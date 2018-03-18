/*
 * Copyright (C) 2016 Dimitris Geromichalos
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

 /**
  * This module is responsible for the communication between the platform and the CAN bus.
  * Implements the functionality needed to receive and transmit a CAN frame using
  * the hardware's CAN driver.
  * All the possible Rx and Tx CAN IDs are registered upon initialization.
  *
  * This interface is hosted in BCM2837 and uses the MCP2515 CAN controller.
  * It is hosted under Linux as a network driver, using SocketCan.
  * The CAN IRQ task of the scheduler (that is running in parallel to the main task)
  * is also implemented here.
  *
  * On initialization, a socket is opened and the Rx IDs are registered to a filter.
  * On transmition, the socket is polled for a specific amount of time until it is
  * available for use.
  * On reception, the received CAN frame is stored in a list that is copied
  * synchronously to the main module.
  */

/******************************** Inclusions *********************************/

#define _GNU_SOURCE

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>

#include <pthread.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "common_types.h"

#include "reconfigure.h"

#include "platform_params.h"
#include "can_protocol.h"
#include "sensor_interface.h"
#include "main_interface.h"

#include "can_interface.h"

/****************************** Macro Definitions ****************************/

/** The name of interface to receive from any CAN interface. */
#define DEVICE "can0"

/** The id of the CAN frame that contains the configuration parameters. */
#define CFG_FRAME_ID (0x735u)

/** The time that the can socket is polled until available for transmition. */
#define TX_TIMEOUT (1u)  /* ms */

/************************** Socket Static Variables **************************/

static int canSocket;
static struct sockaddr_can addr;
static struct ifreq ifr;

static struct can_filter rfilter[NUM_RX_OBJS + 1];

static struct pollfd fds;

static fd_set rdfs;
static char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
static struct iovec iov;
static struct msghdr msg;

static struct canfd_frame rxFrame;
static struct canfd_frame txFrame;

/* static pthread_mutex_t mutex_irq = PTHREAD_MUTEX_INITIALIZER; */
/* static pthread_mutex_t mutex_socket = PTHREAD_MUTEX_INITIALIZER; */

/************************* General Static Variables **************************/

/** The CAN frame that holds a received configuration. */
CanFrame_t cfgFrame;

/** The flag array containing the reception status of each frame for a cycle. */
static u8_t prefusedFrameReceived[NUM_RX_OBJS];

/** The list that holds the received CAN frames. */
static CanFrame_t rxFrameList[NUM_RX_OBJS];

/** The index of the frame list associated with a received CAN frame's ID. */
static u8_t frameListIndex;

/************************ Static Function Prototypes *************************/

/**
  * @brief Transmit the given CAN frame.
  * @details Attempt to write the given frame to the can socket.
  *          If it's not available try resending (poll the socket).
  * @todo Make param const.
  * @return Whether the transmition was successful.
  */
static int SocketCanTransmit(struct canfd_frame* frame);

/***************************** Static Functions ******************************/

int SocketCanTransmit(struct canfd_frame* frame)
{
        int ret = 0u;

        /* (void)pthread_mutex_lock(&mutex_socket); */

resend:
        if (write(canSocket, frame, CAN_MTU) != CAN_MTU)
        {
                if (errno != ENOBUFS)
                {
                        perror("write");
                        ret = 1u;
                }
                else
                {
                        if (poll(&fds, 1, TX_TIMEOUT) < 0)
                        {
                                perror("poll");
                                ret = 1u;
                        }
                        else
                        {
                                goto resend;
                        }
                }
        }

        /* (void)pthread_mutex_unlock(&mutex_socket); */

        return ret;
}

/******************************* Public Functions ****************************/

void InitializeCanInterface(void)
{
        u8_t i;

        /* Socket settings */
        if ((canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
                perror("socket");
                /* return 1; */
        }

        memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
        strcpy(ifr.ifr_name, DEVICE);
        if (ioctl(canSocket, SIOCGIFINDEX, &ifr) < 0)
        {
                perror("SIOCGIFINDEX");
                /* return 1; */
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
                perror("bind");
                /* return 1; */
        }

        /* Filter settings */
        for (i = 0; i < NUM_RX_OBJS; i++)
        {
                MapIndexToIdRx(i, (u16_t*)(&rfilter[i].can_id));

                rfilter[i].can_mask = 0xFFFu & (~CAN_ERR_FLAG);
        }

        rfilter[NUM_RX_OBJS].can_id = 0x735;
        rfilter[NUM_RX_OBJS].can_mask = 0xFFFu & (~CAN_ERR_FLAG);

        setsockopt(canSocket, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, (NUM_RX_OBJS + 1) * sizeof(struct can_filter));

        /* Transmition settings */
        fds.fd = canSocket;
        fds.events = POLLOUT;

        /* Reception settings */
        iov.iov_base = &rxFrame;
        iov.iov_len = CAN_MTU;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = &addr;
        msg.msg_namelen = sizeof(addr);
        msg.msg_control = &ctrlmsg;
        msg.msg_controllen = sizeof(ctrlmsg);  
        msg.msg_flags = 0;

        /* return 0; */
}

void CopyPrefusedFrameList(u8_t* receivedList, CanFrame_t* frameList)
{
        /* (void)pthread_mutex_lock(&mutex_irq); */

        (void)memcpy(receivedList, prefusedFrameReceived, sizeof(u8_t) * (u32_t)NUM_RX_OBJS);
        (void)memcpy(frameList, rxFrameList, sizeof(CanFrame_t) * (u32_t)NUM_RX_OBJS);

        /* (void)pthread_mutex_unlock(&mutex_irq); */
}

void TransmitCanFrame(CanFrame_t* canFrame)
{
        txFrame.can_id = (u32_t)canFrame->id;
        txFrame.len = canFrame->dlc;
        memcpy(&txFrame.data[0], &canFrame->data8[0], sizeof(u8_t) * txFrame.len);

        (void)SocketCanTransmit(&txFrame);
}

void ResetRxBuffers(void)
{
        (void)memset(prefusedFrameReceived, 0, sizeof(u8_t) * (u32_t)NUM_RX_OBJS);
        (void)memset(rxFrameList, 0, sizeof(CanFrame_t) * (u32_t)NUM_RX_OBJS);
}

/***************************** Scheduler Task ********************************/

void* CAN_IRQ_TASK(void* ptr)
{
        while (1)
        {
                /* (void)pthread_mutex_lock(&mutex_socket); */

                FD_ZERO(&rdfs);
                FD_SET(canSocket, &rdfs);

                if (FD_ISSET(canSocket, &rdfs))
                {
                        /* (void)pthread_mutex_lock(&mutex_irq); */

                        if (recvmsg(canSocket, &msg, 0) != CAN_MTU)
                        {
                                perror("read");
                        }
                        else
                        {
                                if (rxFrame.can_id == CFG_FRAME_ID)
                                {
                                    (void)memcpy(cfgFrame.data8, &rxFrame.data[0], sizeof(u8_t) * rxFrame.len);

                                    CfgCallback((u8_t)cfgFrame.data8[4], *(f32_t*)&cfgFrame.data32[0]);
                                }

                                if (MapIdToIndexRx(&frameListIndex, (u16_t)(rxFrame.can_id)))
                                {
                                    prefusedFrameReceived[frameListIndex] = TRUE;

                                    rxFrameList[frameListIndex].id = (u16_t)(rxFrame.can_id);
                                    (void)memcpy(rxFrameList[frameListIndex].data8, &rxFrame.data[0], sizeof(u8_t) * rxFrame.len);
                                }
                        }

                        /* (void)pthread_mutex_unlock(&mutex_irq); */
                }

                /* (void)pthread_mutex_unlock(&mutex_socket); */
        }

        return (void*)NULL;
}
