RAFT
=====

RAFT is a system for fusing tracked objects from multiple radar sensors and can be used in a self-driving application with the aim to provide surround sensing capabilities.
The algorithm for fusion and tracking uses a Kalman Filter that estimates the distance and velocity of each tracked object.

The experimental platform was designed for a BCM2837 (Raspberry Pi 3) target running on a Linux kernel patched with RT-PREEMPT (real-time preemption).
The sensor (radar) interface is implemented using SocketCAN with a MCP2515 board.
