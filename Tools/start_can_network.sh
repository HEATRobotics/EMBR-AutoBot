#!/usr/bin/env bash

sudo ip link set can0 down
sudo ip link set can0 type can bitrate ${CAN_BIT_RATE}
sudo ip link set can0 up