#!/bin/sh
MODULE_NAME=pcnet32

rmmod ${MODULE_NAME}
insmod ./${MODULE_NAME}.ko

sleep 5
ping -c 5 192.168.27.120
