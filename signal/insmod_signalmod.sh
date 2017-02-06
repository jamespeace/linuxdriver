#!/bin/sh

B26=`uname -r|cut -d'.' -f2`
if [ "$B26" -eq "4" ]; then
  module="signalmod.o"
else
  module="signalmod.ko"
fi

./ctrlc&

sleep 1  # wait ctrlc exectuation

pid=`ps aux|grep ctrlc|grep -v grep|cut -d' ' -f7`
insmod $module user_pid="$pid" 
