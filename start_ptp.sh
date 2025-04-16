#!/bin/bash

INTERFACE="eth1" # Change this based on your connection

case "$1" in
	
	start)
		echo "Starting PTP on $INTERFACE..."
		sudo killall ptp4l phc2sys 2>/dev/null
		sleep 1
		sudo ptp4l -i $INTERFACE -m &
		sudo phc2sys -w -s $INTERFACE -O 0 -m
		echo "PTP services started"
		;;
	
	stop)
		echo "Stopping PTP services..."
		sudo killall ptp4l phc2sys
		echo "PTP services stopped"
		;;
	
	restart)
		echo "Restarting PTP services..."
		sudo killall ptp4l phc2sys 2>/dev/null
		sleep 1
		sudo ptp4l -i $INTERFACE -m &
		sudo phc2sys -w -s $INTERFACE -O 0 -m
		echo "PTP services restarted"
		;;
	
	status)
		if pgrep -x "ptp4l" > /dev/null && pgrep -x "phc2sys" > /dev/null
		then
			echo "PTP services are runing"
			ps aux | grep -E 'ptp4l|phc2sys' | grep -v grep
		else
			echo "PTP services are NOT running"
		fi
		;;

	*)
		echo "Usage: $0 {start|stop|restart|status}"
		exit 1
esac

exit 0

