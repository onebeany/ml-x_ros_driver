#!/bin/bash

INTERFACES=("eth0" "eth1")  # Change this based on your connection
CONF="/etc/linuxptp/ptp4l.conf"

start_ptp(){
	sudo killall ptp4l phc2sys 2>/dev/null
	sleep 1
	for IF in "${INTERFACES[@]}"; do
		echo "Starting ptp4l on $IF"
		sudo ptp4l -i $IF -m 
		echo "Starting phc2sys on $IF"
		sudo phc2sys -w -s $IF -O 0 -m
	done
	echo "All PTP services started"
}

stop_ptp(){
	echo "Stopping all PTP services"
	sudo killall ptp4l phc2sys
}



case "$1" in
	
	start) start_ptp ;;
	stop) stop_ptp ;;
	restart) stop_ptp; sleep 1; start_ptp ;;
	status)
		pgrep -x ptp4l && pgrep -x phc2sys \
		&& ps aux | grep -E 'ptp4l|phc2sys' | grep -v grep \
		|| echo "PTP services are NOT running"
		;;
	*)
		echo "Usage: $0 {start|stop|restart|status}"
		exit 1
		;;
esac
exit 0

