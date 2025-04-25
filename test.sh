#!/bin/bash

INTERFACES=("eth0" "eth1")  # adjust as needed

start_ptp(){
    sudo killall ptp4l phc2sys 2>/dev/null
    sleep 1
    for IF in "${INTERFACES[@]}"; do
        echo "[Starting ptp4l on] $IF"
        sudo ptp4l -i "$IF" -2 >/dev/null 2>&1 &
    done
    echo "[Starting phc2sys chain] ${INTERFACES[1]} → ${INTERFACES[0]} → system clock"
    sudo phc2sys -s "${INTERFACES[1]}" -c "${INTERFACES[0]}" -O 0 >/dev/null 2>&1 &
    sudo phc2sys -s "${INTERFACES[0]}" -c CLOCK_REALTIME -O 0 >/dev/null 2>&1 &
    echo "All PTP services started."
}

stop_ptp(){
    echo "Stopping all PTP services..."
    sudo killall ptp4l phc2sys 2>/dev/null
}

status_ptp(){
    if pgrep -x ptp4l >/dev/null && pgrep -x phc2sys >/dev/null; then
        echo "PTP services are running."
    else
        echo "PTP services are NOT running."
    fi
}

monitor_ptp(){
    stop_ptp
    echo "=== MONITOR MODE: ptp4l & phc2sys logs ==="
    echo "Press Ctrl-C to exit."
    for IF in "${INTERFACES[@]}"; do
        sudo ptp4l -i "$IF" -2 -m 2>&1 | sed "s/^/[$IF] /" &
    done
    sudo phc2sys -s "${INTERFACES[1]}" -c "${INTERFACES[0]}" -O 0 2>&1 \
        | sed "s/^/[${INTERFACES[1]}→${INTERFACES[0]}] /" &
    sudo phc2sys -s "${INTERFACES[0]}" -c CLOCK_REALTIME -O 0 2>&1 \
        | sed "s/^/[${INTERFACES[0]}→RTC] /" &
    wait
}

case "$1" in
    start)    start_ptp   ;;
    stop)     stop_ptp    ;;
    restart)  stop_ptp; sleep 1; start_ptp ;;
    status)   status_ptp  ;;
    monitor)  monitor_ptp ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|monitor}"
        exit 1
        ;;
esac

exit 0

