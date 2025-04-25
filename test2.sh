#!/bin/bash

INTERFACES=("eth0" "eth1")  # adjust as needed

start_ptp(){
    sudo killall ptp4l phc2sys 2>/dev/null
    sleep 1

    for IF in "${INTERFACES[@]}"; do
        echo "[STARTING] ptp4l on $IF"
        sudo ptp4l -i "$IF" -2 >/dev/null 2>&1 &
    done

    echo "[STARTING] phc2sys: ${INTERFACES[1]} => ${INTERFACES[0]} => system clock"
    sudo phc2sys -s "${INTERFACES[1]}" -c "${INTERFACES[0]}" -O 0 >/dev/null 2>&1 &
    sudo phc2sys -s "${INTERFACES[0]}" -c CLOCK_REALTIME -O 0 >/dev/null 2>&1 &

    echo "[DONE] All PTP services started."
}

stop_ptp(){
    echo "[STOPPING] all PTP services..."
    sudo killall ptp4l phc2sys 2>/dev/null
}

status_ptp(){
    if pgrep -x ptp4l >/dev/null && pgrep -x phc2sys >/dev/null; then
        echo "[OK] PTP services are running."
    else
        echo "[FAIL] PTP services are NOT running."
    fi
}

monitor_ptp(){
    local LEVEL=$1
    stop_ptp
    echo "=== MONITOR MODE (level $LEVEL) ==="
    echo "Press Ctrl-C to exit."

    # always show ptp4l
    for IF in "${INTERFACES[@]}"; do
        sudo ptp4l -i "$IF" -2 -m 2>&1 \
          | sed "s/^/[$IF] /" &
    done

    # level 2 adds phc2sys
    if [ "$LEVEL" -ge 2 ]; then
        sudo phc2sys -s "${INTERFACES[1]}" -c "${INTERFACES[0]}" -O 0 -m 2>&1 \
          | sed "s/^/[${INTERFACES[1]}=>${INTERFACES[0]}] /" &
        sudo phc2sys -s "${INTERFACES[0]}" -c CLOCK_REALTIME -O 0 -m 2>&1 \
          | sed "s/^/[${INTERFACES[0]}=>RTC] /" &
    fi

    wait
}

case "$1" in
    start)    start_ptp                ;;
    stop)     stop_ptp                 ;;
    restart)  stop_ptp; sleep 1; start_ptp ;;
    status)   status_ptp               ;;
    monitor)
        LEVEL=${2:-1}
        if [[ "$LEVEL" =~ ^[12]$ ]]; then
            monitor_ptp "$LEVEL"
        else
            echo "Usage: $0 monitor {1|2}"
            exit 1
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|monitor}"
        exit 1
        ;;
esac

exit 0

