#!/bin/bash

LOG_FILE="/var/log/ps5-auto-pair.log"

echo "-------------------------------------" | tee -a $LOG_FILE
echo "$(date): PS5 Auto-Pair Script Started" | tee -a $LOG_FILE

# Enable Bluetooth and scanning
echo "Enabling Bluetooth..." | tee -a $LOG_FILE
bluetoothctl power on >> $LOG_FILE 2>&1
bluetoothctl agent on >> $LOG_FILE 2>&1
bluetoothctl default-agent >> $LOG_FILE 2>&1

echo "Scanning for PS5 controllers..." | tee -a $LOG_FILE
bluetoothctl scan on &

# Allow more time for multiple controllers to be detected
sleep 10  

# Get all detected PS5 controllers dynamically
FOUND_MACS=$(bluetoothctl devices | grep "Wireless Controller" | awk '{print $2}')

if [ -n "$FOUND_MACS" ]; then
    echo "PS5 Controllers found:" | tee -a $LOG_FILE
    echo "$FOUND_MACS" | tee -a $LOG_FILE

    for MAC in $FOUND_MACS; do
        # Check if the controller is already paired
        PAIRED=$(bluetoothctl paired-devices | grep "$MAC")

        if [ -z "$PAIRED" ]; then
            echo "Pairing new controller: $MAC" | tee -a $LOG_FILE
            bluetoothctl pair "$MAC" >> $LOG_FILE 2>&1
        else
            echo "Controller already paired: $MAC" | tee -a $LOG_FILE
        fi

        echo "Trusting controller: $MAC" | tee -a $LOG_FILE
        bluetoothctl trust "$MAC" >> $LOG_FILE 2>&1

        echo "Connecting to controller: $MAC" | tee -a $LOG_FILE
        bluetoothctl connect "$MAC" >> $LOG_FILE 2>&1
    done

    echo "✅ All detected controllers have been paired/trusted/connected" | tee -a $LOG_FILE
else
    echo "❌ No PS5 controllers found via Bluetooth scan" | tee -a $LOG_FILE
fi

echo "Stopping Bluetooth scan..." | tee -a $LOG_FILE
bluetoothctl scan off >> $LOG_FILE 2>&1

echo "Script execution complete." | tee -a $LOG_FILE
