#!/bin/bash

while true; do
	DISK=$(lsblk --fs | grep sd.[0-9])
	IFS=' '

	read -ra ADDR <<< "$DISK"
	len=${#ADDR[@]}

	if [ $len -lt 6 ]
	then
		echo + Disk unmounted, trying to mount it...
		tmp=$(echo ${ADDR[0]} | cut -d "s" -f2)
		mountpoint=$(echo "s${tmp}")
		sudo mount -t ext4 /dev/$mountpoint /disks/DataVolume
	fi
	sleep 1m
done

