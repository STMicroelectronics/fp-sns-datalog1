#!/bin/bash
# Setup script for hsdatalog under Linux on Raspberry pi 3b
# NOTE: unplug the board before running the script
echo "NOTE: unplug the board before running the script"

sudo cp ../lib/hs_datalog/libhs_datalog_rpi3b.so /usr/lib

sudo cp 30-hsdatalog.rules /etc/udev/rules.d


# Check if the group already exists
if grep -q "hsdatalog" /etc/group
then
	echo "hsdatalog group exists"
else
	#create hsdatalog group
	echo "Adding hsdatalog group"
	sudo addgroup hsdatalog
	echo "Adding user to hsdatalog group"
	sudo usermod -aG hsdatalog $USER
fi

# Then restart udev
echo "Reloading udev rules"
sudo udevadm control --reload

chmod +x ../bin/cli_example_rpi3b


