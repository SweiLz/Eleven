#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the usb connection."
echo ""

sudo cp `rospack find eleven_bringup`/99-eleven.rules /etc/udev/rules.d/

echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
