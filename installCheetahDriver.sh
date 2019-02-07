#! /bin/bash
echo "Tested on Ubuntu 18.04"
echo "Installing the Cheetah driver..."
sudo cp 99-totalphase.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger 

