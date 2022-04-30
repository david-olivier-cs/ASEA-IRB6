
#!/bin/bash
# Setup script for the Raspberrypi
#
# Requirements
#   Only runs on ubuntu
#   Needs internet access and root privileges (run as root)
#   Needs python3
# Usage: 
# ./rpi_setup.sh
# -----------------------------------------------------------------------------

echo -e "\nInstalling the Phidget22 libraries ... \n"
wget -qO- http://www.phidgets.com/gpgkey/pubring.gpg |\
    apt-key add - && echo 'deb http://www.phidgets.com/debian bullseye main' > \
    /etc/apt/sources.list.d/phidgets.list
apt-get -y update
apt-get -y install libphidget22

echo -e "\nInstalling pip3 ... \n"
apt-get -y install python3-pip

echo -e "\nInstalling the required python modules ... \n"
pip3 install -e ../init_irb6

echo -e "\nScheduling the execution of the control script on device (RPI4) power up ... \n"

# creating a systemd service to launch the control script upon startup
cp ./robot.service /lib/systemd/system/robot.service
systemctl daemon-reload
systemctl enable robot.service

echo -e "\nSetup complete, restarting the host device \n"
systemctl -i reboot