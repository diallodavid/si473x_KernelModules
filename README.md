# kernel modules for the fm radio receiver with rds for raspberry pi

https://www.elektormagazine.com/labs/fm-radio-receiver-with-rds-for-raspberry-pi

How to build:

install headers

    sudo apt-get install -y binutils bc libncurses5-dev
    sudo apt-get install -y v4l-utils
    sudo wget https://raw.githubusercontent.com/notro/rpi-source/master/rpi-source -O /usr/bin/rpi-source && sudo chmod +x /usr/bin/rpi-source && /usr/bin/rpi-source -q --tag-update
    rpi-source

build and install modules

    sudo /home/pi/rpi-receiver*/install.sh -c

