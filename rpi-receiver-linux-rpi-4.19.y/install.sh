#!/bin/bash
RED='\033[0;31m'
NC='\033[0m' # No Color

# check if sudo is used
if [ "$(id -u)" != 0 ]; then
  echo 'Sorry, you need to run this script with sudo'
  exit 1
fi

cd "$(dirname "$0")"

if [ ! -d "$(uname -r)" ] || [ "$1" == "-c" ] || [ "$1" == "--compile" ]; then
  echo "Checking if build essential is installed"
  if [ $(dpkg-query -W -f='${Status}' make 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    echo -e "${RED}Install build essential first - needed to make modules!${NC}"
    exit $?
  fi
  if [ -d "/lib/modules/$(uname -r)/extra/" ]; then
    rm -r "/lib/modules/$(uname -r)/extra/"
  fi
  make compile install
  if [ $? -ne 0 ]; then # Test exit status of make.
    echo -e "${RED}make error ${$?}!${NC}"
    exit $?
  fi

  # copy modules to a folder named after the kernelversion
  mkdir -p $(uname -r)
  cp -r /lib/modules/$(uname -r)/extra/* $(uname -r)/
  dtc -@ -H epapr -O dtb -o "$(uname -r)/rpi-receiver.dtbo" -W no-unit_address_vs_reg './rpi-receiver-overlay.dts'
  if [ $? -ne 0 ]  # Test exit status of dtc.
  then
    echo -e "${RED}dtc error ${$?}!${NC}"
    exit $?
  fi
  cp "$(uname -r)/rpi-receiver.dtbo" "/boot/overlays/"
fi

if [ "$1" == "--clean" ]; then
  echo "Cleaning up"
  make clean
  if [ -d "/lib/modules/$(uname -r)/extra/" ]; then
    rm -r "/lib/modules/$(uname -r)/extra/"
  fi
fi

if [ -d "$(uname -r)" ]; then
  cp -rv "$(uname -r)/drivers" "/lib/modules/$(uname -r)/kernel/"
  cp -rv "$(uname -r)/sound" "/lib/modules/$(uname -r)/kernel/"
fi

depmod -a

