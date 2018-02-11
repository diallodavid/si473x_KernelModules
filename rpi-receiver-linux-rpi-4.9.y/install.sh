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
  make
  if [ $? -ne 0 ]; then # Test exit status of make.
    echo -e "${RED}make error ${$?}!${NC}"
    exit $?
  fi
  # copy modules to a folder named after the kernelversion
  mkdir -p $(uname -r)
  cp "codecs/snd-soc-ssm2518.ko" "$(uname -r)/"
  cp "si473x/si473x-core.ko" "$(uname -r)/"
  cp "si473x/radio-si473x.ko" "$(uname -r)/"
  cp "si473x/snd-soc-si473x.ko" "$(uname -r)/"
  cp "clk/clk-cs2300.ko" "$(uname -r)/"
  cp "rpiReceiver/snd-soc-rpi-receiver.ko" "$(uname -r)/"
fi

if [ -f "$(uname -r)/snd-soc-ssm2518.ko" ]; then
  cp "$(uname -r)/snd-soc-ssm2518.ko" "/lib/modules/$(uname -r)/kernel/sound/soc/codecs/"
else
  echo -e "${RED}codecs/snd-soc-ssm2518.ko not found!${NC}"
  exit $?
fi

if [ -f "$(uname -r)/si473x-core.ko" ]; then
  cp -n "$(uname -r)/si473x-core.ko" "/lib/modules/$(uname -r)/kernel/drivers/mfd/"
else
  echo -e "${RED}si473x/si473x-core.ko not found!${NC}"
  exit $?
fi
if [ -f "$(uname -r)/radio-si473x.ko" ]; then
  cp "$(uname -r)/radio-si473x.ko" "/lib/modules/$(uname -r)/kernel/drivers/media/radio/"
else
  echo -e "${RED}si473x/radio-si473x.ko not found!${NC}"
  exit $?
fi
if [ -f "$(uname -r)/snd-soc-si473x.ko" ]; then
  cp "$(uname -r)/snd-soc-si473x.ko" "/lib/modules/$(uname -r)/kernel/sound/soc/codecs/"
else
  echo -e "${RED}si473x/snd-soc-si473x.ko not found!${NC}"
  exit $?
fi
if [ -f "$(uname -r)/clk-cs2300.ko" ]; then
  cp "$(uname -r)/clk-cs2300.ko" "/lib/modules/$(uname -r)/kernel/drivers/clk/"
else
  echo -e "${RED}clk/clk-cs2300.ko not found!${NC}"
  exit $?
fi
if [ -f "$(uname -r)/snd-soc-rpi-receiver.ko" ]; then
  cp "$(uname -r)/snd-soc-rpi-receiver.ko" "/lib/modules/$(uname -r)/kernel/sound/soc/bcm/"
else
  echo -e "${RED}rpiReceiver/snd-soc-rpi-receiver.ko not found!${NC}"
  exit $?
fi
depmod -a
