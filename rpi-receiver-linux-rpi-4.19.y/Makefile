obj-$(CONFIG_SND_SOC)	+= drivers/
obj-$(CONFIG_SND_SOC)	+= sound/

export CONFIG_RADIO_SI473X := m
export CONFIG_SND_SOC_SI473X := m
export CONFIG_MFD_SI473X_CORE := m
# export CONFIG_RADIO_SI476X := m
# export CONFIG_SND_SOC_SI476X := m
# export CONFIG_MFD_SI476X_CORE := m
export CONFIG_SND_SOC_SSM2518 := m
export CONFIG_COMMON_CLK_CS2300 := m
export CONFIG_SND_RPI_RECEIVER_CARD := m

export INCLUDEDIR :=  $(dir $(abspath $(lastword $(MAKEFILE_LIST))))include
$(info $$INCLUDEDIR is [${INCLUDEDIR}])
LINUXINCLUDE += -I$(INCLUDEDIR)
#CC += -I$(CURDIR)/../rpi-receiver-linux-rpi-4.19.y/include

all: clean compile

compile:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install