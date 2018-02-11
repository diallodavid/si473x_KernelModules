obj-$(CONFIG_SND_SOC)	+= si473x/
obj-$(CONFIG_SND_SOC)	+= codecs/
obj-$(CONFIG_SND_SOC)	+= clk/
obj-$(CONFIG_SND_SOC)	+= rpiReceiver/

all: clean compile

compile:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
    
install:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install