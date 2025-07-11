CRAZYFLIE_BASE := external/crazyflie-firmware

OOT_CONFIG := $(PWD)/config 
EXTRA_CFLAGS += -I$(PWD) -I$(PWD)/external/stedgeai/Inc -I$(PWD)/external/rl_tools/include -DRL_TOOLS_CONTROLLER -Wno-error=double-promotion -Wno-error=unused-local-typedefs -Wno-error=missing-braces -Wno-error=sign-compare -Wno-error=unused-variable

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

.PHONY: clean-all
clean-all:
	$(MAKE) clean
	rm -f *.o .*.cmd .*.d
	find . -type f \( -name '*.o' -o -name '.*.cmd' -o -name '.*.d' \) -delete
	rm -rf build/