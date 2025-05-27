CRAZYFLIE_BASE := external/crazyflie-firmware

OOT_CONFIG := $(PWD)/config 
EXTRA_CFLAGS += -I$(PWD) -I$(PWD)/external/rl_tools/include -DRL_TOOLS_CONTROLLER -Wno-error=double-promotion -Wno-error=unused-local-typedefs -Wno-error=missing-braces -Wno-error=sign-compare

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
