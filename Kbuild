obj-y += rl_tools_controller.o
# obj-y += rl_tools_adapter_old.o
obj-y += rl_tools_adapter.o
obj-y += network/network.o network/network_data.o
# obj-y += baseline_adapter.o

ldflags-y += -Wl,--whole-archive $(PWD)/external/stedgeai/Lib/GCC/ARMCortexM4/NetworkRuntime1010_CM4_GCC.a -Wl,--no-whole-archive