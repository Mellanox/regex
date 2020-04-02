export RTE_TARGET=x86_64-native-linuxapp-gcc
export RTE_SDK=$PWD

#make   EXTRA_CFLAGS=" -O0 -g3"
#sudo gdb --args ./build/app/dpdk-test-pmd-regexdev -c 0x3 -n 1 -w 82:00.0

make install -j 18 T=x86_64-native-linuxapp-gcc  EXTRA_CFLAGS=" -O0 -g3" CONFIG_RTE_LIBRTE_MLX5_DEBUG=n CONFIG_RTE_IBVERBS_LINK_DLOPEN=n CONFIG_RTE_LIBRTE_MLX5_PMD=n CONFIG_RTE_LIBRTE_MLX5_REGEX_PMD=y CONFIG_RTE_LIBRTE_REGEXDEV=y


