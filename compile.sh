export CROSS_COMPILE=arm-linux-gnueabihf-
export ARCH=arm

make buyutech_rk3288_defconfig
make -j8
