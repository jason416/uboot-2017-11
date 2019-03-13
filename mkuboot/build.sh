#!/bin/bash

cd ../

if [ ! -f .config ]
then
	make itop4412_defconfig
fi

make -j4 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

cp u-boot.bin ./mkuboot/
echo "copy u-boot.bin done."

cd spl/
if [ ! -f itop4412-spl.bin ] ; then
	echo "notice: not found itop4412-spl.bin !"
	exit 0
else
	echo "copying itop4412-spl.bin..."
fi

cp itop4412-spl.bin ../mkuboot/
echo "copy u-boot-spl.bin done."

echo "build success !!!"
