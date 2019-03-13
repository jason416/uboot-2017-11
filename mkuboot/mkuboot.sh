#!/bin/bash

if [ ! -f E4412_N.bl1.bin ] ; then
	echo "not find files: E4412_N.bl1.bin !!!"
	exit 0
fi

if [ ! -f env.bin ] ; then
	echo "not find files: env.bin !!!"
	exit 0
fi

cat E4412_N.bl1.bin itop4412-spl.bin env.bin u-boot.bin > u-boot-iTOP-4412.bin

if [ -f u-boot-iTOP-4412.bin ] ; then
	echo "created u-boot-iTOP-4412.bin success!!!"
else
	echo "created u-boot-iTOP-4412.bin failed!!!"
	exit 0
fi

echo "writting ..."

if [ -z $1 ] ; then
	dd iflag=dsync oflag=dsync if=u-boot-iTOP-4412.bin of=/dev/sdb seek=1
else
	dd iflag=dsync oflag=dsync if=u-boot-iTOP-4412.bin of=$1 seek=1
fi

echo "writting success"
