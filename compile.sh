#!/bin/bash

echo "Set the KERNEL variable"
export KERNEL=kernel8

echo "Build the media drivers"
sudo make modules_install

echo "Copy the necessary device tree blobs to /boot directory"
sudo cp arch/arm64/boot/dts/broadcom/*.dtb /boot/
sudo cp arch/arm64/boot/dts/overlays/*.dtb* /boot/overlays/
sudo cp arch/arm64/boot/dts/overlays/README /boot/overlays/

echo "Copy the kernel image to /boot directory"
sudo cp arch/arm64/boot/Image.gz /boot/$KERNEL.img

