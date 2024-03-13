This is a camera module driver for Raspberry Pi that is running debian/raspbian. The driver was developed for the ov5647 camera sensor using the Arducam 5MP OV5647. 
It was only developed for educational purposes and was inspired by the imx219 as a model recommended by Raspberry for developing third-party drivers. The official release of the ov5647 as part of the Raspberry Linux repo wasn't consulted. 

The driver was developed using the V4L2 framework and lib-camera as well as its applications like libcamera-hello, libcamera-vid
and libcamera-jpeg tools for testing the driver. 

Hardware required: 
- raspberry pi 4: https://www.raspberrypi.com/products/raspberry-pi-4-model-b/
- camera module: https://www.arducam.com/product/arducam-ov5647-standard-raspberry-pi-camera-b0033/

Prerequisite 
- In the `/boot/config.txt`. comment out `dtoverlay ov5647` or any other preload camera modulule.
- This prevents the module from loading automially on boot up so you can manually load it when its compiled correctly.

Build instructions (the ones I followed): 
- clone the Raspberry Pi raspbian kernel on your pi - https://github.com/raspberrypi/linux
- update the release ov5647 driver. This can be done with sftp to overwrite ov5647.c with this repo's. Copy the file to this path `linux/drivers/media/i2c`.
- compile the modules with the following command `make -j16 modules`. This will generate .ko files including one for the ov5647.
- copy the `compile.sh` in the repo to the root path of your raspberry Linux folder. Then run the compile.sh to update the kernel.
- reboot the pi
- after rebooting run `sudo dtoverlay ov5647` to install the driver using the device tree.
  
