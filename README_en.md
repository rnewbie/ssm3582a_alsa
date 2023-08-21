[Russian version](README.md)
# Linux ALSA driver for Analog Devices SSM3582A
Simple ALSA Codec class driver for AD SSM3582A I2S amplifier with i2c control. It was developed and tested with [Raspberry Pi Zero W](http://www.raspberrypi.org) and [rdc2-0059 board by ChipDip](https://www.chipdip.ru/product/rdc2-0059). Source code provided as is. Please open an issue on bugs found. If it works with you custom card, please let me know.
## Installation
Standard linux module compilation process described in internet better than I could. Reference Makefile provided. DeviceTree description example provided using  snd-simple-sound-card in file (rdc2-0059.dts). Dont't forget to compile it with  dtc and put at proper location (/boot/overlays/ for raspbian), for example:`dtc -O dtb -o /boot/overlays/rdc2-0059.dtbo rdc2-0059.dts`, and uncomment/add following strings at /boot/config.txt:
```
dtparam=i2c_arm=on
dtparam=i2s=on
dtoverlay=rdc2-0059
```
## Tested
- DAC bit/frequency settings according to PCM stream
- Most of registers exported to sysfs with write access as:`echo 0xNN > /sys/bus/i2c/devices/1-0010/register`
- Stereo mode switches on from device tree with `stereo` parameter
- Limiters voltage settings from device tree with volts parameter
- Limiters control from alsa mixer
## ToDo
- TDM support for chaining multiple chips on single I2S
- Why voltage and temperature measurements don't show on my boards?
## License
GNU General Public License v2.0