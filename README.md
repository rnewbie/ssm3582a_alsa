[English version](README_en.md)
# Linux драйвер для усилителя Analog Devices SSM3582A
В данном репозитории расположен исходный код ALSA Codec class драйвера для усилителя на микросхеме SSM3582A с управлением по i2c. Код отлаживался на [Raspberry Pi Zero W](http://www.raspberrypi.org) с платой [rdc2-0059 производства ChipDip](https://www.chipdip.ru/product/rdc2-0059). Исходный код предоставляется как есть. У меня работает, если у вас нет - пишите, чем смогу - помогу.
## Установка
Процесс компиляции - стандартный для модуля ядра linux и неоднократно описан в интернете. Пример описания DeviceTree, с использованием общего модуля snd-simple-sound-card приведён в файле (rdc2-0059.dts). Для raspberry его надо скомпилировать с помощью dtc и поместить в /boot/overlays/, например так `dtc -O dtb -o /boot/overlays/rdc2-0059.dtbo rdc2-0059.dts`, после чего добавить/раскомментировать в /boot/config.txt следующий строки:
```
dtparam=i2c_arm=on
dtparam=i2s=on
dtoverlay=rdc2-0059
```
и перезагрузиться.
## Рабочий функционал
- Переключение битности/частоты DAC в соответствии с потоком
- Экспорт в sysfs большинства регистров усилителя для просмотра и управления в ручном режиме путем записи командой `echo 0xNN > /sys/bus/i2c/devices/1-0010/register`
- включение стерео режима усилителя из device tree опцией stereo
- установка напряжения ограничителей, в соответствии с поданым питанием, опцией volts
- управление ограничителями и громкостью с помощью стандартных средств ALSA
## Планы
- Поддержка режима TDM для установки до 8 усилителей на один интерфейс I2S, для чего видимо придётся написать Machine Class driver т.к. в simple card я этого не нашёл.
- Почему-то не работают регистры температуры и напряжения (на моих экземплярах).
## Лицензия
GNU General Public License v2.0