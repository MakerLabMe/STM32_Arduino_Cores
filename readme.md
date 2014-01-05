# MakerLabBoard
This is MakerLabBoard core source files, make Arduino IDE supports STM32F103XX cortex-m3 arm cpu.

中文详细教程:http://makerlab.me/guides/2

# how to install
## download Arduino IDE 1.5
known working version 1.5.4, 1.5.5 beta
http://arduino.cc/en/Main/Software
## download The MakerLabBoard core files.
You can use git or 
zip(https://github.com/MakerLabMe/MakerLabBoard/archive/master.zip).
Then put the folder in the Arduino IDE hardware/arduino directory.Like this:

Mac: right click the Arduino.app --> "Show Package Contents" and go to `Contents/Resources/Java/hardware/arduino`
and copy the core files to there.

Like this:

```sh
hardware/arduino > ls    
avr	sam	stm32
#The folder name must be **stm32**,because I hardcoded the folder name.
hardware/arduino > cd stm32
hardware/arduino/stm32 > ls
firmwares	programmers.txt	system
boards.txt	libraries	readme.md	variants
cores		platform.txt	sloadhost
```

Now restart the Arduino IDE, you can see the Tools>Board will add MakerLab Board.
Then you can use Arduino IDE for stm32f103 based board.

# Todo
more info.
