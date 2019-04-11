###############################
#Raspberry Pi OS and ROS Installation 
###############################

#Download from following website:
	https://downloads.ubiquityrobotics.com/pi.html
#the image with version:
	2019-02-19-ubiquity-xenial-lxde

#Burn it on a SD-Card and start it with the Raspberry Pi 3

#Install updates after Start and Login
sudo apt-get install update
sudo apt-get install upgrade

###############################
#Run DS3231 RTC on Raspberry PI 
###############################

#install updates and dependencies
sudo apt-get install update
sudo apt-get install upgrade
sudo apt-get install i2c-tools
sudo apt-get install ntp

#Enable I2C 
Run: sudo raspi-config
- Choose Interfacing Options -> I2C -> Enable 

#Check, if RTC is detected on I2C Bus
sudo i2cdetect -y 1
- You should see the number 68 (this is the adress of the rtc)

#Load the clock at boot
sudo nano /etc/rc.local
- Add the following lines before exit 0:
	echo ds1307 0x68 > /sys/class/i2c-adapter/i2c-1/new_device
	hwclock -s

#Set date and time options
- set the pi's system time to Internet time:
	sudo nptd -g -q 
- check the system time:
	date
- check the date and time of the RTC:
	sudo hwclock -r 
- write the system time to the RTC:
	sudo hwclock -w
- set the system time from the RTC:
	sudo hwclock -s
- Compair system and rtc time:
	sudo timedatectl

###############################
#Install PIGPIO on Raspberry PI 
###############################
#Install PIGPIO (for parallel GPIO write)
	http://abyz.me.uk/rpi/pigpio/cif.html
#activate pigpio
	sudo pigpiod

