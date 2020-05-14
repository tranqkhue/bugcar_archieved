# Assign fixed device to static USB port
 - **REFERENCE:** https://www.domoticz.com/wiki/Assign_fixed_device_name_to_USB_port
 - Get USB port currently being used by the device: 
 	- **~$ dmesg | grep tty**

 - Check device attributes, replace PORT with the usb port the device is using:
 	- **~$ udevadm info --name=PORT --attribute-walk**

 	- **IMPORTANT**: Take note of values of 
		- **SUBSYSTEM** 
		- **ATTRS{idVendor}**
		- **ATTRS{idProduct}**
		- **ATTRS{serial}**

 - Writing new udev rules:
	- Using Vim to edit:
		- **~$ sudo vi /etc/udev/rules.d/10-usb-serial.rules**
	- Write the following line for each devie, replace the appropriate value for each attributes:
		- **SUBSYSTEM=="TYPE", ATTRS{idVendor}=="ID_VENDOR", ATTRS{idProduct}=="ID_PRODUCT", ATTRS{serial}=="SERIAL",  SYMLINK+="NEW_PORT_NAME"**

 - Load new rule:
	- **~$ sudo udevadm control --reload-rules**

 - Reboot system and verify:
	- **~$ sudo reboot**
	- __~$ ls -l /dev/tty*__ 
	- (see expected results in reference)

 - Repeat step two for the new ports and check if the attributes of device match.
