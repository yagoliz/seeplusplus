sudo cp 2-seeplusplus-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
