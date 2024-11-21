echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="2a1940393c83eb118a10cc7f9693f7bc", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_ins622"' >/etc/udev/rules.d/wheeltec_ins622.rules

service udev reload
sleep 2
service udev restart


