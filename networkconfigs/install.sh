sudo apt install hostapd haveged dnsmasq
sudo cp hostapd.conf /etc/hostapd/hostapd.conf
sudo cp hostapd /etc/default/hostapd
sudo cp sysctl.conf /etc/sysctl.conf
sudo cp rc.local /etc/rc.local
sudo cp dnsmasq.conf /etc/dnsmasq.conf
sudo cp dnsmasq.service /lib/systemd/system/dnsmasq.service
sudo cp interfaces /etc/network/interfaces
sudo cp NetworkManager.conf /etc/NetworkManager/NetworkManager.conf
sudo cp blacklist.conf /etc/modprobe.d/blacklist.conf
sudo cp vncserver /etc/init.d/vncserver
sudo chmod +x /etc/rc.local
sudo update-rc.d hostapd defaults
sudo update-rc.d hostapd enable

