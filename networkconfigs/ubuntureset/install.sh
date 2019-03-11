sudo update-rc.d hostapd disable
sudo rm /etc/hostapd/hostapd.conf
sudo rm /etc/default/hostapd
sudo rm /etc/rc.local
sudo rm /etc/dnsmasq.conf
sudo apt remove hostapd haveged dnsmasq net-tools


