https://doc.miivii.com/Apex-AD10-User-Manual-CH/wiki/Apex-AD10-CAN.html

#加载驱动
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
#配置参数（以can1为例）
sudo ip link set can1 up type can bitrate 500000 dbitrate 5000000 sjw 4 restart-ms 1000 berr-reporting on fd on loopback off


echo 1024 > /sys/class/net/can2/tx_queue_len

#运行can_setup.sh   开机自启
sudo cp can_setup.sh /etc/init.d/can_setup
sudo chmod +x /etc/init.d/can_setup
sudo update-rc.d can_setup defaults


