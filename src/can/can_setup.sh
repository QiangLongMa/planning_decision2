#!/bin/bash

### BEGIN INIT INFO
# Provides:          setup_can
# Required-Start:    $network $syslog
# Required-Stop:     $network $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Setup CAN interface
# Description:       Configure CAN interface on startup
### END INIT INFO
# 加载驱动
sudo -S modprobe can << EOF 
nvidia
EOF

sudo -S modprobe can_raw << EOF
nvidia
EOF

sudo -S modprobe mttcan << EOF
nvidia
EOF

# 配置参数（以can0为例）
sudo -S ip link set can0 up type can bitrate 500000 dbitrate 5000000 sjw 4 restart-ms 1000 berr-reporting on fd on loopback off << EOF
nvidia
EOF
