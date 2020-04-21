./imu_test /dev/video0   #测试重力加速度
./spi_test /dev/video0   #擦除SPI FLASH 
./（vcmipidemo-c，vcmipidemo-yuv-8bit-c）vcmipidemo-dual-8bit-c -s (曝光值) -g 0(Gain值，增益1,2,3) ，
 #color，dual代表双目

./vcmipidemo-dual-8bit-y,vcmipidemo-y，vcmipidemo-yuv-8bit-y -s (曝光值) -g 0(Gain值，增益1,2,3) ，               #mono