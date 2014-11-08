#!/bin/bash -x
RED5_INSTALL_DIR=/home/cubie/wkdir/red5/red5-server-1.0
STREAMER_INSTALL_DIR=/home/cubie/wkdir/rtmpstreamer

sleep 10

#cd /home/cubie/wkdir/rtmptest/crtmp/builders/cmake
#su -s /bin/bash -c "nohup crtmpserver/crtmpserver crtmpserver/crtmpserver.lua &" cubie

#crtmpserver should be daemon!!! when use nohup, the behavior of crtmpserver will be worng!! 
#su -s /bin/bash -c "crtmpserver/crtmpserver crtmpserver/crtmpserver.lua" cubie
cd $RED5_INSTALL_DIR
su -s /bin/bash -c "./red5.sh &" cubie

sleep 30

#enable alsa capture.
amixer -c 0 cset numid=25 0

cd $STREAMER_INSTALL_DIR

chmod 777 /dev/sunxi_mem
chown cubie:cubie /dev/sunxi_mem

#echo "cubie" | sudo chmod a+r /dev/sunxi_mem
#echo "cubie" | sudo chmod a+w /dev/sunxi_mem
#chmod a+r /dev/sunxi_mem
#chmod a+w /dev/sunxi_mem

#echo "rtmp_streamer start !!"

#su -s /bin/bash -c "arecord -Dplughw:0,0 -fS16_LE -r44100 -c1 > WavInFifo.wav 2>arec.txt &" cubie
#sleep 2
su -s /bin/bash -c "./rtmp_streamer rtmp://localhost/oflaDemo/streamer>/dev/null 2>./error.log&" cubie
#su -s /bin/bash -c "./rtmp_streamer rtmp://192.168.2.107/flvplayback/streamer > /dev/null 2>&1 &" cubie
#su -s /bin/bash -c "nohup ./rtmp_streamer rtmp://172.16.200.1/flvplayback/streamer &" cubie
#su -s /bin/bash -c "./rtmp_streamer rtmp://192.168.2.107/flvplayback/streamer > ./log.txt 2>./log.txt &" cubie
