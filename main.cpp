#include <stdio.h>  
#include "RTMPStream.h"  
  
int main(int argc,char* argv[])  
{  
    CRTMPStream rtmpSender;  
  
    //bool bRet = rtmpSender.Connect("rtmp://192.168.1.104/live/test");
	bool bRet = rtmpSender.Connect(argv[2]);
	printf("connect to rtmp server:%s", argv[2]);
  
    //rtmpSender.SendH264File("E:\\video\\test.264");
	rtmpSender.SendH264File(argv[1]);
	printf("send data:%s", argv[1]);

    rtmpSender.Close();  
}  