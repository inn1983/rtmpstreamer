#include <stdio.h>  
#include "RTMPStream.h"  
  
int main(int argc,char* argv[])  
{  
	char* server;
	bool bEncode;
	if(argc == 2)
	{
		server = argv[1];
		bEncode = true;
	}
    else {
		server = argv[2];
		bEncode = false;
	}
	
	CRTMPStream rtmpSender(bEncode);
	bool bRet = rtmpSender.Connect(server);
	printf("connect to rtmp server:%s.\n", server);
  
    //rtmpSender.SendH264File("E:\\video\\test.264");
	if(argc == 2){
		rtmpSender.SendCapEncode();
	}
	else if(argc == 3){
		rtmpSender.SendH264File(argv[1]);
		printf("send data:%s.\n", argv[1]);
	}
	
    rtmpSender.Close();  
}  
