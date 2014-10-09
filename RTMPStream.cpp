/******************************************************************** 
filename:   RTMPStream.h
created:    2014.10.07
author:     inn 
purpose:    librtmpライブラリを使用しH264データをRed5に送信
*********************************************************************/ 
#include "RTMPStream.h"
#include "SpsDecode.h"
#ifdef WIN32  
#include <windows.h>
#endif

#ifdef WIN32
#pragma comment(lib,"WS2_32.lib")
#pragma comment(lib,"winmm.lib")
#endif

#define LOG_NDEBUG 0
#define LOG_TAG "venc-file"
#include "CDX_Debug.h"



AVfifo_t* g_avfifo; //グローバルfifo

enum
{
	FLV_CODECID_H264 = 7,
};

int InitSockets()  
{  
#ifdef WIN32  
	WORD version;  
	WSADATA wsaData;  
	version = MAKEWORD(1, 1);  
	return (WSAStartup(version, &wsaData) == 0);  
#else  
	return TRUE;  
#endif  
}  

inline void CleanupSockets()  
{  
#ifdef WIN32  
	WSACleanup();  
#endif  
}  

char * put_byte( char *output, uint8_t nVal )
{  
	output[0] = nVal;  
	return output+1;  
}  
char * put_be16(char *output, uint16_t nVal )
{  
	output[1] = nVal & 0xff;  
	output[0] = nVal >> 8;  
	return output+2;  
}  
char * put_be24(char *output,uint32_t nVal )  
{  
	output[2] = nVal & 0xff;  
	output[1] = nVal >> 8;  
	output[0] = nVal >> 16;  
	return output+3;  
}  
char * put_be32(char *output, uint32_t nVal )  
{  
	output[3] = nVal & 0xff;  
	output[2] = nVal >> 8;  
	output[1] = nVal >> 16;  
	output[0] = nVal >> 24;  
	return output+4;  
}  
char *  put_be64( char *output, uint64_t nVal )  
{  
	output=put_be32( output, nVal >> 32 );  
	output=put_be32( output, nVal );  
	return output;  
}  
char * put_amf_string( char *c, const char *str )  
{  
	uint16_t len = strlen( str );  
	c=put_be16( c, len );  
	memcpy(c,str,len);  
	return c+len;  
}  
char * put_amf_double( char *c, double d )  
{  
	*c++ = AMF_NUMBER;  /* type: Number */  
	{  
		unsigned char *ci, *co;  
		ci = (unsigned char *)&d;  
		co = (unsigned char *)c;  
		co[0] = ci[7];  
		co[1] = ci[6];  
		co[2] = ci[5];  
		co[3] = ci[4];  
		co[4] = ci[3];  
		co[5] = ci[2];  
		co[6] = ci[1];  
		co[7] = ci[0];  
	}  
	return c+8;  
}

static slice_t *slice_alloc(const void *data, int len, int64_t pts)
{
	slice_t *ns = (slice_t *)malloc(sizeof(slice_t));
	ns->data_ = malloc(len);
	memcpy(ns->data_, data, len);
	ns->pts_ = pts;
	ns->len_ = len;

	return ns;
}

static slice_t *slice_alloc(const void *p1, int l1, const void *p2, int l2, int64_t pts)
{
	slice_t *ns = (slice_t*)malloc(sizeof(slice_t));
	ns->data_ = malloc(l1+l2);
	memcpy(ns->data_, p1, l1);
	memcpy((unsigned char*)ns->data_ + l1, p2, l2);
	ns->pts_ = pts;
	ns->len_ = l1+l2;
	return ns;
}

static void slice_free(slice_t *s)
{
	free(s->data_);
	free(s);
}

static void uyvy_nv12(const unsigned char *puyvy, unsigned char *pnv12, int width, int height)
{
	unsigned char *Y = pnv12;
	unsigned char *UV = Y + width * height;
	//unsigned char *V = U + width * height / 4;
	int i, j;

	for (i = 0; i < height / 2; i++) {
		// 奇数行保留 U/V
		for (j = 0; j < width / 2; j++) {
			*UV++ = *puyvy++;	//U
			*Y++ = *puyvy++;
			*UV++ = *puyvy++;	//V
			*Y++ = *puyvy++;
		}

		// 偶数行的 UV 直接扔掉
		for (j = 0; j < width / 2; j++) {
			puyvy++;		// 跳过 U
			*Y++ = *puyvy++;
			puyvy++;		// 跳过 V
			*Y++ = *puyvy++;
		}
	}

}

int get_next_slice(NaluUnit &nalu)
{
	g_avfifo->cs_fifo_.enter();
	while (g_avfifo->fifo_.empty()) {
		g_avfifo->cs_fifo_.leave();
		g_avfifo->sem_fifo_.wait();
		g_avfifo->cs_fifo_.enter();
	}

	slice_t *s = g_avfifo->fifo_.front();
	g_avfifo->fifo_.pop_front();
	fprintf(stderr, "fifo size is %d.\n", g_avfifo->fifo_.size());

	if (s->len_ > g_avfifo->outbuf_size_) {
		g_avfifo->outbuf_size_ = (s->len_ + 4095)/4096*4096;
		g_avfifo->outbuf_ = realloc(g_avfifo->outbuf_, g_avfifo->outbuf_size_);
	}

	memcpy(g_avfifo->outbuf_, s->data_, s->len_);
	nalu.data = (unsigned char*)g_avfifo->outbuf_+4;
	nalu.type = nalu.data[0]&0x1f;
	nalu.size = s->len_-4;

	int rc = nalu.size;
	slice_free(s);

	g_avfifo->cs_fifo_.leave();

	return rc;
}

int CameraSourceCallback(void *cookie,  void *data)
{
	CCapEncoder * venc_cam_cxt = (CCapEncoder *)cookie;
	cedarv_encoder_t *venc_device = venc_cam_cxt->m_venc_device;
	AWCameraDevice *CameraDevice = venc_cam_cxt->m_CameraDevice;
	static int has_alloc_buffer = 0;
	
	VencInputBuffer input_buffer;
	int result = 0;
	//int has_alloc_buffer = 0;
	

	struct v4l2_buffer *p_buf = (struct v4l2_buffer *)data;
	v4l2_mem_map_t* p_v4l2_mem_map = GetMapmemAddress(getV4L2ctx(CameraDevice));	

	void *buffer = (void *)p_v4l2_mem_map->mem[p_buf->index];
	int size_y = venc_cam_cxt->m_base_cfg.input_width*venc_cam_cxt->m_base_cfg.input_height; 

	memset(&input_buffer, 0, sizeof(VencInputBuffer));
	
	do{	//retry
		result = venc_device->ioctrl(venc_device, VENC_CMD_GET_ALLOCATE_INPUT_BUFFER, &input_buffer);
		LOGD("no alloc input buffer right now");
		usleep(10*1000);
	}while(result !=0 );
	
	uyvy_nv12( (unsigned char*)buffer, input_buffer.addrvirY, 720, 480);
	
	//input_buffer.id = p_buf->index;
	//input_buffer.addrphyY = p_buf->m.offset;
	//input_buffer.addrphyC = p_buf->m.offset + size_y;

	//LOGD("buffer address is %x", buffer);
	//input_buffer.addrvirY = buffer;
	//input_buffer.addrvirC = buffer + size_y;

	input_buffer.pts = 1000000 * (long long)p_buf->timestamp.tv_sec + (long long)p_buf->timestamp.tv_usec;
	LOGD("pts = %ll", input_buffer.pts);

#if 1
	if(input_buffer.addrphyY >=  (void*)0x40000000)
		input_buffer.addrphyY -=0x40000000;
#endif

	//if(!venc_cam_cxt->mstart) {
		LOGD("p_buf->index = %d\n", p_buf->index);
	//	CameraDevice->returnFrame(CameraDevice, p_buf->index);
	//}

    // enquene buffer to input buffer quene
    
	LOGD("ID = %d\n", input_buffer.id);
	result = venc_device->ioctrl(venc_device, VENC_CMD_ENQUENE_INPUT_BUFFER, &input_buffer);

	if(result < 0) {
		//CameraDevice->returnFrame(CameraDevice, p_buf->index);
		LOGW("video input buffer is full , skip this frame");
	}
	CameraDevice->returnFrame(CameraDevice, p_buf->index);

	return 0;
}

CCapEncoder::CCapEncoder(void)
{
	m_base_cfg.codectype = VENC_CODEC_H264;
	m_base_cfg.framerate = 30;
	m_base_cfg.input_width = 720;
	m_base_cfg.input_height= 480;
	m_base_cfg.dst_width = 720;
	m_base_cfg.dst_height = 480;
	m_base_cfg.maxKeyInterval = 25;
	m_base_cfg.inputformat = VENC_PIXEL_YUV420; //uv combined
	m_base_cfg.targetbitrate = 3*1024*1024;
	
	m_alloc_parm.buffernum = 4;
	
	LOGD("cedarx_hardware_init");
	cedarx_hardware_init(0);
	
	LOGD("Codec version = %s", getCodecVision());
	
	m_venc_device = cedarvEncInit();
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_BASE_CONFIG, &m_base_cfg);
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_ALLOCATE_INPUT_BUFFER, &m_alloc_parm);
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_OPEN, 0);
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_HEADER_DATA, &m_header_data);
	
	LOGD("create encoder ok");
	
	/* create source */
	m_CameraDevice = CreateCamera(m_base_cfg.input_width, m_base_cfg.input_height);
	LOGD("create camera ok");
	
	/* set camera source callback */
	m_CameraDevice->setCameraDatacallback(m_CameraDevice, 
		(void *)this, (void *)&CameraSourceCallback);
	
	/* start camera */
	m_CameraDevice->startCamera(m_CameraDevice);
	
	/* start encoder */
	m_mstart = 1;
	start(); //start thread
	
}


CCapEncoder::~CCapEncoder(void)
{
	m_CameraDevice->stopCamera(m_CameraDevice);
	
	DestroyCamera(m_CameraDevice);
	m_CameraDevice = NULL;
	
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_CLOSE, 0);
	cedarvEncExit(m_venc_device);
	m_venc_device = NULL;

	cedarx_hardware_exit(0);
	m_mstart = 0;
	join();
}

VencSeqHeader CCapEncoder::GetHeader()
{
	return m_header_data;
}

int CCapEncoder::Encode(void)
{
	while (m_mstart) {
		int result = 0;
		VencInputBuffer input_buffer;
		//VencOutputBuffer output_buffer;
		
		// FIXME: encode 需要消耗一定时间，这里不准确
		//usleep(1000 * 1000 / m_base_cfg.maxKeyInterval);	// 25fps
		msleep(20);	// 25fps
		
		memset(&input_buffer, 0, sizeof(VencInputBuffer));
		
		// dequene buffer from input buffer quene;
		result = m_venc_device->ioctrl(m_venc_device, VENC_CMD_DEQUENE_INPUT_BUFFER, &input_buffer);
		
		if(result<0)
		{
			LOGV("enquene input buffer is empty");
			usleep(10*1000);
			continue;
		}
		
		cedarx_cache_op((long int)input_buffer.addrvirY, 
					(long int)input_buffer.addrvirY + m_base_cfg.input_width * m_base_cfg.input_width * 3/2, 0);
		
		result = m_venc_device->ioctrl(m_venc_device, VENC_CMD_ENCODE, &input_buffer);
		
		// return the buffer to the alloc buffer quene after encoder
		m_venc_device->ioctrl(m_venc_device, VENC_CMD_RETURN_ALLOCATE_INPUT_BUFFER, &input_buffer);
		
		if (result != 0) {
			usleep(10000);
			printf("not encode, ret: %d\n", result);
			::exit(-1);
		}
		
		memset(&m_output_buffer, 0, sizeof(VencOutputBuffer));
		result = m_venc_device->ioctrl(m_venc_device, VENC_CMD_GET_BITSTREAM, &m_output_buffer);
		
		if (m_output_buffer.size0 > 0) {
			ost::MutexLock al(g_avfifo->cs_fifo_);
			
			if (g_avfifo->fifo_.size() > 200) {
				fprintf(stderr, "fifo overflow ...\n");
				// 当积累的太多时清除fifo
				while (!g_avfifo->fifo_.empty()) {
					slice_t *s = g_avfifo->fifo_.front();
					g_avfifo->fifo_.pop_front();
					slice_free(s);
					fprintf(stderr, "E");
				}
			}
		
			if(m_output_buffer.size1 > 0){
				g_avfifo->fifo_.push_back(slice_alloc(m_output_buffer.ptr0, m_output_buffer.size0, m_output_buffer.ptr1, m_output_buffer.size1, m_output_buffer.pts));
			}
			else{
				g_avfifo->fifo_.push_back(slice_alloc(m_output_buffer.ptr0, m_output_buffer.size0, m_output_buffer.pts));
			
			}
			g_avfifo->sem_fifo_.post();
		}
		
		result = m_venc_device->ioctrl(m_venc_device, VENC_CMD_RETURN_BITSTREAM, &m_output_buffer);
	}
	
	return 0;

}

void CCapEncoder::run()
{
	Encode();
}


#if 0
int CCapEncoder::ReturnBitstream(void)
{
	int result = 0;
	result = m_venc_device->ioctrl(m_venc_device, VENC_CMD_RETURN_BITSTREAM, &m_output_buffer);
	
	return result;
}
#endif

CRTMPStream::CRTMPStream(bool bEncode):
m_pRtmp(NULL),
m_nFileBufSize(0),
m_nCurPos(0)
{
	m_pFileBuf = new unsigned char[FILEBUFSIZE];
	memset(m_pFileBuf,0,FILEBUFSIZE);
	InitSockets();
	m_pRtmp = RTMP_Alloc();  
	RTMP_Init(m_pRtmp);
	m_venc_cam_cxt = NULL;
	
	if(bEncode)
		m_venc_cam_cxt = new CCapEncoder();
	
	g_avfifo = new AVfifo_t;

	g_avfifo->outbuf_ = malloc(128*2048);
	g_avfifo->outbuf_size_ = 128*2048;
}

CRTMPStream::~CRTMPStream(void)
{
	Close();
	#ifdef WIN32
	WSACleanup();
	#endif
	delete[] m_pFileBuf;
	
	if(m_venc_cam_cxt)
		delete m_venc_cam_cxt;
	
	if(g_avfifo){
		free(g_avfifo->outbuf_);
		delete g_avfifo;
	}
	
}

bool CRTMPStream::Connect(const char* url)
{
	if(RTMP_SetupURL(m_pRtmp, (char*)url)<0)
	{
		return FALSE;
	}
	RTMP_EnableWrite(m_pRtmp);
	if(RTMP_Connect(m_pRtmp, NULL)<0)
	{
		return FALSE;
	}
	if(RTMP_ConnectStream(m_pRtmp,0)<0)
	{
		return FALSE;
	}
	return TRUE;
}

void CRTMPStream::Close()
{
	if(m_pRtmp)
	{
		RTMP_Close(m_pRtmp);
		RTMP_Free(m_pRtmp);
		m_pRtmp = NULL;
	}
}

int CRTMPStream::SendPacket(unsigned int nPacketType,unsigned char *data,unsigned int size,unsigned int nTimestamp)
{
	if(m_pRtmp == NULL)
	{
		return FALSE;
	}

	RTMPPacket packet;
	RTMPPacket_Reset(&packet);
	RTMPPacket_Alloc(&packet,size);

	packet.m_packetType = nPacketType;
	packet.m_nChannel = 0x04;  
	packet.m_headerType = RTMP_PACKET_SIZE_LARGE;  
	packet.m_nTimeStamp = nTimestamp;  
	packet.m_nInfoField2 = m_pRtmp->m_stream_id;
	packet.m_nBodySize = size;
	memcpy(packet.m_body,data,size);

	int nRet = RTMP_SendPacket(m_pRtmp,&packet,0);

	RTMPPacket_Free(&packet);

	return nRet;
}

bool CRTMPStream::SendMetadata(LPRTMPMetadata lpMetaData)
{
	if(lpMetaData == NULL)
	{
		return false;
	}
	char body[1024] = {0};;
    
    char * p = (char *)body;  
	p = put_byte(p, AMF_STRING );
	p = put_amf_string(p , "@setDataFrame" );

	p = put_byte( p, AMF_STRING );
	p = put_amf_string( p, "onMetaData" );

	p = put_byte(p, AMF_OBJECT );  
	p = put_amf_string( p, "copyright" );  
	p = put_byte(p, AMF_STRING );  
	p = put_amf_string( p, "firehood" );  

	p =put_amf_string( p, "width");
	p =put_amf_double( p, lpMetaData->nWidth);

	p =put_amf_string( p, "height");
	p =put_amf_double( p, lpMetaData->nHeight);

	p =put_amf_string( p, "framerate" );
	p =put_amf_double( p, lpMetaData->nFrameRate); 

	p =put_amf_string( p, "videocodecid" );
	p =put_amf_double( p, FLV_CODECID_H264 );

	p =put_amf_string( p, "" );
	p =put_byte( p, AMF_OBJECT_END  );

	int index = p-body;

	SendPacket(RTMP_PACKET_TYPE_INFO,(unsigned char*)body,p-body,0);

	int i = 0;
	body[i++] = 0x17; // 1:keyframe  7:AVC
	body[i++] = 0x00; // AVC sequence header

	body[i++] = 0x00;
	body[i++] = 0x00;
	body[i++] = 0x00; // fill in 0;

	// AVCDecoderConfigurationRecord.
	body[i++] = 0x01; // configurationVersion
	body[i++] = lpMetaData->Sps[1]; // AVCProfileIndication
	body[i++] = lpMetaData->Sps[2]; // profile_compatibility
	body[i++] = lpMetaData->Sps[3]; // AVCLevelIndication 
    body[i++] = 0xff; // lengthSizeMinusOne  

    // sps nums
	body[i++] = 0xE1; //&0x1f
	// sps data length
	body[i++] = lpMetaData->nSpsLen>>8;
	body[i++] = lpMetaData->nSpsLen&0xff;
	// sps data
	memcpy(&body[i],lpMetaData->Sps,lpMetaData->nSpsLen);
	i= i+lpMetaData->nSpsLen;

	// pps nums
	body[i++] = 0x01; //&0x1f
	// pps data length 
	body[i++] = lpMetaData->nPpsLen>>8;
	body[i++] = lpMetaData->nPpsLen&0xff;
	// sps data
	memcpy(&body[i],lpMetaData->Pps,lpMetaData->nPpsLen);
	i= i+lpMetaData->nPpsLen;

	return SendPacket(RTMP_PACKET_TYPE_VIDEO,(unsigned char*)body,i,0);

}

bool CRTMPStream::SendH264Packet(unsigned char *data,unsigned int size,bool bIsKeyFrame,unsigned int nTimeStamp)
{
	if(data == NULL && size<11)
	{
		return false;
	}

	unsigned char *body = new unsigned char[size+9];

	int i = 0;
	if(bIsKeyFrame)
	{
		body[i++] = 0x17;// 1:Iframe  7:AVC
	}
	else
	{
		body[i++] = 0x27;// 2:Pframe  7:AVC
	}
	body[i++] = 0x01;// AVC NALU
	body[i++] = 0x00;
	body[i++] = 0x00;
	body[i++] = 0x00;

	// NALU size
	body[i++] = size>>24;
	body[i++] = size>>16;
	body[i++] = size>>8;
	body[i++] = size&0xff;;

	// NALU data
	memcpy(&body[i],data,size);

	bool bRet = SendPacket(RTMP_PACKET_TYPE_VIDEO,body,i+size,nTimeStamp);

	delete[] body;

	return bRet;
}

bool CRTMPStream::SendCapEncode(void)
{
/*
	if(pFileName == NULL)
	{
		return FALSE;
	}
	FILE *fp = fopen(pFileName, "rb");  
	if(!fp)  
	{  
		printf("ERROR:open file %s failed!",pFileName);
	}  
	fseek(fp, 0, SEEK_SET);
	m_nFileBufSize = fread(m_pFileBuf, sizeof(unsigned char), FILEBUFSIZE, fp);
	if(m_nFileBufSize >= FILEBUFSIZE)
	{
		printf("warning : File size is larger than BUFSIZE\n");
	}
	fclose(fp);  
*/
	RTMPMetadata metaData;
	memset(&metaData,0,sizeof(RTMPMetadata));

    NaluUnit naluUnit;
	// 读取SPS帧
    ReadOneNaluFromBuf_enc(naluUnit);
	metaData.nSpsLen = naluUnit.size;
	memcpy(metaData.Sps,naluUnit.data,naluUnit.size);

	// 读取PPS帧
	ReadOneNaluFromBuf_enc(naluUnit);
	metaData.nPpsLen = naluUnit.size;
	memcpy(metaData.Pps,naluUnit.data,naluUnit.size);

	// 解码SPS,获取视频图像宽、高信息
	int width = 0,height = 0;
	h264_decode_sps(metaData.Sps,metaData.nSpsLen,width,height);
	metaData.nWidth = width;
	metaData.nHeight = height;
	//metaData.nFrameRate = m_venc_cam_cxt->m_base_cfg.framerate;
	metaData.nFrameRate = m_venc_cam_cxt->m_base_cfg.maxKeyInterval;
	LOGD("metaData.nWidth is %d, metaData.nHeight is %d.\n", metaData.nWidth, metaData.nHeight);
   
	// 发送MetaData
	SendMetadata(&metaData);

	unsigned int tick = 0;
	//while(ReadOneNaluFromBuf(naluUnit))
	while(get_next_slice(naluUnit))
	{
		bool bKeyframe  = (naluUnit.type == 0x05) ? TRUE : FALSE;
		// 发送H264数据帧
		SendH264Packet(naluUnit.data,naluUnit.size,bKeyframe,tick);
		LOGD("naluUnit.size:%d, bKeyframe:%d, tick:%d.\n", naluUnit.size, bKeyframe, tick);
		
		msleep(40);
		tick +=40;
	}

	return TRUE;
}

bool CRTMPStream::SendH264File(const char *pFileName)
{
	if(pFileName == NULL)
	{
		return FALSE;
	}
	FILE *fp = fopen(pFileName, "rb");  
	if(!fp)  
	{  
		printf("ERROR:open file %s failed!",pFileName);
	}  
	fseek(fp, 0, SEEK_SET);
	m_nFileBufSize = fread(m_pFileBuf, sizeof(unsigned char), FILEBUFSIZE, fp);
	if(m_nFileBufSize >= FILEBUFSIZE)
	{
		printf("warning : File size is larger than BUFSIZE\n");
	}
	fclose(fp);  

	RTMPMetadata metaData;
	memset(&metaData,0,sizeof(RTMPMetadata));

    NaluUnit naluUnit;
	// 读取SPS帧
    ReadOneNaluFromBuf(naluUnit);
	metaData.nSpsLen = naluUnit.size;
	memcpy(metaData.Sps,naluUnit.data,naluUnit.size);

	// 读取PPS帧
	ReadOneNaluFromBuf(naluUnit);
	metaData.nPpsLen = naluUnit.size;
	memcpy(metaData.Pps,naluUnit.data,naluUnit.size);

	// 解码SPS,获取视频图像宽、高信息
	int width = 0,height = 0;
    h264_decode_sps(metaData.Sps,metaData.nSpsLen,width,height);
	metaData.nWidth = width;
    metaData.nHeight = height;
	metaData.nFrameRate = 25;
   
	// 发送MetaData
    SendMetadata(&metaData);

	unsigned int tick = 0;
	while(ReadOneNaluFromBuf(naluUnit))
	{
		bool bKeyframe  = (naluUnit.type == 0x05) ? TRUE : FALSE;
		// 发送H264数据帧
		SendH264Packet(naluUnit.data,naluUnit.size,bKeyframe,tick);
		LOGD("naluUnit.size:%d, bKeyframe:%d, tick:%d.\n", naluUnit.size, bKeyframe, tick);
		
		usleep(30*1000);
		tick +=30;
	}

	return TRUE;
}

bool CRTMPStream::ReadOneNaluFromBuf_enc(NaluUnit &nalu)
{
	int i = m_nCurPos;
	while(i < m_venc_cam_cxt->m_header_data.length)
	{
		if(m_venc_cam_cxt->m_header_data.bufptr[i++] == 0x00 &&
			m_venc_cam_cxt->m_header_data.bufptr[i++] == 0x00 &&
			m_venc_cam_cxt->m_header_data.bufptr[i++] == 0x00 &&
			m_venc_cam_cxt->m_header_data.bufptr[i++] == 0x01)
		{
			int pos = i;
			while (pos<m_venc_cam_cxt->m_header_data.length)
			{
				if(m_venc_cam_cxt->m_header_data.bufptr[pos++] == 0x00 &&
					m_venc_cam_cxt->m_header_data.bufptr[pos++] == 0x00 &&
					m_venc_cam_cxt->m_header_data.bufptr[pos++] == 0x00 &&
					m_venc_cam_cxt->m_header_data.bufptr[pos++] == 0x01
					)
				{
					break;
				}
			}
			if(pos == m_venc_cam_cxt->m_header_data.length)
			{
				nalu.size = pos-i;	
			}
			else
			{
				nalu.size = (pos-4)-i;
			}
			nalu.type = m_venc_cam_cxt->m_header_data.bufptr[i]&0x1f;
			nalu.data = &m_venc_cam_cxt->m_header_data.bufptr[i];

			m_nCurPos = pos-4;
			return TRUE;
		}
	}
	return FALSE;
}
bool CRTMPStream::ReadOneNaluFromBuf(NaluUnit &nalu)
{
	int i = m_nCurPos;
	while(i<m_nFileBufSize)
	{
		if(m_pFileBuf[i++] == 0x00 &&
			m_pFileBuf[i++] == 0x00 &&
			m_pFileBuf[i++] == 0x00 &&
			m_pFileBuf[i++] == 0x01
			)
		{
			int pos = i;
			while (pos<m_nFileBufSize)
			{
				if(m_pFileBuf[pos++] == 0x00 &&
					m_pFileBuf[pos++] == 0x00 &&
					m_pFileBuf[pos++] == 0x00 &&
					m_pFileBuf[pos++] == 0x01
					)
				{
					break;
				}
			}
			if(pos == m_nFileBufSize)
			{
				nalu.size = pos-i;	
			}
			else
			{
				nalu.size = (pos-4)-i;
			}
			nalu.type = m_pFileBuf[i]&0x1f;
			nalu.data = &m_pFileBuf[i];

			m_nCurPos = pos-4;
			return TRUE;
		}
	}
	return FALSE;
}
