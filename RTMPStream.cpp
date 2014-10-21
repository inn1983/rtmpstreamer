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
//#define LOG_FILE 1
#define LOG_FILE_DUMMY 1
#include "CDX_Debug.h"



//AVfifo_t* g_avfifo; //グローバルfifo
AVmap_t* g_av_map;
TimstampFifo_t* g_ptsfifo;

FILE* g_debuglog = NULL;

long long g_starttime = 0;


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

static slice_t *slice_alloc(const void *data, int len, int64_t pts, unsigned char type)
{
	slice_t *ns = (slice_t *)malloc(sizeof(slice_t));
	ns->data_ = malloc(len);
	memcpy(ns->data_, data, len);
	ns->pts_ = pts;
	ns->len_ = len;
	ns->pkt_type = type;
	return ns;
}

static slice_t *slice_alloc(const void *p1, int l1, const void *p2, int l2, int64_t pts, unsigned char type)
{
	slice_t *ns = (slice_t*)malloc(sizeof(slice_t));
	ns->data_ = malloc(l1+l2);
	memcpy(ns->data_, p1, l1);
	memcpy((unsigned char*)ns->data_ + l1, p2, l2);
	ns->pts_ = pts;
	ns->len_ = l1+l2;
	ns->pkt_type = type;
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
	g_ptsfifo->cs_fifo_.enter();
	LOGD(g_debuglog, "g_ptsfifo->cs_fifo_.enter()");
	
	while (g_ptsfifo->fifo_.empty() ) {
		g_ptsfifo->cs_fifo_.leave();
		g_ptsfifo->sem_fifo_.wait();
		g_ptsfifo->cs_fifo_.enter();
	}

	slice_t *s;
	long long pts = g_ptsfifo->fifo_.front();
	g_ptsfifo->fifo_.pop_front();
	LOGD(g_debuglog, "g_ptsfifo->fifo_.front():%lld.", pts);
	
	g_ptsfifo->cs_fifo_.leave();
	LOGD(g_debuglog, "g_ptsfifo->cs_fifo_.leave()");
	
	g_av_map->cs_map_.enter();
	std::map<long long, slice_t*>::iterator it;
	it = g_av_map->map_.find(pts);
	
	while(it == g_av_map->map_.end())
	{
		g_av_map->cs_map_.leave();
		LOGD(g_debuglog, "g_av_map->cs_map_.leave()");
		g_av_map->sem_map_.wait();
		LOGD(g_debuglog, "g_av_map->sem_map_.wait()");
		g_av_map->cs_map_.enter();
		LOGD(g_debuglog, "g_av_map->cs_map_.enter()");
		it = g_av_map->map_.find(pts);
	}
	LOGD(g_debuglog, "g_av_map->cs_map_.enter()");
	s = it->second;

	//fprintf(stderr, "avc_fifo_ size is %d.\n", g_avfifo->avc_fifo_.size());

	if (s->len_ > g_av_map->outbuf_size_) {
		g_av_map->outbuf_size_ = (s->len_ + 4095)/4096*4096;
		g_av_map->outbuf_ = realloc(g_av_map->outbuf_, g_av_map->outbuf_size_);
	}

	memcpy(g_av_map->outbuf_, s->data_, s->len_);
	
	if (s->pkt_type == RTMP_PACKET_TYPE_VIDEO){
		nalu.data = (unsigned char*)g_av_map->outbuf_+4;
		nalu.frame_type = nalu.data[0]&0x1f;
		nalu.size = s->len_-4;
	}
	else {
		nalu.data = (unsigned char*)g_av_map->outbuf_+7;
		nalu.size = s->len_-7;
	}
	nalu.pkt_type = s->pkt_type;
	nalu.pts = s->pts_/1000;

	int rc = nalu.size;
	slice_free(s);
	g_av_map->map_.erase(it);	//mapから要素を削除


	g_av_map->cs_map_.leave();
	LOGD(g_debuglog, "g_av_map->cs_map_.leave()");

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
	//get timestamp 
	struct timeval tv;
	struct timezone tz;
	long long timestamp = 0;
	LOGD(g_debuglog, "CameraSourceCallback()!!");

	if(true)
	{
		ost::MutexLock al(g_ptsfifo->cs_fifo_);
		LOGD(g_debuglog, "CameraSourceCallback()!!");
		gettimeofday (&tv, &tz);
		LOGD(g_debuglog, "CameraSourceCallback()!!");
		timestamp = (tv.tv_sec - g_starttime)*1000000 + tv.tv_usec;	//usec
		g_ptsfifo->fifo_.push_back(timestamp);
		LOGD(g_debuglog, "avc pts push:%lld", timestamp);
	}
	
	struct v4l2_buffer *p_buf = (struct v4l2_buffer *)data;
	v4l2_mem_map_t* p_v4l2_mem_map = GetMapmemAddress(getV4L2ctx(CameraDevice));

	void *buffer = (void *)p_v4l2_mem_map->mem[p_buf->index];
	int size_y = venc_cam_cxt->m_base_cfg.input_width*venc_cam_cxt->m_base_cfg.input_height; 

	memset(&input_buffer, 0, sizeof(VencInputBuffer));
	
	do{	//retry
		result = venc_device->ioctrl(venc_device, VENC_CMD_GET_ALLOCATE_INPUT_BUFFER, &input_buffer);
		LOGD(g_debuglog, "no alloc input buffer right now");
		usleep(10*1000);
	}while(result !=0 );
	
	uyvy_nv12( (unsigned char*)buffer, input_buffer.addrvirY, 720, 480);

	//input_buffer.pts = 1000000 * (long long)p_buf->timestamp.tv_sec + (long long)p_buf->timestamp.tv_usec;
	
	//save starttime 
	//if (g_starttime == 0){
	//	g_starttime = input_buffer.pts;	//input_buffer.pts long long is 64bit
	//	LOGD(g_debuglog, "g_starttime:%d", g_starttime);
	//}
	//input_buffer.pts = input_buffer.pts - g_starttime ;


#if 1
	if(input_buffer.addrphyY >=  (void*)0x40000000)
		input_buffer.addrphyY -=0x40000000;
#endif

	//if(!venc_cam_cxt->mstart) {
		LOGD(g_debuglog, "p_buf->index = %d\n", p_buf->index);
	//	CameraDevice->returnFrame(CameraDevice, p_buf->index);
	//}

    // enquene buffer to input buffer quene
    
	LOGD(g_debuglog, "ID = %d\n", input_buffer.id);
	input_buffer.pts = timestamp;
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
	m_base_cfg.maxKeyInterval = 30;
	m_base_cfg.inputformat = VENC_PIXEL_YUV420; //uv combined
	m_base_cfg.targetbitrate = 3*1024*1024;
	
	m_alloc_parm.buffernum = 4;
	
	LOGD(g_debuglog, "cedarx_hardware_init");
	cedarx_hardware_init(0);
	
	LOGD(g_debuglog, "Codec version = %s", getCodecVision());
	
	m_venc_device = cedarvEncInit();
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_BASE_CONFIG, &m_base_cfg);
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_ALLOCATE_INPUT_BUFFER, &m_alloc_parm);
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_OPEN, 0);
	m_venc_device->ioctrl(m_venc_device, VENC_CMD_HEADER_DATA, &m_header_data);
	
	LOGD(g_debuglog, "create encoder ok");
	
	/* create source */
	m_CameraDevice = CreateCamera(m_base_cfg.input_width, m_base_cfg.input_height);
	LOGD(g_debuglog, "create camera ok");
	
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
		msleep(30);	// 30fps
		
		memset(&input_buffer, 0, sizeof(VencInputBuffer));
		
		// dequene buffer from input buffer quene;
		result = m_venc_device->ioctrl(m_venc_device, VENC_CMD_DEQUENE_INPUT_BUFFER, &input_buffer);
		
		if(result<0)
		{
			LOGD(g_debuglog, "enquene input buffer is empty");
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
			ost::MutexLock al(g_av_map->cs_map_);
			slice_t* s;
			if(m_output_buffer.size1 > 0){
				s = slice_alloc(m_output_buffer.ptr0, m_output_buffer.size0, m_output_buffer.ptr1, m_output_buffer.size1, m_output_buffer.pts, RTMP_PACKET_TYPE_VIDEO);
			}
			else{
				s = slice_alloc(m_output_buffer.ptr0, m_output_buffer.size0, m_output_buffer.pts, RTMP_PACKET_TYPE_VIDEO);
			}
			
			g_av_map->map_.insert(std::map<long long, slice_t*>::value_type(m_output_buffer.pts, s));
			g_av_map->sem_map_.post();
		}
		
		result = m_venc_device->ioctrl(m_venc_device, VENC_CMD_RETURN_BITSTREAM, &m_output_buffer);
	}
	
	return 0;

}

void CCapEncoder::run()
{
	Encode();
}

CAlsaEncoder::CAlsaEncoder()
{
	m_buffer_frames =  128;
	//m_rate = 11025;
	m_rate = 44100;
	m_format = SND_PCM_FORMAT_S16_LE;
	m_nChannels = 1;
	m_nPCMBitSize = 16;
	
	//AlsaInit();
	FaacInit();
	
	//struct timeval tv;
    //struct timezone tz;
    //gettimeofday (&tv, &tz);
	//m_starttime = tv.tv_sec;
	
	m_mstart = 1;
	start(); //start thread
	
}


CAlsaEncoder::~CAlsaEncoder()
{   
	m_mstart = 0;
	join();
	// (4) Close FAAC engine
	int nRet;
    nRet = faacEncClose(m_hEncoder);
	delete[] m_pbPCMBuffer;
    delete[] m_pbAACBuffer;
    fclose(m_fpWavIn);
	fclose(m_fpAacOut);
}


int CAlsaEncoder::AlsaInit(void)
{
#if 0
	int i;
	int err;
	if ((err = snd_pcm_open (&capture_handle, argv[1], SND_PCM_STREAM_CAPTURE, 0)) < 0) {
		fprintf (stderr, "cannot open audio device %s (%s)\n", 
				argv[1],
				snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "audio interface opened\n");
		   
	if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
		fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "hw_params allocated\n");
	
	if ((err = snd_pcm_hw_params_any (capture_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "hw_params initialized\n");
	
	if ((err = snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
		fprintf (stderr, "cannot set access type (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "hw_params access setted\n");
	
	if ((err = snd_pcm_hw_params_set_format (capture_handle, hw_params, format)) < 0) {
		fprintf (stderr, "cannot set sample format (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "hw_params format setted\n");
	
	if ((err = snd_pcm_hw_params_set_rate_near (capture_handle, hw_params, &rate, 0)) < 0) {
		fprintf (stderr, "cannot set sample rate (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "hw_params rate setted\n");
 
	if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, 2)) < 0) {
		fprintf (stderr, "cannot set channel count (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "hw_params channels setted\n");
	
	if ((err = snd_pcm_hw_params (capture_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot set parameters (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "hw_params setted\n");
	
	snd_pcm_hw_params_free (hw_params);
	
	fprintf(stdout, "hw_params freed\n");
	
	if ((err = snd_pcm_prepare (capture_handle)) < 0) {
		fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
             snd_strerror (err));
		exit (1);
	}
	fprintf(stdout, "audio interface prepared\n");

	return 0;
#endif
}

int CAlsaEncoder::FaacInit(void)
{
	int nRet;
	m_fpWavIn = fopen("./WavInFifo.wav", "rb");	//input fifo
	if (m_fpWavIn) 
		printf("WavInFifo is open!!\n");
		
	m_fpAacOut =  fopen("./dump.aac", "wb");
	
	// (1) Open FAAC engine
	m_hEncoder = faacEncOpen(m_rate, m_nChannels, &m_nInputSamples, &m_nMaxOutputBytes);//初始化aac句柄，同时获取最大输入样本，及编码所需最小字节
	
	m_nMaxInputBytes=m_nInputSamples * m_nPCMBitSize / 8;//计算最大输入字节,跟据最大输入样本数
	printf("m_nInputSamples:%d m_nMaxInputBytes:%d m_nMaxOutputBytes:%d\n", m_nInputSamples, m_nMaxInputBytes,m_nMaxOutputBytes);
    
	if(m_hEncoder == NULL)
    {
        printf("[ERROR] Failed to call faacEncOpen()\n");
        return -1;
    }
	
	m_pbPCMBuffer = new BYTE [m_nMaxInputBytes];
	m_pbAACBuffer = new BYTE [m_nMaxOutputBytes];
	
	// (2.1) Get current encoding configuration
	m_pConfiguration = faacEncGetCurrentConfiguration(m_hEncoder);//获取配置结构指针
	m_pConfiguration->inputFormat = FAAC_INPUT_16BIT;
	m_pConfiguration->outputFormat=1;
	m_pConfiguration->useTns=true;
	m_pConfiguration->useLfe=false;
	m_pConfiguration->aacObjectType=LOW;
	m_pConfiguration->shortctl=SHORTCTL_NORMAL;
	m_pConfiguration->quantqual=60;
	m_pConfiguration->bandWidth=0;
	m_pConfiguration->bitRate=0;
	
	// (2.2) Set encoding configuration
	nRet = faacEncSetConfiguration(m_hEncoder, m_pConfiguration);//设置配置，根据不同设置，耗时不一样
	
	return 0;
}

int CAlsaEncoder::Encode(void)
{
	//snd_pcm_readi(capture_handle, buffer, buffer_frames);
	int nBytesRead;
	int nRet;
	int nInputSamples;
	
	faacEncGetDecoderSpecificInfo(m_hEncoder, &m_enc_spec_buf, (long unsigned int*)&m_enc_spec_len);
	while(m_mstart){
		
		long long timestamp = 0;
		struct timeval tv;
		struct timezone tz;
		static int cont = 0;
		static bool en = false;

		//LOGD(g_debuglog, "(tv.tv_sec - g_starttime)*1000000 + tv.tv_usec is %lld", timestamp);
		
		// 读入的实际字节数，最大不会超过m_nMaxInputBytes，一般只有读到文件尾时才不是m_nMaxInputBytes
        nBytesRead = fread(m_pbPCMBuffer, 1, m_nMaxInputBytes, m_fpWavIn);
		cont++;
		if (cont > 5) en = true;
		
		LOGD(g_debuglog, "CAlsaEncoder::Encode()");

		if(en)
		{
			ost::MutexLock al(g_ptsfifo->cs_fifo_);
			LOGD(g_debuglog, "CAlsaEncoder::Encode()");
			gettimeofday (&tv, &tz);
			timestamp = (tv.tv_sec - g_starttime)*1000000 + tv.tv_usec;	//usec
			g_ptsfifo->fifo_.push_back(timestamp);
			LOGD(g_debuglog, "aac pts push: %lld.", timestamp);
			g_ptsfifo->sem_fifo_.post();
			LOGD(g_debuglog, "CAlsaEncoder::Encode()");
		}
		
		// 输入样本数，用实际读入字节数计算
		nInputSamples = nBytesRead / (m_nPCMBitSize / 8);
		// (3) Encode
		nRet = faacEncEncode(m_hEncoder, (int*) m_pbPCMBuffer, nInputSamples, m_pbAACBuffer, m_nMaxOutputBytes);
		//usleep(70*1000);
		if (nRet > 0) {
			ost::MutexLock al(g_av_map->cs_map_);
			
			//g_avfifo->aac_fifo_.push_back(slice_alloc(m_pbAACBuffer, nRet, timestamp, RTMP_PACKET_TYPE_AUDIO));
			g_av_map->map_.insert(std::map<long long, slice_t*>::value_type(timestamp, slice_alloc(m_pbAACBuffer, nRet, timestamp, RTMP_PACKET_TYPE_AUDIO)));
	
			fprintf(stderr, "aac map insert!! nRet is %d.\n", nRet);
			g_av_map->sem_map_.post();
			
			fwrite(m_pbAACBuffer, 1, nRet, m_fpAacOut);
		}
		//else {
			//先保存したptsを捨てる
			//LOGD(g_debuglog, "aac map is not insert!!nRet is %d", nRet);
			//g_ptsfifo->fifo_.pop_front();
		//}
	}
	
}


void CAlsaEncoder::run()
{
	Encode();
}

CRTMPStream::CRTMPStream(bool bEncode):
m_pRtmp(NULL),
m_nFileBufSize(0),
m_nCurPos(0)
{
#if LOG_FILE
	time_t t = time(0);
	char tmp[64];
	strftime( tmp, sizeof(tmp), "%Y%m%d%H%M%S_log.txt", localtime(&t) ); //格式化输出.
	g_debuglog = fopen(tmp, "w+");
#endif

	m_pFileBuf = new unsigned char[FILEBUFSIZE];
	memset(m_pFileBuf,0,FILEBUFSIZE);
	InitSockets();
	m_pRtmp = RTMP_Alloc();
	RTMP_Init(m_pRtmp);
	m_venc_cam_cxt = NULL;
	m_alsa_enc = NULL;
	
	if(bEncode){
		m_venc_cam_cxt = new CCapEncoder();
		m_alsa_enc = new CAlsaEncoder();
	}
		
	
	g_av_map = new AVmap_t;
	g_av_map->outbuf_ = malloc(128*2048);
	g_av_map->outbuf_size_ = 128*2048;
	
	g_ptsfifo = new TimstampFifo_t;
	
	struct timeval tv;
    struct timezone tz;
    gettimeofday (&tv, &tz);
	g_starttime = tv.tv_sec;
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
	
	if(m_alsa_enc)
		delete m_alsa_enc;
	
	if(g_av_map){
		free(g_av_map->outbuf_);
		delete g_av_map;
	}
	
	if(g_ptsfifo)
		delete g_ptsfifo;
	
	if(g_debuglog)
		fclose(g_debuglog);
}

bool CRTMPStream::Connect(const char* url)
{
	int ret;
	LOGD(g_debuglog, "CRTMPStream::Connect start!.");
	ret = RTMP_SetupURL(m_pRtmp, (char*)url);
	if(ret < 0)
	{
		LOGD(g_debuglog, "RTMP_SetupURL error: %d.", ret);
		return FALSE;
	}
	RTMP_EnableWrite(m_pRtmp);
	
	ret = RTMP_Connect(m_pRtmp, NULL);
	if(ret<0)
	{
		LOGD(g_debuglog, "RTMP_Connect error: %d.", ret);
		return FALSE;
	}
	
	ret = RTMP_ConnectStream(m_pRtmp,0);
	if(ret<0)
	{
		LOGD(g_debuglog, "RTMP_ConnectStream error: %d.", ret);
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

int CRTMPStream::SendPacket(unsigned int nPacketType,unsigned char *data,unsigned int size,unsigned int nTimestamp, unsigned char type)
{
	if(m_pRtmp == NULL)
	{
		return FALSE;
	}

	RTMPPacket packet;
	RTMPPacket_Reset(&packet);
	RTMPPacket_Alloc(&packet,size);
	
	packet.m_hasAbsTimestamp = 0;
	packet.m_packetType = nPacketType;
	packet.m_nChannel = 0x04;  
	packet.m_headerType = type; //RTMP_PACKET_SIZE_LARGE;  
	packet.m_nTimeStamp = nTimestamp;  
	packet.m_nInfoField2 = m_pRtmp->m_stream_id;
	packet.m_nBodySize = size;
	memcpy(packet.m_body,data,size);
	
	//fprintf(stderr, "packet.m_hasAbsTimestamp is %d!!\n", packet.m_hasAbsTimestamp);
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
	p = put_amf_string( p, "inn" );  

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

	SendPacket(RTMP_PACKET_TYPE_INFO,(unsigned char*)body,p-body,0, RTMP_PACKET_SIZE_LARGE);

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

	SendPacket(RTMP_PACKET_TYPE_VIDEO,(unsigned char*)body,i,0, RTMP_PACKET_SIZE_LARGE);
	
	SendAacSpec();
	
	return 0;

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

	bool bRet = SendPacket(RTMP_PACKET_TYPE_VIDEO,body,i+size,nTimeStamp, RTMP_PACKET_SIZE_MEDIUM);

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
	LOGD(g_debuglog, "metaData.nWidth is %d, metaData.nHeight is %d.\n", metaData.nWidth, metaData.nHeight);
   
	// 发送MetaData
	SendMetadata(&metaData);

	unsigned int tick = 0x00ff0000;
	unsigned int cont = 0;
	//unsigned int tick = 0;
	//while(ReadOneNaluFromBuf(naluUnit))
	while(get_next_slice(naluUnit))
	{
		if (naluUnit.pkt_type == RTMP_PACKET_TYPE_VIDEO){
			bool bKeyframe  = (naluUnit.frame_type == 0x05) ? TRUE : FALSE;
			// 发送H264数据帧
			//SendH264Packet(naluUnit.data,naluUnit.size,bKeyframe,naluUnit.pts);
			LOGD(g_debuglog, "RTMP_PACKET_TYPE_VIDEO send.");
			LOGD(g_debuglog, "naluUnit.size:%d, bKeyframe:%d, naluUnit.pts:%u.", naluUnit.size, bKeyframe, naluUnit.pts);
		}
		else {
			// 发送AAC数据
			SendAacPacket(naluUnit.data, naluUnit.size, naluUnit.pts);
			LOGD(g_debuglog, "RTMP_PACKET_TYPE_AUDIO send.");
			LOGD(g_debuglog, "naluUnit.pts:%u.", naluUnit.pts);
		}
		
		cont++;
		if (cont > 2000){
			// 发送MetaData
			//SendMetadata(&metaData);
			cont = 0;
		}

#if LOG_FILE
		//logファイルを新たに作る
		if (cont > 2000){
			fclose(g_debuglog);
			time_t t = time(0);
			char tmp[64];
			strftime( tmp, sizeof(tmp), "%Y%m%d%H%M%S_log.txt", localtime(&t) ); //格式化输出.
			g_debuglog = fopen(tmp, "w+");
			cont = 0;
		}
#endif //LOG_FILE
	}

	return TRUE;
}

int CRTMPStream::SendAacPacket(unsigned char *data, unsigned int size, unsigned long pts)
{
/*
	long timestamp;
	
	struct timeval tv;
    struct timezone tz;
    gettimeofday (&tv, &tz);
	
	timestamp = (tv.tv_sec*1000000 + tv.tv_usec - m_starttime)/1000;
*/	
	RTMPPacket packet;
	RTMPPacket_Reset(&packet);
	RTMPPacket_Alloc(&packet,size+2);
	
	/*AF 00 + AAC RAW data*/
	packet.m_body[0] = 0xAE;
	packet.m_body[1] = 0x01;
	memcpy(&packet.m_body[2],data, size);
	
	packet.m_packetType = RTMP_PACKET_TYPE_AUDIO;
    packet.m_nBodySize = size+2;
    packet.m_nChannel = 0x04;
    packet.m_nTimeStamp = pts;//timestamp;
    packet.m_headerType = RTMP_PACKET_SIZE_MEDIUM;
    packet.m_nInfoField2 = m_pRtmp->m_stream_id;
	
	fprintf(stderr, "packet.m_nTimeStamp is %d.\n", packet.m_nTimeStamp);
	fprintf(stderr, "pts is %d.\n", pts);
	int nRet = RTMP_SendPacket(m_pRtmp,&packet,0);
	
	RTMPPacket_Free(&packet);

	return 0;
}

int CRTMPStream::SendAacSpec(void)
{
	
	RTMPPacket packet;
	RTMPPacket_Reset(&packet);
	RTMPPacket_Alloc(&packet,m_alsa_enc->m_enc_spec_len+2);
	
	//unsigned char debugbuf[20];
	
	/*AF 00 + AAC RAW data*/
	packet.m_body[0] = 0xAE;
	packet.m_body[1] = 0x00;
	memcpy(&packet.m_body[2],m_alsa_enc->m_enc_spec_buf, m_alsa_enc->m_enc_spec_len); /*spec_buf是AAC sequence header数据*/
	packet.m_body[2] = 0x11;
	packet.m_body[3] = 0x88;
	
	packet.m_packetType = RTMP_PACKET_TYPE_AUDIO;
    packet.m_nBodySize = m_alsa_enc->m_enc_spec_len+2;
    packet.m_nChannel = 0x04;
    packet.m_nTimeStamp = 0;
    packet.m_hasAbsTimestamp = 0;
    packet.m_headerType = RTMP_PACKET_SIZE_LARGE;
    packet.m_nInfoField2 = m_pRtmp->m_stream_id;
	
	//memcpy(&debugbuf[0],m_alsa_enc->m_enc_spec_buf, m_alsa_enc->m_enc_spec_len);
	//debugbuf[m_alsa_enc->m_enc_spec_len+1] = '\0';
	fprintf(stderr, "m_alsa_enc->m_enc_spec_len is 0x%x!!\n", m_alsa_enc->m_enc_spec_len);
	fprintf(stderr, "packet.m_body[2] is 0x%x!!\n", packet.m_body[2]);
	fprintf(stderr, "packet.m_body[3] is 0x%x!!\n", packet.m_body[3]);
	
	/*调用发送接口*/
    int nRet = RTMP_SendPacket(m_pRtmp,&packet,0);
	RTMPPacket_Free(&packet);
	
	return 0;
	
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
		bool bKeyframe  = (naluUnit.frame_type == 0x05) ? TRUE : FALSE;
		// 发送H264数据帧
		SendH264Packet(naluUnit.data,naluUnit.size,bKeyframe,tick);
		LOGD(g_debuglog, "naluUnit.size:%d, bKeyframe:%d, tick:%d.\n", naluUnit.size, bKeyframe, tick);
		
		usleep(30*1000);
		tick +=30;
	}

	return TRUE;
}

bool CRTMPStream::ReadOneNaluFromBuf_enc(NaluUnit &nalu)
{
	unsigned int i = m_nCurPos;
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
			nalu.frame_type = m_venc_cam_cxt->m_header_data.bufptr[i]&0x1f;
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
			nalu.frame_type = m_pFileBuf[i]&0x1f;
			nalu.data = &m_pFileBuf[i];

			m_nCurPos = pos-4;
			return TRUE;
		}
	}
	return FALSE;
}
