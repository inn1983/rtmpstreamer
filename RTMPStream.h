/******************************************************************** 
filename:   RTMPStream.h
created:    2014.10.07
author:     inn 
purpose:    librtmpライブラリを使用しH264データをRed5に送信
*********************************************************************/ 
#ifndef RTMPSTREAM_H
#define RTMPSTREAM_H
#include <rtmp.h>
#include <rtmp_sys.h>
#include <amf.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>    
#include <unistd.h>

#include <deque>
#include <cc++/thread.h>

//#include "type.h"
#include "H264encLibApi.h"
#include "V4L2.h"
#include "venc.h"
#include "CameraSource.h"
//#include "water_mark.h"
#include "cedarv_osal_linux.h"

#include <alsa/asoundlib.h>
#include <faac.h>

#define FILEBUFSIZE (1024 * 1024 * 10)       //  10M

// 描述一个 h264 slice
typedef struct slice_t
{
	void *data_;
	int len_;
	int64_t pts_;
	unsigned char pkt_type;
} slice_t;

typedef std::deque<slice_t*> SLICES;

// NALU单元
typedef struct _NaluUnit
{
	int frame_type;
	unsigned char pkt_type;
	int size;
	unsigned char *data;
	unsigned long pts;
}NaluUnit;

typedef struct _RTMPMetadata
{
	// video, must be h264 type
	unsigned int	nWidth;
	unsigned int	nHeight;
	unsigned int	nFrameRate;		// fps
	unsigned int	nVideoDataRate;	// bps
	unsigned int	nSpsLen;
	unsigned char	Sps[1024];
	unsigned int	nPpsLen;
	unsigned char	Pps[1024];

	// audio, must be aac type
	bool	        bHasAudio;
	unsigned int	nAudioSampleRate;
	unsigned int	nAudioSampleSize;
	unsigned int	nAudioChannels;
	char		    pAudioSpecCfg;
	unsigned int	nAudioSpecCfgLen;

} RTMPMetadata,*LPRTMPMetadata;

typedef struct avfifo
{	
	SLICES avc_fifo_;		// 用于缓冲
	SLICES aac_fifo_;		// 用于缓冲
	ost::Mutex cs_fifo_;	
	ost::Semaphore sem_fifo_;
	
	void *outbuf_;		// 
	int outbuf_size_;

} AVfifo_t;

class CCapEncoder : ost::Thread
{
public:
	CCapEncoder(void);
	~CCapEncoder(void);
	
	VencSeqHeader GetHeader();
	int Encode(void);
	//int ReturnBitstream(void);
	
	VencBaseConfig m_base_cfg;
public:	
	cedarv_encoder_t* m_venc_device;
	AWCameraDevice* m_CameraDevice;
	VencSeqHeader m_header_data;
	
private:
	void run();
	
	VencInputBuffer m_input_buffer;
	VencOutputBuffer m_output_buffer;
	VencAllocateBufferParam m_alloc_parm;
	pthread_t m_thread_enc_id;
	int m_mstart;
};

class CAlsaEncoder : ost::Thread
{
public:
	CAlsaEncoder(void);
	~CAlsaEncoder(void);
	int AlsaInit(void);
	int FaacInit(void);
	
	int Encode(void);
	
	unsigned char* m_enc_spec_buf;
	int m_enc_spec_len;
	
	time_t m_starttime;

private:
	void run();
	int m_mstart;
	int m_buffer_frames;
	unsigned int m_rate;
	snd_pcm_t *m_capture_handle;
	snd_pcm_hw_params_t *m_hw_params;
	snd_pcm_format_t m_format;
	int m_nChannels;
	
	FILE* m_fpWavIn;
	FILE* m_fpAacOut;
	
	faacEncHandle m_hEncoder;		//aac handler
	faacEncConfigurationPtr m_pConfiguration;//aac设置指针
	unsigned char* m_pbPCMBuffer;
    unsigned char* m_pbAACBuffer;
	//unsigned long m_nPCMBufferSize;
	unsigned long m_nInputSamples;
	unsigned long m_nMaxInputBytes;
	unsigned long m_nMaxOutputBytes;
	unsigned char m_nPCMBitSize;
	
};


class CRTMPStream
{
public:
	CRTMPStream(bool bEncode);
	~CRTMPStream(void);
public:
	// 连接到RTMP Server
	bool Connect(const char* url);
    // 断开连接
	void Close();
    // 发送MetaData
	bool SendMetadata(LPRTMPMetadata lpMetaData);
    // 发送H264数据帧
	bool SendH264Packet(unsigned char *data,unsigned int size,bool bIsKeyFrame,unsigned int nTimeStamp);
	// 发送H264文件
	bool SendH264File(const char *pFileName);
	//
	bool SendCapEncode(void);
	int SendAacSpec(void);
	int SendAacPacket(unsigned char *data, unsigned int size, unsigned long pts);
	
	//time_t m_starttime;
private:
	// 送缓存中读取一个NALU包
	bool ReadOneNaluFromBuf_enc(NaluUnit &nalu);
	bool ReadOneNaluFromBuf(NaluUnit &nalu);
	// 发送数据
	int SendPacket(unsigned int nPacketType,unsigned char *data,unsigned int size,unsigned int nTimestamp, unsigned char type);
private:
	RTMP* m_pRtmp;
	unsigned char* m_pFileBuf;
	unsigned int  m_nFileBufSize;
	unsigned int  m_nCurPos;
	AVfifo_t m_avfifo; 
	CCapEncoder* m_venc_cam_cxt;
	CAlsaEncoder* m_alsa_enc;
};

#endif
