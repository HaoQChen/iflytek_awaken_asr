#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "../include/msc/msp_cmn.h"
#include "../include/msc/qivw.h"
#include "../include/msc/msp_errors.h"

#include "../include/asr_record/asr_record.h"
#include "../include/asr_record/formats.h"
#include "../include/asr_record/linuxrec.h"
#include "../include/asr_record/speech_recognizer.h"



#define E_SR_NOACTIVEDEVICE		1
#define E_SR_NOMEM				2
#define E_SR_INVAL				3
#define E_SR_RECORDFAIL			4
#define E_SR_ALREADY			5


#define DEFAULT_FORMAT		\
{\
	WAVE_FORMAT_PCM,	\
	1,			\
	16000,			\
	32000,			\
	2,			\
	16,			\
	sizeof(WAVEFORMATEX)	\
}


static int record_state = MSP_AUDIO_SAMPLE_CONTINUE;
struct recorder *recorder;
BOOL g_is_awaken_succeed = TRUE;



static void sleep_ms(size_t ms)
{
	usleep(ms*1000);
}



/* the record call back */
static void iat_cb(char *data, unsigned long len, void *user_para)
{
	int errcode;
	const char *session_id = (const char *)user_para;

	if(len == 0 || data == NULL)
		return;
	if(!g_is_awaken_succeed){
		errcode = QIVWAudioWrite(session_id, (const void *)data, len, record_state);
	}
	if (MSP_SUCCESS != errcode)
	{
		printf("QIVWAudioWrite failed! error code:%d\n",errcode);
		int ret = stop_record(recorder);
		if (ret != 0) {
			printf("Stop failed! \n");
			//return -E_SR_RECORDFAIL;
		}
		wait_for_rec_stop(recorder, (unsigned int)-1);
		QIVWAudioWrite(session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST);
		record_state = MSP_AUDIO_SAMPLE_LAST;
		g_is_awaken_succeed = FALSE;
	}
	if(record_state == MSP_AUDIO_SAMPLE_FIRST){
		record_state = MSP_AUDIO_SAMPLE_CONTINUE;
	}
}



int cb_ivw_msg_proc( const char *sessionID, int msg, int param1, int param2, const void *info, void *userData )
{
  if (MSP_IVW_MSG_ERROR == msg) //唤醒出错消息
  {
    printf("\n\nMSP_IVW_MSG_ERROR errCode = %d\n\n", param1);
    g_is_awaken_succeed = FALSE;
    record_state = MSP_AUDIO_SAMPLE_LAST;
  }else if (MSP_IVW_MSG_WAKEUP == msg) //唤醒成功消息
  {
    printf("\n\nMSP_IVW_MSG_WAKEUP result = %s\n\n", (char*)info);
    g_is_awaken_succeed = TRUE;
    record_state = MSP_AUDIO_SAMPLE_LAST;
  }
  int ret = stop_record(recorder);
	if (ret != 0) {
		printf("Stop failed! \n");
	}else{
		printf("stop success\n");
	}
	//wait_for_rec_stop(recorder, (unsigned int)-1);
	//QIVWAudioWrite(sessionID, NULL, 0, MSP_AUDIO_SAMPLE_LAST);
  
  return 0;
}



void run_ivw(const char *grammar_list ,  const char* session_begin_params)
{
	const char *session_id = NULL;
	int err_code = MSP_SUCCESS;
	char sse_hints[128];
	
	WAVEFORMATEX wavfmt = DEFAULT_FORMAT;
	wavfmt.nSamplesPerSec = SAMPLE_RATE_16K;
	wavfmt.nAvgBytesPerSec = wavfmt.nBlockAlign * wavfmt.nSamplesPerSec;


//start QIVW
	session_id=QIVWSessionBegin(grammar_list, session_begin_params, &err_code);
	if (err_code != MSP_SUCCESS)
	{
		printf("QIVWSessionBegin failed! error code:%d\n",err_code);
		goto exit;
	}


	err_code = QIVWRegisterNotify(session_id, cb_ivw_msg_proc,NULL);
	if (err_code != MSP_SUCCESS)
	{
		snprintf(sse_hints, sizeof(sse_hints), "QIVWRegisterNotify errorCode=%d", err_code);
		printf("QIVWRegisterNotify failed! error code:%d\n",err_code);
		goto exit;
	}
//start record
	err_code = create_recorder(&recorder, iat_cb, (void*)session_id);
	if (recorder == NULL || err_code != 0) {
			printf("create recorder failed: %d\n", err_code);
			err_code = -E_SR_RECORDFAIL;
			goto exit;
	}

	err_code = open_recorder(recorder, get_default_input_dev(), &wavfmt);
	if (err_code != 0) {
		printf("recorder open failed: %d\n", err_code);
		err_code = -E_SR_RECORDFAIL;
		goto exit;
	}

	err_code = start_record(recorder);
	if (err_code != 0) {
		printf("start record failed: %d\n", err_code);
		err_code = -E_SR_RECORDFAIL;
		goto exit;
	}
	record_state = MSP_AUDIO_SAMPLE_FIRST;

	
	while(record_state != MSP_AUDIO_SAMPLE_LAST)
	{
		sleep_ms(200); //模拟人说话时间间隙，10帧的音频时长为200ms
		printf("waiting for awaken%d\n", record_state);
	}
	snprintf(sse_hints, sizeof(sse_hints), "success");

exit:
	if (recorder) {
		if(!is_record_stopped(recorder))
			stop_record(recorder);
		close_recorder(recorder);
		destroy_recorder(recorder);
		recorder = NULL;
	}
	if (NULL != session_id)
	{
		QIVWSessionEnd(session_id, sse_hints);
	}
}

