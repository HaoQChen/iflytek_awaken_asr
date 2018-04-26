#include "../include/asr_record/play_audio.h"
#include "alsa/asoundlib.h"
int set_pcm_play(FILE *fp, WavHeader *wav_header)
{
    int rc;
    int ret;
    int size;
    snd_pcm_t* handle; //PCI设备句柄
    snd_pcm_hw_params_t* params;//硬件信息和PCM流配置
    unsigned int val;
    int dir=0;
    snd_pcm_uframes_t frames;
    char *buffer;
    int channels=wav_header->wChannels;
    int frequency=wav_header->nSamplesPersec;
    int bit=wav_header->wBitsPerSample;
    int datablock=wav_header->wBlockAlign;
    //unsigned char ch[100]; //用来存储wav文件的头信息



    rc=snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    if(rc<0)
    {
        perror("\nopen PCM device failed:");
        exit(1);
    }


    snd_pcm_hw_params_alloca(&params); //分配params结构体
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_alloca:");
        exit(1);
    }
     rc=snd_pcm_hw_params_any(handle, params);//初始化params
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_any:");
        exit(1);
    }
    rc=snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED); //初始化访问权限
    if(rc<0)
    {
        perror("\nsed_pcm_hw_set_access:");
        exit(1);
    }

    //采样位数
    switch(bit/8)
    {
        case 1:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U8);
                break ;
        case 2:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
                break ;
        case 3:snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S24_LE);
                break ;

    }
    rc=snd_pcm_hw_params_set_channels(handle, params, channels); //设置声道,1表示单声>道，2表示立体声
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_set_channels:");
        exit(1);
    }
    val = frequency;
    rc=snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir); //设置>频率
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_set_rate_near:");
        exit(1);
    }

    rc = snd_pcm_hw_params(handle, params);
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params: ");
        exit(1);
    }

    rc=snd_pcm_hw_params_get_period_size(params, &frames, &dir); /*获取周期长度*/
    if(rc<0)
    {
        perror("\nsnd_pcm_hw_params_get_period_size:");
        exit(1);
    }

    size = frames * datablock; /*4 代表数据快长度*/

    buffer =(char*)malloc(size);
    fseek(fp,58,SEEK_SET); //定位歌曲到数据区

    while (1)
    {
        memset(buffer,0,sizeof(buffer));
        ret = fread(buffer, 1, size, fp);
        if(ret == 0)
        {
            //printf("歌曲写入结束\n");
            break;
        }
         else if (ret != size)
        {
          //printf("歌曲写入结束\n");
          break;
        }

        // 写音频数据到PCM设备
        snd_pcm_writei(handle, buffer, frames);
        if (ret == -EPIPE){
          /* EPIPE means underrun */
          fprintf(stderr, "underrun occurred\n");
          //完成硬件参数设置，使设备准备好
          snd_pcm_prepare(handle);
        }else if (ret < 0){
           fprintf(stderr,
           "error from writei: %s\n",
           snd_strerror(ret));
        }/*else if(ret > 0 || r == -EAGAIN){
          snd_pcm_wait
        }*/
        usleep(2 * 1000);
    }

    snd_pcm_drain(handle);
    snd_pcm_close(handle);
    free(buffer);
    return 0;
}



int play_wav(char *path){
    FILE *fp;
    WavHeader wav_header;
    fp = fopen(path, "rb");
    if(fp == NULL){
      return -1;
    }
    int nread = fread(&wav_header, 1, sizeof(wav_header), fp);
    set_pcm_play(fp, &wav_header);
}













