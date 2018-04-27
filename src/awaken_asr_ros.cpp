#include "ros/ros.h"
#include "std_msgs/String.h"
#include "awaken_asr/sr_order.h"

#include "../include/msc/msp_cmn.h"
#include "../include/msc/msp_errors.h"

#include "../include/asr_record/asr_record.h"
#include "../include/awaken/awaken.h"
#include "../include/asr_record/play_audio.h"


#define lgi_param_a "appid = ,engine_start = ivw,work_dir = .,ivw_res_path =fo|"
#define lgi_param_b concat(lgi_param_a, PACKAGE_PATH)
const char *lgi_param = concat(lgi_param_b, "res/ivw/wakeupresource.jet"); //使用唤醒需要在此设置engine_start = ivw,ivw_res_path =fo|xxx/xx 启动唤醒引擎
const char *ssb_param = "ivw_threshold=0:-20,sst=wakeup";



int16_t g_order = ORDER_NONE;
BOOL g_is_order_publiced = FALSE;
UserData asr_data;


#define MAX_SIZE 100
int main(int argc, char **argv)
{
  ros::init(argc, argv, "asr_record");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<asr_record::sr_order>("sr_order", 1000);
  ros::Rate loop_rate(10);

  char current_absolute_path[MAX_SIZE];
  //获取当前程序绝对路径
  int cnt = readlink("/proc/self/exe", current_absolute_path, MAX_SIZE);
  getcwd(current_absolute_path, MAX_SIZE);
  printf("current absolute path:%s\n", current_absolute_path);
//init iflytek


  int ret = 0 ;
  asr_record::sr_order order_send;

  ret = MSPLogin(NULL, NULL, lgi_param);
  if (MSP_SUCCESS != ret)
  {
    printf("MSPLogin failed, error code: %d.\n", ret);
    goto exit ;//登录失败，退出登录
  }


  memset(&asr_data, 0, sizeof(UserData));
  printf("构建离线识别语法网络...\n");
  ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
  if (MSP_SUCCESS != ret) {
    printf("构建语法调用失败！\n");
    goto exit;
  }
  while (1 != asr_data.build_fini)
    usleep(300 * 1000);
  if (MSP_SUCCESS != asr_data.errcode)
    goto exit;
  printf("离线识别语法网络构建完成，开始识别...\n");


  while (1)
  {

    run_ivw(NULL, ssb_param); 
    printf("finish run_ivw\n");
    if(g_is_awaken_succeed){
      printf("begin to run asr\n");
      run_asr(&asr_data);
      g_is_awaken_succeed = FALSE;
    }
    printf("%d:%d\n", g_is_order_publiced, g_order);
    if(ros::ok() && g_is_order_publiced == FALSE){
      if(g_order==ORDER_BACK_TO_CHARGE){
        printf("%d\n", g_order);
        play_wav((char*)"./src/awaken_asr/audios/back_to_charge.wav");        
      }
      if(g_order == ORDER_FACE_DETECTION){
        printf("%d\n", g_order);
        play_wav((char*)"./src/awaken_asr/audios/operating_face_rec.wav");
      }
      order_send.order = g_order;
      chatter_pub.publish(order_send);
      g_is_order_publiced = TRUE;
		}
    

    ros::spinOnce();

    //loop_rate.sleep();
  }
exit:
  MSPLogout();
  printf("请按任意键退出...\n");
  getchar();

  return 0;
}




