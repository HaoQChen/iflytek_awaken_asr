Tihs package use alsa to record voice and wait for awake, and then recognize order.
My code is refer to iflytek's asr_record_sample and awaken_sample. For more infomation, please check through iflytek's reference.

I do not know why, but the iflytek does not accept relative path, so you need to change your package's absolute path in asr_record.h like this:
#define PACKAGE_PATH "/home/your_name/catkin_ws/src/awaken_asr/"

And you need to change your appid which obtained from http://www.xfyun.cn/ in awaken_asr_ros.cpp. 
Replace the resource files in res folder & libs folder

To use it, just put it in your work space and catkin_make.
It will awaken by saying the word in wordlist.txt and the run order recognition according to the rules specific in call.bnf for 15s. When finish, it will change variate g_order and g_is_order_publiced. You can do your job in awaken_asr_ros.cpp.