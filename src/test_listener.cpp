#include "ros/ros.h"
#include "std_msgs/String.h"
#include "awaken_asr/sr_order.h"

void chatterCallback(const asr_record::sr_order order)
{
  ROS_INFO("I heard: [%04x]\n", order.order);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("sr_order", 1000, chatterCallback);

  ros::spin();

  return 0;
}
