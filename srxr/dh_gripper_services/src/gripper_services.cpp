#include "ros/ros.h"
#include "dh_gripper_services/CtrlGripper.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int s, nbytes;
struct can_frame frame[4] = {{0}};

bool ctrlGripper(dh_gripper_services::CtrlGripper::Request  &req,
         dh_gripper_services::CtrlGripper::Response &res)
{
  long int sum = req.force + req.pos;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.force, (long int)req.pos);
  res.isSuc=true;
  ROS_INFO("sending back response: [%d]", res.isSuc);

  if(req.force<0 || req.force>90)
  {
    ROS_INFO("force out limit: %d in [0, 90]\n!",req.force);
    return false;
  }

  if(req.pos<0 || req.pos>90)
  {
    ROS_INFO("Pos out limit: %d in [0, 90]\n!",req.pos);
    return false;
  }

  frame[2].can_id = 0x01;
  frame[2]. can_dlc = 8;
  frame[2].data[0] = 0x05;
  frame[2].data[1] = 0x02;
  frame[2].data[2] = 0x01;
  frame[2].data[3] = 0x00;
  frame[2].data[4] = req.force;
  frame[2].data[5] = 0x00;
  frame[2].data[6] = 0x00;
  frame[2].data[7] = 0x00;

  frame[3].can_id = 0x01;
  frame[3]. can_dlc = 8;
  frame[3].data[0] = 0x06;
  frame[3].data[1] = 0x02;
  frame[3].data[2] = 0x01;
  frame[3].data[3] = 0x00;
  frame[3].data[4] = req.pos;
  frame[3].data[5] = 0x00;
  frame[3].data[6] = 0x00;
  frame[3].data[7] = 0x00;

  nbytes = write(s, &frame[2], sizeof(frame[2])); //发送frame[1]
  if(nbytes != sizeof(frame[2]))
  {
    ROS_INFO("Send Error frame[2]\n!");
//    printf("Send Error frame[2]\n!");
    return false;
  }
  usleep(1000);
  nbytes = write(s, &frame[3], sizeof(frame[3])); //发送frame[1]
  if(nbytes != sizeof(frame[3]))
  {
    ROS_INFO("Send Error frame[3]\n!");
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ctrlGripperServer");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("ctrlGripper", ctrlGripper);
  ROS_INFO("Ready to move gripper.");

//  int s, nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can0" );
  ioctl(s, SIOCGIFINDEX, &ifr); //指定can0 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0 绑定
  //禁用过滤规则，本进程不接收报文，只负责发送
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
  //生成两个报文
  frame[0].can_id = 0x01;
  frame[0]. can_dlc = 8;
  frame[0].data[0] = 0x08;
  frame[0].data[1] = 0x02;
  frame[0].data[2] = 0x01;
  frame[0].data[3] = 0x00;
  frame[0].data[4] = 0x00;
  frame[0].data[5] = 0x00;
  frame[0].data[6] = 0x00;
  frame[0].data[7] = 0x00;

  frame[1].can_id = 0x01;
  frame[1]. can_dlc = 8;
  frame[1].data[0] = 0x05;
  frame[1].data[1] = 0x02;
  frame[1].data[2] = 0x01;
  frame[1].data[3] = 0x00;
  frame[1].data[4] = 0x1A;
  frame[1].data[5] = 0x00;
  frame[1].data[6] = 0x00;
  frame[1].data[7] = 0x00;

  frame[2].can_id = 0x01;
  frame[2]. can_dlc = 8;
  frame[2].data[0] = 0x06;
  frame[2].data[1] = 0x02;
  frame[2].data[2] = 0x01;
  frame[2].data[3] = 0x00;
  frame[2].data[4] = 0x5A;
  frame[2].data[5] = 0x00;
  frame[2].data[6] = 0x00;
  frame[2].data[7] = 0x00;

  frame[3].can_id = 0x01;
  frame[3]. can_dlc = 8;
  frame[3].data[0] = 0x06;
  frame[3].data[1] = 0x02;
  frame[3].data[2] = 0x01;
  frame[3].data[3] = 0x00;
  frame[3].data[4] = 0x00;
  frame[3].data[5] = 0x00;
  frame[3].data[6] = 0x00;
  frame[3].data[7] = 0x00;
    nbytes = write(s, &frame[0], sizeof(frame[0])); //发送frame[0]
    if(nbytes != sizeof(frame[0]))
    {
      printf("Send Error frame[0]\n!");
      //break; //发送错误，退出
    }
    sleep(2);
    nbytes = write(s, &frame[1], sizeof(frame[1])); //发送frame[1]
    if(nbytes != sizeof(frame[1]))
    {
      printf("Send Error frame[1]\n!");
      //break;
    }
    sleep(1);
  //循环发送两个报文
//  while(1)
//  {
//    nbytes = write(s, &frame[2], sizeof(frame[2])); //发送frame[1]
//    if(nbytes != sizeof(frame[2]))
//    {
//      printf("Send Error frame[2]\n!");
//      break;
//    }
//    sleep(2);
//    nbytes = write(s, &frame[3], sizeof(frame[3])); //发送frame[1]
//    if(nbytes != sizeof(frame[3]))
//    {
//      printf("Send Error frame[3]\n!");
//      break;
//    }
//    sleep(2);
//  }
//  close(s);
    ros::spin();
  return 0;
}
