#include "ros/ros.h"
#include "std_msgs/String.h"
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include <numeric>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "pgv.hpp"
#include "pgv100/pgv100_msg.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pgv_node");
    ros::NodeHandle nh;
    // down
    ros::Publisher PGV100_QRCode_Line_down_pub = nh.advertise<pgv100::pgv100_msg>("pgv100_qrcode/down/raw_info", 1);
    ros::Publisher PGV100_QRCode_Line_pose_down_pub = nh.advertise<geometry_msgs::Pose>("pgv100_qrcode/down/pose", 1);
    // up
    ros::Publisher PGV100_QRCode_Line_up_pub = nh.advertise<pgv100::pgv100_msg>("pgv100_qrcode/up/raw_info", 1);
    ros::Publisher PGV100_QRCode_Line_pose_up_pub = nh.advertise<geometry_msgs::Pose>("pgv100_qrcode/up/pose", 1);
    //
    //pgv100::pgv100_msg PGV100_QRCode;
    //pgv100::pgv100_msg PGV100_Line;
    pgv100::pgv100_msg PGV100_down_Response;
    pgv100::pgv100_msg PGV100_up_Response;
    // 5hz
    // ros::Rate loop_rate(5);
    // 30hz
    ros::Rate loop_rate(10);
    pgv_100 pgv_100_down("/dev/down_qrcode");
    pgv_100 pgv_100_up("/dev/up_qrcode");
    // pgv_100 pgv_100_obj("/dev/ttyS1");
    /*
    // Setting Color Mode
    if( !pgv_100_obj.Set_Color_Mode(1) )
    {
        cout << "Color Setting Success" << endl;
    }
    // Setting Direction Mode
    pgv_100_obj.Set_Direction_Mode(2);
    */
    //pgv_100_obj.Set_Mix_Color_Mode();
    pgv_100_up.Set_Color_Mode(0);
    pgv_100_up.Set_Direction_Mode(1);
    //
    pgv_100_down.Set_Color_Mode(0);
    pgv_100_down.Set_Direction_Mode(1);
    //
	while(ros::ok())
	{
        // Get QR_Code
        pgv_100_down.Auto_Detect_Update(&PGV100_down_Response);
        PGV100_QRCode_Line_down_pub.publish(PGV100_down_Response);
        PGV100_QRCode_Line_pose_down_pub.publish(pgv_100_down.pgv100_msg_to_pose(&PGV100_down_Response));
        /*
        pgv_100_up.Auto_Detect_Update(&PGV100_up_Response);
        PGV100_QRCode_Line_up_pub.publish(PGV100_up_Response);
        PGV100_QRCode_Line_pose_up_pub.publish(pgv_100_up.pgv100_msg_to_pose(&PGV100_up_Response));
        */
        //pgv_100_obj.QR_Code_Update(&PGV100_QRCode);
        //PGV100_QRCode_pub.publish(PGV100_QRCode);
        // Get Line
        //pgv_100_obj.Line_Update(&PGV100_Line);
        //PGV100_Line_pub.publish(PGV100_Line);
        //pgv_100_obj.QR_Code_Update(&PGV100_Response);
        //PGV100_QRCode_Line_pub.publish(PGV100_Response);
        //pgv_100_obj.Line_Update();
        //pgv_100_obj.Line_Update();
		ros::spinOnce();
		loop_rate.sleep();
	}


    return 0;
}