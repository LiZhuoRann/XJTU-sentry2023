// #include "ros/init.h"
// #include "ros/subscriber.h"
// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <string>


// bool initialized = false;
// std::string sentryColor;

// void sentryColorCallback(const std_msgs::String::ConstPtr& msg) {
//     if(strcmp(msg->data.c_str(), "RED")) {
//         initialized = true;
//         sentryColor = "RED";
//     }
//     else if(strcmp(msg->data.c_str(), "BLUE")) {
//         initialized = true;
//         sentryColor = "BLUE";
//     } 
// }

// void fillTransformParam(std::string color, geometry_msgs::TransformStamped& ts) {
//     if(color ==  "RED") {
//         ts.transform.translation.x = 3.4;
//         ts.transform.translation.y = -4;
//         ts.transform.translation.z = 0;
//         //----设置四元数:将 欧拉角数据转换成四元数
//         tf2::Quaternion qtn;
//         qtn.setRPY(0,0,180);
//         ts.transform.rotation.x = qtn.getX();
//         ts.transform.rotation.y = qtn.getY();
//         ts.transform.rotation.z = qtn.getZ();
//         ts.transform.rotation.w = qtn.getW();
//     }
//     else if(color == "BLUE") {
//         ts.transform.translation.x = 0;
//         ts.transform.translation.y = 0;
//         ts.transform.translation.z = 0;
//         //----设置四元数:将 欧拉角数据转换成四元数
//         tf2::Quaternion qtn;
//         qtn.setRPY(0,0,0);
//         ts.transform.rotation.x = qtn.getX();
//         ts.transform.rotation.y = qtn.getY();
//         ts.transform.rotation.z = qtn.getZ();
//         ts.transform.rotation.w = qtn.getW();
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "initMap");
//     ros::NodeHandle nh;
    
//     ros::Subscriber sub_sentryColor = nh.subscribe("sentryColor", 1, sentryColorCallback);

//     tf2_ros::StaticTransformBroadcaster broadcaster;
//     geometry_msgs::TransformStamped ts;
//     ts.header.frame_id = "map";
//     ts.header.stamp = ros::Time::now();
//     ts.child_frame_id = "camera_init";

//     ros::Rate loopRate = 1000;
//     while(ros::ok()) {
//         ros::spinOnce();
//         if(initialized) {
//             fillTransformParam(sentryColor, ts);
//             broadcaster.sendTransform(ts);
//             ROS_INFO("sentry color is %s, initialization finished!", sentryColor.data());
//         } else {
//             ROS_INFO("waiting for sentry color topic ...");
//         }
//         loopRate.sleep();
//     }

//     return 0;
// }

