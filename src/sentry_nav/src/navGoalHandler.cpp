#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include "sentry_communication/navGoal.h" 

class navigationNode_t {
public:
    ros::NodeHandle nh;
    ros::Publisher pub_navState;
    ros::Subscriber sub_navGoal;
    ros::Timer timer;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    move_base_msgs::MoveBaseGoal goal;
    ros::Time statrTimeStamp;

public:
    navigationNode_t() : ac("move_base", true) {
        // 等待 server 变为可用
        while(!ac.waitForServer(ros::Duration(1.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        if(!ac.isServerConnected()) {
            ROS_ERROR("move_base server not connected!!!!");
        } else {
            ROS_INFO("move_base server connected!!!!");
        }
        sub_navGoal = nh.subscribe<sentry_communication::navGoal>("/navGoal", 1, &navigationNode_t::navGoalCallback,this);
        pub_navState = nh.advertise<std_msgs::String>("/move_base1/navState", 1);
        timer = nh.createTimer(ros::Duration(0.1), boost::bind(&navigationNode_t::timerUpdateCallback, this));
        statrTimeStamp = ros::Time::now();
    }
    void navGoalCallback(const sentry_communication::navGoal::ConstPtr& msg) {
        // 设置目标点的位置和方向
        goal.target_pose.pose.orientation.w = 1;
        // 设置目标点的 frame
        tf::StampedTransform tf;
        tf::TransformListener tf_listener;
        geometry_msgs::PointStamped pointInMapFrame;
        geometry_msgs::PointStamped pointInWorldFrame;
        pointInWorldFrame.point.x = msg->coo_x;
        pointInWorldFrame.point.y = msg->coo_y;
        pointInWorldFrame.point.z = 0;
        pointInWorldFrame.header.frame_id = "realWorld";
        pointInWorldFrame.header.stamp = ros::Time::now();
        bool tferr = true;
        while(tferr) {
            tferr = false;
            try{
                // tf 目标点平移
                tf_listener.transformPoint("map", pointInWorldFrame, pointInMapFrame);
            }
            catch (tf::TransformException &ex) {
                tferr = true;
                static uint32_t count = 0;
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.1).sleep();
                ROS_INFO("Trying to transform point: count %d", count++);
            }
        }

        ROS_INFO("Point in map frame : x:[%f], y:[%f], z:[%f]", \
            pointInMapFrame.point.x, pointInMapFrame.point.y, pointInMapFrame.point.z);
        goal.target_pose.pose.position.x = pointInMapFrame.point.x;
        goal.target_pose.pose.position.y = pointInMapFrame.point.y;
        goal.target_pose.pose.position.z = pointInMapFrame.point.z;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("go to the goal (in 'map' frame): x:[%f], y:[%f]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        ac.sendGoal(goal);
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have reached the goal!");
        else
            ROS_INFO("The base failed for some reason");
    }
    void timerUpdateCallback() {
        if(ros::Time::now() - statrTimeStamp < ros::Duration(2)) {
            return ;
        }
        // 定时发布导航状态信息
        // std::cout << "Timer Update Periodically: navState:[" << ac.getState().text_ << "]" << std::endl;
        std_msgs::String navStateStr;
        navStateStr.data = ac.getState().text_;
        pub_navState.publish(navStateStr);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "initMap");
    navigationNode_t navNode;
    ros::spin();

    return 0;
}

