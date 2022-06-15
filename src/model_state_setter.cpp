/**
 * @Author: Esau Ortiz Toribio
 * @File: model_state_setter.cpp
 * @Description:    Model state setter. Calls SetModelState service with an initial pose
 *                  for a given kobuki
 * @Date: November 2021
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include "gazebo_msgs/SetModelState.h"

using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "model_state_setter");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    // position variables
    std::string kobuki_name;
    float initial_pose_x;
    float initial_pose_y;
    float initial_pose_a;
    tf2::Quaternion my_quaternion;

    // read params
    nh.getParam("model_state_setter/robot_name", kobuki_name);
    nh.getParam("model_state_setter/initial_pose_x", initial_pose_x);
    nh.getParam("model_state_setter/initial_pose_y", initial_pose_y);
    nh.getParam("model_state_setter/initial_pose_a", initial_pose_a);

    // position
    geometry_msgs::Point kobuki_position;
    kobuki_position.x = initial_pose_x;
    kobuki_position.y = initial_pose_y;
    kobuki_position.z = 0.0;

    // orientation
    geometry_msgs::Quaternion kobuki_orientation;
    my_quaternion.setRPY(0, 0, initial_pose_a);
    my_quaternion = my_quaternion.normalize();
    kobuki_orientation.x = my_quaternion.x();
    kobuki_orientation.y = my_quaternion.y();
    kobuki_orientation.z = my_quaternion.z();
    kobuki_orientation.w = my_quaternion.w();

    // pose
    geometry_msgs::Pose kobuki_pose;
    kobuki_pose.position = kobuki_position;
    kobuki_pose.orientation = kobuki_orientation;

    // model state
    gazebo_msgs::ModelState kobuki_modelstate;
    kobuki_modelstate.model_name = kobuki_name;
    kobuki_modelstate.pose = kobuki_pose;

    // set model state request
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = kobuki_modelstate;

    // call service
    if(client.call(srv)){
        ROS_INFO("Model state set properly");
    }else{
        ROS_ERROR("Model state could not be set");
    }

    return 0;
}