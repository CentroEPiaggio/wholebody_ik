/* Copyright [2016] [Alessandro Settimi (ale.settimi@gmail.com), Mirko Ferrati, Danilo Caporale, Edoardo Farnioli]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <kdl/frames.hpp>
#include <yarp/sig/Image.h>
#include <mutex>
#include <thread>

class visual_utils
{
public:

int right_arm_index, left_arm_index;

visual_utils(std::string base_frame, std::string e_frame, std::string robot_frame)
{
    // just because tf tree coherence
    e_T_o.frame_id_ = base_frame;
    e_T_o.child_frame_id_ = e_frame;
    r_T_o.frame_id_ = robot_frame;
    r_T_o.child_frame_id_ = base_frame;

    e_T_o.setData(tf::Transform::getIdentity());
    r_T_o.setData(tf::Transform::getIdentity());

    marker.color.a=1;
    marker.color.b=1;
    marker.header.frame_id = base_frame;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 1;
    marker.pose.orientation.w=0.71;
    marker.pose.orientation.x=0.71;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;

    marker_pub = nh.advertise<visualization_msgs::Marker>("object",1);

    joints_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);

    right_arm_index = joints.name.size();

    joints.name.push_back("RShSag");
    joints.name.push_back("RShLat");
    joints.name.push_back("RShYaw");
    joints.name.push_back("RElbj");
    joints.name.push_back("RForearmPlate");
    joints.name.push_back("RWrj1");
    joints.name.push_back("RWrj2");

    left_arm_index = joints.name.size();

    joints.name.push_back("LShSag");
    joints.name.push_back("LShLat");
    joints.name.push_back("LShYaw");
    joints.name.push_back("LElbj");
    joints.name.push_back("LForearmPlate");
    joints.name.push_back("LWrj1");
    joints.name.push_back("LWrj2");

    joints.name.push_back("RHipSag");
    joints.name.push_back("RHipLat");
    joints.name.push_back("RHipYaw");
    joints.name.push_back("RKneeSag");
    joints.name.push_back("RAnkLat");
    joints.name.push_back("RAnkSag");

    joints.name.push_back("LHipSag");
    joints.name.push_back("LHipLat");
    joints.name.push_back("LHipYaw");
    joints.name.push_back("LKneeSag");
    joints.name.push_back("LAnkLat");
    joints.name.push_back("LAnkSag");

    joints.name.push_back("WaistSag");
    joints.name.push_back("WaistLat");
    joints.name.push_back("WaistYaw");
    
    joints.name.push_back("NeckPitchj");
    joints.name.push_back("NeckYawj");
    
    for(auto item:joints.name) joints.position.push_back(0);

    th = new std::thread(&visual_utils::publishing_loop,this);
}

~visual_utils()
{
    delete th;
}

void publishing_loop()
{
    ros::Rate rt(2);
    while(ros::ok())
    {
        data_mutex.lock();

        e_T_o.stamp_ = ros::Time::now();
        r_T_o.stamp_ = ros::Time::now();
        tf_broadcaster.sendTransform(e_T_o);
        tf_broadcaster.sendTransform(r_T_o);
        if(joints.position.size()!=0)
        {
            joints.header.stamp = ros::Time::now();
            joints_pub.publish(joints);
        }

        data_mutex.unlock();
        if(!rt.sleep())
        {
            std::cout<<" -- publishing rate not respected"<<std::endl; 
        }
    }
}

void set_object_robot_tf(KDL::Frame object_T_robot)
{
    data_mutex.lock();
    tf::Transform otr;
    tf::transformKDLToTF(object_T_robot,otr);
    r_T_o.setData(otr.inverse());
    data_mutex.unlock();
}

void set_object_target(KDL::Frame object_T_ee)
{
    data_mutex.lock();
    tf::Transform otr;
    tf::transformKDLToTF(object_T_ee,otr);
    e_T_o.setData(otr);
    data_mutex.unlock();
}

void set_arm_joints( std::vector<double> arm_joints, bool left_arm)
{
    data_mutex.lock();

    int i = (left_arm)?left_arm_index:right_arm_index;
    for(int j=0;j<7;j++)
    {
        this->joints.position.at(i) = arm_joints.at(j);
        i++;
    }
    marker_pub.publish(marker);
    data_mutex.unlock();
}

void set_data(KDL::Frame object_T_ee, KDL::Frame object_T_robot, std::vector<double> arm_joints, bool left_arm)
{
    set_arm_joints(arm_joints,left_arm);
    set_object_robot_tf(object_T_robot);
    set_object_target(object_T_ee);
}

private:
    ros::NodeHandle nh;
    ros::Publisher joints_pub;
    ros::Publisher marker_pub;
    tf::TransformBroadcaster tf_broadcaster;

    tf::StampedTransform r_T_o;
    tf::StampedTransform e_T_o;
    sensor_msgs::JointState joints;
    visualization_msgs::Marker marker;

    std::mutex data_mutex;
    std::thread* th;
};