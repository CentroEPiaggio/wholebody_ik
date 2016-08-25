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
int right_leg_index, left_leg_index;

visual_utils(std::string e_frame, std::string base_frame, std::vector<std::string> chains)
{
    int c=0;
    for(auto chain:chains)
    {
        e_T_b[chain].frame_id_ = base_frame;
        e_T_b.at(chain).child_frame_id_ = e_frame + "_" + std::to_string(c);
        c++;
        e_T_b.at(chain).setData(tf::Transform::getIdentity());
    }

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
    
    right_leg_index = joints.name.size();

    joints.name.push_back("RHipLat");
    joints.name.push_back("RHipYaw");
    joints.name.push_back("RHipSag");
    joints.name.push_back("RKneeSag");
    joints.name.push_back("RAnkSag");
    joints.name.push_back("RAnkLat");
    
    left_leg_index = joints.name.size();
    
    joints.name.push_back("LHipLat");
    joints.name.push_back("LHipYaw");
    joints.name.push_back("LHipSag");
    joints.name.push_back("LKneeSag");
    joints.name.push_back("LAnkSag");
    joints.name.push_back("LAnkLat");

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

        for(auto& T:e_T_b)
        {
            T.second.stamp_ = ros::Time::now();
            tf_broadcaster.sendTransform(T.second);
        }

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

void set_chain_target(KDL::Frame base_T_ee,std::string chain)
{
    data_mutex.lock();
    tf::Transform bTe;
    tf::transformKDLToTF(base_T_ee,bTe);
    e_T_b[chain].setData(bTe);
    data_mutex.unlock();
}

void set_chain_joints( std::vector<double> chain_joints, std::string chain)
{
    data_mutex.lock();

    int i;
    i = (chain=="right_arm")?right_arm_index:i;
    i = (chain=="left_arm")?left_arm_index:i;
    i = (chain=="right_leg")?right_leg_index:i;
    i = (chain=="left_leg")?left_leg_index:i;

    for(int j=0;j<chain_joints.size();j++)
    {
        this->joints.position.at(i) = chain_joints.at(j);
        i++;
    }
    data_mutex.unlock();
}

void set_data(KDL::Frame base_T_ee, std::vector<double> chain_joints, std::string chain)
{
    set_chain_joints(chain_joints,chain);
    set_chain_target(base_T_ee,chain);
}

private:
    ros::NodeHandle nh;
    ros::Publisher joints_pub;
    tf::TransformBroadcaster tf_broadcaster;

    std::map<std::string, tf::StampedTransform> e_T_b;
    sensor_msgs::JointState joints;

    std::mutex data_mutex;
    std::thread* th;
};