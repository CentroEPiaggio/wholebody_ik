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

#include <wholebody_ik/wholebody_ik.h>
#include <iostream>
#include <string>
#include "visual_utils.cpp"
#include <trajectory_generator/trajectory_generator.h>

int main(int argc, char** argv)
{
    std::cout<<" -- This is a test to check the wholebody_ik library for the CoM --"<<std::endl;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile( "bigman_config.ini" ); 
    rf.setDefaultContext( "wholebody_ik" );  
    rf.configure(argc, argv);

    std::string robot = rf.find("robot_name").asString();
    std::string urdf = rf.find("urdf_path").asString();
    std::string srdf = rf.find("srdf_path").asString();
    int period = rf.find("thread_period").asInt();

    wholebody_ik IK(robot,urdf,srdf,period);
    
    std::map<std::string,KDL::Frame> desired_poses;
    std::map<std::string,KDL::Frame> initial_poses; //should come from sensing
    std::map<std::string,yarp::sig::Vector> q_init;
    std::map<std::string,yarp::sig::Vector> q_out;
    std::map<std::string,yarp::sig::Vector> q_sense;
    std::map<std::string,std::vector<double>> joints;

    double traj_duration = 3.0;
    std::map<std::string,trajectory_generator> traj_gens;

    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "wholebody_ik_test", ros::init_options::AnonymousName );
    }

    ros::AsyncSpinner as(2);

    std::vector<std::string> chains;
    chains.push_back("com_left_foot");
    visual_utils vutils("e","l_sole",chains);
    
    initial_poses["com_left_foot"] = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.059, -0.181, 1.137));
    desired_poses["com_left_foot"] = initial_poses["com_left_foot"] * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0,0,0));
    q_out["com_left_foot"] = yarp::sig::Vector(31,0.0);
    q_init["com_left_foot"] = yarp::sig::Vector(q_out.at("com_left_foot").size(),0.0);

    //initial joints
    q_init.at("com_left_foot")[0] = 0.6;
    q_init.at("com_left_foot")[1] = -0.2;
    q_init.at("com_left_foot")[3] = -1.2;
    q_init.at("com_left_foot")[5] = -0.6;
    q_init.at("com_left_foot")[7] = 0.6;
    q_init.at("com_left_foot")[8] = 0.2;
    q_init.at("com_left_foot")[10] = -1.2;
    q_init.at("com_left_foot")[12] = -0.6;
    q_init.at("com_left_foot")[19] = -0.3;
    q_init.at("com_left_foot")[20] = 0.6;
    q_init.at("com_left_foot")[21] = -0.3;
    q_init.at("com_left_foot")[27] = -0.3;
    q_init.at("com_left_foot")[28] = 0.6;
    q_init.at("com_left_foot")[29] = -0.3;

    q_sense["com_left_foot"] = q_init.at("com_left_foot");
    joints["com_left_foot"];
    joints.at("com_left_foot").resize(q_out.at("com_left_foot").size());
    traj_gens["com_left_foot"];

    for(auto& joints_:joints)
    {
        for(int i=0;i<q_out.at(joints_.first).size();i++)
        {
            joints_.second.at(i) = q_sense.at(joints_.first)[i];
        }

        std::cout<<"dim: "<<joints_.second.size()<<std::endl;
        vutils.set_data(desired_poses.at(joints_.first), joints_.second, joints_.first);
        traj_gens.at(joints_.first).line_initialize(traj_duration,initial_poses.at(joints_.first),desired_poses.at(joints_.first));
        IK.initialize(joints_.first,desired_poses.at(joints_.first),q_sense.at(joints_.first));
    }
    
    ros::Time start = ros::Time::now();
    ros::Duration exp;
    KDL::Frame next_pose;
    KDL::Twist next_twist;
    double secs;
    double old_t = secs = 0;

    while(ros::ok())
    {
        old_t = secs;
        secs = exp.toSec() + ((double)exp.toNSec()) / 1000000000.0;

        for(auto& traj_gen:traj_gens)
        {
            traj_gen.second.line_trajectory(secs,next_pose,next_twist);
            IK.set_desired_ee_pose(traj_gen.first,next_pose);

            double cart_error = IK.cartToJnt(traj_gen.first,q_sense.at(traj_gen.first),q_out.at(traj_gen.first));

            if(cart_error==-1) std::cout<<" -- error: "<<traj_gen.first<<std::endl;

            for(int i=0;i<q_out.at(traj_gen.first).size();i++)
            {
                joints.at(traj_gen.first).at(i) = q_out.at(traj_gen.first)[i];
            }

            vutils.set_data(next_pose, joints.at(traj_gen.first), traj_gen.first);

            q_sense.at(traj_gen.first) = q_out.at(traj_gen.first);
        }

        usleep(50000);

        if (secs > traj_duration)
        {
            start = ros::Time::now();
            secs=0;

            for(auto& q_s:q_sense)
            {
                q_s.second=q_init.at(q_s.first); //restarting
            }
        }

        exp = ros::Time::now()-start;
    }

    return 0;
}