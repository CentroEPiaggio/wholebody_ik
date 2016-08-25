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
    std::cout<<" -- This is a test to check the wholebody_ik library in a multicontact like scenario --"<<std::endl;

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
    chains.push_back("right_arm");
    chains.push_back("left_arm");
    chains.push_back("right_leg");
    chains.push_back("left_leg");
    visual_utils vutils("e","Waist",chains);
    
    initial_poses["right_arm"] = KDL::Frame(KDL::Rotation::RPY(-0.122, -0.001, 0.349),KDL::Vector(0.186, -0.413, -0.413));
    desired_poses["right_arm"] = initial_poses["right_arm"] * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0,-0.2,0.2));
    q_out["right_arm"] = yarp::sig::Vector(7,0.0);
    q_init["right_arm"] = yarp::sig::Vector(q_out.at("right_arm").size(),0.0);
    q_init.at("right_arm")[1] = -0.2; //NOTE: arms joints limits
    q_sense["right_arm"] = q_init.at("right_arm");
    joints["right_arm"];
    joints.at("right_arm").resize(q_out.at("right_arm").size());
    traj_gens["right_arm"];

    initial_poses["left_arm"] = KDL::Frame(KDL::Rotation::RPY(0.087, -0.001, -0.349),KDL::Vector(0.176, 0.385, -0.415));
    desired_poses["left_arm"] = initial_poses["left_arm"] * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0,0.2,0.2));
    q_out["left_arm"] = yarp::sig::Vector(7,0.0);
    q_init["left_arm"] = yarp::sig::Vector(q_out.at("left_arm").size(),0.0);
    q_init.at("left_arm")[1] = 0.2; //NOTE: arms joints limits
    q_sense["left_arm"] = q_init.at("left_arm");
    joints["left_arm"];
    joints.at("left_arm").resize(q_out.at("left_arm").size());
    traj_gens["left_arm"];

    initial_poses["right_leg"] = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.006, -0.181, -1.083));
    desired_poses["right_leg"] = initial_poses["right_leg"] * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0,-0.2,0.2));
    q_out["right_leg"] = yarp::sig::Vector(6,0.0);
    q_init["right_leg"] = yarp::sig::Vector(q_out.at("right_leg").size(),0.0);
    q_init.at("right_leg")[2] = -0.3; //NOTE: to start far from the singularity
    q_init.at("right_leg")[3] = 0.6; //NOTE: to start far from the singularity
    q_init.at("right_leg")[4] = -0.3; //NOTE: to start far from the singularity
    q_sense["right_leg"] = q_init.at("right_leg");
    joints["right_leg"];
    joints.at("right_leg").resize(q_out.at("right_leg").size());
    traj_gens["right_leg"];

    initial_poses["left_leg"] = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.006, 0.181, -1.083));
    desired_poses["left_leg"] = initial_poses["left_leg"] * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0,0.2,0.2));
    q_out["left_leg"] = yarp::sig::Vector(6,0.0);
    q_init["left_leg"] = yarp::sig::Vector(q_out.at("left_leg").size(),0.0);
    q_init.at("left_leg")[2] = -0.3; //NOTE: to start far from the singularity
    q_init.at("left_leg")[3] = 0.6; //NOTE: to start far from the singularity
    q_init.at("left_leg")[4] = -0.3; //NOTE: to start far from the singularity
    q_sense["left_leg"] = q_init.at("left_leg");
    joints["left_leg"];
    joints.at("left_leg").resize(q_out.at("left_leg").size());
    traj_gens["left_leg"];

    for(auto& joints_:joints)
    {
        for(int i=0;i<q_out.at(joints_.first).size();i++)
        {
            joints_.second.at(i) = q_sense.at(joints_.first)[i];
        }

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

            std::cout<<"err: "<<cart_error<<std::endl;

            for(int i=0;i<q_out.at(traj_gen.first).size();i++)
            {
                joints.at(traj_gen.first).at(i) = q_out.at(traj_gen.first)[i];
            }

            vutils.set_data(next_pose, joints.at(traj_gen.first), traj_gen.first);

            q_sense.at(traj_gen.first) = q_out.at(traj_gen.first);
        }

        usleep(500000);

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