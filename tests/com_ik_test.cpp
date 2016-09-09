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

using namespace yarp::math;

int main(int argc, char** argv)
{
    std::cout<<" -- This is a test to check the wholebody_ik library for the CoM --"<<std::endl;

    bool right=false; //change this to perform the test for com wrt left or right foot

    if(argc>1)
    {
        if(argc!=2)
        {
            std::cout<<"Please use 1 or none parameter!"<<std::endl;
            return -1;
        }
        right = std::atoi(argv[1]);
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile( "bigman_config.ini" ); 
    rf.setDefaultContext( "wholebody_ik" );  
    rf.configure(argc, argv);

    std::string robot = rf.find("robot_name").asString();
    std::string urdf = rf.find("urdf_path").asString();
    std::string srdf = rf.find("srdf_path").asString();
    int period = rf.find("thread_period").asInt();
    double s_period = (double)period / 1000.0;

    wholebody_ik IK(robot,urdf,srdf,period);
    
    std::map<std::string,KDL::Frame> desired_poses;
    std::map<std::string,KDL::Frame> next_poses;
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

    std::string chain;
    std::string f_frame;
    std::string other_leg;
    double y_sign;

    if(right)
    {
        chain = "wb_right";
        f_frame = "r_sole";
        y_sign = 1;
    }
    else
    {
        chain = "wb_left";
        f_frame = "l_sole";
        y_sign = -1;
    }

    std::map<std::string, std::string> links_to_chains;
    links_to_chains["LSoftHand"] = "left_arm";
    links_to_chains["RSoftHand"] = "right_arm";
    links_to_chains["l_sole"] = "left_leg";
    links_to_chains["r_sole"] = "right_leg";
    links_to_chains["COM"] = chain;

    other_leg = (f_frame=="r_sole")?"left_leg":"right_leg";
    std::vector<std::string> chains;
    chains.push_back(chain);
    chains.push_back(other_leg);
    chains.push_back("left_arm");
    chains.push_back("right_arm");
    visual_utils vutils("e",f_frame,chains);

    iDynUtils idynutils(robot,urdf,srdf);

    //initial joints
    yarp::sig::Vector q_left_arm(7,0.0);
    q_left_arm[0]  = 0.6;
    q_left_arm[1]  = 0.2;
    q_left_arm[3]  = -1.2;
    q_left_arm[5]  = -0.6;
    yarp::sig::Vector q_right_arm(7,0.0);
    q_right_arm[0] =  0.6;
    q_right_arm[1] = -0.2;
    q_right_arm[3] = -1.2;
    q_right_arm[5] = -0.6;
    yarp::sig::Vector q_left_leg(6,0.0);
    q_left_leg[2]  = -0.3;
    q_left_leg[3]  =  0.6;
    q_left_leg[4]  = -0.3;
    yarp::sig::Vector q_right_leg(6,0.0);
    q_right_leg[2] = -0.3;
    q_right_leg[3] =  0.6;
    q_right_leg[4] = -0.3;
    yarp::sig::Vector q_torso(3,0.0);
    yarp::sig::Vector q_head(2,0.0);

    q_out[chain] = yarp::sig::Vector(31,0.0);
    q_init[chain] = yarp::sig::Vector(q_out.at(chain).size(),0.0);

    idynutils.fromRobotToIDyn(q_left_arm,q_init.at(chain),idynutils.left_arm);
    idynutils.fromRobotToIDyn(q_right_arm,q_init.at(chain),idynutils.right_arm);
    idynutils.fromRobotToIDyn(q_left_leg,q_init.at(chain),idynutils.left_leg);
    idynutils.fromRobotToIDyn(q_right_leg,q_init.at(chain),idynutils.right_leg);
    idynutils.fromRobotToIDyn(q_torso,q_init.at(chain),idynutils.torso);
    idynutils.fromRobotToIDyn(q_head,q_init.at(chain),idynutils.head);

    q_sense[chain] = yarp::sig::Vector(q_init.at(chain).size(),0.0);
    q_sense.at(chain) = q_init.at(chain);
    joints[chain];
    joints.at(chain).resize(q_out.at(chain).size());

    for(auto& joints_:joints)
    {
        for(int i=0;i<q_out.at(joints_.first).size();i++)
        {
            joints_.second.at(i) = q_sense.at(joints_.first)[i];
        }

        
        IK.initialize(joints_.first,q_sense.at(joints_.first));
    }

    IK.set_desired_wb_poses_as_current(chain);
    IK.get_desired_wb_poses(chain,initial_poses);

    desired_poses["RSoftHand"] = initial_poses.at("RSoftHand") * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.1,0,0));
    desired_poses["LSoftHand"] = initial_poses.at("LSoftHand");
    desired_poses["r_sole"] = initial_poses.at("r_sole");
    desired_poses["l_sole"] = initial_poses.at("l_sole");
    desired_poses["COM"] = initial_poses.at("COM");

    for(auto pose:desired_poses)
    {
        traj_gens[pose.first].line_initialize(traj_duration,initial_poses.at(pose.first),desired_poses.at(pose.first));
        vutils.set_data(desired_poses.at(pose.first), joints.at(chain), chain);
        vutils.set_chain_target(pose.second,chain);
    }
    
    ros::Time start = ros::Time::now();
    ros::Duration exp;
    KDL::Twist next_twist;
    double secs;
    double old_t = secs = 0;
    double cart_error = 9999;

    while(ros::ok())
    {
        std::cout<<"["<<old_t<<"]: "<<cart_error<<std::endl;

        old_t = secs;
        secs = exp.toSec() + ((double)exp.toNSec()) / 1000000000.0;

        for(auto& traj_gen:traj_gens)
        {
            traj_gen.second.line_trajectory(secs,next_poses[traj_gen.first],next_twist);
            vutils.set_data(next_poses.at(traj_gen.first), joints.at(chain), chain);
        }

        IK.set_desired_wb_poses(chain,next_poses);

        cart_error = IK.cartToJnt(chain,q_sense.at(chain),q_out.at(chain));

        if(cart_error==-1) std::cout<<" -- NOT CONVERGED: "<<chain<<std::endl;

        for(int i=0;i<q_out.at(chain).size();i++)
        {
            joints.at(chain).at(i) = q_out.at(chain)[i];
        }

        q_sense.at(chain) = q_out.at(chain);

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