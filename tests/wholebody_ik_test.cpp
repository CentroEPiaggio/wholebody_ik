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

#ifndef NDEBUG
#include "utils.cpp"
#endif

int main(int argc, char** argv)
{
    std::cout<<" -- This is a test to check the wholebody_ik library --"<<std::endl;

    bool right=true; //change this to perform the test for left or right arm
    bool arm=true; //change this to perform the test for arms or legs (these are also parameters from outside, e.g. "wholebody_ik_test 0 0" is to use the left leg)
    
    if(argc>2)
    {
        if(argc!=3)
        {
            std::cout<<"Please use 2 or none parameter!"<<std::endl;
            return -1;
        }
        right = std::atoi(argv[1]);
        arm = std::atoi(argv[2]);
    }

    std::string chain="";

    if(right)
        if(arm)
            chain = "right_arm";
        else
            chain = "right_leg";
    else
        if(arm)
            chain = "left_arm";
        else
            chain = "left_leg";

    if(chain=="") {std::cout<<"WRONG CHAIN!"<<std::endl; return -1;}

    std::cout<<" -- using chain: "<<chain<<" --"<<std::endl;
    
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

    desired_poses[chain]; //desired ee pose in object frame: t_p_e
    
    yarp::sig::Vector q_out;

    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "wholebody_ik_test", ros::init_options::AnonymousName );
    }

    ros::AsyncSpinner as(2);
    std::vector<std::string> chains;
    visual_utils* vutils;
    KDL::Frame initial_pose; //should come from sensing
    yarp::sig::Vector q_init;
    
    if(chain == "right_arm")
    {
        initial_pose = KDL::Frame(KDL::Rotation::RPY(-0.830, -1.194, 0.831),KDL::Vector(0.410, -0.45, -0.14));
        desired_poses.at(chain) = initial_pose * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.2,-0.2,-0.2));
        q_out = yarp::sig::Vector(7,0.0);
        q_init = yarp::sig::Vector(q_out.size(),0.0);
        q_init[0] = 0.6; //NOTE: to start far from the singularity
        q_init[1] = -0.2; //NOTE: arms joints limits
        q_init[3] = -1.2; //NOTE: to start far from the singularity
        q_init[5] = -0.6; //NOTE: to start far from the singularity
        chains.push_back(chain);
        vutils = new visual_utils("e","Waist",chains);
    }

    if(chain == "left_arm")
    {
        initial_pose = KDL::Frame(KDL::Rotation::RPY(0.872, -1.233, -0.887),KDL::Vector(0.410, 0.45, -0.14));
        desired_poses.at(chain) = initial_pose * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.2,0.2,-0.2));
        q_out = yarp::sig::Vector(7,0.0);
        q_init = yarp::sig::Vector(q_out.size(),0.0);
        q_init[0] = 0.6; //NOTE: to start far from the singularity
        q_init[1] = 0.2; //NOTE: arms joints limits
        q_init[3] = -1.2; //NOTE: to start far from the singularity
        q_init[5] = -0.6; //NOTE: to start far from the singularity
        chains.push_back(chain);
        vutils = new visual_utils("e","Waist",chains);
    }

    if(chain == "right_leg")
    {
        initial_pose = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.006, -0.181, -1.083));
        desired_poses.at(chain) = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.006, -0.181, -0.95));
        q_out = yarp::sig::Vector(6,0.0);
        q_init = yarp::sig::Vector(q_out.size(),0.0);
        q_init[2] = -0.3; //NOTE: to start far from the singularity
        q_init[3] = 0.6; //NOTE: to start far from the singularity
        q_init[4] = -0.3; //NOTE: to start far from the singularity
        chains.push_back(chain);
        vutils = new visual_utils("e","Waist",chains);
    }

    if(chain == "left_leg")
    {
        initial_pose = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.006, 0.181, -1.083));
        desired_poses.at(chain) = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(0.006, 0.181, -0.95));
        q_out = yarp::sig::Vector(6,0.0);
        q_init = yarp::sig::Vector(q_out.size(),0.0);
        q_init[2] = -0.3; //NOTE: to start far from the singularity
        q_init[3] = 0.6; //NOTE: to start far from the singularity
        q_init[4] = -0.3; //NOTE: to start far from the singularity
        chains.push_back(chain);
        vutils = new visual_utils("e","Waist",chains);
    }

    std::vector<double> joints;
    joints.resize(q_out.size());
    yarp::sig::Vector q_sense = q_init;

    IK.initialize(chain,q_sense);

    for(int i=0;i<q_out.size();i++)
    {
        joints.at(i) = q_sense[i];
    }
    vutils->set_data(desired_poses.at(chain), joints, chain);

    double traj_duration = 3.0;
    trajectory_generator traj_gen;
    traj_gen.line_initialize(traj_duration,initial_pose,desired_poses.at(chain));
    
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

        traj_gen.line_trajectory(secs,next_pose,next_twist);
        IK.set_desired_ee_pose(chain,next_pose);

        #ifndef NDEBUG
	utils clock;
	clock.tic();
        #endif

	double cart_error = IK.cartToJnt(chain,q_sense,q_out);

        #ifndef NDEBUG
	clock.toc();
        #endif

        std::cout<<"Cartesian error: "<< cart_error <<std::endl;
    
        for(int i=0;i<q_out.size();i++)
        {
            joints.at(i) = q_out[i];
        }

        vutils->set_data(next_pose, joints, chain);

	usleep(500000);

        q_sense = q_out;

        if (secs > traj_duration)
        {
            start = ros::Time::now();
            secs=0;

            q_sense=q_init; //restarting
        }

        exp = ros::Time::now()-start;
    }

    delete vutils;

    return 0;
}