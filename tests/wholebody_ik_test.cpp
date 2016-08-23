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

    bool right=true; //change this to perform the test for left or right arm (it is also a parameter from outside, e.g. "wholebody_ik_test 0" is to use the left arm)
    
    if(argc>1)
    {
        if(argc!=2)
        {
            std::cout<<"Please use 1 or none parameter!"<<std::endl;
            return -1;
        }
        right = std::atoi(argv[1]);
    }

    std::string arm = right?"right_arm":"left_arm";

    std::cout<<" -- using arm: "<<arm<<" --"<<std::endl;
    
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

    desired_poses[arm]; //desired ee pose in object frame: t_p_e

    yarp::sig::Vector q_out(7,0.0);

    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "wholebody_ik_test", ros::init_options::AnonymousName );
    }

    ros::AsyncSpinner as(2);

    visual_utils vutils("t","e","gaze");

    std::vector<double> joints;
    joints.resize(7);
    double x=-0.6,y=0.5,z=0.5;
    if(!right) y = -0.5;
    double sign=-1;
    
    KDL::Frame initial_pose; //should come from sensing
    KDL::Frame trash_pose;
    
    if(right)
    {
        desired_poses.at(arm) = KDL::Frame(KDL::Rotation::RPY(0,-M_PI/2.0,0),KDL::Vector(0,0,0)) * KDL::Frame(KDL::Rotation::RPY(0,0,-M_PI/2.0),KDL::Vector(0,0,0));
        initial_pose = KDL::Frame(KDL::Rotation::Quaternion(-0.098, -0.017, 0.173, 0.980),KDL::Vector(-0.590, 0.024, -0.505));
        trash_pose = KDL::Frame(KDL::Rotation::RPY(0,-M_PI/2.0,0),KDL::Vector::Zero()) * KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0.3,-0.3,0));
    }
    else
    {
        desired_poses.at(arm) = KDL::Frame(KDL::Rotation::RPY(0,-M_PI/2.0,0),KDL::Vector(0,0,0)) * KDL::Frame(KDL::Rotation::RPY(0,0,M_PI/2.0),KDL::Vector(0,0,0));
        initial_pose = KDL::Frame(KDL::Rotation::Quaternion(0.098, -0.017, -0.173, 0.980),KDL::Vector(-0.590, -0.024, -0.505));
        trash_pose = KDL::Frame(KDL::Rotation::RPY(0,-M_PI/2.0,0),KDL::Vector::Zero()) * KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0.3,0.3,0));
    }

    double qs[7] = {0,right?-0.2:0.2,0,0,0,0,0};
    yarp::sig::Vector q_init(7,qs);
    yarp::sig::Vector q_sense = q_init;

    IK.initialize(arm,desired_poses.at(arm),q_sense);
    KDL::Frame t_T_h (KDL::Rotation::Identity(),KDL::Vector(x,y,z));
    IK.set_new_object_head_transform(arm, t_T_h.Inverse());

    
    for(int i=0;i<q_out.size();i++)
    {
        joints.at(i) = q_sense[i];
    }
    vutils.set_data(desired_poses.at(arm), t_T_h, joints, !right);

    double traj_duration = 3.0;
    trajectory_generator traj_gen, trash_traj_gen;
    trajectory_generator* current_traj = &traj_gen;
    traj_gen.line_initialize(traj_duration,initial_pose,desired_poses.at(arm));
    trash_traj_gen.line_initialize(traj_duration,desired_poses.at(arm),trash_pose);
    
    ros::Time start = ros::Time::now();
    ros::Duration exp;
    KDL::Frame next_pose, obj_next_pose;
    KDL::Twist next_twist, obj_next_twist;
    double secs;
    bool first = true;

    // simulating robot movement
    
    KDL::Frame t_TFinal_h = KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0.4,0,0))*t_T_h;
    trajectory_generator obj_traj_gen, obj_trash_traj_gen;
    trajectory_generator* current_obj_tra = &obj_traj_gen;    
    obj_traj_gen.line_initialize(traj_duration,t_T_h,t_TFinal_h);
    obj_trash_traj_gen.line_initialize(traj_duration,t_TFinal_h,t_TFinal_h);

    double old_t = secs = 0;

    while(ros::ok())
    {
        old_t = secs;
        secs = exp.toSec() + ((double)exp.toNSec()) / 1000000000.0;

        current_traj->line_trajectory(secs,next_pose,next_twist);
        IK.set_desired_ee_pose(arm,next_pose);

        #ifndef NDEBUG
	utils clock;
	clock.tic();
        #endif

        current_obj_tra->line_trajectory(secs,obj_next_pose,obj_next_twist);
        IK.set_new_object_head_transform(arm, obj_next_pose.Inverse()); 
//         IK.set_new_object_head_transform(arm, obj_next_pose.Inverse(),secs-old_t); //pass a delta_time to update the shoulder position w.r.t. the object

	double cart_error = IK.cartToJnt(arm,q_sense,q_out);

        #ifndef NDEBUG
	clock.toc();
        #endif

        std::cout<<"Cartesian error: "<< cart_error <<std::endl;
    
        for(int i=0;i<q_out.size();i++)
        {
            joints.at(i) = q_out[i];
        }

        vutils.set_data(next_pose, obj_next_pose, joints, !right);

	usleep(500000);

        q_sense = q_out;

        if (secs > traj_duration)
        {
            start = ros::Time::now();
            secs=0;

            if(first)
            {
                current_traj = &trash_traj_gen;
                first = false;
                current_obj_tra = &obj_trash_traj_gen;
            }
            else
            {
                current_traj = &traj_gen;
                q_sense=q_init; //restarting
                first = true;
                current_obj_tra = &obj_traj_gen;
            }
        }

        exp = ros::Time::now()-start;
    }

    return 0;
}