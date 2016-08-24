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

#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <iostream>
#include <iCub/iDynTree/yarp_kdl.h>
#include "wholebody_ik_module_thread.h"
#include <kdl/frames_io.hpp>
#include "math_utilities.cpp"

#define DEG2RAD    (M_PI/180.0)

using namespace yarp::math;

using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman;

wholebody_ik_thread::wholebody_ik_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph):
control_thread( module_prefix, rf, ph ), receive_from_pci("pci_control_interface"), send_to_pci("control_pci_interface"), ik(get_robot_name(),get_urdf_path(),get_srdf_path(),get_thread_period())
{
    input.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    output.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    home.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    q_init.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    joint_min.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    joint_max.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    
    yarp::sig::Vector q_right_arm(7,0.0);
    yarp::sig::Vector q_left_arm(7,0.0);
    yarp::sig::Vector q_torso(3,0.0);
    yarp::sig::Vector q_right_leg(6,0.0);
    yarp::sig::Vector q_left_leg(7,0.0);
    yarp::sig::Vector q_head(2,0.0);

    q_head[0] = 0.0;
    q_head[1] = 0.0;

    q_right_arm[0]=30.0*DEG2RAD;
    q_right_arm[1]=-10.0*DEG2RAD;
    q_right_arm[2]=20.0*DEG2RAD;
    q_right_arm[3]=-35.0*DEG2RAD;
    q_right_arm[4]=0.0*DEG2RAD;
    q_right_arm[5]=-30.0*DEG2RAD;
    q_right_arm[6]=0.0*DEG2RAD;

    q_left_arm[0]=30.0*DEG2RAD;
    q_left_arm[1]=10.0*DEG2RAD;
    q_left_arm[2]=-20.0*DEG2RAD;
    q_left_arm[3]=-35.0*DEG2RAD;
    q_left_arm[4]=0.0*DEG2RAD;
    q_left_arm[5]=-30.0*DEG2RAD;
    q_left_arm[6]=0.0*DEG2RAD;
    
    q_torso[0] = 0.0;
    q_torso[1] = -0.4*DEG2RAD;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=2.5*DEG2RAD;
    q_right_leg[1]=0.015*DEG2RAD;
    q_right_leg[2]=-20.8*DEG2RAD;
    q_right_leg[3]=33.4*DEG2RAD;
    q_right_leg[4]=-12.0*DEG2RAD;
    q_right_leg[5]=-2.5*DEG2RAD;
    
    q_left_leg[0]=-2.5*DEG2RAD;
    q_left_leg[1]=-0.015*DEG2RAD;
    q_left_leg[2]=-20.8*DEG2RAD;
    q_left_leg[3]=33.4*DEG2RAD;
    q_left_leg[4]=-12.0*DEG2RAD;
    q_left_leg[5]=2.5*DEG2RAD;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,home);

    initialized["left_arm"]=false;
    initialized["right_arm"]=false;
}

bool wholebody_ik_thread::custom_init()
{
    //  real time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam ( pthread_self(), SCHED_FIFO, &thread_param );
    
    // setup
    model.iDyn3_model.setFloatingBaseLink(model.left_leg.end_effector_index);
    sense();
    output = input;
    robot.setPositionDirectMode();

    // joints limits
    joint_max = model.iDyn3_model.getJointBoundMax();
    joint_min = model.iDyn3_model.getJointBoundMin();
    for (int i=0;i<input.size();i++)
    {
        if (input[i]>joint_max[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<input[i]<<") is outside maximum bound: "<<joint_max[i]<<std::endl;
        }
        if (input[i]<joint_min[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<input[i]<<") is outside minimum bound: "<<joint_min[i]<<std::endl;
        }
    }
    
    // initial position
    go_in_initial_position();

    std::cout<<" - Initialized"<<std::endl;

    return true;
}

void wholebody_ik_thread::go_in_initial_position()
{
    q_init = input;
    going_to_initial_position = true;
}


bool wholebody_ik_thread::prepare_for_new_target()
{
    std::string arm_link;

    if(msg.hand=="right")
    {
        chain = &model.right_arm;
        arm = "right_arm";
        arm_link = "RSoftHand";
    }
    else if(msg.hand=="left")
    {
        chain = &model.left_arm;
        arm = "left_arm";
        arm_link = "LSoftHand";
    }
    else return false;

    math_utilities::FrameYARPToKDL(model.iDyn3_model.getPosition(model.iDyn3_model.getLinkIndex("gaze"),model.iDyn3_model.getLinkIndex(arm_link)),current_pose);
    math_utilities::FrameYARPToKDL(model.iDyn3_model.getPosition(model.iDyn3_model.getLinkIndex("Waist"),model.iDyn3_model.getLinkIndex("gaze")),t_T_h);

    yarp::sig::Vector q_arm(7,0.0);
    model.fromIDynToRobot(input, q_arm, *chain);

    if(!initialized[arm])
    {
        ik.initialize(arm,t_T_h*current_pose,q_arm);
// 	ik.set_new_object_head_transform(arm, t_T_h.Inverse()); // t_T_H = W_T_h

        initialized[arm]=true;
    }

//     ik.reset_init_joints(arm,q_arm);
    done=false;

    return true;
}

void wholebody_ik_thread::run()
{   
    sense();

    // get the command
    if(receive_from_pci.getCommand(msg,recv_num))
    {
        if(msg.command=="locomanipulate")
        {
            std::cout<<"Command received"<<std::endl;

            if(!prepare_for_new_target())
            {
                std::cout<<"Received malformed command, abort"<<std::endl;
                return;
            }
//             t_T_h = msg.t_T_h; NOTE: this is to use the Waist as "object" (to be compliant with walking current implementation, we will see how to change it in the future)
            traj_gen.line_initialize(duration,t_T_h*current_pose,msg.data);

            time=0;

        }

        if(msg.command=="test")
        {
            std::cout<<"Executing Test"<<std::endl;

            prepare_for_new_target();

            double sign;
            if(msg.hand=="right") sign=-1;
            else sign = 1;

            traj_gen.line_initialize(duration,t_T_h*current_pose,KDL::Frame(KDL::Rotation::RPY(0,-M_PI/2.0,0),KDL::Vector(0,0,0)) * KDL::Frame(KDL::Rotation::RPY(0,0,sign*M_PI/2.0),KDL::Vector(0,0,0)));
            time=0;

        }

        if(msg.command=="reset")
        {
            go_in_initial_position();
            time=0;
        }
    }

    control_law();

    move();
}    

void wholebody_ik_thread::sense()
{
    input = robot.sensePosition();
    model.updateiDyn3Model( input, true );
}

void wholebody_ik_thread::control_law()
{
    time = time + get_thread_period()/1000.0;

    if(initialized[arm] && !done)
    {
        if(time>duration)
        {
            send_to_pci.sendCommand("done");
            done=true;
        }

        KDL::Frame next_pose;
        KDL::Twist next_twist;
        traj_gen.line_trajectory(time,next_pose,next_twist);

        ik.set_desired_ee_pose(arm,next_pose);

        yarp::sig::Vector q_arm(7,0.0);
        yarp::sig::Vector q_out(7,0.0);
        model.fromIDynToRobot(input, q_arm, *chain);
	double cart_error = ik.cartToJnt(arm,q_arm,q_out,0.0001);

        std::cout<<"cart_error: "<<cart_error<<std::endl;

        model.fromRobotToIDyn(q_out, output, *chain);

        for (int i=0;i<output.size();i++)
        {
            if (output[i]>joint_max[i])
            {
                std::cout<<"desired output: "<<model.getJointNames().at(i)<<"("<<output[i]<<") is outside maximum bound: "<<joint_max[i]<<" current: "<<input[i]<<std::endl;
            }
            if (output[i]<joint_min[i])
            {
                std::cout<<"desired output: "<<model.getJointNames().at(i)<<"("<<output[i]<<") is outside minimum bound: "<<joint_min[i]<<" current: "<<input[i]<<std::endl;
            }
        }
    }

    if(going_to_initial_position)
    {
        double alpha = (time>1)?1:time;
        output = (1-alpha)*q_init + (alpha)*home;
        if(time>1) going_to_initial_position = false;
    }
}

void wholebody_ik_thread::move()
{
    robot.move(output);
}