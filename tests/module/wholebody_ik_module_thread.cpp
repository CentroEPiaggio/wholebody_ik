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

using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman;

walkman::chain_data::chain_data(std::string name)
{
    this->name = name;

    current_pose = KDL::Frame::Identity();
    desired_pose = KDL::Frame::Identity();
    initialized = false;
    done = false;
}

void walkman::chain_data::print()
{
    std::cout<<std::endl<<"Printing chain data: "<<name<<std::endl;
    std::cout<<" - name: "<<name<<std::endl;
    std::cout<<" - ee_link: "<<ee_link<<std::endl;
    std::cout<<" - initialized: "<<initialized<<std::endl;
    std::cout<<" - done: "<<done<<std::endl;
    std::cout<<" - current_pose: "<<current_pose<<std::endl;
    std::cout<<" - desired_pose: "<<desired_pose<<std::endl<<std::endl;
}

wholebody_ik_thread::wholebody_ik_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph):
control_thread( module_prefix, rf, ph ), recv_interface("wb_interface"), ik(get_robot_name(),get_urdf_path(),get_srdf_path(),get_thread_period())
{
    input.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    output.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    home.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    q_init.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    
    yarp::sig::Vector q_right_arm(7,0.0);
    yarp::sig::Vector q_left_arm(7,0.0);
    yarp::sig::Vector q_torso(3,0.0);
    yarp::sig::Vector q_right_leg(6,0.0);
    yarp::sig::Vector q_left_leg(7,0.0);
    yarp::sig::Vector q_head(2,0.0);

    q_head[0] = 0.0;
    q_head[1] = 0.0;

    q_right_arm[0]=  0.6;
    q_right_arm[1]= -0.2;
    q_right_arm[3]= -1.2;
    q_right_arm[5]= -0.6;

    q_left_arm[0]=  0.6;
    q_left_arm[1]=  0.2;
    q_left_arm[3]= -1.2;
    q_left_arm[5]= -0.6;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
 
    q_right_leg[2]= -0.3;
    q_right_leg[3]=  0.6;
    q_right_leg[4]= -0.3;
    
    q_left_leg[2]= -0.3;
    q_left_leg[3]=  0.6;
    q_left_leg[4]= -0.3;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,home);

    available_chains.push_back("left_arm");
    available_chains.push_back("right_arm");
    available_chains.push_back("left_leg");
    available_chains.push_back("right_leg");

    for(auto name:available_chains)
    {
        chains.emplace(name,chain_data(name));
    }

    chains.at("left_arm").kin_chain = &model.left_arm;
    chains.at("right_arm").kin_chain = &model.right_arm;
    chains.at("left_leg").kin_chain = &model.left_leg;
    chains.at("right_leg").kin_chain = &model.right_leg;

    chains.at("left_arm").ee_link = "LSoftHand";
    chains.at("right_arm").ee_link = "RSoftHand";
    chains.at("left_leg").ee_link = "l_sole";
    chains.at("right_leg").ee_link = "r_sole";

    available_commands.push_back("hands_up");
    available_commands.push_back("hands_down");
    available_commands.push_back("hands_forward");
    available_commands.push_back("hands_backward");
    available_commands.push_back("hands_wide");
    available_commands.push_back("hands_tight");
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

bool wholebody_ik_thread::generate_poses_from_cmd(std::string cmd)
{
    if( std::find(available_commands.begin(), available_commands.end(), cmd)==available_commands.end() )
    {
        std::cout<<" !! ERROR: command not available !!"<<std::endl;
        return false;
    }

    double offset_x=0;
    double offset_y=0;
    double offset_z=0;

    if(cmd=="hands_up") offset_z=0.1;
    else if(cmd=="hands_down") offset_z=-0.1;
    else if(cmd=="hands_forward") offset_x=0.1;
    else if(cmd=="hands_backward") offset_x=-0.1;
    else if(cmd=="hands_wide") offset_y=-0.1;
    else if(cmd=="hands_tight") offset_y=0.1;

    msg.desired_poses["left_arm"];
    msg.desired_poses["right_arm"];

    for(auto& pose:msg.desired_poses)
    {
        math_utilities::FrameYARPToKDL(model.iDyn3_model.getPosition(model.iDyn3_model.getLinkIndex("Waist"),model.iDyn3_model.getLinkIndex(chains.at(pose.first).ee_link)),chains.at(pose.first).current_pose);

        pose.second = chains.at(pose.first).current_pose;

        pose.second.p.x(pose.second.p.x() + offset_x);
        pose.second.p.y(pose.second.p.y() + ((pose.first=="right_arm")?1:-1) * offset_y);
        pose.second.p.z(pose.second.p.z() + offset_z);
    }

    return true;
}

bool wholebody_ik_thread::prepare_for_new_target()
{
    for(auto des_pose:msg.desired_poses)
    {
        if(!chains.count(des_pose.first))
        {
            std::cout<<" !! ERROR: chain not available !!"<<std::endl;
            return false;
        }

        
        math_utilities::FrameYARPToKDL(model.iDyn3_model.getPosition(model.iDyn3_model.getLinkIndex("Waist"),model.iDyn3_model.getLinkIndex(chains.at(des_pose.first).ee_link)),chains.at(des_pose.first).current_pose);

        chains.at(des_pose.first).desired_pose = des_pose.second;

        if(!chains.at(des_pose.first).initialized)
        {
            yarp::sig::Vector q_i(chains.at(des_pose.first).kin_chain->getNrOfDOFs(),0.0);
            model.fromIDynToRobot(input, q_i, *chains.at(des_pose.first).kin_chain);
            ik.initialize(chains.at(des_pose.first).name,chains.at(des_pose.first).current_pose,q_i);
            chains.at(des_pose.first).initialized=true;

//             chains.at(des_pose.first).print();
        }

        chains.at(des_pose.first).done=false;
    }
    
    return true;
}

void wholebody_ik_thread::run()
{   
    sense();

    // get the command
    if(recv_interface.getCommand(msg,recv_num))
    {
        std::cout<<"Command received: "<<msg.command<<std::endl;

        if(msg.command=="reset")
        {
            go_in_initial_position();
            time=0;
        }
        else
        {
            if(msg.command!="poses") generate_poses_from_cmd(msg.command);

            if(!prepare_for_new_target())
            {
                std::cout<<"Received malformed command, abort"<<std::endl;
                return;
            }

            for(auto& chain:chains) chain.second.traj_gen.line_initialize(duration,chain.second.current_pose,chain.second.desired_pose);

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

    for(auto& chain:chains)
    {
        if(chain.second.initialized && !chain.second.done)
        {
            if(time>duration)
            {
                std::cout<<" -- done"<<std::endl;
                chain.second.done=true;
            }

            KDL::Frame next_pose;
            KDL::Twist next_twist;
            chain.second.traj_gen.line_trajectory(time,next_pose,next_twist);

            ik.set_desired_ee_pose(chain.second.name,next_pose);

            yarp::sig::Vector q_arm(chain.second.kin_chain->getNrOfDOFs(), 0.0);
            yarp::sig::Vector q_out(chain.second.kin_chain->getNrOfDOFs(), 0.0);
            model.fromIDynToRobot(input, q_arm, *chain.second.kin_chain);
            double cart_error = ik.cartToJnt(chain.second.name,q_arm,q_out,0.0001);

            if(cart_error==-1)
            {
                std::cout<<" !! ERROR in IK !! ( "<<chain.first<<" ) -> I won't move."<<std::endl;
                chain.second.done=true;
                return;
            }

            model.fromRobotToIDyn(q_out, output, *chain.second.kin_chain);
        }
    }

    if(going_to_initial_position)
    {
        double alpha = (time>3)?1:time/3.0;
        output = (1-alpha)*q_init + (alpha)*home;
        if(time>3)
        {
            going_to_initial_position = false;
            std::cout<<" - Ready"<<std::endl;
        }
    }
}

void wholebody_ik_thread::move()
{
    robot.move(output);
}