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
#include "wholebody_ik_wb_module_thread.h"
#include <kdl/frames_io.hpp>
#include "math_utilities.cpp"

using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman;

wholebody_ik_wb_thread::wholebody_ik_wb_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph):
control_thread( module_prefix, rf, ph ), recv_interface("wb_interface"), IK(get_robot_name(),get_urdf_path(),get_srdf_path(),get_thread_period())
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

	chains.push_back("wb_left");
	chains.push_back("wb_right");

	base_frames["wb_left"] = "l_sole";
	base_frames["wb_right"] = "r_sole";

	for(auto frame:base_frames)
	{
		base_indeces[frame.first] = model.iDyn3_model.getLinkIndex(frame.second);
		initialized[frame.first] = false;
	}

	current_chain = "wb_left";

	ee_names.push_back("LSoftHand");
	ee_names.push_back("RSoftHand");
	ee_names.push_back("l_sole");
	ee_names.push_back("r_sole");

	for(auto name:ee_names)
	{
		ee_indeces.emplace(name,model.iDyn3_model.getLinkIndex(name));
		traj_gens[name];
	}
	traj_gens["COM"];

	available_commands.push_back("poses");
	available_commands.push_back("hands_up");
	available_commands.push_back("hands_down");
	available_commands.push_back("hands_forward");
	available_commands.push_back("hands_backward");
	available_commands.push_back("hands_wide");
	available_commands.push_back("hands_tight");
	available_commands.push_back("com_on_left");
	available_commands.push_back("com_on_right");
	available_commands.push_back("com_up");
	available_commands.push_back("com_down");
}

bool wholebody_ik_wb_thread::custom_init()
{
    //  real time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam ( pthread_self(), SCHED_FIFO, &thread_param );
    
    // setup
	model.iDyn3_model.setFloatingBaseLink(model.iDyn3_model.getLinkIndex("Waist"));
    sense();
	output = input = robot.sensePosition();
    robot.setPositionDirectMode();

    // initial position
    go_in_initial_position();

    std::cout<<" - Initialized"<<std::endl;

    return true;
}

void wholebody_ik_wb_thread::go_in_initial_position()
{
    q_init = input;
    going_to_initial_position = true;
}

bool wholebody_ik_wb_thread::generate_poses_from_cmd(std::string cmd)
{
    if( std::find(available_commands.begin(), available_commands.end(), cmd)==available_commands.end() )
    {
        std::cout<<" !! ERROR: command not available !!"<<std::endl;
        return false;
    }

	if(cmd=="com_on_left")
	{
		current_chain = "wb_left";
	}
	if(cmd=="com_on_right")
	{
		current_chain = "wb_right";
	}
	
	if(!initialized.at(current_chain))
	{
		IK.initialize(current_chain,input);
		initialized.at(current_chain)=true;
	}

	IK.get_current_wb_poses(current_chain,initial_poses);

	if(cmd!="poses")
	{
		double offset_x=0;
		double offset_y=0;
		double offset_z=0;

		if(cmd=="hands_up") offset_z=0.1;
		else if(cmd=="hands_down") offset_z=-0.1;
		else if(cmd=="hands_forward") offset_x=0.1;
		else if(cmd=="hands_backward") offset_x=-0.1;
		else if(cmd=="hands_wide") offset_y=-0.1;
		else if(cmd=="hands_tight") offset_y=0.1;

		msg.desired_poses["LSoftHand"] = initial_poses.at("LSoftHand") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(offset_z,-offset_y,-offset_x)); // :3
		msg.desired_poses["RSoftHand"] = initial_poses.at("RSoftHand") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(offset_z,offset_y,-offset_x));
		msg.desired_poses["l_sole"] = initial_poses.at("l_sole");
		msg.desired_poses["r_sole"] = initial_poses.at("r_sole");

		double com_offset_x=0;
		double com_offset_y=0;
		double com_offset_z=0;

		if(cmd=="com_up") com_offset_z=0.1;
		else if(cmd=="com_down") com_offset_z=-0.1;
		
		msg.desired_poses["COM"] = initial_poses.at("COM") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(com_offset_x,com_offset_y,com_offset_z));

		if(cmd=="com_on_left")
		{
			msg.desired_poses.at("COM").p.x(0.0);
			msg.desired_poses.at("COM").p.y(-0.04);
			msg.desired_poses.at("LSoftHand").p.y(0.35);
			msg.desired_poses.at("RSoftHand").p.y(-0.35);
		}
		else if(cmd=="com_on_right")
		{
			msg.desired_poses.at("COM").p.x(0.0);
			msg.desired_poses.at("COM").p.y(0.04);
			msg.desired_poses.at("LSoftHand").p.y(0.35);
			msg.desired_poses.at("RSoftHand").p.y(-0.35);
		}
	}

	IK.set_desired_wb_poses_as_current(current_chain);

	for(auto pose:msg.desired_poses) traj_gens.at(pose.first).line_initialize(duration,initial_poses.at(pose.first),pose.second);
	if(msg.desired_poses.count("COM")) traj_gens.at("COM").line_initialize(duration,initial_poses.at("COM"),msg.desired_poses.at("COM"));
	
	done=false;

    return true;
}

void wholebody_ik_wb_thread::run()
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
			if(!generate_poses_from_cmd(msg.command))
			{
				std::cout<<"Received malformed command, abort"<<std::endl;
				return;
			}

            time=0;
        }
    }

    control_law();

    move();
}    

void wholebody_ik_wb_thread::sense()
{
    input = output; //robot.sensePosition();
    model.updateiDyn3Model( input, true );
}

void wholebody_ik_wb_thread::control_law()
{
    time = time + get_thread_period()/1000.0;

	if(initialized.at(current_chain) && !done)
	{
		if(time>duration)
		{
			std::cout<<" -- done"<<std::endl;
			done=true;
		}

		KDL::Twist next_twist;
		
		for(auto traj_gen:traj_gens)
		{
			if(msg.desired_poses.count(traj_gen.first))
				traj_gen.second.line_trajectory(time,next_poses[traj_gen.first],next_twist);
		}

		IK.set_desired_wb_poses(current_chain,next_poses);

		yarp::sig::Vector out(output.size(),0.0);
		double cart_error = IK.cartToJnt(current_chain,input,out,0.005);

		if(cart_error==-1)
		{
			std::cout<<" !! ERROR in IK !! ( "<<current_chain<<" ) -> I won't move."<<std::endl;
			done=true;
			return;
		}

		output = out;
    }

    if(going_to_initial_position)
    {
        double alpha = (time>5.0)?1:time/5.0;
        output = (1-alpha)*q_init + (alpha)*home;
        if(time>5.0)
        {
            going_to_initial_position = false;
            std::cout<<" - Ready"<<std::endl;
        }
    }
}

void wholebody_ik_wb_thread::move()
{
    robot.move(output);
}