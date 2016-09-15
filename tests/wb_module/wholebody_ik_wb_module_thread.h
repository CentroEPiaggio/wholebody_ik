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

#ifndef WHOLEBODY_IK_WB_THREAD_H_
#define WHOLEBODY_IK_WB_THREAD_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/comanutils.h>

#include <GYM/control_thread.hpp>
#include <GYM/yarp_command_interface.hpp>
#include <drc_shared/yarp_msgs/multicontact_msg.h>

#include <wholebody_ik/wholebody_ik.h>
#include <trajectory_generator/trajectory_generator.h>

#include <vector>

#include "tf/transform_broadcaster.h"

/**
 * @brief wholebody_ik wb version, control thread
 * 
 **/
namespace walkman
{
    class wholebody_ik_wb_thread : public control_thread
    {
    private:
        void control_law();

        yarp::sig::Vector input;
        yarp::sig::Vector output;
        yarp::sig::Vector home;
        yarp::sig::Vector q_init;

		multicontact_msg msg;
		walkman::yarp_custom_command_interface<multicontact_msg> recv_interface;
        int recv_num=0;

        void go_in_initial_position();
        bool going_to_initial_position=false;

        std::vector<std::string> available_commands;
        bool generate_poses_from_cmd();

        wholebody_ik IK;

        double time = 0;
		double exec_time;

		std::vector<std::string> chains;
		std::map<std::string,std::string> base_frames;
		std::map<std::string,int> base_indeces;
		std::string current_chain;

		std::vector<std::string> ee_names;
		std::map<std::string,int> ee_indeces;
		std::map<std::string,KDL::Frame> initial_poses;
		std::map<std::string,KDL::Frame> next_poses;
		std::map<std::string,trajectory_generator> traj_gens;
		std::map<std::string,int> traj_types;
		std::map<std::string,bool> initialized;
		bool done = false;

		void reset_traj_types();

		tf::TransformBroadcaster br;
		void broadcast_com_tf();
    public:
        /**
        * @brief constructor
        * 
        * @param module_prefix the prefix of the module
        * @param rf resource finderce
        * @param ph param helper
        */
        wholebody_ik_wb_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );

        /**
        * @brief wholebody_ik control thread initialization
        * 
        * @return true on succes, false otherwise
        */
        virtual bool custom_init();

        /**
        * @brief wholebody_ik control thread main loop
        * 
        */
        virtual void run();

        /**
            * @brief sense function
            */
        void sense();

        /**
            * @brief move function
            */
        void move();
    };
}

#endif // WHOLEBODY_IK_WB_THREAD_H_
