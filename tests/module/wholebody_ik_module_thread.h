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

#ifndef WHOLEBODY_IK_THREAD_H_
#define WHOLEBODY_IK_THREAD_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/comanutils.h>

#include <GYM/control_thread.hpp>
#include <GYM/yarp_command_interface.hpp>
#include <drc_shared/yarp_msgs/locomanipulation_msg.h>

#include <wholebody_ik/wholebody_ik.h>
#include <trajectory_generator/trajectory_generator.h>

/**
 * @brief wholebody_ik control thread
 * 
 **/
namespace walkman
{
    class wholebody_ik_thread : public control_thread
    {
    private:
        void control_law();

        yarp::sig::Vector input;
        yarp::sig::Vector output;
        yarp::sig::Vector home;
        yarp::sig::Vector q_init;
        
        locomanipulation_msg msg;
        walkman::yarp_custom_command_interface<locomanipulation_msg> receive_from_pci;
        int recv_num=0;
        walkman::yarp_command_sender_interface send_to_pci;
        int send_num=0;

        void go_in_initial_position();
        bool going_to_initial_position=false;

        bool prepare_for_new_target();
        KDL::Frame current_pose;

        wholebody_ik ik;
        std::map<std::string,bool> initialized;
        bool done=false;
        KDL::Frame t_T_h;
        std::string arm = "right_arm";
        kinematic_chain* chain;
        trajectory_generator traj_gen;
        double time = 0;
        double duration = 3.0;

        yarp::sig::Vector joint_min;
        yarp::sig::Vector joint_max;
    public:
        /**
        * @brief constructor
        * 
        * @param module_prefix the prefix of the module
        * @param rf resource finderce
        * @param ph param helper
        */
        wholebody_ik_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );

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

#endif // WHOLEBODY_IK_THREAD_H_
