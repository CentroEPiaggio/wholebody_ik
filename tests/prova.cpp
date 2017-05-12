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
#include <trajectory_generator/trajectory_generator.h>
#include <tf/transform_broadcaster.h>

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
    std::map<std::string,KDL::Frame> current_poses;
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

    tf::TransformBroadcaster broadcaster;
    ros::AsyncSpinner as(2);
    ros::NodeHandle nh;

    sensor_msgs::JointState joints_msg;
    ros::Publisher joints_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);

    joints_msg.name.push_back("LHipLat");
    joints_msg.name.push_back("LHipYaw");
    joints_msg.name.push_back("LHipSag");
    joints_msg.name.push_back("LKneeSag");
    joints_msg.name.push_back("LAnkSag");
    joints_msg.name.push_back("LAnkLat");
    
    joints_msg.name.push_back("RHipLat");
    joints_msg.name.push_back("RHipYaw");
    joints_msg.name.push_back("RHipSag");
    joints_msg.name.push_back("RKneeSag");
    joints_msg.name.push_back("RAnkSag");
    joints_msg.name.push_back("RAnkLat");
    
    joints_msg.name.push_back("WaistLat");
    joints_msg.name.push_back("WaistSag");
    joints_msg.name.push_back("WaistYaw");
    
    joints_msg.name.push_back("LShSag");
    joints_msg.name.push_back("LShLat");
    joints_msg.name.push_back("LShYaw");
    joints_msg.name.push_back("LElbj");
    joints_msg.name.push_back("LForearmPlate");
    joints_msg.name.push_back("LWrj1");
    joints_msg.name.push_back("LWrj2");
    
    joints_msg.name.push_back("NeckPitchj");
    joints_msg.name.push_back("NeckYawj");
    
    joints_msg.name.push_back("RShSag");
    joints_msg.name.push_back("RShLat");
    joints_msg.name.push_back("RShYaw");
    joints_msg.name.push_back("RElbj");
    joints_msg.name.push_back("RForearmPlate");
    joints_msg.name.push_back("RWrj1");
    joints_msg.name.push_back("RWrj2");

    for(auto item:joints_msg.name) joints_msg.position.push_back(0.0);
    
    std::string chain;
    std::string f_frame;
    std::string other_leg;
    double y_sign;

//     if(right)
//     {
//         chain = "wb_right";
//         f_frame = "r_sole";
//         y_sign = 1;
//     }
//     else
//     {
        chain = "wb_left";
        f_frame = "l_sole";
        y_sign = -1;
//     }

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

    iDynUtils idynutils(robot,urdf,srdf);
    
    yarp::sig::Vector joint_max = idynutils.iDyn3_model.getJointBoundMax();
    yarp::sig::Vector joint_min = idynutils.iDyn3_model.getJointBoundMin();
    
    std::cout<<" ------- LIMITS ------- "<<std::endl;
    for(auto jo:joints_msg.name)
    {
        std::cout<<joint_min[idynutils.iDyn3_model.getDOFIndex(jo)]<<" < "<<jo<<" < "<<joint_max[idynutils.iDyn3_model.getDOFIndex(jo)]<<std::endl;
    }
    
    yarp::sig::Vector q_left_arm(7,0.0);
    q_left_arm[0]  = 1.98;
    q_left_arm[1]  = 2.40;
    q_left_arm[2]  = 1.7;
    q_left_arm[3]  = -0.42;
    q_left_arm[4]  = 0.89;
    q_left_arm[5]  = -0.63;
    q_left_arm[6]  = 0.71;
    yarp::sig::Vector q_right_arm(7,0.0);
    q_right_arm[0] = 1.98;
    q_right_arm[1] = -2.40;
    q_right_arm[2] = -1.70;
    q_right_arm[3] = -0.42;
    q_right_arm[4] = -0.89;
    q_right_arm[5] = -0.63;
    q_right_arm[6] = -0.71;
    yarp::sig::Vector q_left_leg(6,0.0);
    q_left_leg[2]  = -0.2;
    q_left_leg[4]  = -1.35;
    yarp::sig::Vector q_right_leg(6,0.0);
    q_right_leg[2] = -0.2;
    q_right_leg[4] = -1.35;
    yarp::sig::Vector q_torso(3,0.0);
    q_torso[1] = -0.31;
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

    desired_poses["RSoftHand"] = initial_poses.at("RSoftHand");
    desired_poses["LSoftHand"] = initial_poses.at("LSoftHand");
    desired_poses["r_sole"] = initial_poses.at("r_sole");
    desired_poses["l_sole"] = initial_poses.at("l_sole");
    desired_poses["COM"] = initial_poses.at("COM");

    ros::Time start = ros::Time::now();
    ros::Duration exp;
    KDL::Twist next_twist;
    double secs;
    double old_t = secs = 0;
    double cart_error = 9999;
    int k=0;
    int steps = 0;
        
    while(ros::ok())
    {
        old_t = secs;
        secs = exp.toSec() + ((double)exp.toNSec()) / 1000000000.0;     
	
	
	switch(steps)
	{
	  case 1: {
		    desired_poses["RSoftHand"] = desired_poses.at("RSoftHand")* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0,0.1));
		    desired_poses["LSoftHand"] = desired_poses.at("LSoftHand")* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0,0.1));
		    desired_poses["r_sole"] = desired_poses.at("r_sole");// *KDL::Frame(KDL::Rotation::RPY(0,2.7,0), KDL::Vector(0,0,0.5));
		    desired_poses["l_sole"] = desired_poses.at("l_sole");//* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0,0.1));
		    desired_poses["COM"] = desired_poses.at("COM")* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(-0.05,0,0.05));
		    k=0;
		    break;
		    }
		    
	  case 2: {
		    desired_poses["RSoftHand"] = desired_poses.at("RSoftHand");//* KDL::Frame(KDL::Rotation::RPY(0,0,-0.6), KDL::Vector(0,-0.8,0));
		    desired_poses["LSoftHand"] = desired_poses.at("LSoftHand")* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0.3,0));
		    desired_poses["r_sole"] = desired_poses.at("r_sole") ;//*KDL::Frame(KDL::Rotation::RPY(0,2.7,0), KDL::Vector(0,0,0.5));
		    desired_poses["l_sole"] = desired_poses.at("l_sole");//* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0,0.1));
		    desired_poses["COM"] = desired_poses.at("COM");//* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0,0.05));
		    k=0;
		    break;
		    }
		    
	  case 3: {
		      desired_poses["RSoftHand"] = desired_poses.at("RSoftHand")* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,-0.3,0));
		      desired_poses["LSoftHand"] = desired_poses.at("LSoftHand");//* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0.7,0));
		      desired_poses["r_sole"] = desired_poses.at("r_sole"); // *KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0,0.4));
		      desired_poses["l_sole"] = desired_poses.at("l_sole");//* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(0,0,0.1));
		      desired_poses["COM"] = desired_poses.at("COM");//* KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(-0.7,0,0));
		      k=0;
		      break;
		      }
	}
		    
	IK.get_current_wb_poses(chain,current_poses);
	
	for(auto pose:current_poses)
	{
	    tf::Transform T;
	    tf::transformKDLToTF(pose.second,T);
	    tf::StampedTransform ST;
	    ST.frame_id_ = "l_sole";
	    ST.child_frame_id_ = pose.first + "_CUR";
	    ST.setData(T);
	    broadcaster.sendTransform(ST);
	}
	for(int i=0;i<joints.at(chain).size();i++)
	{
		joints_msg.position.at(i) = joints.at(chain).at(i);
	}
	joints_msg.header.frame_id="l_sole";
	joints_msg.header.stamp = ros::Time::now();
	joints_pub.publish(joints_msg);

	if(k == 0)
	{
		IK.set_desired_wb_poses(chain,desired_poses);
		double ret=IK.cartToJnt(chain,q_sense.at(chain),q_out.at(chain));
		std::cout<<" -- error: "<<ret<<std::endl;
		if(ret==-1)
		{
		  std::cout<<"aiuto"<<std::endl;
		  steps++;
		  k++;
		  continue;
		}

		for(int i=0;i<q_out.at(chain).size();i++)
		{
			joints.at(chain).at(i) = q_out.at(chain)[i];
		}
		
		q_sense.at(chain) = q_out.at(chain);
		
		
		k++;
		

	}
	
	steps++;
	
	usleep(500000);
	

        exp = ros::Time::now()-start;
    }

    return 0;
}


