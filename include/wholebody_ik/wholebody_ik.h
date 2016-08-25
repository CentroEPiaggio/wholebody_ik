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

#ifndef WHOLEBODY_IK_H
#define WHOLEBODY_IK_H

#include <string>
#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>
#include <kdl/frames.hpp>
#include <idynutils/idynutils.h>
#include <idynutils/RobotUtils.h>

#define DOFS 7

class chain_data
{
public:
    /**
     * chain_data constructor
     * @param robot_name
     * @param urdf_path
     * @param srdf_path
     * @param ee_link link name of the end-effector
     * @param base_link link name of the chain base, w.r.t. desired positions are defined.
     * @param dofs number of degree of freedom of the chain.
     * 
     */
    chain_data(std::string robot_name,std::string urdf_path, std::string srdf_path, std::string ee_link, std::string base_link, int dofs, std::string chain_name);

    iDynUtils idynutils;
    kinematic_chain* kin_chain;

    bool initialized;

    KDL::Frame ee_current;
    KDL::Frame ee_desired;        
    double car_err;
//     Eigen::Matrix<double,6,7> jacobian;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd Weights;

    bool first_step;

    inline std::string get_ee_link(){return ee_link;}
    inline std::string get_base_link(){return base_link;}
    inline int get_dofs(){return dofs;}
    inline std::string get_name(){return chain_name;}
private:
    /**
     * \brief link name of the end-effector.
     */
    std::string ee_link;
    
    /**
     * \brief link name of the chain base, w.r.t. desired positions are defined.
     */
    std::string base_link;
    
    /**
     * \brief number of degree of freedom of the chain.
     */
    int dofs;
    
    /**
     * \brief chain name
     */
    std::string chain_name;
};

class wholebody_ik
{

public:
    /**
     * wholebody_ik constructor
     * \brief FIXME
     * @param robot_name
     * @param urdf_path
     * @param srdf_path
     * @param period_ms
     * 
     */
    wholebody_ik(std::string robot_name,std::string urdf_path, std::string srdf_path,int period_ms);
    ~wholebody_ik();

    /**
    * initialize
    * \brief Initialize a kinematic chain with a desired cartesian position.
    * @param chain the string representing the kinematic chain
    * @param cartesian_pose the target pose for the end-effector of the kinematic chain
    * @param q_input FIXME
    * 
    */
    bool initialize(std::string chain, KDL::Frame cartesian_pose, const yarp::sig::Vector& q_input);
    
    /**
     * set_desired_ee_pose
     * \brief sets the cartesian pose of the end-effector for a given frame
     * @param chain the string representing the kinematic chain
     * @param cartesian_pose the target pose for the end-effector of the kinematic chain
     */
    void set_desired_ee_pose(std::string chain, KDL::Frame cartesian_pose);
    
    /**
     * next_step
     * \brief computes dq (joint speed update)
     * @param chain
     * @param q_input
     * @return dq (rad/s)
     * 
     */
    yarp::sig::Vector next_step(std::string chain, const yarp::sig::Vector& q_input,double precision);
    
    /**
     * cartToJnt
     * \brief main Inverse Kinematic Loop: computes the target q
     * @param chain name of the kinematic chain considered (e.g. "left_arm", "right_arm", "left_leg", "right_leg").
     * @param q_input
     * @param q_out
     * @param dt_external period of the external thread calling this function (used to compute cartesian speed and joint update law)
     * @param precision tolerance to check if the algorithm has reached the goal configuration.
     * @return -1 if errors occur (initialization or others). Carterian error if success.
     */
    double cartToJnt(std::string chain, const yarp::sig::Vector& q_input, yarp::sig::Vector& q_out,double precision=0.04);
    
    /**
     * cartesian_action_completed
     *\brief 
     * @return true if the cartesian error is below the desired precision level.
     */
    bool cartesian_action_completed(std::string chain, double precision);
    double K;
private:

    std::map<std::string,chain_data*> chains;

    void warn_not_initialized(std::string str);
    void print_eigen_matrix(const Eigen::MatrixXd& data);
    void print_YARP_matrix(const yarp::sig::Matrix& data);

    /**
     * T
     * \brief Period of the thread that is using this class (seconds) used for IK loop.
     */
    double d_t=1;

    /**
     * \brief a utility just for switching between single chains and wholebody joints vectors
     */
    RobotUtils robot;
};

#endif //WHOLEBODY_IK_H