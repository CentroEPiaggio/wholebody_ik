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

#define DOFS 7

class chain_data
{
public:
    chain_data(std::string robot_name,std::string urdf_path, std::string srdf_path);

    KDL::Frame t_p_e_current;
    KDL::Frame t_p_e_desired;

    KDL::Frame s_current;
    KDL::Frame s_previous;
    double delta_T;

    bool initialized;
    iDynUtils idynutils;
    std::string e_link;
    std::string s_link;
    double car_err;

    Eigen::Matrix<double,6,7> jacobian;

    Eigen::MatrixXd Weights;

    bool first_step;

    yarp::sig::Matrix h_T_t;
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
     * reset_init_joint
     * @param q_input FIXME TOO
     */
    void reset_init_joints(std::string arm, const yarp::sig::Vector& q_input);

    /**
    * initialize
    * \brief Initialize a kinematic chain with a desired cartesian position.
    * @param arm the string representing the kinematic chain
    * @param cartesian_pose the target pose for the end-effector of the kinematic chain
    * @param q_input FIXME
    * 
    */
    bool initialize(std::string arm, KDL::Frame cartesian_pose, const yarp::sig::Vector& q_input);
    
    /**
     * set_desired_ee_pose
     * \brief sets the cartesian pose of the end-effector for a given frame
     * @param arm the string representing the kinematic chain
     * @param cartesian_pose the target pose for the end-effector of the kinematic chain
     */
    void set_desired_ee_pose(std::string arm, KDL::Frame cartesian_pose);
    
    /**
     * next_step
     * \brief computes dq (joint speed update)
     * @param arm
     * @param q_input
     * @return dq (rad/s)
     * 
     */
    yarp::sig::Vector next_step(std::string arm, const yarp::sig::Vector& q_input,double precision);
    
    /**
     * cartToJnt
     * \brief main Inverse Kinematic Loop: computes the target q
     * @param arm name of the kinematic chain considered (e.g. "left_arm", "right_arm").
     * @param q_input
     * @param q_out
     * @param dt_external period of the external thread calling this function (used to compute cartesian speed and joint update law)
     * @param precision tolerance to check if the algorithm has reached the goal configuration.
     * @return -1 if errors occur (initialization or others). Carterian error if success.
     */
    double cartToJnt(std::string arm, const yarp::sig::Vector& q_input, yarp::sig::Vector& q_out,double precision=0.04);
    
    /**
     * cartesian_action_completed
     *\brief 
     * @return true if the cartesian error is below the desired precision level.
     */
    bool cartesian_action_completed(std::string arm, double precision);
    double K;

    void set_new_object_head_transform(std::string arm, yarp::sig::Matrix h_T_t, double delta_time=0);
    void set_new_object_head_transform(std::string arm, KDL::Frame h_T_t, double delta_time=0);
private:

    std::map<std::string,chain_data*> wholebody_data;

    void fromRobotToIDyn(std::string arm, const yarp::sig::Vector& q_arm, yarp::sig::Vector& q_all);
    void warn_not_initialized(std::string str);
    void print_eigen_matrix(const Eigen::MatrixXd& data);
    void print_YARP_matrix(const yarp::sig::Matrix& data);
    void set_new_shoulder_position(std::string arm, KDL::Frame shoulder_pos, double delta_time);

    /**
     * T
     * \brief Period of the theard that is using this class (seconds) used for IK loop.
     */
    double d_t=1;

};

#endif //WHOLEBODY_IK_H