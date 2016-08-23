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
#include "math_utilities.cpp"
#include <iostream>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>

#include <locoman/utils/screws.hpp>
#include <locoman/utils/kinematics.hpp>
#include <locoman/utils/kinetostatics.hpp>
#include <locoman/utils/locoman_utils.hpp>
#include <locoman/utils/algebra.hpp>

#include "utils.cpp" // NOTE: to remove

using namespace yarp::math;

chain_data::chain_data(std::string robot_name, std::string urdf_path, std::string srdf_path): idynutils(robot_name,urdf_path,srdf_path)
{

}

wholebody_ik::wholebody_ik(std::string robot_name,std::string urdf_path, std::string srdf_path, int period_ms): d_t(period_ms/1000.0)
{
    wholebody_data["right_arm"] = new chain_data(robot_name,urdf_path,srdf_path);
    wholebody_data["left_arm"] = new chain_data(robot_name,urdf_path,srdf_path);

    wholebody_data.at("right_arm")->e_link = "RSoftHand";
    wholebody_data.at("left_arm")->e_link = "LSoftHand";

    wholebody_data.at("right_arm")->s_link = wholebody_data.at("left_arm")->s_link = "DWYTorso";

    wholebody_data.at("right_arm")->initialized = false;
    wholebody_data.at("left_arm")->initialized = false;
}

void wholebody_ik::print_eigen_matrix(const Eigen::MatrixXd& data)
{
    for(int i=0;i<data.rows();i++)
    {
        for(int j=0;j<data.cols();j++)
        {
            std::cout<<data(i,j)<<' ';
        }
        std::cout<<std::endl;
    }

}

void wholebody_ik::print_YARP_matrix(const yarp::sig::Matrix& data)
{
    for(int i=0;i<data.rows();i++)
    {
        for(int j=0;j<data.cols();j++)
        {
            std::cout<<data[i][j]<<' ';
        }
        std::cout<<std::endl;
    }
}

void wholebody_ik::warn_not_initialized(std::string str)
{
    std::cout<<" --------------- WARNING ARMS IK NOT PROPERLY INITIALIZED FOR '"<< str <<"'--------------- "<<std::endl;
}

void wholebody_ik::fromRobotToIDyn(std::string arm, const yarp::sig::Vector& q_arm, yarp::sig::Vector& q_all)
{
    chain_data *arm_data = wholebody_data.at(arm);

    yarp::sig::Vector q_other_arm(DOFS,0.0);
    yarp::sig::Vector q_left_leg(arm_data->idynutils.left_leg.getNrOfDOFs(),0.0);
    yarp::sig::Vector q_right_leg(arm_data->idynutils.right_leg.getNrOfDOFs(),0.0);
    yarp::sig::Vector q_torso(arm_data->idynutils.torso.getNrOfDOFs(),0.0);
    yarp::sig::Vector q_head(arm_data->idynutils.head.getNrOfDOFs(),0.0);

    if(arm=="left_arm")
    {
        arm_data->idynutils.fromRobotToIDyn(q_other_arm, q_all, arm_data->idynutils.right_arm);
        arm_data->idynutils.fromRobotToIDyn(q_arm, q_all, arm_data->idynutils.left_arm);
    }
    if(arm=="right_arm")
    {
        arm_data->idynutils.fromRobotToIDyn(q_other_arm, q_all, arm_data->idynutils.left_arm);
        arm_data->idynutils.fromRobotToIDyn(q_arm, q_all, arm_data->idynutils.right_arm);
    }
    arm_data->idynutils.fromRobotToIDyn(q_torso, q_all, arm_data->idynutils.torso);
    arm_data->idynutils.fromRobotToIDyn(q_right_leg, q_all, arm_data->idynutils.right_leg);
    arm_data->idynutils.fromRobotToIDyn(q_left_leg, q_all, arm_data->idynutils.left_leg);
    arm_data->idynutils.fromRobotToIDyn(q_head, q_all, arm_data->idynutils.head);
}

void wholebody_ik::reset_init_joints(std::string arm, const yarp::sig::Vector& q_input)
{
    yarp::sig::Vector q_all(wholebody_data.at(arm)->idynutils.getJointNames().size(),0.0);
    fromRobotToIDyn(arm, q_input,q_all);
    wholebody_data.at(arm)->idynutils.updateiDyn3Model(q_all, false);
}

bool wholebody_ik::initialize(std::string arm, KDL::Frame cartesian_pose, const yarp::sig::Vector& q_input)
{
    if(arm!="right_arm" && arm!="left_arm")
    {
        warn_not_initialized(arm);
        return false;
    }

    chain_data *arm_data = wholebody_data.at(arm);

    if(!arm_data->idynutils.iDyn3_model.setFloatingBaseLink(arm_data->idynutils.iDyn3_model.getLinkIndex(arm_data->s_link)))
    {
        std::cout<<"!! - Error in setting floating base - !!"<<std::endl;
        return false;
    }

    reset_init_joints(arm,q_input);

    Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
    int dim=6;
    arm_data->Weights.resize(dim,dim);
    arm_data->Weights.setZero();
    arm_data->Weights.block<3,3>(0,0) = identity*1;
    arm_data->Weights.block<3,3>(3,3) = identity*1;

    arm_data->car_err = 9999.0;
    arm_data->first_step = true;

    arm_data->t_p_e_current = KDL::Frame::Identity();
    arm_data->t_p_e_desired = cartesian_pose;

    arm_data->jacobian.setZero();
    
    arm_data->initialized = true;

    arm_data->s_current = KDL::Frame::Identity();
    arm_data->s_previous = KDL::Frame::Identity(); 
    arm_data->delta_T = 1;
    yarp::sig::Matrix eye_matrix(4,4);
    eye_matrix.eye();
    arm_data->h_T_t = eye_matrix;

    std::cout<<"=---------------------------"<<std::endl;
    std::cout<<" Arms IK Library initialized. Created a "<<dim<<"x" <<DOFS<<" Jacobian ("<<arm<<")"<<std::endl;
    std::cout<<"=---------------------------"<<std::endl;
    
    return true;
}



void wholebody_ik::set_desired_ee_pose(std::string arm, KDL::Frame cartesian_pose)
{
    if(!wholebody_data.at(arm)->initialized) {warn_not_initialized(arm); return;}

    wholebody_data.at(arm)->t_p_e_desired=cartesian_pose;
}

void wholebody_ik::set_new_shoulder_position(std::string arm, KDL::Frame shoulder_pos, double delta_time)
{
    wholebody_data.at(arm)->s_previous = wholebody_data.at(arm)->s_current;
    wholebody_data.at(arm)->s_current = shoulder_pos;
    wholebody_data.at(arm)->delta_T = delta_time;
}

void wholebody_ik::set_new_object_head_transform(std::string arm, yarp::sig::Matrix h_T_t, double delta_time)
{
    wholebody_data.at(arm)->h_T_t = h_T_t;

    if(delta_time!=0)
    {
        int s_index = wholebody_data.at(arm)->idynutils.iDyn3_model.getLinkIndex(wholebody_data.at(arm)->s_link);
        int h_index = wholebody_data.at(arm)->idynutils.iDyn3_model.getLinkIndex("gaze");
        yarp::sig::Matrix h_T_s = wholebody_data.at(arm)->idynutils.iDyn3_model.getPosition(h_index,s_index);
        yarp::sig::Matrix t_T_s = locoman::utils::iHomogeneous(h_T_t)*h_T_s;
        KDL::Frame t_T_s_KDL;
        math_utilities::FrameYARPToKDL(t_T_s,t_T_s_KDL);
        set_new_shoulder_position(arm, t_T_s_KDL, delta_time);
    }
}

void wholebody_ik::set_new_object_head_transform(std::string arm, KDL::Frame h_T_t, double delta_time)
{
    yarp::sig::Matrix mat(4,4);
    math_utilities::FrameKDLToYARP(h_T_t,mat);
    set_new_object_head_transform(arm, mat, delta_time);    
}

bool wholebody_ik::cartesian_action_completed(std::string arm, double precision)
{
    if(!wholebody_data.at(arm)->initialized) {warn_not_initialized(arm); return false;}

    return (wholebody_data.at(arm)->car_err < precision);
}


double wholebody_ik::cartToJnt(std::string arm, const yarp::sig::Vector& q_input, yarp::sig::Vector& q_out,double precision)
{
    if(!wholebody_data.at(arm)->initialized) {warn_not_initialized(arm); return -1;}

    q_out = q_input;
    unsigned int i;

    int maxiter=10000;   //NOTE: parameter

    for(i=0;i<maxiter;i++)
    {
        yarp::sig::Vector temp=q_out;
        q_out=q_out+next_step(arm,temp,precision)*d_t;
        if (cartesian_action_completed(arm,precision)) break;
    }

    if(i >= maxiter)
    {
        std::cout<<" --------------- WARNING Reached maximum number of iterations in cartToJnt --------------- "<<std::endl;
        return -1;
    }

    return wholebody_data.at(arm)->car_err;
}


yarp::sig::Vector wholebody_ik::next_step(std::string arm, const yarp::sig::Vector& q_input, double precision)
{
    chain_data *arm_data = wholebody_data.at(arm);

    if(!arm_data->initialized) {warn_not_initialized(arm); return yarp::sig::Vector();}
    
    // s_link = shoulder {S} , e_link = hand {E}, head  {H} 
    int e_index = arm_data->idynutils.iDyn3_model.getLinkIndex(arm_data->e_link);
    int s_index = arm_data->idynutils.iDyn3_model.getLinkIndex(arm_data->s_link);
    int h_index = arm_data->idynutils.iDyn3_model.getLinkIndex("gaze");
    yarp::sig::Matrix e_J_se, s_J_se;
    KDL::Frame ee_kdl;
    KDL::Frame pos_d;
    KDL::Twist vel_d;
    Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d L;
    yarp::sig::Vector Eo(3);
    Eigen::Vector3d zero; zero.setZero();

    yarp::sig::Vector q_all(arm_data->idynutils.getJointNames().size(),0.0);
    fromRobotToIDyn(arm, q_input,q_all);
    arm_data->idynutils.updateiDyn3Model(q_all,false);

    //
    // ------ transforming the Jacobian in {T}
    //
    arm_data->idynutils.iDyn3_model.getRelativeJacobian(e_index,s_index,e_J_se);
    yarp::sig::Matrix s_T_e = arm_data->idynutils.iDyn3_model.getPosition(s_index,e_index); // THE SECOND W.R.T. THE FIRST
    yarp::sig::Matrix null_vec(3,1); null_vec.zero();
    s_T_e.setSubmatrix(null_vec,0,3);
    s_J_se = locoman::utils::Adjoint(s_T_e) * e_J_se;
    yarp::sig::Matrix h_T_s = arm_data->idynutils.iDyn3_model.getPosition(h_index,s_index);
    const yarp::sig::Matrix& h_T_s_copy = arm_data->idynutils.iDyn3_model.getPosition(h_index,s_index);
    h_T_s.setSubmatrix(null_vec,0,3);
    yarp::sig::Matrix h_T_t = arm_data->h_T_t;
    h_T_t.setSubmatrix(null_vec,0,3);
    yarp::sig::Matrix t_J_se = locoman::utils::Adjoint(locoman::utils::iHomogeneous(h_T_t))*locoman::utils::Adjoint(h_T_s)*s_J_se;

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > ee_jac(t_J_se.data(),t_J_se.rows(),t_J_se.cols());

    //HACK
    int cols_to_remove = 2*6 + 3 ;
    if(arm=="right_arm") cols_to_remove += 7 + 2;

    arm_data->jacobian.block<6,DOFS>(0,0) = ee_jac.block<6,DOFS>(0,cols_to_remove);

    //
    // ------ transforming the ee position in {T}
    //
    math_utilities::FrameYARPToKDL(locoman::utils::iHomogeneous(arm_data->h_T_t)*h_T_s_copy*arm_data->idynutils.iDyn3_model.getPosition(s_index,e_index),arm_data->t_p_e_current);

    Eigen::Matrix<double,6,1> t_v_se;
    Eigen::Vector3d temp;

    //
    // ------ computing the desired velocity t_v_se
    //
    math_utilities::vectorKDLToEigen((arm_data->t_p_e_desired.p - arm_data->t_p_e_current.p), temp);
    t_v_se.block<3,1>(0,0) = temp;

    yarp::sig::Matrix ee_d(3,3);
    yarp::sig::Matrix ee_c(3,3);
    math_utilities::rotationKDLToYarp(arm_data->t_p_e_desired.M,ee_d);
    math_utilities::rotationKDLToYarp(arm_data->t_p_e_current.M,ee_c);
    Eo = locoman::utils::Orient_Error(ee_d, ee_c);
    math_utilities::vectorYARPToEigen(Eo,temp);
    t_v_se.block<3,1>(3,0)=temp;

    //
    // ------ computing the pseudoinverse of the jacobian
    //    
//     Eigen::Matrix<double,7,6> pinvJ =  math_utilities::pseudoInverseQR_76(arm_data->jacobian);

    if(!arm_data->first_step) arm_data->car_err=t_v_se.norm();
    else arm_data->first_step = false;

    Eigen::Matrix<double,DOFS,1> d_q;

    if (!cartesian_action_completed(arm,precision))
    {
        Eigen::Vector3d temp2;
        Eigen::Matrix<double,6,1> shoulder_diff;
        math_utilities::vectorKDLToEigen((arm_data->s_current.p - arm_data->s_previous.p), temp2);
        shoulder_diff.block<3,1>(0,0) = temp2;
        
        yarp::sig::Matrix SH_c(3,3);
        yarp::sig::Matrix SH_p(3,3); 
        yarp::sig::Vector e0(3);
        math_utilities::rotationKDLToYarp(arm_data->s_current.M,SH_c);
        math_utilities::rotationKDLToYarp(arm_data->s_previous.M,SH_p);
        e0 = locoman::utils::Orient_Error(SH_c, SH_p);
        math_utilities::vectorYARPToEigen(e0,temp);
        shoulder_diff.block<3,1>(3,0)=temp;

// 	d_q = arm_data->jacobian.colPivHouseholderQr().solve(t_v_se - shoulder_diff / arm_data->delta_T);

//         yarp::sig::Matrix jacco(6,DOFS);
//         Eigen::MatrixXd pseudo(DOFS,6);
//         math_utilities::matrixEigenToYARP(arm_data->jacobian,jacco);
//         math_utilities::matrixYARPToEigen(locoman::utils::Pinv_trunc_SVD(jacco),pseudo);
//         d_q = pseudo * t_v_se;

         Eigen::Matrix<double,7,6> pinvJ =  math_utilities::pseudoInverseQR_76(arm_data->jacobian);
//          d_q = pinvJ* t_v_se/d_t;

	 Eigen::Matrix<double,7,7> I77;
	 I77.setZero();
	 for(int w=0;w<7;w++) I77(w,w)=1.0;
	 Eigen::Matrix<double,7,1> des_q;
	 des_q.setZero();
	 des_q[1]=(arm=="right_arm")?0.35:-0.35;
	 Eigen::Matrix<double,7,1> input_q;
	 math_utilities::vectorYARPToEigen(q_input,input_q);
	 d_q = pinvJ* t_v_se/d_t + (I77-pinvJ*arm_data->jacobian) *0.05* (des_q-input_q); //HACK to avoid joint limits

//         d_q = pinvJ*( t_v_se - shoulder_diff / arm_data->delta_T );
    }
    else
        d_q.setZero();

    yarp::sig::Vector out(DOFS,0.0);

    for(int i = 0;i<DOFS;i++)
    {
        out[i] = d_q[i];
    }

    return out;
}

wholebody_ik::~wholebody_ik()
{
    delete wholebody_data.at("right_arm");
    delete wholebody_data.at("left_arm");
}