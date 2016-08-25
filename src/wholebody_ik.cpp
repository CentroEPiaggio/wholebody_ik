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

#define ARM_DOFS 7
#define LEG_DOFS 6
#define TORSO_DOFS 3
#define HEAD_DOFS 2

using namespace yarp::math;

chain_data::chain_data(std::string robot_name, std::string urdf_path, std::string srdf_path, std::string ee_link, std::string base_link, int dofs, std::string chain_name): idynutils(robot_name,urdf_path,srdf_path)
{
    yarp::sig::Vector joint_max = idynutils.iDyn3_model.getJointBoundMax();
    joint_max[idynutils.iDyn3_model.getDOFIndex("RElbj")] = -0.02;
    joint_max[idynutils.iDyn3_model.getDOFIndex("LElbj")] = -0.02;
    idynutils.iDyn3_model.setJointBoundMax(joint_max);

    idynutils.iDyn3_model.setAllConstraints(false);

    if(chain_name=="right_arm") { kin_chain = &idynutils.right_arm; jacobian = Eigen::Matrix<double,6,ARM_DOFS>();}
    if(chain_name=="left_arm") {kin_chain = &idynutils.left_arm; jacobian = Eigen::Matrix<double,6,ARM_DOFS>();}
    if(chain_name=="right_leg") {kin_chain = &idynutils.right_leg; jacobian = Eigen::Matrix<double,6,LEG_DOFS>();}
    if(chain_name=="left_leg") {kin_chain = &idynutils.left_leg; jacobian = Eigen::Matrix<double,6,LEG_DOFS>();}

    this->ee_link = ee_link;
    this->base_link = base_link;
    this->dofs = dofs;
    this->chain_name = chain_name;

    initialized = false;
}

wholebody_ik::wholebody_ik(std::string robot_name,std::string urdf_path, std::string srdf_path, int period_ms): d_t(period_ms/1000.0), robot("wb_ik", robot_name, urdf_path, srdf_path)
{    
    chains["right_arm"] = new chain_data(robot_name,urdf_path,srdf_path,"RSoftHand","Waist", ARM_DOFS, "right_arm");
    chains["left_arm"] = new chain_data(robot_name,urdf_path,srdf_path,"LSoftHand","Waist", ARM_DOFS, "left_arm");
    chains["right_leg"] = new chain_data(robot_name,urdf_path,srdf_path,"r_sole","Waist", LEG_DOFS, "right_leg");
    chains["left_leg"] = new chain_data(robot_name,urdf_path,srdf_path,"l_sole","Waist", LEG_DOFS, "left_leg");
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

bool wholebody_ik::initialize(std::string chain, KDL::Frame cartesian_pose, const yarp::sig::Vector& q_input)
{
    if(!chains.count(chain))
    {
        warn_not_initialized(chain);
        return false;
    }

    chain_data *data = chains.at(chain);

    if(!data->idynutils.iDyn3_model.setFloatingBaseLink(data->idynutils.iDyn3_model.getLinkIndex(data->get_base_link())))
    {
        std::cout<<"!! - Error in setting floating base - !!"<<std::endl;
        return false;
    }

    Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
    int dofs = data->get_dofs();
    int dim=6;
    data->Weights.resize(dim,dim);
    data->Weights.setZero();
    data->Weights.block<3,3>(0,0) = identity*1;
    data->Weights.block<3,3>(3,3) = identity*1;

    data->car_err = 9999.0;
    data->first_step = true;

    data->ee_current = KDL::Frame::Identity();
    data->ee_desired = cartesian_pose;

    data->jacobian.setZero();
    
    data->initialized = true;

    std::cout<<"=---------------------------"<<std::endl;
    std::cout<<" WholeBody IK Library initialized. Created a "<<dim<<"x" <<dofs<<" Jacobian ("<<chain<<")"<<std::endl;
    std::cout<<"=---------------------------"<<std::endl;
    
    return true;
}



void wholebody_ik::set_desired_ee_pose(std::string chain, KDL::Frame cartesian_pose)
{
    if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return;}

    chains.at(chain)->ee_desired=cartesian_pose;
}

bool wholebody_ik::cartesian_action_completed(std::string chain, double precision)
{
    if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return false;}

    return (chains.at(chain)->car_err < precision);
}


double wholebody_ik::cartToJnt(std::string chain, const yarp::sig::Vector& q_input, yarp::sig::Vector& q_out,double precision)
{
    if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return -1;}

    q_out = q_input;
    unsigned int i;

    int maxiter=10000;   //NOTE: parameter

    for(i=0;i<maxiter;i++)
    {
        yarp::sig::Vector temp=q_out;
        q_out=q_out+next_step(chain,temp,precision)*d_t;
        if (cartesian_action_completed(chain,precision)) break;
    }

    if(i >= maxiter)
    {
        std::cout<<" --------------- WARNING Reached maximum number of iterations in cartToJnt --------------- "<<std::endl;
        return -1;
    }

    return chains.at(chain)->car_err;
}

int compute_cols_to_remove(std::string chain)
{
    int cols_to_remove = 0;
    if(chain=="right_leg") cols_to_remove = LEG_DOFS;
    if(chain=="left_arm") cols_to_remove = 2*LEG_DOFS + TORSO_DOFS ;
    if(chain=="right_arm") cols_to_remove = 2*LEG_DOFS + TORSO_DOFS + ARM_DOFS + HEAD_DOFS;

    return cols_to_remove;
}

yarp::sig::Vector wholebody_ik::next_step(std::string chain, const yarp::sig::Vector& q_input, double precision)
{
    chain_data *data = chains.at(chain);
    int dofs = data->get_dofs();

    if(!data->initialized) {warn_not_initialized(chain); return yarp::sig::Vector();}
    
    // base_link = {B} , ee_link = {E}
    int e_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_ee_link());
    int b_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_base_link());
    yarp::sig::Matrix e_J_be, b_J_be;
    KDL::Frame ee_kdl;
    KDL::Frame pos_d;
    KDL::Twist vel_d;
    Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d L;
    yarp::sig::Vector Eo(3);
    Eigen::Vector3d zero; zero.setZero();

    yarp::sig::Vector q_all(data->idynutils.getJointNames().size(),0.0);
    data->idynutils.fromRobotToIDyn(q_input,q_all,*data->kin_chain);
    data->idynutils.updateiDyn3Model(q_all,false);

    //
    // ------ transforming the Jacobian in {B}
    //
    data->idynutils.iDyn3_model.getRelativeJacobian(e_index,b_index,e_J_be);
    yarp::sig::Matrix b_T_e = data->idynutils.iDyn3_model.getPosition(b_index,e_index); // THE SECOND W.R.T. THE FIRST
    yarp::sig::Matrix null_vec(3,1); null_vec.zero();
    b_T_e.setSubmatrix(null_vec,0,3);
    b_J_be = locoman::utils::Adjoint(b_T_e) * e_J_be;

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > ee_jac(b_J_be.data(),b_J_be.rows(),b_J_be.cols());

    if(data->get_name()=="right_arm" || data->get_name()=="left_arm")
        data->jacobian.block<6,ARM_DOFS>(0,0) = ee_jac.block<6,ARM_DOFS>(0,compute_cols_to_remove(chain));
    if(data->get_name()=="right_leg" || data->get_name()=="left_leg")
        data->jacobian.block<6,LEG_DOFS>(0,0) = ee_jac.block<6,LEG_DOFS>(0,compute_cols_to_remove(chain));

    //
    // ------ transforming the ee position in {B}
    //
    math_utilities::FrameYARPToKDL(data->idynutils.iDyn3_model.getPosition(b_index,e_index),data->ee_current);

    Eigen::Matrix<double,6,1> b_v_ee_desired;
    Eigen::Vector3d temp;

    //
    // ------ computing the desired velocity b_v_ee_desired
    //
    math_utilities::vectorKDLToEigen((data->ee_desired.p - data->ee_current.p), temp);
    b_v_ee_desired.block<3,1>(0,0) = temp;

    yarp::sig::Matrix ee_d(3,3);
    yarp::sig::Matrix ee_c(3,3);
    math_utilities::rotationKDLToYarp(data->ee_desired.M,ee_d);
    math_utilities::rotationKDLToYarp(data->ee_current.M,ee_c);
    Eo = locoman::utils::Orient_Error(ee_d, ee_c);
    math_utilities::vectorYARPToEigen(Eo,temp);
    b_v_ee_desired.block<3,1>(3,0)=temp;

    //
    // ------ computing the pseudoinverse of the jacobian
    //    
//     Eigen::Matrix<double,7,6> pinvJ =  math_utilities::pseudoInverseQR_76(data->jacobian);

    if(!data->first_step) data->car_err=b_v_ee_desired.norm();
    else data->first_step = false;

    Eigen::MatrixXd d_q;

    if(dofs==ARM_DOFS)
    {
        d_q = Eigen::Matrix<double,ARM_DOFS,1>();
    }
    if(dofs==LEG_DOFS)
    {
        d_q = Eigen::Matrix<double,LEG_DOFS,1>();
    }

    if (!cartesian_action_completed(chain,precision))
    {
//         yarp::sig::Matrix jacco(6,dofs);
//         Eigen::MatrixXd pseudo(dofs,6);
//         math_utilities::matrixEigenToYARP(data->jacobian,jacco);
//         math_utilities::matrixYARPToEigen(locoman::utils::Pinv_trunc_SVD(jacco),pseudo);
//         d_q = pseudo * b_v_ee_desired;

        Eigen::MatrixXd pinvJ;
	Eigen::MatrixXd In;
        Eigen::MatrixXd des_q;
        Eigen::MatrixXd input_q;

        if(dofs==ARM_DOFS)
        {
            In = Eigen::Matrix<double,ARM_DOFS,ARM_DOFS>();
            des_q = Eigen::Matrix<double,ARM_DOFS,1>();
            input_q = Eigen::Matrix<double,ARM_DOFS,1>();
            pinvJ = Eigen::Matrix<double,ARM_DOFS,6>();
            pinvJ = math_utilities::pseudoInverseQR_76(data->jacobian);
        }
        if(dofs==LEG_DOFS)
        {
            In = Eigen::Matrix<double,LEG_DOFS,LEG_DOFS>();
            des_q = Eigen::Matrix<double,LEG_DOFS,1>();
            input_q = Eigen::Matrix<double,LEG_DOFS,1>();
            pinvJ = Eigen::Matrix<double,LEG_DOFS,6>();
            pinvJ = math_utilities::pseudoInverseQR_66(data->jacobian);
        }

	In.setZero();
	for(int w=0;w<dofs;w++) In(w,w)=1.0;
	des_q.setZero();
	des_q(1)=(chain=="right_arm")?0.35:des_q(1);
        des_q(1)=(chain=="left_arm")?-0.35:des_q(1);
        double K_null = (chain=="right_arm" || chain=="left_arm")?0.05:0.0; //HACK to avoid arms joint limits

        math_utilities::vectorYARPToEigen(q_input,input_q);

	d_q = pinvJ* b_v_ee_desired/d_t + (In-pinvJ*data->jacobian) *K_null* (des_q-input_q);
    }
    else
        d_q.setZero();

    yarp::sig::Vector out(dofs ,0.0);

    for(int i = 0;i<dofs;i++)
    {
        out[i] = d_q(i);
    }

    return out;
}

wholebody_ik::~wholebody_ik()
{
    delete chains.at("right_arm");
    delete chains.at("left_arm");
    delete chains.at("right_leg");
    delete chains.at("left_leg");
}