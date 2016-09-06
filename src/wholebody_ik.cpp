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

#include <locoman/utils/declarations.h>

#include "utils.cpp" // NOTE: to remove

#define ARM_DOFS 7
#define LEG_DOFS 6
#define TORSO_DOFS 3
#define HEAD_DOFS 2
#define WB_DOFS 31
#define FLOATING_BASE_DOFS 6

#define CARTESIAN_DIM 6
#define COM_DIM 3

#define COM_FULL_DIM COM_DIM + 3*CARTESIAN_DIM

using namespace yarp::math;

chain_data::chain_data(std::string robot_name, std::string urdf_path, std::string srdf_path, std::string ee_link, std::string base_link, int dofs, std::string chain_name): idynutils(robot_name,urdf_path,srdf_path)
{
    yarp::sig::Vector joint_max = idynutils.iDyn3_model.getJointBoundMax();
    joint_max[idynutils.iDyn3_model.getDOFIndex("RElbj")] = -0.02;
    joint_max[idynutils.iDyn3_model.getDOFIndex("LElbj")] = -0.02;
    idynutils.iDyn3_model.setJointBoundMax(joint_max);

    idynutils.iDyn3_model.setAllConstraints(false); //to use joints limits

    if(chain_name=="right_arm") { kin_chain = &idynutils.right_arm; jacobian = Eigen::Matrix<double,CARTESIAN_DIM,ARM_DOFS>();}
    if(chain_name=="left_arm") {kin_chain = &idynutils.left_arm; jacobian = Eigen::Matrix<double,CARTESIAN_DIM,ARM_DOFS>();}
    if(chain_name=="right_leg") {kin_chain = &idynutils.right_leg; jacobian = Eigen::Matrix<double,CARTESIAN_DIM,LEG_DOFS>();}
    if(chain_name=="left_leg") {kin_chain = &idynutils.left_leg; jacobian = Eigen::Matrix<double,CARTESIAN_DIM,LEG_DOFS>();}

    if(chain_name=="com_left_foot" || chain_name=="com_right_foot") {com = true; jacobian = Eigen::Matrix<double,COM_DIM,WB_DOFS>();}

    this->ee_link = ee_link;
    this->base_link = base_link;
    this->dofs = dofs;
    this->chain_name = chain_name;

    initialized = false;
}

wholebody_ik::wholebody_ik(std::string robot_name,std::string urdf_path, std::string srdf_path, int period_ms): d_t(period_ms/1000.0)
{    
    chains["right_arm"]      = new chain_data(robot_name,urdf_path,srdf_path,"RSoftHand","Waist", ARM_DOFS, "right_arm");
    chains["left_arm"]       = new chain_data(robot_name,urdf_path,srdf_path,"LSoftHand","Waist", ARM_DOFS, "left_arm");
    chains["right_leg"]      = new chain_data(robot_name,urdf_path,srdf_path,"r_sole"   ,"Waist", LEG_DOFS, "right_leg");
    chains["left_leg"]       = new chain_data(robot_name,urdf_path,srdf_path,"l_sole"   ,"Waist", LEG_DOFS, "left_leg");

    chains["com_left_foot"]  = new chain_data(robot_name,urdf_path,srdf_path,"CoM"      ,"l_sole", WB_DOFS, "com_left_foot");
    chains["com_right_foot"] = new chain_data(robot_name,urdf_path,srdf_path,"CoM"      ,"r_sole", WB_DOFS, "com_right_foot");
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

void wholebody_ik::update_limbs_poses(std::string chain)
{
    if(!chains.at(chain)->com) return;

    std::string foot_frame = (chain=="com_left_foot")?"r_sole":"l_sole";
    std::string l_hand_frame = "LSoftHand";
    std::string r_hand_frame = "RSoftHand";

    int foot_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(foot_frame);
    int l_hand_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(l_hand_frame);
    int r_hand_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(r_hand_frame);

    math_utilities::FrameYARPToKDL(chains.at(chain)->idynutils.iDyn3_model.getPosition(foot_index),limbs_poses[foot_frame]);
    math_utilities::FrameYARPToKDL(chains.at(chain)->idynutils.iDyn3_model.getPosition(l_hand_index),limbs_poses[l_hand_frame]);
    math_utilities::FrameYARPToKDL(chains.at(chain)->idynutils.iDyn3_model.getPosition(r_hand_index),limbs_poses[r_hand_frame]);
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

    yarp::sig::Vector q_all(data->idynutils.getJointNames().size(),0.0);
    if(!data->com) data->idynutils.fromRobotToIDyn(q_input,q_all,*data->kin_chain);
    else q_all=q_input;
    data->idynutils.updateiDyn3Model(q_all,false);

    int dofs = data->get_dofs();
    int dim;
    if(data->com) dim=COM_DIM;
    else dim=CARTESIAN_DIM;

    data->car_err = 9999.0;
    data->first_step = true;

    data->ee_current = KDL::Frame::Identity();
    data->ee_desired = cartesian_pose;
    update_limbs_poses(chain);

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

    update_limbs_poses(chain);
}

double wholebody_ik::get_error(std::string chain)
{
    return chains.at(chain)->car_err;
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
    if(chain=="com_left_foot" || chain=="com_right_foot") cols_to_remove = FLOATING_BASE_DOFS;

    return cols_to_remove;
}

yarp::sig::Vector wholebody_ik::next_step(std::string chain, const yarp::sig::Vector& q_input, double precision)
{
    chain_data *data = chains.at(chain);
    int dofs = data->get_dofs();

    if(!data->initialized) {warn_not_initialized(chain); return yarp::sig::Vector();}

    yarp::sig::Matrix e_J_be, b_J_be;
    KDL::Frame ee_kdl;
    KDL::Frame pos_d;
    KDL::Twist vel_d;
    Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d L;
    yarp::sig::Vector Eo(3);
    Eigen::Vector3d zero; zero.setZero();
    yarp::sig::Vector out(dofs ,0.0);
    Eigen::MatrixXd d_q;

    yarp::sig::Vector q_all(data->idynutils.getJointNames().size(),0.0);
    if(!chains.at(chain)->com) data->idynutils.fromRobotToIDyn(q_input,q_all,*data->kin_chain);
    else q_all=q_input;
    data->idynutils.updateiDyn3Model(q_all,false);

    int e_index = 0;
    int b_index = 0;

    //
    // ------ transforming the Jacobian in {B}
    //
    // base_link = {B} , ee_link = {E}
    if(chains.at(chain)->com) //COM
    {
        if(!data->idynutils.iDyn3_model.getCOMJacobian(b_J_be))
        {
            std::cout<<" !! ERROR : UNABLE TO GET COM JACOBIAN !! "<<std::endl;
            return out;
        }
    }
    else //NOT COM
    {
        e_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_ee_link());
        b_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_base_link());

        if(!data->idynutils.iDyn3_model.getRelativeJacobian(e_index,b_index,e_J_be))
        {
            std::cout<<" !! ERROR : UNABLE TO GET JACOBIAN !! "<<std::endl;
            return out;
        }
        
        yarp::sig::Matrix b_T_e = data->idynutils.iDyn3_model.getPosition(b_index,e_index); // THE SECOND W.R.T. THE FIRST
        yarp::sig::Matrix null_vec(3,1); null_vec.zero();
        b_T_e.setSubmatrix(null_vec,0,3);
        b_J_be = locoman::utils::Adjoint(b_T_e) * e_J_be;
    }

    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > ee_jac(b_J_be.data(),b_J_be.rows(),b_J_be.cols());

    if(data->get_name()=="right_arm" || data->get_name()=="left_arm")
        data->jacobian.block<CARTESIAN_DIM,ARM_DOFS>(0,0) = ee_jac.block<CARTESIAN_DIM,ARM_DOFS>(0,compute_cols_to_remove(chain));
    if(data->get_name()=="right_leg" || data->get_name()=="left_leg")
        data->jacobian.block<CARTESIAN_DIM,LEG_DOFS>(0,0) = ee_jac.block<CARTESIAN_DIM,LEG_DOFS>(0,compute_cols_to_remove(chain));
    if(chains.at(chain)->com)
        data->jacobian.block<COM_DIM,WB_DOFS>(0,0) = ee_jac.block<COM_DIM,WB_DOFS>(0,compute_cols_to_remove(chain));

    if(!chains.at(chain)->com) // NOT COM
    {
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

        if(!data->first_step) data->car_err=b_v_ee_desired.norm();
        else data->first_step = false;

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
            Eigen::MatrixXd pinvJ;
            Eigen::MatrixXd In;
            Eigen::MatrixXd des_q;
			Eigen::MatrixXd des_dq;
            Eigen::MatrixXd input_q;

            if(dofs==ARM_DOFS)
            {
                In = Eigen::Matrix<double,ARM_DOFS,ARM_DOFS>();
                des_q = Eigen::Matrix<double,ARM_DOFS,1>();
				des_dq = Eigen::Matrix<double,ARM_DOFS,1>();
                input_q = Eigen::Matrix<double,ARM_DOFS,1>();
                pinvJ = Eigen::Matrix<double,ARM_DOFS,CARTESIAN_DIM>();
                pinvJ = math_utilities::pseudoInverseQR_76(data->jacobian);
            }
            if(dofs==LEG_DOFS)
            {
                In = Eigen::Matrix<double,LEG_DOFS,LEG_DOFS>();
                des_q = Eigen::Matrix<double,LEG_DOFS,1>();
				des_dq = Eigen::Matrix<double,LEG_DOFS,1>();
                input_q = Eigen::Matrix<double,LEG_DOFS,1>();
                pinvJ = Eigen::Matrix<double,LEG_DOFS,CARTESIAN_DIM>();
                pinvJ = math_utilities::pseudoInverseQR_66(data->jacobian);
            }

            In.setZero();
            for(int w=0;w<dofs;w++) In(w,w)=1.0;

			des_q.setZero();
			des_q(1)=(chain=="right_arm")?0.35:des_q(1);
			des_q(1)=(chain=="left_arm")?-0.35:des_q(1);
			math_utilities::vectorYARPToEigen(q_input,input_q);
			des_dq.setZero();

			if(dofs==ARM_DOFS)
			{
				des_dq(1) = ((des_q-input_q)(1))/d_t;
				des_dq(1) = std::min(std::max(des_dq(1),-0.1),0.1);
			}

			d_q = pinvJ* b_v_ee_desired/d_t + (In-pinvJ*data->jacobian) * (des_dq); //null projection to avoid joint limits

		}
        else
            d_q.setZero();
    }
    else // -------------------- COM ---------------------------
    {
        std::string base_frame = (chain=="com_left_foot")?"l_sole":"r_sole";
        std::string foot_frame = (chain=="com_left_foot")?"r_sole":"l_sole";
        std::string l_hand_frame = "LSoftHand";
        std::string r_hand_frame = "RSoftHand";

        int base_index = data->idynutils.iDyn3_model.getLinkIndex(base_frame);
        int foot_index = data->idynutils.iDyn3_model.getLinkIndex(foot_frame);
        int l_hand_index = data->idynutils.iDyn3_model.getLinkIndex(l_hand_frame);
        int r_hand_index = data->idynutils.iDyn3_model.getLinkIndex(r_hand_frame);

        KDL::Vector com_pos;
        math_utilities::vectorYARPToKDL(data->idynutils.iDyn3_model.getCOM(),com_pos);
        data->ee_current.M = KDL::Rotation::Identity();
        data->ee_current.p = com_pos;

        //
        // ------ computing the desired velocity b_v_ee_desired
        //
        Eigen::Matrix<double,COM_DIM,1> b_v_ee_desired;
        Eigen::Matrix<double,3*CARTESIAN_DIM,1> b_v_ee_desired_cart;
        Eigen::Vector3d temp;
        KDL::Frame temp_current_ee;
        yarp::sig::Matrix ee_d(3,3);
        yarp::sig::Matrix ee_c(3,3);

        std::vector<int> ee_index;
        ee_index.push_back(foot_index);
        ee_index.push_back(l_hand_index);
        ee_index.push_back(r_hand_index);
        std::vector<std::string> ee_names;
        ee_names.push_back(foot_frame);
        ee_names.push_back(l_hand_frame);
        ee_names.push_back(r_hand_frame);

        math_utilities::vectorKDLToEigen((data->ee_desired.p - data->ee_current.p), temp);
        b_v_ee_desired.block<COM_DIM,1>(0,0) = temp;

        for(int limb_num=0;limb_num<3;limb_num++)
        {
            math_utilities::FrameYARPToKDL(data->idynutils.iDyn3_model.getPosition(ee_index.at(limb_num)),temp_current_ee);

            math_utilities::vectorKDLToEigen((limbs_poses.at(ee_names.at(limb_num)).p - temp_current_ee.p), temp);
            b_v_ee_desired_cart.block<3,1>(CARTESIAN_DIM*limb_num,0) = temp;

            math_utilities::rotationKDLToYarp(limbs_poses.at(ee_names.at(limb_num)).M,ee_d);
            math_utilities::rotationKDLToYarp(temp_current_ee.M,ee_c);
            Eo = locoman::utils::Orient_Error(ee_d, ee_c);
            math_utilities::vectorYARPToEigen(Eo,temp);
            b_v_ee_desired_cart.block<3,1>(CARTESIAN_DIM*limb_num+3,0)=temp;
        }
        
        if(!data->first_step) data->car_err= b_v_ee_desired.norm();
        else data->first_step = false;

        d_q = Eigen::Matrix<double,WB_DOFS,1>();

        if (!cartesian_action_completed(chain,precision))
        {
            Eigen::MatrixXd pinvJ_com = Eigen::Matrix<double,WB_DOFS,COM_DIM>();
            Eigen::MatrixXd input_q = Eigen::Matrix<double,WB_DOFS,1>();
            Eigen::MatrixXd J_cart = Eigen::Matrix<double,3*CARTESIAN_DIM,WB_DOFS>();
            Eigen::MatrixXd pinvJ_cart = Eigen::Matrix<double,WB_DOFS,3*CARTESIAN_DIM>();
            Eigen::MatrixXd I31 = Eigen::Matrix<double,WB_DOFS,WB_DOFS>();
            I31.setZero();
            for(int w=0;w<dofs;w++) I31(w,w)=1.0;
            double K_null=0.05;

            yarp::sig::Matrix foot_J;
            yarp::sig::Matrix l_hand_J;
            yarp::sig::Matrix r_hand_J;

            if(!data->idynutils.iDyn3_model.getRelativeJacobian(foot_index,base_index,foot_J))
            {
                std::cout<<" !! ERROR : UNABLE TO GET JACOBIAN - "<<foot_frame<<std::endl;
                return out;
            }
            if(!data->idynutils.iDyn3_model.getRelativeJacobian(l_hand_index,base_index,l_hand_J))
            {
                std::cout<<" !! ERROR : UNABLE TO GET JACOBIAN - "<<l_hand_frame<<std::endl;
                return out;
            }
            if(!data->idynutils.iDyn3_model.getRelativeJacobian(r_hand_index,base_index,r_hand_J))
            {
                std::cout<<" !! ERROR : UNABLE TO GET JACOBIAN - "<<r_hand_frame<<std::endl;
                return out;
            }

            Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > eigen_foot_J(foot_J.data(),foot_J.rows(),foot_J.cols());
            Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > eigen_l_hand_J(l_hand_J.data(),l_hand_J.rows(),l_hand_J.cols());
            Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > eigen_r_hand_J(r_hand_J.data(),r_hand_J.rows(),r_hand_J.cols());

            J_cart.block<CARTESIAN_DIM,WB_DOFS>(0,0) = eigen_foot_J;
            J_cart.block<CARTESIAN_DIM,WB_DOFS>(CARTESIAN_DIM,0) = eigen_l_hand_J;
            J_cart.block<CARTESIAN_DIM,WB_DOFS>(2*CARTESIAN_DIM,0) = eigen_r_hand_J;

            pinvJ_cart = math_utilities::pseudoInverseQR_3118(J_cart);

            pinvJ_com = math_utilities::pseudoInverseQR_313(data->jacobian);

            math_utilities::vectorYARPToEigen(q_input,input_q);

            d_q = pinvJ_cart* b_v_ee_desired_cart/d_t + K_null * (I31 - pinvJ_cart*J_cart) * pinvJ_com * b_v_ee_desired/d_t;
        }
        else
        {
            d_q.setZero();
            std::cout<<"converged"<<std::endl;
        }
    }

    for(int i = 0;i<dofs;i++)
    {
//         out[i] = d_q(i);
        out[i] = std::min(std::max(d_q(i),-0.1),0.1);
    }

    return out;
}

wholebody_ik::~wholebody_ik()
{
    delete chains.at("right_arm");
    delete chains.at("left_arm");
    delete chains.at("right_leg");
    delete chains.at("left_leg");
    delete chains.at("com_left_foot");
    delete chains.at("com_right_foot");
}