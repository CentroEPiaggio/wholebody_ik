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
#define FLOATING_BASE_DOFS 6
#define WB_DOFS 31 + FLOATING_BASE_DOFS

#define CARTESIAN_DIM 6
#define COM_DIM 3

#define FULL_DIM COM_DIM + 4*CARTESIAN_DIM

using namespace yarp::math;

std::string green(std::string text)
{
    std::string g="\033[0;32m";
    std::string b="\033[0m";
    return g + text + b;
}

std::string yellow(std::string text)
{
    std::string y="\033[0;33m";
    std::string b="\033[0m";
    return y + text + b;
}

std::string red(std::string text)
{
    std::string r="\033[0;31m";
    std::string b="\033[0m";
    return r + text + b;
}

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

    if(chain_name=="wb_left" || chain_name=="wb_right") {wb = true; jacobian = Eigen::Matrix<double,FULL_DIM,WB_DOFS>();}

    this->ee_link = ee_link;
    this->base_link = base_link;
    this->dofs = dofs;
    this->chain_name = chain_name;

    initialized = false;
    set = false;
}

wholebody_ik::wholebody_ik(std::string robot_name,std::string urdf_path, std::string srdf_path, int period_ms): d_t(period_ms/1000.0)
{    
    chains["right_arm"]      = new chain_data(robot_name,urdf_path,srdf_path,"RSoftHand","Waist", ARM_DOFS, "right_arm");
    chains["left_arm"]       = new chain_data(robot_name,urdf_path,srdf_path,"LSoftHand","Waist", ARM_DOFS, "left_arm");
    chains["right_leg"]      = new chain_data(robot_name,urdf_path,srdf_path,"r_sole"   ,"Waist", LEG_DOFS, "right_leg");
    chains["left_leg"]       = new chain_data(robot_name,urdf_path,srdf_path,"l_sole"   ,"Waist", LEG_DOFS, "left_leg");

    chains["wb_left"]  = new chain_data(robot_name,urdf_path,srdf_path,"WB"      ,"Waist", WB_DOFS, "wb_left");
    chains["wb_right"] = new chain_data(robot_name,urdf_path,srdf_path,"WB"      ,"Waist", WB_DOFS, "wb_right");
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

void wholebody_ik::print_eigen_matrix_ndigits(const Eigen::MatrixXd& data, int n)
{
	for(int i=0;i<data.rows();i++)
	{
		for(int j=0;j<data.cols();j++)
		{
			std::cout<<((int)(data(i,j)*pow(10,n)))/pow(10,n)<<' ';
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

void wholebody_ik::print_YARP_matrix_ndigits(const yarp::sig::Matrix& data, int n)
{
	for(int i=0;i<data.rows();i++)
	{
		for(int j=0;j<data.cols();j++)
		{
			std::cout<<((int)(data[i][j]*pow(10,n)))/pow(10,n)<<' ';
		}
		std::cout<<std::endl;
	}
}

void wholebody_ik::warn_not_initialized(std::string str)
{
    std::cout<<yellow(" --------------- WARNING WB IK NOT PROPERLY INITIALIZED FOR '" + str + "'--------------- ")<<std::endl;
}

void wholebody_ik::warn_desired_not_set(std::string str)
{
    std::cout<<yellow(" --------------- WARNING NEVER SET DESIRED POSE FOR '" + str + "'--------------- ")<<std::endl;
}

bool wholebody_ik::initialize(std::string chain, const yarp::sig::Vector& q_input)
{
    if(!chains.count(chain))
    {
        warn_not_initialized(chain);
        return false;
    }

    chain_data *data = chains.at(chain);

    if(!data->idynutils.iDyn3_model.setFloatingBaseLink(data->idynutils.iDyn3_model.getLinkIndex(data->get_base_link())))
    {
        std::cout<<red("!! - Error in setting floating base - !!")<<std::endl;
        return false;
    }

    yarp::sig::Vector q_all(data->idynutils.getJointNames().size(),0.0);
    if(!data->wb) data->idynutils.fromRobotToIDyn(q_input,q_all,*data->kin_chain);
    else q_all=q_input;
    data->idynutils.updateiDyn3Model(q_all,true);

    int dofs = data->get_dofs();
    int dim;
    if(data->wb)
    {
        dim=FULL_DIM;
        data->desired_poses["l_sole"] = KDL::Frame::Identity();
        data->desired_poses["r_sole"] = KDL::Frame::Identity();
        data->desired_poses["LSoftHand"] = KDL::Frame::Identity();
        data->desired_poses["RSoftHand"] = KDL::Frame::Identity();
        data->desired_poses["COM"] = KDL::Frame::Identity();
    }
    else dim=CARTESIAN_DIM;

    data->car_err = 9999.0;
    data->first_step = true;

    data->ee_current = KDL::Frame::Identity();
    data->ee_desired = KDL::Frame::Identity();

    data->jacobian.setZero();
    
    data->initialized = true;

    std::cout<<green("=---------------------------")<<std::endl;
    std::cout<<green(" WholeBody IK Library initialized. Created a "+ std::to_string(dim) + "x" + std::to_string(dofs) + " Jacobian (" + chain + ")")<<std::endl;
    std::cout<<green("=---------------------------")<<std::endl;

    return true;
}

void wholebody_ik::set_desired_wb_poses_as_current(std::string chain)
{
    if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return;}

    int link_index;
    
    std::string base_frame = (chain=="wb_left")?"l_sole":"r_sole";
    int base_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(base_frame);
    
    for(auto& pose:chains.at(chain)->desired_poses)
    {
        if(pose.first!="COM")
        {
            link_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(pose.first);

            if(link_index==-1)
            {
                std::cout<<red(" !! ERROR : UNABLE TO GET LINK INDEX !! ")<<std::endl;
                return;
            }

            pose.second = chains.at(chain)->idynutils.iDyn3_model.getPositionKDL(base_index,link_index);
        }
        else
        {
			KDL::Vector com;
			math_utilities::vectorYARPToKDL(get_com_position_wrt_base_frame(chain,base_index),com);
            pose.second = KDL::Frame(KDL::Rotation::Identity(), com);
        }
    }

    chains.at(chain)->set=true;
}

void wholebody_ik::set_desired_wb_poses(std::string chain, std::map<std::string, KDL::Frame> cartesian_poses)
{
    if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return;}

    for(auto pose:cartesian_poses)
    {
        if(chains.at(chain)->desired_poses.count(pose.first)) chains.at(chain)->desired_poses.at(pose.first) = pose.second;
        else std::cout<<yellow(" --------------- WARNING wrong link name : " + pose.first + " --------------- ")<<std::endl;
    }

    chains.at(chain)->set=true;
}

void wholebody_ik::get_current_wb_poses(std::string chain, std::map< std::string, KDL::Frame >& cartesian_poses)
{
	if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return;}

	int link_index;

	std::string base_frame = (chain=="wb_left")?"l_sole":"r_sole";
	int base_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(base_frame);

	for(auto pose:chains.at(chain)->desired_poses)
	{
		if(pose.first!="COM")
		{
			link_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(pose.first);
			
			if(link_index==-1)
			{
				std::cout<<red(" !! ERROR : UNABLE TO GET LINK INDEX !! ")<<std::endl;
				return;
			}

			cartesian_poses[pose.first] = chains.at(chain)->idynutils.iDyn3_model.getPositionKDL(base_index,link_index);
		}
		else
		{
			KDL::Vector com;
			math_utilities::vectorYARPToKDL(get_com_position_wrt_base_frame(chain,base_index),com);
			cartesian_poses[pose.first] = KDL::Frame(KDL::Rotation::Identity(), com);
		}
	}
}

void wholebody_ik::get_desired_wb_poses(std::string chain, std::map<std::string, KDL::Frame>& cartesian_poses)
{
    for(auto pose:chains.at(chain)->desired_poses)
        cartesian_poses[pose.first] = pose.second;
}

void wholebody_ik::get_current_ee_pose(std::string chain, KDL::Frame& cartesian_pose)
{
    std::string frame;

    if(chain=="right_arm") frame="RSoftHand";
    if(chain=="left_arm") frame="LSoftHand";
    if(chain=="right_leg") frame="r_sole";
    if(chain=="left_leg") frame="l_sole";

    int link_index;

    link_index = chains.at(chain)->idynutils.iDyn3_model.getLinkIndex(frame);

    cartesian_pose = chains.at(chain)->idynutils.iDyn3_model.getPositionKDL(link_index);
}

void wholebody_ik::set_desired_ee_pose(std::string chain, KDL::Frame cartesian_pose)
{
    if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return;}

    chains.at(chain)->set=true;

    chains.at(chain)->ee_desired=cartesian_pose;
}

void wholebody_ik::set_desired_ee_pose_as_current(std::string chain)
{
    if(!chains.at(chain)->initialized) {warn_not_initialized(chain); return;}

    get_current_ee_pose(chain, chains.at(chain)->ee_desired);

    chains.at(chain)->set=true;
}

yarp::sig::Vector wholebody_ik::get_com_position_wrt_base_frame(std::string chain, int base_index)
{
	yarp::sig::Vector w_p_com = chains.at(chain)->idynutils.iDyn3_model.getCOM();
	yarp::sig::Matrix w_T_b = chains.at(chain)->idynutils.iDyn3_model.getPosition(base_index);
	yarp::sig::Matrix b_R_w = locoman::utils::getRot(locoman::utils::iHomogeneous(w_T_b)); // only rotation
	yarp::sig::Vector b_p_com_w = b_R_w * w_p_com; // COM w.r.t. World but expressed in base_frame
	yarp::sig::Vector b_p_w = locoman::utils::getTrasl(locoman::utils::iHomogeneous(w_T_b));
	yarp::sig::Vector b_p_com = b_p_w + b_p_com_w;

	return b_p_com;
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
    if(!chains.at(chain)->set) {warn_desired_not_set(chain); return -1;}

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
        std::cout<<yellow(" --------------- WARNING Reached maximum number of iterations in cartToJnt --------------- ")<<std::endl;
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
    if(!data->set) {warn_desired_not_set(chain); return yarp::sig::Vector();}

    std::string l_foot_frame = "l_sole";
    std::string r_foot_frame = "r_sole";
    std::string l_hand_frame = "LSoftHand";
    std::string r_hand_frame = "RSoftHand";

    int l_foot_index = data->idynutils.iDyn3_model.getLinkIndex(l_foot_frame);
    int r_foot_index = data->idynutils.iDyn3_model.getLinkIndex(r_foot_frame);
    int l_hand_index = data->idynutils.iDyn3_model.getLinkIndex(l_hand_frame);
    int r_hand_index = data->idynutils.iDyn3_model.getLinkIndex(r_hand_frame);
    
    std::vector<int> ee_index;
    ee_index.push_back(l_hand_index);
    ee_index.push_back(r_hand_index);
    ee_index.push_back(l_foot_index);
    ee_index.push_back(r_foot_index);
    
    std::vector<std::string> ee_names;
    ee_names.push_back(l_hand_frame);
    ee_names.push_back(r_hand_frame);
    ee_names.push_back(l_foot_frame);
    ee_names.push_back(r_foot_frame);
    
    yarp::sig::Matrix e_J_be, b_J_be;
    KDL::Frame ee_kdl;
    KDL::Frame pos_d;
    KDL::Twist vel_d;
    Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
    Eigen::Matrix3d L;
    yarp::sig::Vector Eo(3);
    Eigen::Vector3d zero; zero.setZero();
    yarp::sig::Vector out(WB_DOFS-FLOATING_BASE_DOFS ,0.0);
    if(!data->wb) out.resize(dofs ,0.0);
    Eigen::MatrixXd d_q;

    yarp::sig::Vector q_all(data->idynutils.getJointNames().size(),0.0);
    if(!data->wb) data->idynutils.fromRobotToIDyn(q_input,q_all,*data->kin_chain);
    else q_all=q_input;
    data->idynutils.updateiDyn3Model(q_all,true);

    //
    // ------ transforming the Jacobian in {B}
    //
    // base_link = {B} , ee_link = {E}
    
    /**
     * From now on, we express the jacobians of the e-e w.r.t. the distal frame
     * The CoM jacobian is expressed either w.r.t. one foot or the other
     */
    
    std::string base_frame;
    if(chain=="wb_left") base_frame="l_sole";
    if(chain=="wb_right") base_frame="r_sole";

    int base_index = data->idynutils.iDyn3_model.getLinkIndex(base_frame);

    if(data->wb) //WB
    {
        //COM
        yarp::sig::Matrix w_J_com;
        if(!data->idynutils.iDyn3_model.getCOMJacobian(w_J_com)) //NOTE: it has also orientation part, to be removed :3
        {
            std::cout<<" !! ERROR : UNABLE TO GET COM JACOBIAN !! "<<std::endl;
            return out;
        }
        yarp::sig::Matrix w_T_b = data->idynutils.iDyn3_model.getPosition(base_index);
	yarp::sig::Vector null_vec(3,0.0);
	yarp::sig::Matrix b_Tr_w = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(w_T_b)),null_vec); // only rotation
	yarp::sig::Matrix b_J_com = locoman::utils::Adjoint(b_Tr_w) * w_J_com;
	Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > com_jac(b_J_com.data(),b_J_com.rows(),b_J_com.cols());

	// LEFT HAND
	yarp::sig::Matrix w_J_lh;
        if(!data->idynutils.iDyn3_model.getJacobian(l_hand_index,w_J_lh))
        {
            std::cout<<" !! ERROR : UNABLE TO GET LSoftHand JACOBIAN !! "<<std::endl;
            return out;
        }
        yarp::sig::Matrix w_T_lh = data->idynutils.iDyn3_model.getPosition(l_hand_index);
	yarp::sig::Matrix lh_Tr_w = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(w_T_lh)),null_vec);
	yarp::sig::Matrix lh_J_lh = locoman::utils::Adjoint(lh_Tr_w) * w_J_lh;
        Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > lh_jac(lh_J_lh.data(),lh_J_lh.rows(),lh_J_lh.cols());

	// RIGHT HAND
        yarp::sig::Matrix w_J_rh;
        if(!data->idynutils.iDyn3_model.getJacobian(r_hand_index,w_J_rh))
        {
            std::cout<<" !! ERROR : UNABLE TO GET RSoftHand JACOBIAN !! "<<std::endl;
            return out;
        }
        yarp::sig::Matrix w_T_rh = data->idynutils.iDyn3_model.getPosition(r_hand_index);
	yarp::sig::Matrix rh_Tr_w = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(w_T_rh)),null_vec);
	yarp::sig::Matrix rh_J_rh = locoman::utils::Adjoint(rh_Tr_w) * w_J_rh;
        Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > rh_jac(rh_J_rh.data(),rh_J_rh.rows(),rh_J_rh.cols());

	// LEFT FOOT
        yarp::sig::Matrix w_J_lf;
        if(!data->idynutils.iDyn3_model.getJacobian(l_foot_index,w_J_lf))
        {
            std::cout<<" !! ERROR : UNABLE TO GET l_sole JACOBIAN !! "<<std::endl;
            return out;
        }
	yarp::sig::Matrix w_T_lf = data->idynutils.iDyn3_model.getPosition(l_foot_index);
	yarp::sig::Matrix lf_Tr_w = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(w_T_lf)),null_vec);
	yarp::sig::Matrix lf_J_lf = locoman::utils::Adjoint(lf_Tr_w) * w_J_lf;
        Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > lf_jac(lf_J_lf.data(),lf_J_lf.rows(),lf_J_lf.cols());

	// RIGHT FOOT
        yarp::sig::Matrix w_J_rf;        
        if(!data->idynutils.iDyn3_model.getJacobian(r_foot_index,w_J_rf))
        {
            std::cout<<" !! ERROR : UNABLE TO GET r_sole JACOBIAN !! "<<std::endl;
            return out;
        }
	yarp::sig::Matrix w_T_rf = data->idynutils.iDyn3_model.getPosition(r_foot_index);
	yarp::sig::Matrix rf_Tr_w = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(w_T_rf)),null_vec);
	yarp::sig::Matrix rf_J_rf = locoman::utils::Adjoint(rf_Tr_w) * w_J_rf;
        Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > rf_jac(rf_J_rf.data(),rf_J_rf.rows(),rf_J_rf.cols());

        // we use the floating base to perform the pseudoinverse, then we remove the extra joints velocity

	// final jacobian:
	// | Jcom |
	// | JLH  |
	// | JRH  |
	// | JLF  |
	// | JRF  |
		
		// matrix.block<p,q>(i,j) = Block of size (p,q), starting at (i,j)
        data->jacobian.block<COM_DIM,WB_DOFS>(0,0) = com_jac.block<COM_DIM,WB_DOFS>(0,0); // removing orientation part :3
        data->jacobian.block<CARTESIAN_DIM,WB_DOFS>(COM_DIM,0) = lh_jac.block<CARTESIAN_DIM,WB_DOFS>(0,0);
        data->jacobian.block<CARTESIAN_DIM,WB_DOFS>(COM_DIM + CARTESIAN_DIM,0) = rh_jac.block<CARTESIAN_DIM,WB_DOFS>(0,0);
        data->jacobian.block<CARTESIAN_DIM,WB_DOFS>(COM_DIM + 2*CARTESIAN_DIM,0) = lf_jac.block<CARTESIAN_DIM,WB_DOFS>(0,0);
        data->jacobian.block<CARTESIAN_DIM,WB_DOFS>(COM_DIM + 3*CARTESIAN_DIM,0) = rf_jac.block<CARTESIAN_DIM,WB_DOFS>(0,0);
    }
    else //NOT WB
    {
        int e_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_ee_link());
        int b_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_base_link());

        if(!data->idynutils.iDyn3_model.getRelativeJacobian(e_index,b_index,e_J_be))
        {
            std::cout<<" !! ERROR : UNABLE TO GET JACOBIAN !! "<<std::endl;
            return out;
        }
        
        yarp::sig::Matrix b_T_e = data->idynutils.iDyn3_model.getPosition(b_index,e_index); // THE SECOND W.R.T. THE FIRST
        yarp::sig::Matrix null_vec(3,1); null_vec.zero();
        b_T_e.setSubmatrix(null_vec,0,3);
        b_J_be = locoman::utils::Adjoint(b_T_e) * e_J_be;

        Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > ee_jac(b_J_be.data(),b_J_be.rows(),b_J_be.cols());

        if(data->get_name()=="right_arm" || data->get_name()=="left_arm")
            data->jacobian.block<CARTESIAN_DIM,ARM_DOFS>(0,0) = ee_jac.block<CARTESIAN_DIM,ARM_DOFS>(0,compute_cols_to_remove(chain));
        if(data->get_name()=="right_leg" || data->get_name()=="left_leg")
            data->jacobian.block<CARTESIAN_DIM,LEG_DOFS>(0,0) = ee_jac.block<CARTESIAN_DIM,LEG_DOFS>(0,compute_cols_to_remove(chain));
    }

    if(!data->wb) // NOT WB
    {
		int e_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_ee_link());
		int b_index = data->idynutils.iDyn3_model.getLinkIndex(data->get_base_link());
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
    else // -------------------- WB ---------------------------
    {
        //
        // ------ computing the desired velocity
        //
      
	// d_C: delta configuration
	// | d_C_com |
	// | d_C_LH  |
	// | d_C_RH  |
	// | d_C_LF  |
	// | d_C_RF  |
      
        Eigen::Matrix<double,FULL_DIM,1> d_C;
        Eigen::Vector3d temp;
        yarp::sig::Matrix ee_d(3,3);

	KDL::Vector com_pos;
	math_utilities::vectorYARPToKDL(get_com_position_wrt_base_frame(chain,base_index),com_pos);

        math_utilities::vectorKDLToEigen((data->desired_poses.at("COM").p - com_pos), temp);
        d_C.block<COM_DIM,1>(0,0) = temp;

	KDL::Frame temp_current_ee;
	yarp::sig::Matrix Eye_3(3,3);
	Eye_3.eye() ;

        for(int limb_num=0;limb_num<4;limb_num++)
        {
			yarp::sig::Matrix f_T_ee_cur = data->idynutils.iDyn3_model.getPosition(ee_index.at(limb_num));
			yarp::sig::Matrix f_T_b = data->idynutils.iDyn3_model.getPosition(base_index);
			yarp::sig::Matrix b_T_ee_cur = locoman::utils::iHomogeneous(f_T_b) * f_T_ee_cur;

			yarp::sig::Matrix b_T_ee_des(4,4);
			math_utilities::FrameKDLToYARP(data->desired_poses.at(ee_names.at(limb_num)),b_T_ee_des);

			yarp::sig::Matrix cur_T_des = locoman::utils::iHomogeneous(b_T_ee_cur) * b_T_ee_des;
			math_utilities::vectorYARPToEigen(locoman::utils::getTrasl(cur_T_des),temp);
			d_C.block<3,1>(COM_DIM+CARTESIAN_DIM*limb_num,0) = temp;

			ee_d = locoman::utils::getRot(cur_T_des);
			Eo = locoman::utils::Orient_Error(ee_d, Eye_3);
			math_utilities::vectorYARPToEigen(Eo,temp);
			d_C.block<3,1>(COM_DIM+CARTESIAN_DIM*limb_num+3,0) = temp;
        }

        if(!data->first_step) data->car_err= d_C.norm();
        else data->first_step = false;

        d_q = Eigen::Matrix<double,WB_DOFS-FLOATING_BASE_DOFS,1>();

        Eigen::MatrixXd full_d_q = Eigen::Matrix<double,WB_DOFS,1>();

        if (!cartesian_action_completed(chain,precision)) // NOTE maybe not the best stop criterion
        {
			Eigen::MatrixXd pinvJ = Eigen::Matrix<double,WB_DOFS,FULL_DIM>();

			pinvJ = math_utilities::pseudoInverse(data->jacobian, 1E-8);
			
			full_d_q = pinvJ* d_C/d_t;

            d_q.block<WB_DOFS-FLOATING_BASE_DOFS,1>(0,0) = full_d_q.block<WB_DOFS-FLOATING_BASE_DOFS,1>(6,0);
        }
        else
        {
            d_q.setZero();
            std::cout<<"converged"<<std::endl;
        }
    }
    
    for(int i = 0;i<WB_DOFS-FLOATING_BASE_DOFS;i++)
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
    delete chains.at("wb_left");
    delete chains.at("wb_right");
}