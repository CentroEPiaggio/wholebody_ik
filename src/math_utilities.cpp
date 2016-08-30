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

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace yarp::math; //NOTE: otherwise the operators defined within the YARP types do not work!

class math_utilities
{
public:

  /**
   * @brief pseudoInverse
   * @return Pseudo inverse based on SVD decomposition
   */
template<typename _Matrix_Type_>
static _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows())*svd.singularValues().array().abs()(0);
    return svd.matrixV() *
    (svd.singularValues().array().abs() >tolerance).
    select(svd.singularValues().array().inverse(),0).
    matrix().asDiagonal() * 
    svd.matrixU().adjoint();
}

/**
 * @brief pseudoInverseDamped
 * @param lambda scalad damping factor
 * @return Damped pseudo inverse based on SVD decomposition
 */
template<typename _Matrix_Type_>
static _Matrix_Type_ pseudoInverseDamped(const _Matrix_Type_ &a, double lambda, double epsilon = std::numeric_limits<double>::epsilon())
{
  _Matrix_Type_ Sigma;
  _Matrix_Type_ S;
  Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(a.cols(), a.rows())*svd.singularValues().array().abs()(0);
  Sigma = svd.singularValues().array().matrix().asDiagonal();
  _Matrix_Type_ damp = lambda*Eigen::MatrixBase<_Matrix_Type_>::Identity(Sigma.rows(), Sigma.rows()); // Identity matrix times a damping factor
  S = Sigma.transpose()*(Sigma*Sigma.transpose() + damp).inverse();
  return svd.matrixV() *
  S * 
  svd.matrixU().adjoint();
}

/**
 * @brief pseudoInverseQR
 * @param
 * @return Pseudo inverse based on QR decomposition
 */
template<typename _Matrix_Type_>
static _Matrix_Type_ pseudoInverseQR(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::HouseholderQR<_Matrix_Type_> qr(a.transpose());
  // Full rank R
  _Matrix_Type_ Rtot = qr.matrixQR().template triangularView<Eigen::Upper>();
//   // Rank deficient R //FIXME
//   Eigen::FullPivLU<Eigen::MatrixXd>lu_decomp(a.transpose());
//   int Rank = lu_decomp.rank();                 //retrieve rank of matrix
//   if(Rank<R.cols()) //FIXME 
//   {
    //     Eigen::MatrixXd Rtot = qr.matrixQR().topLeftCorner(Rank, Rank).triangularView<Eigen::Upper>();
    //     Eigen::MatrixXd R = Rtot;
    //       R+= Eigen::MatrixBase< Eigen::MatrixXd >::Identity(R.cols(),R.cols())*epsilon;
//   }
  
  _Matrix_Type_ Q = qr.householderQ();
  
//   assert(a.transpose().isApprox(Q*Rtot) && "Q*R in QR decomposition does not return the original matrix. Perhaps Jacobian is singular?");
//   assert(Eigen::MatrixBase< Eigen::MatrixXd >::Identity(Q.rows(),Q.cols()).isApprox(Q.transpose()*Q) && "Q*Q' is not the identity");
  
  return Q.leftCols(Rtot.topRows(Rtot.cols()).cols())*Rtot.topRows(Rtot.cols()).transpose().inverse();
}


static Eigen::Matrix<double,7,6> pseudoInverseQR_76(const Eigen::Matrix<double,6,7> &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::HouseholderQR<Eigen::Matrix<double,7,6>> qr(a.transpose());
  // Full rank R
  Eigen::Matrix<double,7,6> Rtot = qr.matrixQR().template triangularView<Eigen::Upper>();
  //   // Rank deficient R //FIXME
  //   Eigen::FullPivLU<Eigen::MatrixXd>lu_decomp(a.transpose());
  //   int Rank = lu_decomp.rank();                 //retrieve rank of matrix
  //   if(Rank<R.cols()) //FIXME 
  //   {
  //     Eigen::MatrixXd Rtot = qr.matrixQR().topLeftCorner(Rank, Rank).triangularView<Eigen::Upper>();
  //     Eigen::MatrixXd R = Rtot;
  //       R+= Eigen::MatrixBase< Eigen::MatrixXd >::Identity(R.cols(),R.cols())*epsilon;
  //   }
  
  Eigen::Matrix<double,7,7> Q = qr.householderQ();
  
  //   assert(a.transpose().isApprox(Q*Rtot) && "Q*R in QR decomposition does not return the original matrix. Perhaps Jacobian is singular?");
  //   assert(Eigen::MatrixBase< Eigen::MatrixXd >::Identity(Q.rows(),Q.cols()).isApprox(Q.transpose()*Q) && "Q*Q' is not the identity");
  
  return Q.leftCols(Rtot.topRows(Rtot.cols()).cols())*Rtot.topRows(Rtot.cols()).transpose().inverse();
}

static Eigen::Matrix<double,6,6> pseudoInverseQR_66(const Eigen::Matrix<double,6,6> &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::HouseholderQR<Eigen::Matrix<double,6,6>> qr(a.transpose());

  Eigen::Matrix<double,6,6> Rtot = qr.matrixQR().template triangularView<Eigen::Upper>();

  Eigen::Matrix<double,6,6> Q = qr.householderQ();
  
  return Q.leftCols(Rtot.topRows(Rtot.cols()).cols())*Rtot.topRows(Rtot.cols()).transpose().inverse();
}

static Eigen::Matrix<double,31,3> pseudoInverseQR_313(const Eigen::Matrix<double,3,31> &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::HouseholderQR<Eigen::Matrix<double,31,3>> qr(a.transpose());

  Eigen::Matrix<double,31,3> Rtot = qr.matrixQR().template triangularView<Eigen::Upper>();

  Eigen::Matrix<double,31,31> Q = qr.householderQ();
  
  return Q.leftCols(Rtot.topRows(Rtot.cols()).cols())*Rtot.topRows(Rtot.cols()).transpose().inverse();
}

static Eigen::Matrix<double,31,21> pseudoInverseQR_3121(const Eigen::Matrix<double,21,31> &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::HouseholderQR<Eigen::Matrix<double,31,21>> qr(a.transpose());

  Eigen::Matrix<double,31,21> Rtot = qr.matrixQR().template triangularView<Eigen::Upper>();

  Eigen::Matrix<double,31,31> Q = qr.householderQ();

  return Q.leftCols(Rtot.topRows(Rtot.cols()).cols())*Rtot.topRows(Rtot.cols()).transpose().inverse();
}

static inline void vectorKDLToEigen(const KDL::Vector& k, Eigen::Vector3d& e)
{
  e[0]=k.x();
  e[1]=k.y();
  e[2]=k.z();
}

static inline void vectorYARPToEigen(const yarp::sig::Vector& k, Eigen::Vector3d& e)
{
  e[0]=k[0];
  e[1]=k[1];
  e[2]=k[2];
}

static inline void vectorYARPToEigen(const yarp::sig::Vector& k, Eigen::MatrixXd& e)
{
  for(int i=0;i<k.size();i++)
  {
    e(i)=k[i];
  }
}

static inline void vectorYARPToKDL(const yarp::sig::Vector& k, KDL::Vector& e)
{
  e = KDL::Vector(k[0],k[1],k[2]);
}

static inline void vectorYARPToKDL(const yarp::sig::Matrix& k, KDL::Vector& e)
{
  assert((k.cols()== 1 && k.rows()==3));
  e = KDL::Vector(k[0][0],k[1][0],k[2][0]);
}

static inline void vectorKDLToYARP(const KDL::Vector& in, yarp::sig::Vector& out)
{
  out = yarp::sig::Vector(3,in.data);
}

static inline void vectorKDLToYARP(const KDL::Vector& in, yarp::sig::Matrix& out)
{
  out[0][0] = in.x();
  out[1][0] = in.y();
  out[2][0] = in.z();
}

static inline void matrixYARPToEigen(const yarp::sig::Matrix& in, Eigen::MatrixXd& out)
{
    for(int r=0;r<3;r++)
        for(int c=0;c<3;c++)
            out.data()[r*3+c] = in(r,c);
}

static inline void matrixEigenToYARP(const Eigen::MatrixXd& in, yarp::sig::Matrix& out)
{
    for(int r=0;r<in.rows();r++)
        for(int c=0;c<in.cols();c++)
            out(r,c)=in.data()[r*in.cols()+c];
}

static inline void rotationKDLToYarp(const KDL::Rotation& in, yarp::sig::Matrix& out)
{
    for(int r=0;r<3;r++)
        for(int c=0;c<3;c++)
            out.data()[r*3+c] = in(r,c);
}

static inline void rotationYARPToKDL(const yarp::sig::Matrix& in, KDL::Rotation& out)
{
    for(int r=0;r<3;r++)
        for(int c=0;c<3;c++)
            out(r,c) = in.data()[r*3+c];
}

static inline void FrameYARPToKDL(const yarp::sig::Matrix& in, KDL::Frame& out)
{
    rotationYARPToKDL(in.submatrix(0,2,0,2),out.M);
    vectorYARPToKDL(in.submatrix(0,2,3,3),out.p);
}

static inline void FrameKDLToYARP(const KDL::Frame& in, yarp::sig::Matrix& out)
{
    yarp::sig::Matrix pos(3,1);
    yarp::sig::Matrix rot(3,3);
    rotationKDLToYarp(in.M,rot);
    vectorKDLToYARP(in.p,pos);
    out.setSubmatrix(rot,0,0);
    out.setSubmatrix(pos,0,3);
}

template<class Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> & vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}

};
