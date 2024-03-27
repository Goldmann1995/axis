#include "andreffextendedaxxbsolver.h"
#include "conventionalaxxbsvdsolver.h"
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/LU>
#include <Eigen/QR>
#include <stdlib.h>

Pose AndreffExtendedAXXBSolver::SolveX()
{
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(9*A_.size(),9);
  Eigen::MatrixXd n = Eigen::MatrixXd::Zero(3*A_.size(),4);
  for(int i=0;i<A_.size();i++)
  {
    //extract R,t from homogophy matrix
    Eigen::Matrix3d Ra = A_[i].topLeftCorner(3,3);
    Eigen::Vector3d Ta = A_[i].topRightCorner(3,1);
    Eigen::Matrix3d Rb = B_[i].topLeftCorner(3,3);
//    Eigen::Vector3d Tb = B_[i].topRightCorner(3,1);

    m.block<9,9>(9*i,0) = Eigen::MatrixXd::Identity(9,9) - Eigen::kroneckerProduct(Ra,Rb);
    n.block<3,3>(3*i,0) = Ra-Eigen::MatrixXd::Identity(3,3);
    n.block<3,1>(3*i,3) = Ta;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd( m, Eigen::ComputeFullV | Eigen::ComputeFullU );
  CHECK(svd.computeV())<<"fail to compute V";
  // std::cout <<"svd.singularValues()=" << svd.singularValues()<< "\n";
  // std::cout <<"svd.matrixV()=" << svd.matrixV()<< "\n";

  Eigen::Matrix3d R_alpha;
  R_alpha.row(0) = svd.matrixV().block<3,1>(0,8).transpose();
  R_alpha.row(1) = svd.matrixV().block<3,1>(3,8).transpose();
  R_alpha.row(2) = svd.matrixV().block<3,1>(6,8).transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_r( R_alpha, Eigen::ComputeFullV | Eigen::ComputeFullU );

  std::cout <<"svd_r.singularValues()=" << svd_r.singularValues()<< "\n";
  std::cout <<"svd_r.matrixV()=" << svd_r.matrixV()<< "\n";
  Eigen::Matrix3d svd_V = svd_r.matrixV();
  Eigen::Vector3d angle= svd_V.eulerAngles(2,1,0);
  std::cout <<"angle [y,p,r]=" << angle<< "\n";


  //double a = std::fabs(R_alpha.determinant());
  double det = R_alpha.determinant();
  double alpha = std::pow(std::abs(det),4./3.)/det;
  // std::cout <<"R_alpha ====(AndreffExtendedAXXBSolver) =" << R_alpha<< "\n";
  // std::cout <<"alpha ====(AndreffExtendedAXXBSolver) =" << alpha<< "\n";

  Eigen::HouseholderQR<Eigen::Matrix3d> qr(R_alpha/alpha);
  // std::cout <<"qr ====(AndreffExtendedAXXBSolver) =" << qr<< "\n";

  Pose handeyetransformation = Pose::Identity(4,4);
  Eigen::Matrix3d Q = qr.householderQ();
  Eigen::Matrix3d R = Q.transpose()*R_alpha/alpha;
  // std::cout <<"R ====(AndreffExtendedAXXBSolver) =" << R<< "\n";

  Eigen::Vector3d R_diagonal = R.diagonal();

  Eigen::Matrix3d Rx;
  for(int i=0;i<3;i++)
  {
    Rx.block<3,1>(0,i) = int(R_diagonal(i)>=0?1:-1)*Q.col(i);
  }
  handeyetransformation.topLeftCorner(3,3) = Rx;
  // std::cout <<"Rx(AndreffExtendedAXXBSolver) =" << Rx<< "\n";

  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(3*A_.size(),1);
  for(int i=0;i<A_.size();i++)
  {
    Eigen::Vector3d Tb = B_[i].topRightCorner(3,1);
    b.block<3,1>(3*i,0) = Rx*Tb;
  }


  handeyetransformation.topRightCorner(4,1) = n.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  return handeyetransformation;
}
