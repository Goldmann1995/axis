#ifndef AXXBSOLVER_H
#define AXXBSOLVER_H

#include<Eigen/Core>
#include <Eigen/Geometry>

#include"type.h"
#include<glog/logging.h>
#include <iostream>
//used for hand eye calibration
class AXXBSolver
{
public:
  AXXBSolver();
  AXXBSolver(const Poses A, const Poses B):A_(A),B_(B)
  {
    if (A_.size()!=B_.size())
    {
      size_t min = A_.size() > B_.size()?B_.size():A_.size();
      A_ = Poses{A_.begin(),A_.begin()+min};
      B_ = Poses{B_.begin(),B_.begin()+min};
    }
    CHECK(A_.size()==B_.size())<<"two sizes should be equal";

    
    CHECK(A_.size()>=2)<<"at least two motions are needed";
  }

  virtual Pose SolveX()=0;

  Poses A_;
  Poses B_;
};

#endif // AXXBSOLVER_H
