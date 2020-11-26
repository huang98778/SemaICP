/*
 * @Author: your name
 * @Date: 2020-10-23 13:20:21
 * @LastEditTime: 2020-10-23 13:20:21
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /semICP/sqloss.h
 */
#ifndef __SQ_LOSS_H__
#define __SQ_LOSS_H__

#include<ceres/loss_function.h>

#include<cmath>
#include<math.h>

namespace SemanticICP {

class SQLoss : public ceres::LossFunction {
  public:
    void Evaluate(double s, double rho[3]) const {
     double v = s + std::numeric_limits<double>::epsilon();
     rho[0] = std::sqrt(v);
     rho[1] = 1.0/(2.0*std::sqrt(v));
     rho[2] = -1.0/(4.0*std::pow(v,1.5));
    }
};

}
#endif // _SQ_LOSS_H_
