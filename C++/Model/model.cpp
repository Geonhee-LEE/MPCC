// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "model.h"
namespace mpcc{
Model::Model()
:Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Model::Model(double Ts,const PathToJson &path)
:Ts_(Ts),param_(Param(path.param_path))
{
}

/* double Model::getSlipAngleFront(const State &x) const
{
    // compute slip angels given current state
    return -std::atan2(x.vy+x.r*param_.lf,x.vx) + x.delta;
} 

double Model::getSlipAngleRear(const State &x) const
{
    // compute slip angels given current state
    return -std::atan2(x.vy-x.r*param_.lr,x.vx);
}

TireForces Model::getForceFront(const State &x) const
{
    const double alpha_f = getSlipAngleFront(x);
    const double F_y = param_.Df * std::sin(param_.Cf * std::atan(param_.Bf * alpha_f ));
    const double F_x = 0.0;

    return {F_y,F_x};
}

TireForces Model::getForceRear(const State &x) const
{
    const double alpha_r = getSlipAngleRear(x);
    const double F_y = param_.Dr * std::sin(param_.Cr * std::atan(param_.Br * alpha_r ));
    const double F_x = param_.Cm1*x.D - param_.Cm2*x.D*x.vx;// - param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);

    return {F_y,F_x};
}

double Model::getForceFriction(const State &x) const
{
    return -param_.Cr0 - param_.Cr2*std::pow(x.vx,2.0);
}

NormalForces Model::getForceNormal(const State &x) const
{
    // at this point aero forces could be modeled
    const double f_n_front = param_.lr/(param_.lf + param_.lr)*param_.m*param_.g;
    const double f_n_rear = param_.lf/(param_.lf + param_.lr)*param_.m*param_.g;
    return {f_n_front,f_n_rear};
}

TireForcesDerivatives Model::getForceFrontDerivatives(const State &x) const
{
    const double alpha_f = getSlipAngleFront(x);
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;

    // F_fx
    const double dF_x_vx    = 0.0;
    const double dF_x_vy    = 0.0;
    const double dF_x_r     = 0.0;
    const double dF_x_D     = 0.0;
    const double dF_x_delta = 0.0;
    // F_fy
    const double dF_y_vx    = (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
                                            /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2))*((param_.lf*r + vy)
                                            /(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
    const double dF_y_vy    = (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
                                            /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2))
                                            *(-vx/(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
    const double dF_y_r     =  (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
                                            /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2))*((-param_.lf*vx)
                                            /(std::pow((param_.lf*r + vy),2)+std::pow(vx,2)));
    const double dF_y_D     =  0.0;
    const double dF_y_delta = (param_.Bf*param_.Cf*param_.Df*std::cos(param_.Cf*std::atan(param_.Bf*alpha_f)))
                                            /(1.+std::pow(param_.Bf,2)*std::pow(alpha_f,2));

    return {dF_y_vx,dF_y_vy,dF_y_r,dF_y_D,dF_y_delta,dF_x_vx,dF_x_vy,dF_x_r,dF_x_D,dF_x_delta};
}

TireForcesDerivatives Model::getForceRearDerivatives(const State &x) const
{
    const double alpha_r = getSlipAngleRear(x);
    const double vx = x.vx;
    const double vy = x.vy;
    const double r  = x.r;
    const double D  = x.D;

    //F_rx
    const double dF_x_vx    = -param_.Cm2*D;// - 2.0*param_.Cr2*vx;
    const double dF_x_vy    = 0.0;
    const double dF_x_r     = 0.0;
    const double dF_x_D     = param_.Cm1 - param_.Cm2*vx;
    const double dF_x_delta = 0.0;
    // F_ry
    const double dF_y_vx    = ((param_.Br*param_.Cr*param_.Dr*std::cos(param_.Cr*std::atan(param_.Br*alpha_r)))
                                            /(1.+std::pow(param_.Br,2)*std::pow(alpha_r,2)))*(-(param_.lr*r - vy)
                                            /(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
    const double dF_y_vy    = ((param_.Br*param_.Cr*param_.Dr*std::cos(param_.Cr*std::atan(param_.Br*alpha_r)))
                                            /(1.+std::pow(param_.Br,2)*std::pow(alpha_r,2)))
                                            *((-vx)/(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
    const double dF_y_r     = ((param_.Br*param_.Cr*param_.Dr*std::cos(param_.Cr*std::atan(param_.Br*alpha_r)))
                                            /(1.+std::pow(param_.Br,2)*std::pow(alpha_r,2)))*((param_.lr*vx)
                                            /(std::pow((-param_.lr*r + vy),2)+std::pow(vx,2)));
    const double dF_y_D     = 0.0;
    const double dF_y_delta = 0.0;

    return {dF_y_vx,dF_y_vy,dF_y_r,dF_y_D,dF_y_delta,dF_x_vx,dF_x_vy,dF_x_r,dF_x_D,dF_x_delta};
}

FrictionForceDerivatives Model::getForceFrictionDerivatives(const State &x) const
{
    return {-2.0*param_.Cr2*x.vx,0.0,0.0,0.0,0.0};
}
*/

StateVector Model::getF(const State &x,const Input &u) const
{
    const double phi = x.phi;
    const double vx = x.vx;
    const double vs = x.vs;

    const double dVx = u.dVx;
    const double dPhi = u.dPhi;
    const double dVs = u.dVs;

    //const TireForces tire_forces_front = getForceFront(x);
    //const TireForces tire_forces_rear  = getForceRear(x);
    //const double friction_force = getForceFriction(x);

    StateVector f;

    f(0) = vx*std::cos(phi); 
    f(1) = vx*std::sin(phi);
    f(2) = dPhi;
    f(3) = vs;
    f(4) = dVx;
    f(5) = dVs;

    /*     
    f(0) = vx*std::cos(phi) - vy*std::sin(phi);
    f(1) = vy*std::cos(phi) + vx*std::sin(phi);
    f(2) = r;
    f(3) = 1.0/param_.m*(tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*std::sin(delta) + param_.m*vy*r);
    f(4) = 1.0/param_.m*(tire_forces_rear.F_y + tire_forces_front.F_y*std::cos(delta) - param_.m*vx*r);
    f(5) = 1.0/param_.Iz*(tire_forces_front.F_y*param_.lf*std::cos(delta) - tire_forces_rear.F_y*param_.lr);
    f(6) = vs;
    f(7) = dD;
    f(8) = dDelta;
    f(9) = dVs;
    */

    return f;
}

LinModelMatrix Model::getModelJacobian(const State &x, const Input &u) const
{
    // compute jacobian of the model
    // state values
    const double phi = x.phi;
    const double vx = x.vx;

//    LinModelMatrix lin_model_c;
    A_MPC A_c = A_MPC::Zero();
    B_MPC B_c = B_MPC::Zero();
    g_MPC g_c = g_MPC::Zero();

    const StateVector f = getF(x,u);

    // Derivatives of function
    // f1 = v_x*std::cos(phi)
    const double df1_dphi = -vx*std::sin(phi);
    const double df1_dvx  = std::cos(phi);

    // f2 = v_x*std::sin(phi);
    const double df2_dphi = vx*std::cos(phi);
    const double df2_dvx  = std::sin(phi);

    // f3 = delta phi;

    // f4 = s
    const double df4_dvs  = 1;

    // f5 = delta vx

    // f6 = delta vs

    // Jacobians
    // Matrix A
    // Column 1
    // all zero
    // Column 2
    // all zero
    // Column 3
    A_c(0,2) = df1_dphi;
    A_c(1,2) = df2_dphi;
    // Column 4
    // all zero
    // Column 5
    A_c(0,4) = df1_dvx;
    A_c(1,4) = df2_dvx;
    // Column 6
    A_c(3,5) = df4_dvs;

    // Matrix B
    // Column 1
    B_c(4,0) = 1.0;
    // Column 2
    B_c(2,1) = 1.0;
    // Column 3
    B_c(5,2) = 1.0;

    //zero order term
    g_c = f - A_c*stateToVector(x) - B_c*inputToVector(u);

    return {A_c,B_c,g_c};
}

LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c) const
{
    // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
    Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp = Eigen::Matrix<double,NX+NU+1,NX+NU+1>::Zero();
    // building matrix necessary for expm
    // temp = Ts*[A,B,g;zeros]
    temp.block<NX,NX>(0,0) = lin_model_c.A;
    temp.block<NX,NU>(0,NX) = lin_model_c.B;
    temp.block<NX,1>(0,NX+NU) = lin_model_c.g;
    temp = temp*Ts_;
    // take the matrix exponential of temp
    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_res = temp.exp();
    // extract dynamics out of big matrix
    // x_{k+1} = Ad x_k + Bd u_k + gd
    //temp_res = [Ad,Bd,gd;zeros]
    const A_MPC A_d = temp_res.block<NX,NX>(0,0);
    const B_MPC B_d = temp_res.block<NX,NU>(0,NX);
    const g_MPC g_d = temp_res.block<NX,1>(0,NX+NU);

    return {A_d,B_d,g_d};
}

//LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c) const
//{
//    // disctetize the continuous time linear model \dot x = A x + B u + g using ZHO
//    Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp = Eigen::Matrix<double,NX+NU+1,NX+NU+1>::Zero();
//    // building matrix necessary for expm
//    // temp = Ts*[A,B,g;zeros]
//    temp.block<NX,NX>(0,0) = lin_model_c.A;
//    temp.block<NX,NU>(0,NX) = lin_model_c.B;
//    temp.block<NX,1>(0,NX+NU) = lin_model_c.g;
//    temp = temp*TS;
//    Eigen::Matrix<double,NX+NU+1,NX+NU+1> eye;
//    eye.setIdentity();
//    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_mult = temp * temp;
//
//    const Eigen::Matrix<double,NX+NU+1,NX+NU+1> temp_res = eye + temp + 1./2.0 * temp_mult + 1./6.0 * temp_mult * temp;
//
//    // x_{k+1} = Ad x_k + Bd u_k + gd
//    const A_MPC A_d = temp_res.block<NX,NX>(0,0);
//    const B_MPC B_d = temp_res.block<NX,NU>(0,NX);
//    const g_MPC g_d = temp_res.block<NX,1>(0,NX+NU);
//
//    return {A_d,B_d,g_d};
//
//}

LinModelMatrix Model::getLinModel(const State &x, const Input &u) const
{
    // compute linearized and discretized model
    const LinModelMatrix lin_model_c = getModelJacobian(x,u);
    // discretize the system
    return discretizeModel(lin_model_c);
}
}