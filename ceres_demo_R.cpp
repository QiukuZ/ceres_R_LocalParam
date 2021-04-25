#include "ceres/ceres.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

using namespace std;

class RLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 9; };
    virtual int LocalSize() const { return 3; };
};

bool RLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_new(x_plus_delta);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(x);
    Eigen::Map<const Eigen::Vector3d> dT(delta);
    Sophus::SO3<double> dphi;
    dphi =  Sophus::SO3<double>::exp(dT);
    R_new = dphi.matrix() * R;
    return true;
}

bool RLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> j(jacobian);
    j.topRows<3>().setIdentity();
    j.bottomRows<6>().setZero();
    return true;
}

class RpCostFun : public ceres::SizedCostFunction<3,9> {
public:
    RpCostFun(const Eigen::Vector3d p, const Eigen::Vector3d pt) : x_(p[0]), y_(p[1]), z_(p[2]), xt_(pt[0]), yt_(pt[1]), zt_(pt[2]) {}
    virtual ~RpCostFun() {}
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(parameters[0]);

        const Eigen::Vector3d po(x_, y_, z_);
        const Eigen::Vector3d pt(xt_, yt_, zt_);
        const Eigen::Vector3d p = R * po;
        
        Eigen::Vector3d dp = p - pt;
        residuals[0] = dp[0];
        residuals[1] = dp[1];
        residuals[2] = dp[2];

        Eigen::Matrix3d Rpo;
        Rpo << 0, -p[2], p[1],
               p[2], 0, -p[0],
               -p[1], p[0], 0;

        if (jacobians)
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor> > J(jacobians[0]);
                J.setZero();
                J.block<3,3>(0,0 ) = - Rpo;
            }
        }

    return true;
    }

private:
    const double x_;
    const double y_;
    const double z_;

    const double xt_;
    const double yt_;
    const double zt_;
 };

int main(int argc, char const *argv[])
{   
    // Init
    Eigen::Matrix3d R_init;
    R_init << -0.3649686492030002, 0.9297555845003839, -0.04850227669832963,
              -0.22443096743320423, -0.037300048179599976, 0.973775883907204,
               0.9035644186285601, 0.3662830658283426, 0.22227927407224402;
    
    double r[9];
    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 3; j++)
            r[ i*3 + j ] = R_init(i, j);

    // Map r and R
    Eigen::Map<Eigen::Matrix<double,3 ,3, Eigen::RowMajor>> R(r);

    // Target R
    Eigen::Matrix3d Rt;
    Rt << 0,1,0,0,0,1,1,0,0;

    // Data
    Eigen::Vector3d Po(1,2,3);
    Eigen::Vector3d Pt = Rt*Po;

    // ceres Problem 
    ceres::Problem problem;
    ceres::LocalParameterization *local_parameterization = new RLocalParameterization();

    ceres::CostFunction *cost_function = new RpCostFun( Po, Pt);
    problem.AddResidualBlock(cost_function, NULL, r);
    problem.SetParameterization(r, local_parameterization);

    // Run the solver
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;
    
    // Display Result
    cout << endl;
    cout << "Init R = " << endl << R_init << endl;
    cout << "Optim R = " << endl << R << endl;

    return 0;
}
