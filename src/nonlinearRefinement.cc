/*
Copyright (C) 2018 Manuel Fritsche, Felix Graule, Thomas Ziegler

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


Created on 04.04.2018
    Author: Thomas Ziegler
  Modified: Manuel Fritsche

*/

#include "nonlinearRefinement.h"
#include "minimal.h"
#include <Eigen/Core>
#include <ceres/ceres.h>

namespace nonlinear_refinement {

    // Ceres Residual
    template<typename T>
    bool RsResidual::operator()(const T *const lin_velocity, const T *const angl_velocity, const T *const k,
                                 const T *const inv_depth, T *residuals) const {
        T predicted_velocity[2];
        T beta = (T(2) / (T(2) + (*k)))*(T(alpha_) + (*k) * T(alphaK_));

        predicted_velocity[0] = beta * T(-1) * (*inv_depth * (T(coordinates_[0]) * lin_velocity[2] - lin_velocity[0]) +
                                                 (T(coordinates_[0]) * T(coordinates_[1]) * angl_velocity[0]) -
                                                 (T(1) + T(coordinates_[0]) * T(coordinates_[0])) * angl_velocity[1]
                                                 + T(coordinates_[1]) * angl_velocity[2]);

        predicted_velocity[1] = beta * T(-1) * (*inv_depth * (T(coordinates_[1]) * lin_velocity[2] - lin_velocity[1]) +
                                                 (T(1) + T(coordinates_[1]) * T(coordinates_[1])) * angl_velocity[0] -
                                                 T(coordinates_[0]) * T(coordinates_[1]) * angl_velocity[1]
                                                 - T(coordinates_[0]) * angl_velocity[2]);

        residuals[0] = T(observed_velocity_[0]) - predicted_velocity[0];
        residuals[1] = T(observed_velocity_[1]) - predicted_velocity[1];

        return true;
    }

    // inverse depth estimation for a single pixel
    double estimateInverseDepth(const Eigen::Vector2d &normalized_coordinates, const Eigen::Vector3d &linear_velocity,
                                const Eigen::Vector3d &angular_velocity, const Eigen::Vector2d &flow, const double& k,
                                const double& alpha, const double& alphaK, bool show_messages) {

        double lin_vel_double[linear_velocity.size()];
        double ang_vel_double[angular_velocity.size()];
        double coordinates[2];
        double flow_vec[2];

        // need non const variable for ceres
        double k_local = k;

        // initialize depth value
        double inverse_depth = 1.0;

        // map Eigen vectors, arrays into double arrays
        Eigen::Map<Eigen::MatrixXd>(lin_vel_double, linear_velocity.rows(), linear_velocity.cols()) = linear_velocity;
        Eigen::Map<Eigen::MatrixXd>(ang_vel_double, angular_velocity.rows(),
                                    angular_velocity.cols()) = angular_velocity;
        coordinates[0] = normalized_coordinates.coeff(0);
        coordinates[1] = normalized_coordinates.coeff(1);
        flow_vec[0] = flow.coeff(0);
        flow_vec[1] = flow.coeff(1);


        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        //loss_function = new ceres::HuberLoss(1.0);
        //loss_function = new ceres::CauchyLoss(1.0);
        loss_function = nullptr;

        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<RsResidual, 2, 3, 3, 1, 1>(
                new RsResidual(coordinates, flow_vec, alpha, alphaK));

        problem.AddResidualBlock(cost_function, loss_function, lin_vel_double, ang_vel_double, &k_local, &inverse_depth);

        // set parameters as constants -> only optimize over depth value
        problem.SetParameterBlockConstant(lin_vel_double);
        problem.SetParameterBlockConstant(ang_vel_double);
        problem.SetParameterBlockConstant(&k_local);

        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (show_messages) {
            std::cout << summary.BriefReport() << std::endl;
            std::cout << inverse_depth << std::endl;
        }

        return inverse_depth;
    }

    // inverse depth estimation for an array of pixels
    Eigen::ArrayXd estimateInverseDepths(const Eigen::Array2Xd &normalized_coordinates, const Eigen::Array2Xd &flow,
                                              const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity,
                                              const double& k, const Eigen::ArrayXd& alpha, const Eigen::ArrayXd& alphaK,
                                              bool show_messages) {

        double lin_vel_double[linear_velocity.size()];
        double ang_vel_double[angular_velocity.size()];

        // store depth values on the heap
        auto inverse_depth = new double[normalized_coordinates.cols()];

        // need non const variable for ceres
        double k_local = k;

        // map Eigen vectors into double arrays
        Eigen::Map<Eigen::MatrixXd>(lin_vel_double, linear_velocity.rows(),
                                    linear_velocity.cols()) = linear_velocity;
        Eigen::Map<Eigen::MatrixXd>(ang_vel_double, angular_velocity.rows(),
                                    angular_velocity.cols()) = angular_velocity;


        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        //loss_function = new ceres::HuberLoss(1.0);
        //loss_function = new ceres::CauchyLoss(1.0);
        loss_function = nullptr;
        for (int i = 0; i < normalized_coordinates.cols(); ++i) {
            double coordinates[2];
            double flow_vec[2];

            // initalize depth value
            inverse_depth[i] = 1.0;

            // map Eigen array into double array
            coordinates[0] = normalized_coordinates(0, i);
            coordinates[1] = normalized_coordinates(1, i);
            flow_vec[0] = flow(0, i);
            flow_vec[1] = flow(1, i);

            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<RsResidual, 2, 3, 3, 1, 1>(
                    new RsResidual(coordinates, flow_vec, alpha(i), alphaK(i)));
            problem.AddResidualBlock(cost_function, loss_function, lin_vel_double, ang_vel_double, &k_local,
                                     &inverse_depth[i]);


        }
        // set parameters as constants -> only optimize over depth values
        problem.SetParameterBlockConstant(&k_local);
        problem.SetParameterBlockConstant(lin_vel_double);
        problem.SetParameterBlockConstant(ang_vel_double);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (show_messages) {
            std::cout << std::endl;
            std::cout << summary.BriefReport() << std::endl;
            std::cout << "Total time for solving optimization: " << summary.total_time_in_seconds << " s" << std::endl;
        }

        // map double array into Eigen array
        Eigen::ArrayXd inverse_depth_array = Eigen::ArrayXd::Zero(normalized_coordinates.cols());
        for (int i=0; i< normalized_coordinates.cols(); ++i)
        {
            inverse_depth_array(i) = inverse_depth[i];
        }
        delete inverse_depth;

        return inverse_depth_array;
    }

    // minimize geometric, reprojection error
    RansacValues nonLinearRefinement(const Eigen::Array2Xd &flow, const RansacValues &inliers, bool const_acceleration, bool show_messages) {

        // extract variables from inlier set
        int num_inliers = inliers.num_inliers;
        Eigen::Vector3d linear_velocity  = inliers.v;
        Eigen::Vector3d angular_velocity  = inliers.w;

        double k = inliers.k;
        double lin_vel_double[3];
        double ang_vel_double[3];

        // map Eigen vectors into double arrays
        Eigen::Map<Eigen::MatrixXd>(lin_vel_double,linear_velocity.rows(),
                                    linear_velocity.cols()) = linear_velocity;
        Eigen::Map<Eigen::MatrixXd>(ang_vel_double, angular_velocity.rows(),
                                    angular_velocity.cols()) = angular_velocity;

        // store inverse depth on heap
        auto inverse_depths = new double[num_inliers];

        ceres::Problem problem;
        for (int i = 0; i < num_inliers; ++i) {
            double coordinates[2];
            double flow_vec[2];

            // map Eigen arrays to double arrays
            coordinates[0] = inliers.inliers(0,i);
            coordinates[1] = inliers.inliers(1,i);
            flow_vec[0] = flow(0, i);
            flow_vec[1] = flow(1, i);
            inverse_depths[i] = 1.0 / inliers.inliers(2,i);

            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<RsResidual, 2, 3, 3, 1, 1>(
                    new RsResidual(coordinates, flow_vec, inliers.alpha(i), inliers.alpha_k(i)));
            problem.AddResidualBlock(cost_function, nullptr, lin_vel_double, ang_vel_double, &k, &inverse_depths[i]);
        }

        // only optimize over k if in constant acceleration assumption
        if (!const_acceleration){
            problem.SetParameterBlockConstant(&k);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (show_messages) {
            std::cout << summary.BriefReport() << std::endl;
            std::cout << "Total time for solving optimization: " << summary.total_time_in_seconds << " s" << std::endl;
            std::cout << std::endl;
        }


        // Map the double arrays back into the Eigen vectors
        linear_velocity << lin_vel_double[0], lin_vel_double[1], lin_vel_double[2];
        angular_velocity << ang_vel_double[0], ang_vel_double[1], ang_vel_double[2];

        Eigen::Array3Xd new_inliers = Eigen::Array3Xd::Zero(3, num_inliers);

        // create new inlier set with optimized variables
        for(int i=0; i<inliers.num_inliers; ++i) {
            new_inliers(0,i) = inliers.inliers(0,i);
            new_inliers(1,i) = inliers.inliers(1,i);
            new_inliers(2,i) = 1.0 / inverse_depths[i];
        }
        delete[] inverse_depths;

        return RansacValues(inliers.num_inliers, new_inliers, inliers.alpha, inliers.alpha_k, angular_velocity, linear_velocity, k);
    }

}//nonlinear_refinement


