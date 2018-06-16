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



#ifndef RS_SFM_NONLINEARREFINMENT_H
#define RS_SFM_NONLINEARREFINMENT_H

#include <Eigen/Core>
#include <ceres/ceres.h>
//#include "minimal.h"

struct Velocities;
struct RansacValues;


namespace nonlinear_refinement {

    /**
    * Struct implementing the residual for the ceres solver
    */
    struct RsResidual {
        RsResidual(double coordinates[], double observed_velocity[], double alpha, double alphaK) :
                alpha_(alpha), alphaK_(alphaK) {
            for (int i = 0; i < 2; ++i) {
                coordinates_[i] = coordinates[i];
                observed_velocity_[i] = observed_velocity[i];
            }
        }

        template<typename T>
        bool
        operator()(const T *const lin_velocity, const T *const angl_velocity, const T *const beta,
                   const T *const inv_depth, T *residuals) const;

    private:
        double coordinates_[2];
        double observed_velocity_[2];
        double alpha_;
        double alphaK_;
    };

    /**
     * Uses the ceres solver for estimating the inverse depth value. This method estimates a single depth value. The
     * rsult is an algebraic optimum.
     *
     * @param normalized_coordinates (2 x 1) vector containing the image coordinates normalized to [-1, 1]
     * @param linear_velocity (3 x 1) vector containing the linear velocity in (x, y, z) direction
     * @param angular_velocity (3 x 1) vector containing the angular velocity
     * @param flow (2 x 1) vector containing the relative pixel displacement (dx, dy)
     * @param k (double) constant acceleration parameter
     * @param alpha (double) the alpha parameter
     * @param alphaK (double) the alphaK parameter
     * @param show_messages (bool) true if the debug messages should be printed
     * @return inverse_depth (double) estimated inverse depth
     */
    double estimateInverseDepth(const Eigen::Vector2d &normalized_coordinates, const Eigen::Vector3d &linear_velocity,
                                const Eigen::Vector3d &angular_velocity, const Eigen::Vector2d &flow,
                                const double& k, const double& alpha, const double& alphaK, bool show_messages);

    /**
     * Uses the ceres solver for estimating the inverse depth of the given set. This method estimates the depth value
     * for the given set of points. Result is an algebraic optimum.
     *
     * @param normalized_coordinates rows containing the normalized image coordinates [-1, 1]
     * @param flow rows containing the relative pixel displacements
     * @param linear_velocity (3 x 1) vector containing the linear velocity in (x, y, z) direction
     * @param angular_velocity (3 x 1) vector containing the angular velocit
     * @param k (double) constant acceleration parameter
     * @param alpha (n x 1) the alpha parameters
     * @param alphaK(n x 1) the alphaK parameters
     * @param show_messages (bool) true if the debug messages should be printed
     * @return inverse_depth (Eigen Array) containing the estimated inverse depth values
     */
    Eigen::ArrayXd estimateInverseDepths(const Eigen::Array2Xd& normalized_coordinates, const Eigen::Array2Xd& flow,
                                              const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity,
                                              const double& k, const Eigen::ArrayXd& alpha, const Eigen::ArrayXd& alphaK,
                                              bool show_messages);

    /**
     * Uses the ceres solver to optimize the nonlinear refinement over all the given input parameters. This minimizes
     * the geometric reprojection error.
     *
     * @param flow std::vector<(2x1) vector> containing the relative pixel displacements stored in an Eigen vector.
     * @param inliers (struct RansacValues) containing the inliers set from the RANSAC (linear velocity, angular velocity
     *        beta and inverse depth)
     * @param const_acceleration (bool) true if constant acceleration assumption holds, k is also part of optimization
     * @param show_messages (bool) true if the debug messages should be printed
     * @return (struct RansacValues) optimized inliers set
     */
    RansacValues nonLinearRefinement(const Eigen::Array2Xd &flow, const RansacValues& inliers, bool const_velocity, bool show_messages);

}//nonlinear_refinement
#endif //RS_SFM_NONLINEARREFINMENT_H

