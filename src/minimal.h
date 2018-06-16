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


Created on 12.04.2018
    Author: Manuel Fritsche
  Modified: Thomas Ziegler

*/

#ifndef INC_3DVMINIMAL_MINIMAL_H
#define INC_3DVMINIMAL_MINIMAL_H

#endif //INC_3DVMINIMAL_MINIMAL_H


#define _USE_MATH_DEFINES

#include "Eigen/Dense"


typedef Eigen::Matrix<double, 9, 1> VectorE;
typedef Eigen::Matrix<double, 9, 9> MatrixZ;

/**
     * Used for the return value of calculateVelocities
     */
struct Velocities {
    Eigen::Vector3d w;
    Eigen::Vector3d v;
    double k;

    Velocities(Eigen::Vector3d w_init, Eigen::Vector3d v_init) :
            w(w_init), v(v_init), k(0) {}

    Velocities(Eigen::Vector3d w_init, Eigen::Vector3d v_init, double k_init) :
            w(w_init), v(v_init), k(k_init) {}
};


/**
 * Used for the return value of ransac
 */
struct RansacValues {
    int num_inliers;
    Eigen::Array3Xd inliers;
    Eigen::VectorXd beta;
    Eigen::VectorXd alpha;
    Eigen::VectorXd alpha_k;
    Eigen::Vector3d w;
    Eigen::Vector3d v;
    double k;

    RansacValues(int num_inliers_init, Eigen::Array3Xd inliers_init, Eigen::VectorXd beta_init, Eigen::Vector3d w_init,
                 Eigen::Vector3d v_init) :
            inliers(inliers_init), num_inliers(num_inliers_init), beta(beta_init), alpha(beta_init), alpha_k(beta_init),
            w(w_init), v(v_init), k(0) {}

    RansacValues(int num_inliers_init, Eigen::Array3Xd inliers_init, Eigen::VectorXd alpha_init,
                 Eigen::VectorXd alpha_k_init, Eigen::Vector3d w_init, Eigen::Vector3d v_init, double k_init) :
            inliers(inliers_init), num_inliers(num_inliers_init), beta(alpha_init), alpha(alpha_init),
            alpha_k(alpha_k_init), w(w_init), v(v_init), k(k_init) {}
};


namespace minimal {

    /**
     * Returns the angular velocity omega (w) and linear velocity v of equation (1). Also returns k
     *
     * @param q (2 x 9) Array2Xd containing the 2D x vectors
     * @param u (2 x 9) Array2Xd containing the 2D optical flow vectors
     * @param alpha (9) ArrayXd containing the calculated alpha values
     * @param alphaK (9) ArrayXd containing the factor of the k part in beta
     * @param useAlphaK True for constant acceleration, False for constant velocity
     * @return OmegaV struct containing the angular velocity w, linear velocity v and k
     */
    Velocities calculateVelocities(const Eigen::Array2Xd& q, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha,
                                   const Eigen::ArrayXd& alpha_k, bool use_alpha_k);


    /**
     * Returns the alpha parameters
     *
     * @param q (2 x n) Array containing the points for which alpha should be calculated
     * @param h the maximum distance between a y and yRef possible. i.e. h = max(q(1, i)/q(2, i) - qRef(1)/qRef(2)) if a point with max y is part of q
     * @param gamma (double) readout time ratio
     * @return alpha (n x 1) the alpha parameters
     */
    //Eigen::ArrayXd getAlpha(const Eigen::Array2Xd& q, double h, double gamma);
    Eigen::ArrayXd getAlpha(const Eigen::Array2Xd& flow, double h, double gamma);

    /**
     * Returns the alphaK parameters, which is the factor in front of k in beta
     *
     * @param q (2 x n) Array containing the points for which alpha should be calculated
     * @param h the maximum distance between a y and yRef possible. i.e. h = max(q(1, i)/q(2, i) - qRef(1)/qRef(2)) if a point with max y is part of q
     * @param gamma (double) readout time ratio
     * @return alpha (n x 1) the alphaK parameters
     */
    //Eigen::ArrayXd getAlphaK(const Eigen::Array2Xd& q, double h, double gamma);
    Eigen::ArrayXd getAlphaK(const Eigen::Array2Xd& q, const Eigen::Array2Xd& flow, double h, double gamma);

    /**
     * Returns the consensus set and velocities after using ransac
     *
     * @param q (2 x 9) Array2Xd containing the 2D x vectors
     * @param u (2 x 9) Array2Xd containing the 2D optical flow vectors
     * @param alpha (9) ArrayXd containing the calculated alpha values
     * @param alphaK (9) ArrayXd containing the calculated alphaK values: beta = alpha + k * alphaK
     * @param useAlphaK True for constant acceleration, False for constant velocity
     * @param iterations the number of ransac trails
     * @param tolerance the maximum deviation that is allowed for a point to be an inlier
     * @param show_messages true if the debug messages should be printed
     * @return the consensus set, the velocities and the inverse depths that were estimated with RANSAC
     */
    RansacValues ransac(const Eigen::Array2Xd& x, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha,
                        const Eigen::ArrayXd& alpha_k, bool use_alpha_k, int iterations, double tolerance, bool schow_messages);

    /**
     * Returns the consensus set and velocities after using ransac
     *
     * @param q (2 x 9) Array2Xd containing the 2D x vectors
     * @param u (2 x 9) Array2Xd containing the 2D optical flow vectors
     * @param alpha (9) ArrayXd containing the calculated alpha values
     * @param iterations the number of ransac trails
     * @param tolerance the maximum deviation that is allowed for a point to be an inlier
     * @param show_messages true if the debug messages should be printed
     * @return the consensus set, the velocities and the inverse depths that were estimated with RANSAC
     */
    RansacValues ransac(const Eigen::Array2Xd& x, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha, int iterations,
                        double tolerance, bool schow_messages);

    /**
     * Returns the consensus set and velocities after using ransac
     *
     * @param q (2 x 9) Array2Xd containing the 2D x vectors
     * @param u (2 x 9) Array2Xd containing the 2D optical flow vectors
     * @param alpha (9) ArrayXd containing the calculated alpha values
     * @param alphaK (9) ArrayXd containing the calculated alphaK values: beta = alpha + k * alphaK
     * @param iterations the number of ransac trails
     * @param tolerance the maximum deviation that is allowed for a point to be an inlier
     * @param show_messages true if the debug messages should be printed
     * @return the consensus set, the velocities and the inverse depths that were estimated with RANSAC
     */
    RansacValues ransac(const Eigen::Array2Xd& x, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha,
                        const Eigen::ArrayXd& alpha_k, int iterations, double tolerance, bool schow_messages);
}