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

#include <ctime>
#include <cmath>
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "Eigen/Geometry"

#include "nonlinearRefinement.h"
#include "minimal.h"

namespace minimal {
    Velocities calculateVelocities(const Eigen::Array2Xd& q, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha,
                                   const Eigen::ArrayXd& alpha_k, bool use_alpha_k) {
        // set constants
        const double THRESHOLD_LAMBDA = 0.000001;
        const double TOL_IMAG = 0.00001; // tolerance of the imaginary part of k
        // initialize variables
        int n = q.cols();
        double k = 0;
        Eigen::ArrayXd beta = Eigen::ArrayXd::Ones(n);
        Eigen::ArrayXXd z = Eigen::ArrayXXd::Ones(n, 9);
        // calculate Z
        z.col(0) = -u.row(1);
        z.col(1) = u.row(0);
        z.col(2) = u.row(1) * q.row(0) - u.row(0) * q.row(1);
        z.col(3) = q.row(0) * q.row(0);
        z.col(4) = 2.0 * q.row(0) * q.row(1);
        z.col(5) = 2.0 * q.row(0);
        z.col(6) = q.row(1) * q.row(1);
        z.col(7) = 2 * q.row(1);
        if (use_alpha_k) {
            // The following code estimates k from det(Z(k)) = 0. The Problem is reduced to an eigenvalue decomposition
            // build p matrix
            Eigen::MatrixXd a = z.block(0, 0, 3, 3);
            Eigen::MatrixXd a_inv = a.inverse();
            Eigen::MatrixXd efhj = z.block(3,3,6,6); // [E, F; H, J]
            Eigen::MatrixXd dg = z.block(3, 0, 6, 3); // [D; G]
            Eigen::MatrixXd bc = z.block(0, 3, 3, 6); // [B, C]
            Eigen::Vector3d alpha_f3 = alpha.head(3); // first 3 entries of alpha
            Eigen::VectorXd alpha_l6 = alpha.tail(6); // last 6 entries of alpha
            Eigen::MatrixXd p = alpha_l6.asDiagonal() * efhj - dg * a_inv * alpha_f3.asDiagonal() * bc;
            // build PK matrix
            Eigen::Vector3d alpha_kf3 = alpha_k.head(3); // first 3 entries of alpha
            Eigen::VectorXd alpha_kl6 = alpha_k.tail(6); // last 6 entries of alpha
            Eigen::MatrixXd p_k = alpha_kl6.asDiagonal() * efhj - dg * a_inv * alpha_kf3.asDiagonal() * bc;
            // calculate Eigenvalues of p * inv(p_k)
            Eigen::EigenSolver<Eigen::MatrixXd> eig_sol;
            eig_sol.compute(p * p_k.inverse(), false);
            Eigen::VectorXcd k_vals = eig_sol.eigenvalues();
            // calculate real k value with smallest abs(k)
            k = INFINITY;
            for (int i = 0; i < 6; i++) {
                if ((std::abs(k_vals(i).imag()) < TOL_IMAG) && std::abs(k_vals(i).real()) < std::abs(k))  {
                    k = k_vals(i).real();
                }
            }
            // set beta
            beta = (alpha + k * alpha_k) * (2.0 / (2.0 + k));
        }
        else {
            // use alpha instead of beta
            beta = alpha;
        }
        // complete Z matrix
        z.col(3) *= beta;
        z.col(4) *= beta;
        z.col(5) *= beta;
        z.col(6) *= beta;
        z.col(7) *= beta;
        z.col(8) *= beta;
        z.matrix();

        // calculate normalized vector e (Step 1)
        Eigen::JacobiSVD<MatrixZ> svd(z, Eigen::ComputeFullV);
        Eigen::MatrixXd v_e = svd.matrixV();
        // e is the singular vector corresponding to the smallest singular value of z
        VectorE e = v_e.col(8);
        double norm_v0 = sqrt(e(0) * e(0) + e(1) * e(1) + e(2) * e(2));
        e = e / norm_v0;
        // get the velocity v0 and the s matrix from the vector e (Step 1)
        Eigen::Vector3d v0 = Eigen::Vector3d(e(0), e(1), e(2));
        Eigen::Matrix3d s;
        s << e(3), e(4), e(5),
                e(4), e(6), e(7),
                e(5), e(7), e(8);
        // calculate the new eigenvalues (step 2)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_sol = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(s);
        Eigen::Vector3d lamb = eig_sol.eigenvalues();
        Eigen::Matrix3d v1 = eig_sol.eigenvectors();
        v1.col(0).swap(v1.col(2)); // swap column, because order is reversed in algorithm
        Eigen::Vector3d sigma;
        sigma << (2 * lamb(2) + lamb(1) - lamb(0)) / 3,
                (lamb(2) + 2 * lamb(1) + lamb(0)) / 3,
                (-lamb(2) + lamb(1) + 2 * lamb(0)) / 3;
        // calculate the new transformation matrices V (v_) and U (u_) (Step 3)
        double lambda = sigma(0) - sigma(2);
        double theta = 0;
        if (lambda < THRESHOLD_LAMBDA) {
            if (sigma(1) >= THRESHOLD_LAMBDA) {
                std::cout << "Warning: lambda was 0 but sigma(1) was " << sigma(1) << std::endl;
            }
        }
        else {
            theta = acos(-sigma(1) / lambda);
        }
        Eigen::Matrix3d r_v = Eigen::AngleAxisd((theta - M_PI) / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d r_u = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d v_ = v1 * r_v.transpose();
        Eigen::Matrix3d u_ = -v_ * r_u;
        // find the 4 possible vHats corresponding to s (Step 3)
        Eigen::Matrix3d sig1;
        sig1 << 1, 0, 0,
                0, 1, 0,
                0, 0, 0;
        Eigen::Matrix3d sig_lamb = lambda * sig1;
        Eigen::Matrix3d r_z1 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d r_z2 = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d v_hat_v1 = v_ * r_z1 * sig1 * v_.transpose();
        Eigen::Matrix3d v_hat_v2 = v_ * r_z2 * sig1 * v_.transpose();
        Eigen::Matrix3d v_hat_u1 = u_ * r_z1 * sig1 * u_.transpose();
        Eigen::Matrix3d v_hat_u2 = u_ * r_z2 * sig1 * u_.transpose();
        // calculate the optimal vHat, where v_hat' * v0 is max (Step 4)
        Eigen::Matrix<double, 4, 3> v_vecs; // matrix contains the vectors v_hat in the columns
        //  Skew symmetric matrix:  [ 0  -p3  p2]
        //						    [ p3  0  -p1] --> [p1 p2 p3]'
        //							[-p2  p1  0 ]
        v_vecs << v_hat_v1(2, 1), v_hat_v1(0, 2), v_hat_v1(1, 0),
                v_hat_v2(2, 1), v_hat_v2(0, 2), v_hat_v2(1, 0),
                v_hat_u1(2, 1), v_hat_u1(0, 2), v_hat_u1(1, 0),
                v_hat_u2(2, 1), v_hat_u2(0, 2), v_hat_u2(1, 0);
        Eigen::Vector4d v_dot_v0 = v_vecs * v0;
        int index_max;
        v_dot_v0.maxCoeff(&index_max);
        // calculate the omega that corresponds to the optimal vHat (Step 3/4)
        Eigen::Matrix3d w_hat;
        switch (index_max) {
            case 0:
                w_hat = u_ * r_z1 * sig_lamb * u_.transpose();
                break;
            case 1:
                w_hat = u_ * r_z2 * sig_lamb * u_.transpose();
                break;
            case 2:
                w_hat = v_ * r_z1 * sig_lamb * v_.transpose();
                break;
            case 3:
                w_hat = v_ * r_z2 * sig_lamb * v_.transpose();
                break;
        }
        Eigen::Vector3d w = Eigen::Vector3d(w_hat(2, 1), w_hat(0, 2), w_hat(1, 0));
        // w and the original v0 are chosen as results
        return Velocities(w, v0, k);
    }

    Eigen::ArrayXd getAlpha(const Eigen::Array2Xd& flow, double h, double gamma) {
        int n = flow.cols();
        Eigen::ArrayXd alpha = Eigen::ArrayXd::Ones(n);
        for (int i = 0; i < n; i++) {
            alpha(i) = 1 + gamma * flow(1,i) / double(h);
        }
        return alpha;
    }

    Eigen::ArrayXd getAlphaK(const Eigen::Array2Xd& q, const Eigen::Array2Xd& flow, double h, double gamma) {
        int n = q.cols();
        Eigen::ArrayXd alpha_k = Eigen::ArrayXd::Ones(n);
        for (int i = 0; i < n; i++) {
            double part1 = gamma * q(1,i) / double(h);
            double part2 = 1.0 + gamma * (q(1, i) + flow(1,i)) / double(h);
            alpha_k(i) = 0.5 * (part2 * part2 - part1 * part1);
        }
        return alpha_k;
    }

    RansacValues ransac(const Eigen::Array2Xd& q, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha,
                        int iterations, double tolerance, bool show_messages) {
        return ransac(q, u, alpha, alpha, false, iterations, tolerance, show_messages);
    }

    RansacValues ransac(const Eigen::Array2Xd& q, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha,
                        const Eigen::ArrayXd& alpha_k, int iterations, double tolerance, bool show_messages) {
        return ransac(q, u, alpha, alpha_k, true, iterations, tolerance, show_messages);
    }

    RansacValues ransac(const Eigen::Array2Xd& q, const Eigen::Array2Xd& u, const Eigen::ArrayXd& alpha,
                        const Eigen::ArrayXd& alpha_k, bool use_alpha_k, int iterations, double tolerance, bool show_messages) {
        int n = q.cols();

        // initialize arrays
        Eigen::Array<double, 2, 9> choice_q = Eigen::Array<double, 2, 9>::Zero();
        Eigen::Array<double, 2, 9> choice_u = Eigen::Array<double, 2, 9>::Zero();
        Eigen::ArrayXd choice_alpha = Eigen::ArrayXd::Zero(9);
        Eigen::ArrayXd choice_alpha_k = Eigen::ArrayXd::Zero(9);
        std::vector<double> choice_k(n);
        Eigen::ArrayXd inv_depth = Eigen::ArrayXd::Zero(n);
        Eigen::ArrayXd inv_depth_best = Eigen::ArrayXd::Zero(n);
        std::vector<bool> inliers_best(n);
        Velocities vel_best(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
        int num_inliers_best = -1;
        double inlier_error_best = 0;
        // create index vector that is used to pick the columns
        std::vector<int> indices(n);
        for (int i = 0; i < n; i++) {
            indices[i] = i;
        }
        for (int i = 0; i < iterations; i++) {
            // pick 9 distinct points at random
            srand(time(NULL));
            int n_temp = n;
            for (int j = 0; j < 9; j++) {
                // make sure to pick the elements distinctly --> don't reuse elements in indices
                int random_choice = rand() % n_temp;
                std::swap(indices[n_temp - 1], indices[random_choice]);
                int index = indices[n_temp - 1];
                n_temp--; // do not reuse the element that was swaped previously
                choice_q.col(j) = q.col(index);
                choice_u.col(j) = u.col(index);
                choice_alpha(j) = alpha(index);
                choice_alpha_k(j) = alpha_k(index);
            }

            // fit the Velocities
            Velocities vel = calculateVelocities(choice_q, choice_u, choice_alpha, choice_alpha_k, use_alpha_k);
            // find the inliers
            std::vector<bool> inliers(n);
            int num_inliers = 0;
            double inlier_error = 0;
            // calculate the inverse depth
            std::vector<double> betas(n);
            inv_depth = nonlinear_refinement::estimateInverseDepths(q, u, vel.v, vel.w, vel.k, alpha, alpha_k, show_messages);
            for (int j = 0; j < n; j++) {
                // calculate the optical flow given w and v
                double x = q(0, j);
                double y = q(1, j);
                Eigen::Matrix<double, 2, 3> A;
                A << 1, 0, -x,
                        0, 1, -y;
                Eigen::Matrix<double, 2, 3> B;
                B << -x * y, (1 + x * x), -y,
                        -(1 + y * y), x * y, x;
                double beta = (alpha(j) + vel.k * alpha_k(j))*(2.0/(2.0+vel.k));
                Eigen::Vector2d u_est = beta * (A * vel.v * inv_depth(j) + B * vel.w);
                // check if the point is an inlier
                Eigen::Vector2d u_j = Eigen::Vector2d(u(0, j), u(1, j));
                double error = (u_est - u_j).norm();
                inliers[j] = (error < tolerance);
                if (inliers[j]) {
                    num_inliers++;
                    inlier_error += error;
                }
            }

            // update the current best estimate
            if (num_inliers > num_inliers_best || (num_inliers == num_inliers_best && inlier_error < inlier_error_best)) {
                num_inliers_best = num_inliers;
                inliers_best = inliers;
                vel_best = vel;
                inlier_error_best = inlier_error;
                inv_depth_best = inv_depth;
                inv_depth = Eigen::ArrayXd::Zero(n);
            }
            if (show_messages) {
                std::cout << "Finished " << i+1 << " RANSAC trials. The current maximum number of inliers is " << num_inliers_best << "." << std::endl;
            }
        }
        // collect the inlier points and put them in an array
        Eigen::Array3Xd inlier_pts = Eigen::Array3Xd::Zero(3, num_inliers_best);
        Eigen::VectorXd inlier_alphas = Eigen::VectorXd::Zero(num_inliers_best);
        Eigen::VectorXd inlier_alpha_ks = Eigen::VectorXd::Zero(num_inliers_best);
        int j = 0;
        for (int i = 0; i < n; i++) {
            if (inliers_best[i]) {
                inlier_pts(0, j) = q(0, i);
                inlier_pts(1, j) = q(1, i);
                inlier_pts(2, j) = 1.0 / inv_depth_best(i);
                inlier_alphas(j) = alpha[i];
                inlier_alpha_ks(j) = alpha_k[i];
                j++;
            }
        }
        return RansacValues(num_inliers_best, inlier_pts, inlier_alphas, inlier_alpha_ks, vel_best.w, vel_best.v, vel_best.k);
    }
}