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


Created on 07.05.2018
    Author: Manuel Fritsche
  Modified: Thomas Ziegler

*/



#include "errorMeasure.h"
#include "minimal.h"
#include "nonlinearRefinement.h"
#include "camera.h"

#include <iostream>
#include <cmath>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include "Eigen/Dense"
#include "camera.h"


namespace error_measure {
    VelocityErrors evaluateVelocities(Camera camera, TrueValues true_values, double gamma, int ransac_trials, int num_evaluations,
                  bool use_deep_flow, bool constant_acceleration, bool global_shutter, bool optimize_results, bool show_messages, std::string image_path) {
        // defining constants
        const double THRESHOLD_FLOW = 0.0000000001; // threshold for assuming that flow is 0
        const double TOL_RANSAC = 0.05; // tolerance of the RANSAC inliers

        // calculate optical flow
        cv::Mat_<cv::Point_<double>> flow_image;
        if (use_deep_flow) {
            flow_image = camera.calculateDeepFlow(1,2);
        } else {
            flow_image = camera.calculateTrueFlow(1,2);
        }
        int rows = flow_image.rows;
        int cols = flow_image.cols;

        // get the camera intrinsics
        Eigen::Matrix3d K = camera.getIntrinsics();
        double f_x = K(0, 0);
        double f_y = K(1,1);
        double c_x = K(0, 2);
        double c_y = K(1,2);
        // set the gamma value for the camera
        camera.setGamma(gamma);

        // calculate the (flattened) coordinates and flatten the flow
        Eigen::Matrix2Xd coord = Eigen::Matrix2Xd::Ones(2, rows * cols);
        Eigen::Matrix2Xd coord_full = Eigen::Matrix2Xd::Ones(2, rows * cols);
        Eigen::Matrix2Xd flow = Eigen::Matrix2Xd::Zero(2, rows * cols);
        Eigen::Matrix2Xd flow_full = Eigen::Matrix2Xd::Zero(2, rows * cols);
        int position = 0;
        for (int i = 0; i < cols; i++) {
            for (int j = 0; j < rows; j++) {
                cv::Point_<double> point = flow_image.at<cv::Point_<double>>(j, i); // correct ordering?
             double dx = point.x;
             double dy = point.y;

            // only use the coordinates with flow != 0
            double norm = dx * dx + dy * dy;
            if (norm > THRESHOLD_FLOW) {
                coord_full(0, position) = double(i);
                coord_full(1, position) = double(j);
                flow_full(0, position) = point.x;
                flow_full(1, position) = point.y;

                flow(0, position) = point.x*gamma/f_x;
                flow(1, position) = point.y*gamma/f_y;
                coord(0, position) = (i-c_x)*1.0/f_x;
                coord(1, position) = (j-c_y)*1.0/f_y;

                position++;
                }
            }
        }
        // remove 0 columns from the coord and flow matrices
        coord.conservativeResize(2, position);
        flow.conservativeResize(2, position);
        if (show_messages) {
            std::cout << std::endl << "coordinate columns: " << coord.cols() << std::endl;
            std::cout << "flow columns: " << flow.cols() << std::endl << std::endl;
        }

        // calculate both parts of the beta factor
        Eigen::ArrayXd alpha = minimal::getAlpha(flow_full, rows, gamma);
        Eigen::ArrayXd alphaK = minimal::getAlphaK(coord_full, flow_full, rows, gamma);
        if (global_shutter) {
            // in the GS case set all alpha values to 1
            alpha *= 0;
            alpha += 1;
            constant_acceleration = false;
        }

        // initialize storage and variables
        Eigen::ArrayXd v_errors = Eigen::ArrayXd::Zero(num_evaluations);
        Eigen::ArrayXd w_errors = Eigen::ArrayXd::Zero(num_evaluations);
        Eigen::ArrayXd mean_errors = Eigen::ArrayXd::Zero(num_evaluations);
        Eigen::Array3Xd w = Eigen::Array3Xd::Zero(3, num_evaluations);
        Eigen::Array3Xd v = Eigen::Array3Xd::Zero(3, num_evaluations);
        Eigen::ArrayXd k = Eigen::ArrayXd::Zero(num_evaluations);

        // initialize true values
        double true_v_norm = true_values.v.norm();
        Eigen::Matrix3d true_rot;
        //  Skew symmetric matrix:  [ 0  -p3  p2]
        //						    [ p3  0  -p1] --> [p1 p2 p3]'
        //							[-p2  p1  0 ]
        true_rot << 1, -true_values.w(2), true_values.w(1),
                true_values.w(2), 1, -true_values.w(0),
                -true_values.w(1), true_values.w(0), 1;

        if (show_messages) {
            std::cout << std::endl;
        }
        // iterate several times to compte the average result
        for (int eval_num = 0; eval_num < num_evaluations; eval_num++) {
            if (show_messages) {
                std::cout << std::endl << "Computing evaluation " << eval_num+1 << "/" << num_evaluations;
            }
            // execute ransac
            RansacValues ransac_results = minimal::ransac(coord, flow, alpha, alphaK, constant_acceleration, ransac_trials, TOL_RANSAC, show_messages);
            if (show_messages) {
                // print ransac results
                std::cout << std::endl << "ransac numInliers: " << ransac_results.num_inliers << std::endl;
                std::cout << "ransac w: " << ransac_results.w.transpose() << std::endl;
                std::cout << "ransac v: " << ransac_results.v.transpose() << std::endl;
                std::cout << "ransac k: " << ransac_results.k << std::endl << std::endl;
            }

            // optimize solution with nonlinear refinement:
            RansacValues results = ransac_results; // in case no optimization is used
            if (optimize_results) {
                results = nonlinear_refinement::nonLinearRefinement(flow, ransac_results, constant_acceleration, show_messages);
                if (show_messages) {
                    // print optimized results
                    std::cout << std::endl << "After nonlinear refinement: " << std::endl;
                    std::cout << "final w: " << results.w.transpose() << std::endl;
                    std::cout << "final v: " << results.v.transpose() << std::endl;
                    std::cout << "final k: " << results.k << std::endl << std::endl;
                }
            }

            // flip sign of z and v values if the z values are negative
            double z_count=0;
            for (int i = 0; i < results.num_inliers; i++) {
                z_count += results.inliers(2,i);
            }
            double z_mean = z_count * 1.0 / results.num_inliers;
            if (z_mean < 0) {
                results.inliers.row(2) *= -1.0;
                results.v *= -1.0;
            }

            // store velocities and k
            w.col(eval_num) = results.w;
            v.col(eval_num) = results.v;
            k(eval_num) = results.k;

            // calculate rotation error
            Eigen::Matrix3d results_w_rot;
            results_w_rot << 1, -results.w(2), results.w(1),
                    results.w(2), 1, -results.w(0),
                    -results.w(1), results.w(0), 1;
            Eigen::Matrix3d error_rot = results_w_rot * true_rot.transpose();
            w_errors(eval_num) = Eigen::Vector3d(error_rot(2, 1), error_rot(0, 2), error_rot(1, 0)).norm();
            // calculate translation error
            v_errors(eval_num) = std::acos(results.v.dot(true_values.v) / (results.v.norm() * true_v_norm));
            // calculate reprojection error and save depth map
            double z_min = 100000;
            double z_max = 0;
            for (int i=0; i< results.num_inliers; ++i) {
                if (results.inliers(2,i) < z_min) {
                    z_min =  results.inliers(2,i);
                }
                if (results.inliers(2,i) > z_max) {
                    z_max =  results.inliers(2,i);
                }
            }

            // create image of depth map
            const int min_z_value = 10;
            double multiplier = 244.0 /(z_max- z_min);
            cv::Mat depth_est(rows, cols, CV_8UC1, cv::Scalar(0));
            Eigen::MatrixXd depth_map = Eigen::MatrixXd::Zero(rows, cols);
            for (int i = 0; i < results.num_inliers; i++) {
                int x = int(f_x*results.inliers(0,i) + c_x + 0.5);
                int y = int(f_y*results.inliers(1,i) + c_y + 0.5);
                int z = min_z_value + int((results.inliers(2, i) - z_min) * multiplier);
                depth_est.at<uchar>(cv::Point_<double>(x, y)) = z;
                depth_map(y,x) = results.inliers(2,i);
            }

            // output image
            std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);
            cv::imwrite(image_path + std::to_string(eval_num) + ".png", depth_est, compression_params);

            // set realative pose
            camera.setPose(1, results.k, results.v, results.w);
            camera.setDepthMap(1, depth_map);

            if (global_shutter){
                camera.backProjectGs(1);
            } else {
                camera.backProject(1);
            }

            // calculate reprojection error and save point cloud
            mean_errors(eval_num) = camera.meanReprojectionError(1);
            camera.createPointCloud(1, image_path + std::to_string(eval_num) + ".ply");

            // print current errors
            if (show_messages) {
                std::cout << std::endl << "The rotation error for the current evaluation is " << w_errors(eval_num) << std::endl;
                std::cout << "The translation error for the current evaluation is " << v_errors(eval_num) << std::endl;
                std::cout << "The reprojection error for the current evaluation is " << mean_errors(eval_num) << std::endl;
            }
            else {
                std::cout << "Finished evaluation " << eval_num+1 << "/" << num_evaluations << ". error_w = " << w_errors(eval_num)
                        << ". error_v = " << v_errors(eval_num)
                        << ". reprojection error = "<< mean_errors(eval_num) << std::endl;
            }
        }

        // calculate and print average errors
        double ave_v_error = v_errors.mean();
        double ave_w_error = w_errors.mean();
        double ave_mean_error = mean_errors.mean();
        std::cout << std::endl << "The average rotation error is " << ave_w_error << std::endl;
        std::cout << "The average translation error is " << ave_v_error << std::endl;
        std::cout << "The average reprojection error is " << ave_mean_error << std::endl;

        return VelocityErrors(w, v, k, mean_errors, v_errors, w_errors, ave_w_error, ave_v_error, ave_mean_error);
    }
}