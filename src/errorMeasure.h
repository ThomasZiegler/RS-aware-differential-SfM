//
// Created by manuel on 5/7/18.
//


#ifndef DIFF_RS_SFM_ERRORMEASURE_H
#define DIFF_RS_SFM_ERRORMEASURE_H


#include "Eigen/Dense"
#include "camera.h"


namespace error_measure {
    /**
     * Struct containing the true values of the angular and translational velocities
     */
    struct TrueValues{
        Eigen::Vector3d w;
        Eigen::Vector3d v;

        TrueValues(Eigen::Vector3d w_init, Eigen::Vector3d v_init) :
                w(w_init), v(v_init) {}
    };

    /**
     * Struct containing the errors that were calculated by evaluateVelocities
     */
    struct VelocityErrors{
        Eigen::Array3Xd w;
        Eigen::Array3Xd v;
        Eigen::ArrayXd k;
        Eigen::ArrayXd error_reproject_vec;
        Eigen::Array3Xd error_v_vec;
        Eigen::Array3Xd error_w_vec;
        double error_w;
        double error_v;
        double error_reproject;

        VelocityErrors(Eigen::Array3Xd w_init, Eigen::Array3Xd v_init, Eigen::ArrayXd k_init, Eigen::ArrayXd error_reproject_vec_init,
                       Eigen::Array3Xd error_v_vec_init, Eigen::Array3Xd error_w_vec_init, double error_w_init, double error_v_init, double error_reproject_init) :
                w(w_init), v(v_init), k(k_init), error_reproject_vec(error_reproject_vec_init),
                error_v_vec(error_v_vec_init), error_w_vec(error_w_vec_init), error_w(error_w_init), error_v(error_v_init), error_reproject(error_reproject_init) {}
    };

    /**
     * Evaluate the algorithm on the images stored in camera
     *
     * @param camera the camera that is used for evaluation. It should contain the images
     * @param gamma the gamma value that is used for beta
     * @param ransac_trials the number of trials used in RANSAC
     * @param num_evaluations the number of times the whole algorithm should be evaluated
     * @param use_deep_flow true if Deep Flow should be used. Otherwise the ground truth flow is used
     * @param constant_acceleration true if the constant acceleration assumption should be used instead of constant velocity
     * @param global_shutter true if the global shutter assumption should be used instead of const velocity / acceleration
     *        this will overwrite the constant_acceleration parameter
     * @param optimize_results true if the results should be optimized by the Nonlinear Refinement after RANSAC
     * @param show_messages true if the debug messages should be print
     * @param image_path path where the depth map be saved
     */
    VelocityErrors evaluateVelocities(Camera camera, TrueValues true_values, double gamma, int ransac_trials, int num_evaluations,
                                      bool use_deep_flow, bool constant_acceleration, bool global_shutter, bool optimize_results, bool show_messages, std::string image_path);
}

#endif // DIFF_RS_SFM_ERRORMEASURE_H