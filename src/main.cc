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


Created on 29.03.2018
    Author: Thomas Ziegler
  Modified: Manuel Fritsche
            Felix Graule

*/

#include <boost/filesystem.hpp>
#include <ctime>
#include <ceres/ceres.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "Eigen/Core"
#include "camera.h"
#include "nonlinearRefinement.h"
#include "minimal.h"
#include "errorMeasure.h"


using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Array2d;
using Eigen::ArrayXd;
using ceres::CostFunction;
using ceres::Problem;
using ceres::AutoDiffCostFunction;
using ceres::Solver;
using cv::Mat;
using nonlinear_refinement::estimateInverseDepth;
using nonlinear_refinement::nonLinearRefinement;
using minimal::ransac;

using std::cout;
using std::cin;
using std::endl;

// Method declaration
// ------------------------------------------------------------------------------------------------------------------
/**
 * Method to evaluate the reprojection error over several different tasks (e.g. parameter sweeps)
 *
 * Set the desired constants at the beginning of this method implementation.
 *
 * !! This method works only with synthetic data since the ground truth values are required for the repojection error !!
 * !! calculation                                                                                                     !!
 */
void evaluateParameterSweep();

/**
 * Method to evaluate a single task (synthetic or real world)
 *
 * Set the desired constants at the beginning of the method implementation.
 *
 * Some examples are included in the Git repository. To run them uncomment the desired one. The required data have to
 * be extracted beforehand!
 */
void evaluateSingleRun();


/**
 * Method to evaluate calculated flow. It calculates the flow from the two frames and adds it to frame 1. This should
 * coincide with frame 2. Hence the difference between calculated frame 2 and correct frame 2.
 *
 * @param use_deep_flow (bool) set true if Deep Flow should be used instead of ground truth flow
 */
void testFlow(bool use_deep_flow);

// helper methods

/**
 * Method to initialize the camera class with synthetic data.
 *
 * Adds frame from synthetic data (RS and GS image, depth_map, unprojection map and correct poses)
 * additional adds the unprojection maps for the GS images (used for testing)
 *
 * @param data_prefix (string) path to location of the data
 * @param show_messages (bool) indicating if information text should be displayed
 * @return initialized Camera object
 */
Camera setupCameraSynthetic(std::string data_prefix, bool show_messages);


/**
 * Method to initialize the camera class with real world data.
 *
 * Adds the two RS frames and the camera intrinsic
 *
 * @param data_prefix (string) path to location of the data
 * @param intrinsic_selection (string) selecting the camera intinsic (see Camera.cc file)
 * @return initialized Camera object
 */
Camera setupCameraReal(std::string data_prefix, std::string intrinsic_selection);

/**
 * Method that returns the current date and time as string
 *
 * @return (string) date and time
 */
std::string getDateString();

/**
 * Method that converts integer to string. It ensures that in case the integer is only one digit long an additional
 * zero is added at the beginning of the string.
 *
 * @param number (int) number to be converted
 * @return (string) number converted to string
 */
std::string stringToFormat(const int number);

// select desired method by uncommenting
int main() {
    // set constants in function!
    //evaluateParameterSweep();

    // set constants in function!
    //evaluateSingleRun();

    //testFlow(false);
    return 0;
}

// Method implementation
// ------------------------------------------------------------------------------------------------------------------


void evaluateParameterSweep() {
    // structure of the test files:
    // create a folder ../data/evaluate/
    // create a txt file ../data/evaluate/tasks.txt that includes the names of the tasks:
    //      e.g.:
    //          task1
    //          task2
    //          task3
    // create folders with the name of each task: ../data/evaluate/<task>
    // create the csv files v.csv, w.csv, gamma.csv, k.csv in each of the folders
    // v.csv contains the ground truth translational velocities (e.g.: 0.5,0.25,0.25)
    // w.csv contains the ground truth rotational velocities (e.g.: 0.1,0.2,0.3)
    // gamma.csv contains only the ground truth gamma value (e.g. 0.96)
    // k.csv contains only the ground truth k value (e.g. 0.1)
    // For each task create a folder ../data/evaluate/<task>/images containing the images as they are used in setupCameraSynthetic()

    // set constants
    const int RANSAC_TRIALS = 5; // number of RANSAC trials used
    const int NUM_EVALUATIONS = 2; // number of evaluation runs for each gamma value
    const bool USE_DEEP_FLOW = false; // true if Deep Flow should be used instead of Ground Truth Flow
    const bool USE_CONST_ACC = false; // true if the constant acceleration assumption should be used instead of constant velocity
    const bool USE_GLOBAL_SHUTTER = false; // true if global shutter assumpiton should be used (will overwrite const acceleration mode)
    const bool OPTIMIZE_RESULTS = false; // true if the nonlinear refinement should be used. If false it is skipped.
    const bool SHOW_MSG = true; // reduces the numer of messages that are printed

    const std::string PATH = "../../examples/synthetic/example_sweep/";
    const std::string date_time = getDateString();
    const std::string PATH_RESULTS = PATH + "results/" + date_time + "/";
    boost::filesystem::create_directories(PATH_RESULTS);

    // save config file
    std::ofstream file_config;
    file_config.open(PATH_RESULTS+"configuration");
    file_config << "ransac trials: " << RANSAC_TRIALS << endl;
    file_config << "evaluation runs: " << NUM_EVALUATIONS << endl;
    file_config << "use deep flow: " << USE_DEEP_FLOW << endl;
    file_config << "use GS assumption: " << USE_GLOBAL_SHUTTER << endl;
    file_config << "use const acceleration: " << USE_CONST_ACC << endl;
    file_config << "use refinement: " << OPTIMIZE_RESULTS << endl;
    file_config << "----------------------------------------------" << endl;


    // initialize ofstream files in new folder
    std::ofstream file_errors_out;
    file_errors_out.open(PATH_RESULTS+"errors.csv");
    file_errors_out << "task,error_w,error_v,reproject_error" << std::endl;
    std::ofstream file_w_out;
    file_w_out.open(PATH_RESULTS+"w.csv");
    std::ofstream file_v_out;
    file_v_out.open(PATH_RESULTS+"v.csv");
    std::ofstream file_k_out;
    file_k_out.open(PATH_RESULTS+"k.csv");
    std::ofstream file_reproject_error_out;
    file_reproject_error_out.open(PATH_RESULTS+"reproject_errors.csv");
    std::ofstream file_v_error_out;
    file_v_error_out.open(PATH_RESULTS+"error_v.csv");
    std::ofstream file_w_error_out;
    file_w_error_out.open(PATH_RESULTS+"error_w.csv");


    // initialize constants
    // load tasks
    std::ifstream file_task_in (PATH + "tasks.txt");
    std::vector<std::string> tasks;
    std::string task;
    while(std::getline(file_task_in, task)) {
        tasks.push_back(task);
        file_config << task << endl;
    }
    // run tasks
    for (int i = 0; i < tasks.size(); i++) {
        std::cout << std::endl << "Executing task " << tasks[i] << "..." << std::endl;
        std::cout << "Ground Truth values:" << std::endl;
        // load values:
        // initialize value storage:
        double v[3];
        double w[3];
        double gamma;
        double k;
        // 1. line: v, 2: w, 3: gamma, 4: k
        std::string csv_line;
        // read v from the stream
        std::ifstream file_v_in (PATH + tasks[i] + "/v.csv");
        for (int j = 0; j < 3; j++) {
            std::getline(file_v_in, csv_line, ',');
            std::stringstream stream_v(csv_line);
            stream_v >> v[j];
        }
        std::cout << "v: " << v[0] << ", " << v[1] << ", " << v[2] << std::endl;
        std::ifstream file_w_in (PATH + tasks[i] + "/w.csv");
        // read w from the stream
        for (int j = 0; j < 3; j++) {
            std::getline(file_w_in, csv_line, ',');
            std::stringstream stream_w(csv_line);
            stream_w >> w[j];
        }
        std::cout << "w: " << w[0] << ", " << w[1] << ", " << w[2] << std::endl;
        error_measure::TrueValues true_values(Eigen::Vector3d(w[0], w[1], w[2]), Eigen::Vector3d(v[0], v[1], v[2]));
        // read gamma from the stream
        std::ifstream file_gamma_in (PATH + tasks[i] + "/gamma.csv");
        std::getline(file_gamma_in, csv_line, ',');
        std::stringstream stream_gamma(csv_line);
        stream_gamma >> gamma;
        std::cout << "gamma: " << gamma << std::endl;
        // read k from the stream
        std::ifstream file_k_in (PATH + tasks[i] + "/k.csv");
        std::getline(file_k_in, csv_line, ',');
        std::stringstream stream_k(csv_line);
        stream_k >> k;
        std::cout << "k: " << k << std::endl << std::endl;

        // setup camera
        std::streambuf* orig_buf = std::cout.rdbuf();
        std::cout.rdbuf(NULL);
        Camera camera = setupCameraSynthetic(PATH + tasks[i] + "/images/", SHOW_MSG);
        camera.setGamma(gamma);
        cout.rdbuf(orig_buf);


        std::string image_path = PATH_RESULTS + "/depthMaps/" + std::to_string(i) + "/";
        boost::filesystem::create_directories(image_path);

        // run evaluation
        error_measure::VelocityErrors errors = error_measure::evaluateVelocities(camera, true_values, gamma, RANSAC_TRIALS,
                                                                                 NUM_EVALUATIONS, USE_DEEP_FLOW,
                                                                                 USE_CONST_ACC, USE_GLOBAL_SHUTTER, OPTIMIZE_RESULTS, SHOW_MSG, image_path);
        // write results to files
        file_errors_out << tasks[i] << "," << errors.error_w << "," << errors.error_v << "," << errors.error_reproject << std::endl;
        for (int j = 0; j < NUM_EVALUATIONS; j++) {
            file_w_out << errors.w.col(j).transpose() << ",";
            file_v_out << errors.v.col(j).transpose() << ",";
            file_k_out << errors.k(j) << ",";
            file_reproject_error_out << errors.error_reproject_vec(j) << ",";
            file_v_error_out << errors.error_v_vec.col(j).transpose() << ",";
            file_w_error_out << errors.error_w_vec.col(j).transpose() << ",";
        }
        file_w_out << std::endl;
        file_v_out << std::endl;
        file_k_out << std::endl;
        file_reproject_error_out << std::endl;
        file_v_error_out << std::endl;
        file_w_error_out << std::endl;
    }
    file_errors_out.close();
    file_w_out.close();
    file_v_out.close();
    file_k_out.close();
    file_config.close();
    file_reproject_error_out.close();
    file_v_error_out.close();
    file_w_error_out.close();
}

// evaluate algorithm for two consecutive frames
void evaluateSingleRun() {
    // set desired parameters and here
    const int ransac_trials = 5; // number of RANSAC trials used
    const bool use_global_shutter_mode = false; // true if global shutter assumption should be used (This overrides use_const_acleration_mode)
    const bool use_acceleration_mode = false; // true if const acceleration assumption should be used instead of constant velocity
    const bool use_refinement = true; // true if the nonlinear refinement should be used
    bool use_deep_flow = false; // true if Deep Flow should be used instead of Ground Truth Flow
    const bool use_synthetic_data = true; // true if synthetic data is used (might override used flow method)
    const double ransac_tol = 0.05;  // RANSAC threshold
    const double flow_threshold = 1e-10; // flow threshold values below are set to zero


    // load data (uncomment desired example)
    // -----------------------------------------------
    // Synthetic data
    // -----------------------------------------------
    // Example 1:
    std::string data_path = "../../examples/synthetic/example1/";
    Camera camera = setupCameraSynthetic(data_path+"test__v=[0.03;0.03;0]_w=[0;0;0.5]_k=0_gamma=0.8/images/",true);
    double gamma = 0.8;
    // Example 2:
//    std::string data_path = "../../examples/synthetic/example2/";
//    Camera camera = setupCameraSynthetic(data_path+"test__v=[0.01;0.01;0]_w=[0;0;0.5]_k=0_gamma=0.8/images/",true);
//    double gamma = 0.8;
    // Example 3:
//    std::string data_path = "../../examples/synthetic/example3/";
//    Camera camera = setupCameraSynthetic(data_path+"test__v=[0.07;0.07;0]_w=[0;0;0.5]_k=0_gamma=0.8/images/",true);
//    double gamma = 0.8;
    // Example 4:
//    std::string data_path = "../../examples/synthetic/example4/";
//    Camera camera = setupCameraSynthetic(data_path+"test__v=[0.03;0.03;0]_w=[0;0;0]_k=0_gamma=0.8/images/",true);
//    double gamma = 0.8;
    // Example 5:
//    std::string data_path = "../../examples/synthetic/example5/";
//    Camera camera = setupCameraSynthetic(data_path+"test__v=[0.03;0.03;0]_w=[0;0;2]_k=0_gamma=0.8/images/",true);
//    double gamma = 0.8;

    // -----------------------------------------------
    // Real world data
    // -----------------------------------------------
    // Example 1:
//    std::string data_path = "../../examples/real_world/example1/";
//    Camera camera = setupCameraReal(data_path, "galaxy_stabil");
//    double gamma = 0.95;
    // Example 2:
//    std::string data_path = "../../examples/real_world/example2/";
//    Camera camera = setupCameraReal(data_path, "galaxy");
//    double gamma = 0.95;
    // Example 3:
//    std::string data_path = "../../examples/real_world/example3/";
//    Camera camera = setupCameraReal(data_path, "galaxy");
//    double gamma = 0.95;
    // Example 4:
//    std::string data_path = "../../examples/real_world/example4/";
//    Camera camera = setupCameraReal(data_path, "galaxy_stabil");
//    double gamma = 0.95;
    // Example 5:
//    std::string data_path = "../../examples/real_world/example5/";
//    Camera camera = setupCameraReal(data_path, "galaxy");
//    double gamma = 0.95;


    camera.setGamma(gamma);
    // ensure that for real world images deep flow is used
    if (!use_synthetic_data) {
        use_deep_flow = true;
    }

    // get camera intrinsics (used to calculate normalized image coordinates)
    Eigen::Matrix3d K = camera.getIntrinsics();
    double f_x = K(0, 0);
    double f_y = K(1,1);
    double c_x = K(0, 2);
    double c_y = K(1,2);


    // calculate flow
    cv::Mat_<cv::Point_<double>> flow_image;
    if (use_deep_flow) {
        flow_image = camera.calculateDeepFlow(1,2);
    } else {
        flow_image = camera.calculateTrueFlow(1,2);
    }

    // save flow image
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    cv::Mat flow_image_save = camera.getImageOpticalFlow(flow_image);
    flow_image_save.convertTo(flow_image_save, CV_8UC3, 255.0);
    cv::imwrite(data_path + "optical_flow.png", flow_image_save, compression_params);
    cv::Mat flow_image_arrow = camera.flowArrows(camera.getFrame(1).getRsImage().clone(), flow_image, 50, 50);
    cv::imwrite(data_path + "optical_flow_arrow.png", flow_image_arrow, compression_params);



    // calculate the (flattened) coordinates and flatten the flow
    int rows = flow_image.rows;
    int cols = flow_image.cols;
    Eigen::Matrix2Xd coord = Eigen::Matrix2Xd::Ones(2, rows * cols);
    Eigen::Matrix2Xd coord_pixel = Eigen::Matrix2Xd::Ones(2, rows * cols);
    Eigen::Matrix2Xd flow = Eigen::Matrix2Xd::Zero(2, rows * cols);
    Eigen::Matrix2Xd flow_pixel = Eigen::Matrix2Xd::Zero(2, rows * cols);

    // get flow values (for normalized coordinates and for pixel coordinates)
    int position = 0;
    for (int i = 0; i < cols; i++) {
        for (int j = 0; j < rows; j++) {
            cv::Point_<double> point = flow_image.at<cv::Point_<double>>(j, i);
             double dx = point.x;
             double dy = point.y;

            // only use the coordinates with flow != 0
            double norm = dx * dx + dy * dy;
            if (norm > flow_threshold) {
                // get values in pixel coordinates
                coord_pixel(0, position) = double(i);
                coord_pixel(1, position) = double(j);
                flow_pixel(0, position) = point.x;
                flow_pixel(1, position) = point.y;

                // calculate normalized values
                flow(0, position) = point.x*gamma/f_x;
                flow(1, position) = point.y*gamma/f_y;
                coord(0, position) = (i-c_x)*1.0/f_x;
                coord(1, position) = (j-c_y)*1.0/f_y;

                position++;
            }
        }
    }
    cout << endl << "coordinate columns: " << coord.cols() << endl;
    cout << "flow columns: " << flow.cols() << endl << endl;

    // calculate beta values
    ArrayXd alpha = minimal::getAlpha(flow_pixel, rows, gamma);
    ArrayXd alphaK = minimal::getAlphaK(coord_pixel, flow_pixel, rows, gamma);

    // set beta = 1 if GS assumption is used
    if(use_global_shutter_mode){
        alpha *= 0;
        alpha += 1;
    }

    // run ransac
    RansacValues ransac_results = minimal::ransac(coord, flow, alpha, alphaK, use_acceleration_mode, ransac_trials, ransac_tol, true);
    cout << endl << "ransac numInliers: " << ransac_results.num_inliers << endl;
    cout << "ransac w: " << ransac_results.w.transpose() << endl;
    cout << "ransac v: " << ransac_results.v.transpose() << endl;
    cout << "ransac k: " << ransac_results.k << endl << endl;


    // optimize solution with nonlinear refinement:
    RansacValues results = ransac_results;
    if (use_refinement){
        results = nonLinearRefinement(flow, ransac_results, use_acceleration_mode, false);
    }
    cout << "After nonlinear refinment: " << endl;
    cout << "ransac w: " << results.w.transpose() << endl;
    cout << "ransac v: " << results.v.transpose() << endl;
    cout << "ransac k: " << results.k << endl;



    // flip sign of z and v values if average depth is negative
    double count_z = 0;
    for (int i=0; i<results.num_inliers; ++i)
    {
        count_z += results.inliers(2,i);
    }
    double z_mean = count_z * 1.0 / results.num_inliers;
    cout << "z mean: " << z_mean << endl;

    if (z_mean < 0){
        results.inliers.row(2) *= -1.0;
        results.v *= -1.0;
    }


    // get min and max depth value (needed for depth image)
    double z_min = INFINITY;
    double z_max = 0;
    for (int i=0; i< results.num_inliers; ++i) {
        if (results.inliers(2,i) < z_min) {
            z_min =  results.inliers(2,i);
        }
        if (results.inliers(2,i) > z_max) {
            z_max =  results.inliers(2,i);
        }
    }


    // set depth map (effective value) and depth image (scaled between min and max depth value)
    const int min_z_value = 10;
    double multiplier = 244.0 / (z_max-z_min);
    Mat depth_est(rows, cols, CV_8UC1, cv::Scalar(0));
    Eigen::MatrixXd depth_map = Eigen::MatrixXd::Zero(rows, cols);

    for (int i = 0; i < results.num_inliers; i++) {
        int x = int(f_x*results.inliers(0,i) + c_x + 0.5);
        int y = int(f_y*results.inliers(1,i) + c_y + 0.5);
        int z = min_z_value + int((results.inliers(2, i) - z_min) * multiplier);
        if (z > 255) {
            cout << "Warning: Saturated pixel for z of size " << z << endl;
        }
        depth_est.at<uchar>(cv::Point_<double>(x, y)) = z;
        depth_map(y, x) = results.inliers(2, i);
    }

    // get original images
    Mat original_gs = camera.getFrame(1).getGsImage().clone();
    Mat original_rs = camera.getFrame(1).getRsImage().clone();

    // set relative pose and project RS image back into GS image
    camera.setPose(1, results.k, results.v, results.w );
    camera.setDepthMap(1, depth_map);
    if (use_global_shutter_mode){
        camera.backProjectGs(1);
    } else {
        camera.backProject(1);
    }
    Mat backprojection = camera.interpolateCrackyImage(camera.getFrame(1).getGsImage(), 1);

    // save point cloud
    camera.createPointCloud(1, data_path + "point_cloud.ply");

    // save depth image, RS image and backprojection
    cv::imwrite(data_path + "MinimalDepth.png", depth_est, compression_params);
    cv::imwrite(data_path + "rs_image.png", camera.getFrame(1).getRsImage().clone(), compression_params);
    cv::imwrite(data_path + "backprojection.png", backprojection, compression_params);

    // save additional images for evaluation (only possible with synthetic data!)
    if (use_synthetic_data) {
        // store GS image
        cv::imwrite(data_path + "gs_image.png", original_gs, compression_params);

        // reprojection error image (Euclidean distance between estimate and ground truth)
        cv::Mat error_image = camera.createErrorImage(1, 10.0);
        cv::imwrite(data_path + "error_image.png", error_image, compression_params);

        cv::Mat difference = abs(backprojection-original_gs);
        cv::imwrite(data_path + "difference.png", difference, compression_params);
        cv::Mat remainder = abs(original_gs-difference);
        cv::imwrite(data_path + "remainder.png", remainder, compression_params);

        cv::Mat warp_shift = abs(original_rs-original_gs);
        cv::Mat warp_overlay = camera.createOverlayImage(camera.shiftChannelBGR(original_gs, 1, 1, 1),
                                                         camera.shiftChannelBGR(warp_shift, 2, 0.5, 0.5));
        cv::imwrite(data_path + "overlay_gs_rs.png", warp_overlay, compression_params);

        cv::Mat overlay_gs_bp = camera.createOverlayImage(camera.shiftChannelBGR(original_gs, 1, 1, 1),
                                                          camera.shiftChannelBGR(abs(backprojection-original_gs), 2, 0.5, 0.5));
        cv::imwrite(data_path + "overlay_gs_bp.png", overlay_gs_bp, compression_params);

        // calculate mean reprojection error
        cout << "Euclidean error: " << camera.meanReprojectionError(1) << endl;
    }
}


// Test flow calculation by adding flow to first RS image and compare the result with the second RS image
void testFlow(bool use_deep_flow){
    std::string data_path = "../../examples/synthetic/example4/";
    Camera camera = setupCameraSynthetic(data_path+"test__v=[0.03;0.03;0]_w=[0;0;0]_k=0_gamma=0.8/images/",true);

    Mat gs_image_1 = camera.getFrame(1).getGsImage().clone();
    Mat gs_image_2 = camera.getFrame(2).getGsImage().clone();
    Mat rs_image_1 = camera.getFrame(1).getRsImage().clone();
    Mat rs_image_2 = camera.getFrame(2).getRsImage().clone();
    Mat test_image = camera.getFrame(2).getGsImage().clone();
    test_image *= 0;

    cv::Mat_<cv::Point_<double>> flow_image;
    if (use_deep_flow) {
        flow_image = camera.calculateDeepFlow(1,2);
    } else {
        flow_image = camera.calculateTrueFlow(1,2);
    }

    for(int u=0; u<flow_image.cols; ++u) {
        for(int v=0; v<flow_image.rows; ++v){
            cv::Point_<double> point = flow_image.at<cv::Point_<double>>(v,u);

            int dx = floor(point.x+0.5);
            int dy = floor(point.y+0.5);

            if (std::abs(dx) > 0 || std::abs(dy)>0) {
                std::cout << point <<", " << dx << ", " << dy << std::endl;
                if (v+dy < flow_image.rows && v+dy >= 0 && u+dx < flow_image.cols && u+dx >= 0 )
                    test_image.at<cv::Vec3b>(v + dy, u + dx) = rs_image_1.at<cv::Vec3b>(v, u);
            }
       }
    }

    cv::imshow("rs 1", gs_image_1);
    cv::imshow("rs 2", gs_image_2);
    cv::imshow("test", test_image);
    cv::imshow("difference from GS 2 (target)", abs(rs_image_2-test_image));

    cv::waitKey(0);

    // save figure
//    std::vector<int> compression_params;
//    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//    compression_params.push_back(0);
//    test_image = abs(gs_image_2-test_image);
//    cv::imwrite(data_path + "Difference.png", test_image, compression_params);
}


// setup camera for synthetic images
Camera setupCameraSynthetic(std::string data_prefix, bool show_messages) {
    // Frame 1
    std::string poses_csv_1 = data_prefix + "1_rs_t.csv";
    std::string orientation_csv_1 = data_prefix + "1_rs_r.csv";
    std::string csv_unprojetion_x_1 = data_prefix + "1_rs_unproject_x.csv";
    std::string csv_unprojetion_y_1 = data_prefix + "1_rs_unproject_y.csv";
    std::string csv_unprojetion_z_1 = data_prefix + "1_rs_unproject_z.csv";
    std::string csv_gs_unprojetion_x_1 = data_prefix + "1_initial_gs_unproject_x.csv";
    std::string csv_gs_unprojetion_y_1 = data_prefix + "1_initial_gs_unproject_y.csv";
    std::string csv_gs_unprojetion_z_1 = data_prefix + "1_initial_gs_unproject_z.csv";

    std::string path_rs_image_1 = data_prefix + "1_rs.png";
    std::string path_gs_image_1 = data_prefix + "1_initial_gs.png";
    std::string path_depth_image_1 = data_prefix + "1_initial_depth.png";


    Mat rs_image_1 = cv::imread(path_rs_image_1.c_str(), CV_LOAD_IMAGE_COLOR);
    Mat gs_image_1 = cv::imread(path_gs_image_1.c_str(), CV_LOAD_IMAGE_COLOR);
    Mat depth_image_inv_1 = cv::imread(path_depth_image_1.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    Mat depth_image_1 = depth_image_inv_1.clone();

    // Frame 2
    std::string poses_csv_2 = data_prefix + "2_rs_t.csv";
    std::string orientation_csv_2 = data_prefix + "2_rs_r.csv";
    std::string csv_unprojetion_x_2 = data_prefix + "2_rs_unproject_x.csv";
    std::string csv_unprojetion_y_2 = data_prefix + "2_rs_unproject_y.csv";
    std::string csv_unprojetion_z_2 = data_prefix + "2_rs_unproject_z.csv";
    std::string csv_gs_unprojetion_x_2 = data_prefix + "2_initial_gs_unproject_x.csv";
    std::string csv_gs_unprojetion_y_2 = data_prefix + "2_initial_gs_unproject_y.csv";
    std::string csv_gs_unprojetion_z_2 = data_prefix + "2_initial_gs_unproject_z.csv";

    std::string path_rs_image_2= data_prefix + "2_rs.png";
    std::string path_gs_image_2= data_prefix + "2_initial_gs.png";
    std::string path_depth_image_2= data_prefix + "2_initial_depth.png";


    Mat rs_image_2 = cv::imread(path_rs_image_2.c_str(), CV_LOAD_IMAGE_COLOR);
    Mat gs_image_2 = cv::imread(path_gs_image_2.c_str(), CV_LOAD_IMAGE_COLOR);
    Mat depth_image_inv_2 = cv::imread(path_depth_image_2.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    Mat depth_image_2 = depth_image_inv_2.clone();

    Mat test_image = gs_image_2.clone();
    test_image *= 0;

    MatrixXd depth_map_1(depth_image_1.rows, depth_image_1.cols);
    cv::cv2eigen(depth_image_1, depth_map_1);
    MatrixXd depth_map_2(depth_image_2.rows, depth_image_2.cols);
    cv::cv2eigen(depth_image_2, depth_map_2);

    Camera camera;
    camera.loadIntrinsicsFromFile(data_prefix + "A.csv", show_messages);
    camera.addFrameSynthetic(rs_image_1, gs_image_1, depth_image_1, poses_csv_1, orientation_csv_1, csv_unprojetion_x_1,
                             csv_unprojetion_y_1, csv_unprojetion_z_1, csv_gs_unprojetion_x_1, csv_gs_unprojetion_y_1,
                             csv_gs_unprojetion_z_1);
    camera.addFrameSynthetic(rs_image_2, gs_image_2, depth_image_2, poses_csv_2, orientation_csv_2, csv_unprojetion_x_2,
                             csv_unprojetion_y_2, csv_unprojetion_z_2, csv_gs_unprojetion_x_2, csv_gs_unprojetion_y_2,
                             csv_gs_unprojetion_z_2);

    return camera;
}

// setup camera for real world images
Camera setupCameraReal(std::string data_prefix, std::string intrinsic_selection) {
    // Frame 1
    std::string path_rs_image_1 = data_prefix + "frame1.png";
    Mat rs_image_1 = cv::imread(path_rs_image_1.c_str(), CV_LOAD_IMAGE_COLOR);

    // Frame 2
    std::string path_rs_image_2= data_prefix + "frame2.png";
    Mat rs_image_2 = cv::imread(path_rs_image_2.c_str(), CV_LOAD_IMAGE_COLOR);

    Camera camera;
    camera.setIntrinsics(intrinsic_selection);
    camera.addFrameReal(rs_image_1);
    camera.addFrameReal(rs_image_2);

    return camera;
}


// Method which returns current date and time as string
std::string getDateString() {
    time_t currentTime;
    struct tm *localTime;

    time(&currentTime);
    localTime = localtime(&currentTime);
    int day = localTime->tm_mday;
    int month = localTime->tm_mon + 1;
    int year = localTime->tm_year + 1900;
    int hour = localTime->tm_hour;
    int min = localTime->tm_min;
    int sec = localTime->tm_sec;

    // ensure seconds, minutes, hours are two digits
    std::string dateTime = std::to_string(year) + stringToFormat(month) + stringToFormat(day)+ "-" + stringToFormat(hour) + stringToFormat(min) + stringToFormat(sec);
    return dateTime;
}

// Method that converts integer to desired string format (i.e. two letters wide)
std::string stringToFormat(const int number) {
    std::stringstream stream;
    stream << std::setw(2) << std::setfill('0') << number;
    return stream.str();
}
