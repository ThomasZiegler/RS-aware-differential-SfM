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
  Modified: Felix Graule

*/


#include "camera.h"
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/optflow.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>

// return camera intrinsics
Eigen::Matrix3d Camera::getIntrinsics() {
    return K_;
}

// add frame from real data (only RS image and camera intrinsics)
void Camera::addFrameReal(cv::Mat rs_image) {
    RsFrame frame;

    frame.setIntrinsics(K_);
    frame.setImage(rs_image);

    frames_.push_back(frame);
}

// add frame from synthetic data (RS and GS image, depth_map, unprojection map and correct poses)
void Camera::addFrameSynthetic(cv::Mat rs_image, cv::Mat gs_image, cv::Mat depth_image, const std::string poses_csv,
                               const std::string orientation_csv, const std::string csv_unproject_x,
                               const std::string csv_unproject_y, const std::string csv_unproject_z) {
    RsFrame frame;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> depth_map;
    cv::cv2eigen(depth_image, depth_map);

    // add images and set intrinsics
    frame.setImage(rs_image);
    frame.setGsImage(gs_image);
    frame.setDepthMap(depth_map);
    frame.setIntrinsics(K_);

    // set poses of scanlines
    frame.setPoses(poses_csv, orientation_csv);
    // add unprojection map
    frame.setUnprojectionMapRs(csv_unproject_x, csv_unproject_y, csv_unproject_z);

    frames_.push_back(frame);
}


// add frame from synthetic data (RS and GS image, depth_map, unprojection map and correct poses)
// additional add the unprojection maps for the GS images (used for testing)
void Camera::addFrameSynthetic(cv::Mat rs_image, cv::Mat gs_image, cv::Mat depth_image, const std::string poses_csv,
                               const std::string orientation_csv, const std::string csv_unproject_x,
                               const std::string csv_unproject_y, const std::string csv_unproject_z,
                               const std::string csv_gs_unproject_x, const std::string csv_gs_unproject_y,
                               const std::string csv_gs_unproject_z) {
    RsFrame frame;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> depth_map;
    cv::cv2eigen(depth_image, depth_map);

    // add images and set intrinsics
    frame.setImage(rs_image);
    frame.setGsImage(gs_image);
    frame.setDepthMap(depth_map);
    frame.setIntrinsics(K_);

    // initialize GS depth_map (with wrong values!)
    frame.setDepthMapGs(depth_map);

    frame.setPoses(poses_csv, orientation_csv);
    frame.setUnprojectionMapRs(csv_unproject_x, csv_unproject_y, csv_unproject_z);
    frame.setUnprojectionMapGs(csv_gs_unproject_x, csv_gs_unproject_y, csv_gs_unproject_z);

    frames_.push_back(frame);
}

// set camera intrinsics for synthetic data (use provided file)
bool Camera::loadIntrinsicsFromFile(const std::string csv_intrinsic_matrix, bool show_messages) {
    bool return_value = true;
    bool files_correct = true;
    bool nr_lines_correct = true;

    std::string line_entry_1, line_entry_2, line_entry_3;

    long file_nr_lines = 0;

    // check file names
    std::ifstream intrinsics(csv_intrinsic_matrix);

    if (!intrinsics) {
        if (show_messages)
            std::cout << csv_intrinsic_matrix<< " Is not a valid file path for the intrinsic file!" << std::endl;
        files_correct = false;
    }

    if (files_correct) {
        // load data
        if (show_messages)
            std::cout << "Loading of the intrinsic files was successful" << std::endl;

        file_nr_lines = std::count(std::istreambuf_iterator<char>(intrinsics), std::istreambuf_iterator<char>(), '\n');

        // reset file to start at top line
        intrinsics.clear();
        intrinsics.seekg(0, std::ios::beg);

        if (file_nr_lines != 3) {
            if (show_messages)
                std::cout << "The number of lines: " << file_nr_lines << " in the file: " << csv_intrinsic_matrix
                          << " does not conform with the number of lines of an intrinsic matrix (3)!" << std::endl;
            nr_lines_correct = false;
        }

        if (nr_lines_correct) {
            int i = 0;
            while (intrinsics.good()) {
                getline(intrinsics, line_entry_1, ',');
                getline(intrinsics, line_entry_2, ',');
                getline(intrinsics, line_entry_3, '\n');
                // break if no line left in file
                if (intrinsics.eof()) {
                    break;
                }

                if (i == 0) {          // 1st line
                    K_(0,0) = ::atof(line_entry_1.c_str());
                    K_(0,1) = ::atof(line_entry_2.c_str());
                    K_(0,2) = ::atof(line_entry_3.c_str());

                } else if (i == 1) {   // 2nd line
                    K_(1,0) = ::atof(line_entry_1.c_str());
                    K_(1,1) = ::atof(line_entry_2.c_str());
                    K_(1,2) = ::atof(line_entry_3.c_str());
                } else if (i == 2) {   // 3rd line
                    K_(2,0) = ::atof(line_entry_1.c_str());
                    K_(2,1) = ::atof(line_entry_2.c_str());
                    K_(2,2) = ::atof(line_entry_3.c_str());
                }
                ++i;
            }
        }
        else {
            if (show_messages)
                std::cout << "No itrinsics were set" << std::endl;
            return_value = false;
        }
    }
    else {
        if (show_messages)
            std::cout << "Not able to load data file, no intrinsics were set" << std::endl;
        return_value = false;
    }

    return return_value;
}

// set intrinsics for real world data
void Camera::setIntrinsics(const std::string source_camera) {
    Eigen::Matrix3d intrinsics;
    if (source_camera == "iphone") { // IP 4 (RS dataset)
        intrinsics << 1505.1283359786307, 0, 657.81734686405991,
                0, 1513.7789208311444, 349.91807538147589,
                0, 0, 1.0;
    } else if (source_camera == "galaxy_stabil") {  // S8 full HD without video stabilization
        intrinsics << 1803.29785922382, 0, 945.304708272490,
                0, 1799.35406531529, 544.684292978344,
                0, 0, 1;
    } else if (source_camera == "galaxy") { // S8 full HD with video stabilization
        intrinsics << 1492.41306997746, 0, 949.571146410704,
                0, 1491.09286590722, 554.675409391795,
                0, 0, 1;
    } else if (source_camera == "galaxy_old") { // first clips
        intrinsics << 3154.53208221173, 0, 1969.87107268891,
                0, 3152.28696217577, 1521.27056048818,
                0, 0, 1;
    } else if (source_camera == "galaxy_vga") { // S8 VGA resolution without video stabilization
        intrinsics << 484.450845764569, 0,  313.442094604855,
                0, 485.345469134313, 241.383116350144,
                0, 0, 1;
    } else {
        std::cerr << "No valid source camera specified";
    }
    K_ = intrinsics;
    std::cout << "Intrinsics set to: " << std::endl << intrinsics << std::endl << std::endl;
}

// calculate flow between two rs frames from the groundtruth data
cv::Mat_<cv::Point_<double>> Camera::calculateTrueFlow(const int frameNr1, const int frameNr2) {
    std::cout << "Calculating Optical Flow using synthetic data" << std::endl;
    int rows = frames_[frameNr1 - 1].getRows();
    int cols = frames_[frameNr1 - 1].getCols();

    // initialize with zero value
    cv::Mat_<cv::Point_<double>> flow(rows, cols, cv::Point(0, 0));

    RsFrame frame1 = frames_[frameNr1 - 1];
    RsFrame frame2 = frames_[frameNr2 - 1];

    Eigen::Vector3d world_coordinates;
    Eigen::Vector2d image_coordinates_f1;
    Eigen::Vector2d image_coordinates_f2;

    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {

            // get corresponding 3D point for given pixel
            image_coordinates_f1 << u, v;
            world_coordinates = frame1.getUnprojectedWorldCoordinates(image_coordinates_f1);

            // check if coordinate correspond to the 3D object or void
            if (world_coordinates.norm() == 0) {
                image_coordinates_f2 << u, v;
            } else {
                // get corresponing 2D plane coordinate in 2nd image
                image_coordinates_f2 = frame2.calculateImageCoordinatesRsFrame(world_coordinates);
                if (image_coordinates_f2.norm() == 0) {
                    image_coordinates_f2 << u, v;
                }

            }

            // calculate flow by subtracting pixel position of 1st frame from 2nd frame
            flow.at<cv::Point_<double>>(v, u) = cv::Point_<double>(image_coordinates_f2.coeff(0) - double(u),
                                                 image_coordinates_f2.coeff(1) - double(v));
        }
    }
    return flow;
}


// calculate flow between two rs frames using Deep-Flow
cv::Mat_<cv::Point_<double>> Camera::calculateDeepFlow(const int frameNr1, const int frameNr2) {
    std::cout << "Calculating Optical Flow using Deep-Flow" << std::endl;

    int rows = frames_[frameNr1 - 1].getRows();
    int cols = frames_[frameNr1 - 1].getCols();

    cv::Mat_<cv::Point_<double>> flow(rows, cols, cv::Point_<double>(0.0, 0.0));

    cv::Mat cflow;
    cv::UMat gray_f1, gray_f2, uflow;

    // convert color to gray image
    cv::cvtColor(this->getFrame(1).getRsImage(), gray_f1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(this->getFrame(2).getRsImage(), gray_f2, cv::COLOR_BGR2GRAY);

    // Use DeepFlow method to get flow
    cv::Ptr<cv::DenseOpticalFlow> dfl = cv::optflow::createOptFlow_DeepFlow();
    dfl->calc(gray_f1, gray_f2, uflow);

    // create color flow image
    cv::cvtColor(gray_f1, cflow, cv::COLOR_GRAY2BGR);
    uflow.copyTo(flow);

    return flow;
}

// create color image representing the flow direction and magnitude
cv::Mat Camera::getImageOpticalFlow(cv::Mat_<cv::Point_<double>> flow) {
    cv::Mat float_flow;
    flow.convertTo(float_flow, CV_32F);

    //extract x and y channels
    cv::Mat xy[2]; // X,Y
    cv::split(float_flow, xy);

    //calculate angle and magnitude
    cv::Mat magnitude, angle;
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

    //translate magnitude to range [0;1]
    double mag_max;
    cv::minMaxLoc(magnitude, 0, &mag_max);
    magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

    //build hsv image
    cv::Mat _hsv[3], hsv;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magnitude;
    cv::merge(_hsv, 3, hsv);

    //convert to BGR and show
    cv::Mat bgr;//CV_32FC3 matrix
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

    return bgr;
}

// representing flow by adding arrows onto the first frame
cv::Mat Camera::flowArrows(const cv::Mat image, const cv::Mat_<cv::Point_<double>> flow, const int delta_x, const int delta_y) {
    for (int y = 0; y < image.rows; y+=delta_y) {
        for (int x = 0; x < image.cols; x+=delta_x) {
            // get flow
            double dx = flow.at<cv::Point_<double>>(y,x).x;
            double dy = flow.at<cv::Point_<double>>(y,x).y;

            // get length of flow
            double l = sqrt(dx*dx + dy*dy);

            cv::Point p = cv::Point(x, y);
            if (l > 0)
            {
                // add arrow between starting point in frame 1 and corresponding end point in frame 2
                cv::Point p2 = cv::Point(p.x + (int)(dx), p.y + (int)(dy));
                cv::arrowedLine( image, p, p2, CV_RGB(255,0,0), 1, cv::LINE_AA, 0 );
            }
        }
    }
    return image;
}

// get image frame
RsFrame Camera::getFrame(const int frameNr) {
    return frames_[frameNr - 1];
}

// set relative pose for given frame using angular and linear velocity
void Camera::setPose(const int frameNr, const double k, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity) {
    frames_[frameNr -1].setRelativePose(linear_velocity, angular_velocity, k);
}

// set gamma in each frame
void Camera::setGamma(const double gamma) {
    for(std::vector<RsFrame>::iterator it = frames_.begin(); it != frames_.end(); ++it) {
        it->setGamma(gamma);
    }
}

// backproject RS image back into GS image
// calculate corresponding 3D points in world coordinate system
void Camera::backProject(const int frameNr) {
    frames_[frameNr-1].backProject();
}

// backproject RS image back into GS image using GS assumption
// calculate corresponding 3D points in world coordinate system
void Camera::backProjectGs(const int frameNr) {
    frames_[frameNr-1].backProjectGs();
}

// set depthmap
void Camera::setDepthMap(const int frameNr, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> depth_map) {
    frames_[frameNr-1].setDepthMap(depth_map);
}

// set synthetic (groundtruth) depth map
void Camera::setSyntheticDepthMap(const int frameNr) {
    frames_[frameNr-1].setSyntheticDepthMapRs();
}

// test method to ensure correct projection
void Camera::testProjection() {
    RsFrame frame = frames_[0];
    frame.setSyntheticDepthMapRs();

    Eigen::Vector3d true_coordinates;
    Eigen::Vector3d world_coordinates;
    Eigen::Vector3d camera_coordinates1;
    Eigen::Vector3d camera_coordinates2;

    Eigen::Vector2d image_coordinates1;
    Eigen::Vector2d image_coordinates2;
    for (int x=0; x<frame.getCols(); ++x) {
        for (int y=0; y<frame.getRows(); ++y) {

            true_coordinates = frame.getUnprojectedWorldCoordinates(Eigen::Vector2d(x,y));
            camera_coordinates1 = frame.worldToCameraFrame(true_coordinates, y);
            camera_coordinates2 = frame.planeToSpace(Eigen::Vector2d(x,y));
            image_coordinates1 = frame.spaceToPlane(camera_coordinates1);
            image_coordinates2 = frame.spaceToPlane(camera_coordinates2);
            world_coordinates = frame.cameraToWorldFrame(camera_coordinates2, y);

            if (true_coordinates.norm()>0){

                std::cout << "x, y: " << x << ", " << y << std::endl;
                std::cout << "true coordinates: " << true_coordinates << std::endl;
                std::cout << "camera coordinates1: " << camera_coordinates1 << std::endl;
                std::cout << "camera coordinates2: " << camera_coordinates2 << std::endl;
                std::cout << "calculated coordinates: " << world_coordinates << std::endl;
                std::cout << "image coordinates1: " << image_coordinates1 << std::endl;
                std::cout << "image coordinates2: " << image_coordinates2 << std::endl;
                std::cout << "---------------------------------------------" << std::endl;
            }
        }
    }
}

// return depth map
Eigen::MatrixXd Camera::getDepthMap(const int frameNr) {
    return frames_[frameNr-1].getDepthMap();
}

// get ground truth depth map (only possible with synthetic data)
// !!!!This might override the stored depth map!!!!
Eigen::MatrixXd Camera::getGroundTruthDepthMap(const int frameNr) {
    frames_[frameNr-1].setSyntheticDepthMapRs();
    return frames_[frameNr-1].getDepthMap();
}

// create point cloud of the 3D points in world coordinate system
void Camera::createPointCloud(const int frameNr, const std::string fileName) {

    RsFrame frame = frames_[frameNr-1];
    cv::Mat coordinates = frame.get3dCoordinates();
    cv::Mat colors = frame.getRsImage();

    // safty checks
    CV_Assert(coordinates.total() == colors.total());       // If not same size, assert
    CV_Assert(coordinates.type() == CV_32FC3 &&
                  colors.type() == CV_8UC3);                // If not 3 channels and good type
    CV_Assert(coordinates.isContinuous() &&
                  colors.isContinuous());                   // If not continuous in memory

    int cols = frame.getCols();
    int rows = frame.getRows();

    // borders of point cloud (in case only part of image should be extracted)
    int dx_start = 00;
    int dx_end = 00;
    int dy_start = 0;
    int dy_end = 00;

    // extract wanted area
    cv::Mat_<cv::Vec3f> coordinates_reduced(rows-dy_start-dy_end, cols-dx_start-dx_end);
    cv::Mat_<cv::Vec3b> colors_reduced(rows-dy_start-dy_end, cols-dx_start-dx_end);
    for (int x=dx_start; x<cols-dx_end; ++x) {
        for (int y=dy_start; y<rows-dy_end; ++y) {
            coordinates_reduced.at<cv::Vec3f>(y-dy_start,x-dx_start) = coordinates.at<cv::Vec3f>(y,x);
            colors_reduced.at<cv::Vec3b>(y-dy_start,x-dx_start) = colors.at<cv::Vec3b>(y,x);
        }
    }

    // pointer to data
    const float* pData = coordinates_reduced.ptr<float>(0);
    const unsigned char* pColor = colors_reduced.ptr<unsigned char>(0);
    const unsigned long number_iterations = 3*coordinates_reduced.total();

    std::ofstream outputfile(fileName);

    // create header
    outputfile << "ply" << std::endl
               << "format ascii 1.0" << std::endl
               << "comment PLY File created by RS aware SfM wrapper" << std::endl
               << "element vertex " << coordinates_reduced.total() << std::endl
               << "property float x" << std::endl
               << "property float y" << std::endl
               << "property float z" << std::endl
               << "property uchar red" << std::endl
               << "property uchar green" << std::endl
               << "property uchar blue" << std::endl
               << "end_header" << std::endl;


    // create output
    for (int i=0; i<number_iterations;  i+=3) {
        // loop through the coordinates
        for (unsigned int j = 0; j<3; j++) {
            outputfile << std::setprecision(9) << pData[i+j] << " ";
        }
        // loop through the colors
        // OpenCV uses BGR format, so the order of writing is reverse to comply with the RGB format
        for (int j = 2; j>=0; j--) {
            outputfile << (unsigned short)pColor[i+j] << (j==0?"":" ");
        }
        outputfile << std::endl;
    }
    outputfile.close();
    std::cout << "Point cloud file " << fileName << " created." << std::endl;
}

// use small motion wrapping to create GS image out of RS image with help of angular and linear velocity
void Camera::smallMotionWrapping(const int frameNr, const Eigen::Vector3d &linear_velocity,
                                 const Eigen::Vector3d &angular_velocity, const double k) {
    frames_[frameNr-1].smallMotionWrapping(linear_velocity, angular_velocity, k);
}


// create image where each pixel represents the Euclidean error of the 3D points corresponding to the RS image
// at the same pixel position
// variable max_norm corresponds to width (100%) pixel value
cv::Mat Camera::createErrorImage(const int frameNr, const double max_norm) {
    RsFrame frame = frames_[frameNr-1];
    const int cols = frame.getCols();
    const int rows = frame.getRows();

    cv::Mat error_image(rows, cols, CV_8U, cv::Scalar(0));
    cv::Mat world_points = frame.get3dCoordinates();
    Eigen::Vector3d Point_true;
    Eigen::Vector3d Point_estimate;

    // get real depth map
    Eigen::MatrixXd real_depth_map = frame.getGroundtruthDepthMap();
    Eigen::VectorXd scales = Eigen::VectorXd::Zero(3*cols*rows);
    cv::Mat_<cv::Vec3f> estimated_coordinates_3d = frame.get3dCoordinates();
    cv::Mat_<cv::Vec3f> true_coordinates_3d(rows, cols);

    // relocate camera, to be able to compare estimate with ground truth
    frame.relocatePose();

    int number_outliers = 0;
    for (int x=0; x<cols; ++x) {
        for (int y=0; y<rows; ++y) {
            double true_z = real_depth_map(y,x);

            // get ground truth 3D point in world coordinates (from synthetic depth map)
            Point_true = frame.planeToSpace(Eigen::Vector2d(x,y), true_z);
            Point_true = frame.cameraToWorldFrame(Point_true, y, false);
            cv::Vec3f &point_true = true_coordinates_3d.at<cv::Vec3f>(y,x);
            point_true[0] = Point_true.x();
            point_true[1] = Point_true.y();
            point_true[2] = Point_true.z();

            // estimation of 3D point
            cv::Vec3f &point_est = estimated_coordinates_3d.at<cv::Vec3f>(y,x);

            // get scale between ground truth and estimate
            scales(x*3*rows + 3*y) = point_est[0]/point_true[0];
            scales(x*3*rows + 3*y+1) = point_est[1]/point_true[1];
            scales(x*3*rows + 3*y+2) = point_est[2]/point_true[2];

            // remove outliers (if scale is to big or negative)
            if (std::abs(point_est[0]/point_true[0]) > 10 ) {
                scales(x*3*rows + 3*y) = 0;
                number_outliers++;
            }
            if (std::abs(point_est[1]/point_true[1]) > 10 ) {
                scales(x * 3 * rows + 3*y+1) = 0;
                number_outliers++;
            }
            if (std::abs(point_est[2]/point_true[2]) > 10 ) {
                scales(x * 3 * rows + 3*y+2) = 0;
                number_outliers++;
            }
        }
    }

    std::cout << "number of outliers: " << number_outliers << std::endl;
    // calculate mean scale while only using the inliers
    double sum=0;
    int inliers=0;
    for (int i = 0; i<3*cols*rows; ++i) {
        if (scales(i) != 0 && scales(i) == scales(i)) {
            inliers++;
            sum += scales(i);
        }
    }
    double scale = sum/double(inliers);
    std::cout << "mean scale: " << scales.mean() << std::endl;

    // write Euclidean error as pixel value
    for (int x=0; x<cols; ++x) {
        for (int y=0; y<rows; ++y) {

            cv::Vec3f &point_est = estimated_coordinates_3d.at<cv::Vec3f>(y,x);
            Point_estimate.x() = point_est[0] / scale;
            Point_estimate.y() = point_est[1] / scale;
            Point_estimate.z() = point_est[2] / scale;

            cv::Vec3f &point_true = true_coordinates_3d.at<cv::Vec3f>(y,x);
            Point_true.x() = point_true[0];
            Point_true.y() = point_true[1];
            Point_true.z() = point_true[2];

            error_image.at<char>(y, x) = int((Point_estimate - Point_true).norm()*255/max_norm + 0.5);
        }
     }
    return error_image;
}

// calculate mean reprojection error over all 3D points of the corresponding RS image
double Camera::meanReprojectionError(const int frameNr) {
    RsFrame frame = frames_[frameNr-1];
    const int cols = frame.getCols();
    const int rows = frame.getRows();

    cv::Mat world_points = frame.get3dCoordinates();
    Eigen::Vector3d Point_true;
    Eigen::Vector3d Point_estimate;

    Eigen::MatrixXd real_depth_map = frame.getGroundtruthDepthMap();
    Eigen::VectorXd scales = Eigen::VectorXd::Zero(3*cols*rows);


    cv::Mat_<cv::Vec3f> estimated_coordinates_3d = frame.get3dCoordinates();
    cv::Mat_<cv::Vec3f> true_coordinates_3d(rows, cols);

    // relocate camera, to be able to compare ground truth with estimate
    frame.relocatePose();

    int number_outliers = 0;
    for (int x=0; x<cols; ++x) {
        for (int y=0; y<rows; ++y) {
            double true_z = real_depth_map(y,x);

            // get ground truth 3D point
            Point_true = frame.planeToSpace(Eigen::Vector2d(x,y), true_z);
            Point_true = frame.cameraToWorldFrame(Point_true, y, false);
            cv::Vec3f &point_true = true_coordinates_3d.at<cv::Vec3f>(y,x);
            point_true[0] = Point_true.x();
            point_true[1] = Point_true.y();
            point_true[2] = Point_true.z();

            // get estimated 3D point
            cv::Vec3f &point_est = estimated_coordinates_3d.at<cv::Vec3f>(y,x);

            // calculate scale between ground truth and estimate
            scales(x*3*rows + 3*y) = point_est[0]/point_true[0];
            scales(x*3*rows + 3*y+1) = point_est[1]/point_true[1];
            scales(x*3*rows + 3*y+2) = point_est[2]/point_true[2];

            // remove outliers (if scale is to big or negative)
            if (std::abs(point_est[0]/point_true[0]) > 10 ) {
                scales(x*3*rows + 3*y) = 0;
                number_outliers++;
            }
            if (std::abs(point_est[1]/point_true[1]) > 10 ) {
                scales(x * 3 * rows + 3*y+1) = 0;
                number_outliers++;
            }
            if (std::abs(point_est[2]/point_true[2]) > 10 ) {
                scales(x * 3 * rows + 3*y+2) = 0;
                number_outliers++;
            }
        }
    }

    std::cout << "number of outliers: " << number_outliers << std::endl;
    // calculate mean scale while only using the inliers
    double sum=0;
    int inliers=0;
    for (int i = 0; i<3*cols*rows; ++i) {
        if (scales(i) != 0 && scales(i) == scales(i)) {
            inliers++;
            sum += scales(i);
        }
    }
    double scale = sum/double(inliers);
    std::cout << "mean scale: " << scales.mean() << std::endl;


    // add all Euclidian errors up
    double sum_error = 0;
    inliers = 0;
    for (int x=0; x<cols; ++x) {
        for (int y=0; y<rows; ++y) {

            cv::Vec3f &point_est = estimated_coordinates_3d.at<cv::Vec3f>(y,x);
            Point_estimate.x() = point_est[0] / scale;
            Point_estimate.y() = point_est[1] / scale;
            Point_estimate.z() = point_est[2] / scale;

            cv::Vec3f &point_true = true_coordinates_3d.at<cv::Vec3f>(y,x);
            Point_true.x() = point_true[0];
            Point_true.y() = point_true[1];
            Point_true.z() = point_true[2];

            if ((Point_estimate.array() == Point_estimate.array()).all() && (Point_true.array() == Point_true.array()).all()  ) {
                if ((Point_estimate - Point_true).norm() < 50){
                    sum_error += (Point_estimate - Point_true).norm();
                    inliers++;
                }
            }
        }
     }
    std::cout << "points used for error meassure: " << inliers << std::endl;

    // return mean error
    return sum_error * 1.0/inliers;
}

// check if a pixel should be considered black (darker than threshold)
bool Camera::isBlackPixel(const cv::Vec3b point, const unsigned threshold) {
    return cv::norm(point) <= threshold;
}

// check if a pixel is within a colorful area i.e. all pixels in distance offset are not black
bool Camera::isColorfulArea(const cv::Mat image_in, const unsigned row, const unsigned col, const unsigned offset) {
    const auto point_above = image_in.at<cv::Vec3b>(row-offset, col);
    const auto point_below = image_in.at<cv::Vec3b>(row+offset, col);
    const auto point_left = image_in.at<cv::Vec3b>(row, col-offset);
    const auto point_right = image_in.at<cv::Vec3b>(row, col+offset);

    const unsigned black_threshold = 15;

    return (!isBlackPixel(point_above, black_threshold) || !isBlackPixel(point_below, black_threshold) ||
            !isBlackPixel(point_left, black_threshold) || !isBlackPixel(point_right, black_threshold));
}

// interpolate a pixel to have the averaged color of all pixels in distance offset that are not black
cv::Vec3b Camera::interpolateAreaColor(const cv::Mat image_in, const unsigned row, const unsigned col, const unsigned offset) {
    cv::Vec3d sum = cv::Vec3d(0,0,0);
    unsigned count = 0;
    const unsigned black_threshold = 15;

    // add the color value for all non-black pixels
    const auto point_above = image_in.at<cv::Vec3b>(row-offset, col);
    if (!isBlackPixel(point_above, black_threshold)) {
        sum += static_cast<cv::Vec3d>(point_above);
        count++;
    }

    const auto point_below = image_in.at<cv::Vec3b>(row+offset, col);
    if (!isBlackPixel(point_below, black_threshold)) {
        sum += static_cast<cv::Vec3d>(point_below);
        count++;
    }

    const auto point_left = image_in.at<cv::Vec3b>(row, col-offset);
    if (!isBlackPixel(point_left, black_threshold)) {
        sum += static_cast<cv::Vec3d>(point_left);
        count++;
    }

    const auto point_right = image_in.at<cv::Vec3b>(row, col+offset);
    if (!isBlackPixel(point_right, black_threshold)) {
        sum += static_cast<cv::Vec3d>(point_right);
        count++;
    }

    // calculate average color
    cv::Vec3b average;
    if (count > 0) {
        average = static_cast<cv::Vec3b>((1 / static_cast<double>(count)) * sum);
    } else {
        average = cv::Vec3b(0, 0, 0);
    }
    return average;
}

// fix cracky images taken from a discretized depth map using interpolation
cv::Mat Camera::interpolateCrackyImage(cv::Mat image_in, const unsigned offset) {
    std::cout << "Interpolating image to reduce cracks." << std::endl;

    cv::Mat image_out = image_in.clone();
    const unsigned black_threshold = 15;

    // iterate over image expect outer-most pixels
    for (int row = offset; row < image_in.rows-offset; row++) {
        for (int col = offset; col < image_in.cols-offset; col++) {
            const auto point_of_interest = image_in.at<cv::Vec3b>(row, col);
            if (isBlackPixel(point_of_interest, black_threshold)) { // reached black pixel
                if (isColorfulArea(image_in, row, col, offset)) { // black pixel in colorful area
                    // get interpolation around pixel
                    cv::Vec3b average = interpolateAreaColor(image_in, row, col, offset);
                    // set pixel to interpolated value
                    image_out.at<cv::Vec3b>(row, col) = average;
                }
            }
        }
    }
    return image_out;
}

// shift BGR color channels in an entire image
cv::Mat Camera::shiftChannelBGR(cv::Mat image_in, double shift_blue, double shift_green, double shift_red) {
    cv::Mat shifted = image_in.clone()*0;

    // iterate over entire image
    for (int row = 0; row < image_in.rows; row++) {
        for(int col = 0; col < image_in.cols; col++) {
            cv::Vec3b point = image_in.at<cv::Vec3b>(row, col);

            // shift blue channel
            double blue_shifted = point.val[0] * shift_blue;
            if (blue_shifted > 255) {
                blue_shifted = 255;
            } else if (blue_shifted < 0) {
                blue_shifted = 0;
            }

            // shift green channel
            double green_shifted = point.val[1] * shift_green;
            if (green_shifted > 255) {
                green_shifted = 255;
            } else if (green_shifted < 0) {
                green_shifted = 0;
            }

            // shift red channel
            double red_shifted = point.val[2] * shift_red;
            if (red_shifted > 255) {
                red_shifted = 255;
            } else if (red_shifted < 0) {
                red_shifted = 0;
            }

            shifted.at<cv::Vec3b>(row,col) = cv::Vec3b(static_cast<uint8_t >(blue_shifted),
                                                       static_cast<uint8_t >(green_shifted),
                                                       static_cast<uint8_t >(red_shifted));
        }
    }
    return shifted;
}

// generate overlay between two images
cv::Mat Camera::createOverlayImage(cv::Mat original_image, cv::Mat shift_image) {
    const unsigned black_threshold = 15;

    cv::Mat image_out = original_image.clone()*0;

    // iterate over entire image
    for (int row = 0; row < original_image.rows; row++) {
        for(int col = 0; col < original_image.cols; col++) {
            cv::Vec3b point_shift = shift_image.at<cv::Vec3b>(row, col);
            cv::Vec3b point_original = original_image.at<cv::Vec3b>(row, col);

            if (norm(point_shift) > black_threshold) { // reached non-black pixel
                // get multiplier (use more of the more colorful pixel)
                double multiplier = norm(point_original) / (norm(point_original) + norm(point_shift));
                image_out.at<cv::Vec3b>(row, col) = multiplier * point_original + (1-multiplier) * point_shift;
            } else {
                image_out.at<cv::Vec3b>(row, col) = point_original;
            }
        }
    }
    return image_out;
}

// reconstruct the second image from a sequence only given the first frame and the flow to the second frame
cv::Mat Camera::reconstructImageFromFlow(cv::Mat_<cv::Point_<double>> flow_image) {
    cv::Mat original_image = this->getFrame(1).getRsImage().clone();
    cv::Mat reconstructed_image = original_image.clone();
    reconstructed_image.setTo(cv::Scalar(0, 0, 0));

    // iterate over entire image
    for(int u = 0; u < flow_image.cols; ++u) {
        for (int v = 0; v < flow_image.rows; ++v) {
            cv::Point_<double> flow_in_point = flow_image.at<cv::Point_<double>>(v, u);

            int dx = floor(flow_in_point.x + 0.5);
            int dy = floor(flow_in_point.y + 0.5);

            int new_y = v + dy;
            int new_x = u + dx;

            // shift pixel if it is still inside the image
            if ((new_x > 0 && new_x < flow_image.cols) && (new_y > 0 && new_y < flow_image.rows)) {
                reconstructed_image.at<cv::Vec3b>(new_y, new_x) = original_image.at<cv::Vec3b>(v, u);
            }
        }
    }
    return reconstructed_image;
}

