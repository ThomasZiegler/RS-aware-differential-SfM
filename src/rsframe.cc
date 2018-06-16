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


Created on 27.04.2018
    Author: Felix Graule
  Modified: Thomas Ziegler

*/


#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "rsframe.h"
#include <fstream>
#include <iostream>

// add an image to RS frame
void RsFrame::setImage(cv::Mat image) {
    rows_ = image.rows;
    cols_ = image.cols;
    image_ = image;

    for (int i = 0; i < rows_; ++i) {
        scanlines_.push_back(Scanline());
    }
}

// add depth map to RS frame
void RsFrame::setDepthMap(const Eigen::MatrixXd &depth_map) {
    depth_map_ = depth_map;
}

// add GS depth map to RS frame (used for testing)
void RsFrame::setDepthMapGs(const Eigen::MatrixXd &depth_map_gs) {
    gs_depth_map_ = depth_map_gs;
}

// add GS image to RS frame
void RsFrame::setGsImage(cv::Mat image) {
    gs_image_ = image;
}

// add RS unprojection map (corresponding 3D coordinate for a pixel in RS image)
bool RsFrame::setUnprojectionMapRs(const std::string csv_unprojection_x, const std::string csv_unprojection_y,
                                   const std::string csv_unprojection_z) {
    bool return_value = true;
    bool files_correct = true;
    bool nr_scanlines_correct = true;

    std::string input;
    Eigen::MatrixXd unprojection_map_x = Eigen::MatrixXd::Zero(rows_, cols_);
    Eigen::MatrixXd unprojection_map_y = Eigen::MatrixXd::Zero(rows_, cols_);
    Eigen::MatrixXd unprojection_map_z = Eigen::MatrixXd::Zero(rows_, cols_);

    unsigned long nr_scanlines = scanlines_.size();
    long nr_lines_x = 0;
    long nr_lines_y = 0;
    long nr_lines_z = 0;

    // check file names
    std::ifstream map_x(csv_unprojection_x);
    std::ifstream map_y(csv_unprojection_y);
    std::ifstream map_z(csv_unprojection_z);

    if (!map_x) {
        std::cout << csv_unprojection_x << " Is not a valid file path for the unprojection x map file!"
                  << std::endl;
        files_correct = false;
    }
    if (!map_y) {
        std::cout << csv_unprojection_y << " Is not a valid file path for the unprojection y map file!"
                  << std::endl;
        files_correct = false;
    }
    if (!map_z) {
        std::cout << csv_unprojection_z << " Is not a valid file path for the unprojection z map file!"
                  << std::endl;
        files_correct = false;
    }

    if (files_correct) {
        // load data
        std::cout << "Loading of unproject data files successful" << std::endl;

        nr_lines_x = std::count(std::istreambuf_iterator<char>(map_x), std::istreambuf_iterator<char>(), '\n');
        nr_lines_y = std::count(std::istreambuf_iterator<char>(map_y), std::istreambuf_iterator<char>(), '\n');
        nr_lines_z = std::count(std::istreambuf_iterator<char>(map_z), std::istreambuf_iterator<char>(), '\n');

        // reset file, starting at top line
        map_x.clear();
        map_x.seekg(0, std::ios::beg);
        map_y.clear();
        map_y.seekg(0, std::ios::beg);
        map_z.clear();
        map_z.seekg(0, std::ios::beg);

        if (nr_lines_x != nr_scanlines) {
            std::cout << "The number of lines: " << nr_lines_x << " in the file: " << csv_unprojection_x
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }
        if (nr_lines_y != nr_scanlines) {
            std::cout << "The number of lines: " << nr_lines_y << " in the file: " << csv_unprojection_y
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }
        if (nr_lines_z != nr_scanlines) {
            std::cout << "The number of lines: " << nr_lines_z << " in the file: " << csv_unprojection_z
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }

        if (nr_scanlines_correct) {

            // process X coordinates line-by-line
            int i = 0;
            while (map_x.good()) {

                // 1st column element
                getline(map_x, input, ',');
                // break if no line left in file
                if (map_x.eof()) {
                    break;
                }
                unprojection_map_x(i, 0) = ::atof(input.c_str());

                // 2nd to 2nd last column element
                for (int j = 1; j < cols_ - 1; ++j) {
                    getline(map_x, input, ',');
                    unprojection_map_x(i, j) = ::atof(input.c_str());
                }

                // last column
                getline(map_x, input, '\n');
                unprojection_map_x(i, cols_ - 1) = ::atof(input.c_str());

                ++i;
            }

            // process Y coordinates line-by-line
            i = 0;
            while (map_y.good()) {
                // 1st column element
                getline(map_y, input, ',');
                // break if no line left in file
                if (map_y.eof()) {
                    break;
                }
                unprojection_map_y(i, 0) = ::atof(input.c_str());

                // 2nd to 2nd last column elements
                for (int j = 1; j < cols_ - 1; ++j) {
                    getline(map_y, input, ',');
                    unprojection_map_y(i, j) = ::atof(input.c_str());
                }

                // last column
                getline(map_y, input, '\n');
                unprojection_map_y(i, cols_ - 1) = ::atof(input.c_str());

                ++i;
            }

            // process Z coordinates line-by-line
            i = 0;
            while (map_z.good()) {
                // 1st column
                getline(map_z, input, ',');
                // break if no line left in file
                if (map_z.eof()) {
                    break;
                }
                unprojection_map_z(i, 0) = ::atof(input.c_str());

                // 2nd to 2nd last column elements
                for (int j = 1; j < cols_ - 1; ++j) {
                    getline(map_z, input, ',');
                    unprojection_map_z(i, j) = ::atof(input.c_str());

                }

                // last column
                getline(map_z, input, '\n');
                unprojection_map_z(i, cols_ - 1) = ::atof(input.c_str());

                ++i;
            }

            unprojection_map_x_ = unprojection_map_x;
            unprojection_map_y_ = unprojection_map_y;
            unprojection_map_z_ = unprojection_map_z;

        } else {
            std::cout << "Unprojection maps not set" << std::endl;
            return_value = false;
        }

    } else {
        std::cout << "Unprojection maps not set" << std::endl;
        return_value = false;
    }

    return return_value;
}


// add GS unprojection map (corresponding 3D coordinate for a pixel in GS image) (for testing used)
bool RsFrame::setUnprojectionMapGs(const std::string csv_unprojection_x, const std::string csv_unprojection_y,
                                   const std::string csv_unprojection_z) {
    bool return_value = true;
    bool files_correct = true;
    bool nr_scanlines_correct = true;

    std::string input;
    Eigen::MatrixXd unprojection_map_x = Eigen::MatrixXd::Zero(rows_, cols_);
    Eigen::MatrixXd unprojection_map_y = Eigen::MatrixXd::Zero(rows_, cols_);
    Eigen::MatrixXd unprojection_map_z = Eigen::MatrixXd::Zero(rows_, cols_);

    unsigned long nr_scanlines = scanlines_.size();
    long nr_lines_x = 0;
    long nr_lines_y = 0;
    long nr_lines_z = 0;

    // check file names
    std::ifstream map_x(csv_unprojection_x);
    std::ifstream map_y(csv_unprojection_y);
    std::ifstream map_z(csv_unprojection_z);

    if (!map_x) {
        std::cout << csv_unprojection_x << " Is not a valid file path for the unprojection x map file!"
                  << std::endl;
        files_correct = false;
    }
    if (!map_y) {
        std::cout << csv_unprojection_y << " Is not a valid file path for the unprojection y map file!"
                  << std::endl;
        files_correct = false;
    }
    if (!map_z) {
        std::cout << csv_unprojection_z << " Is not a valid file path for the unprojection z map file!"
                  << std::endl;
        files_correct = false;
    }

    if (files_correct) {
        // load data
        std::cout << "Loading of unproject data files for GS image successful" << std::endl;

        nr_lines_x = std::count(std::istreambuf_iterator<char>(map_x), std::istreambuf_iterator<char>(), '\n');
        nr_lines_y = std::count(std::istreambuf_iterator<char>(map_y), std::istreambuf_iterator<char>(), '\n');
        nr_lines_z = std::count(std::istreambuf_iterator<char>(map_z), std::istreambuf_iterator<char>(), '\n');

        // reset file, starting at top line
        map_x.clear();
        map_x.seekg(0, std::ios::beg);
        map_y.clear();
        map_y.seekg(0, std::ios::beg);
        map_z.clear();
        map_z.seekg(0, std::ios::beg);

        if (nr_lines_x != nr_scanlines) {
            std::cout << "The number of lines: " << nr_lines_x << " in the file: " << csv_unprojection_x
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }
        if (nr_lines_y != nr_scanlines) {
            std::cout << "The number of lines: " << nr_lines_y << " in the file: " << csv_unprojection_y
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }
        if (nr_lines_z != nr_scanlines) {
            std::cout << "The number of lines: " << nr_lines_z << " in the file: " << csv_unprojection_z
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }

        if (nr_scanlines_correct) {
            // process X coordinates
            int i = 0;
            while (map_x.good()) {
                // first column
                getline(map_x, input, ',');
                // break if no line left in file
                if (map_x.eof()) {
                    break;
                }
                unprojection_map_x(i, 0) = ::atof(input.c_str());

                // 2nd to 2nd last column
                for (int j = 1; j < cols_ - 1; ++j) {
                    getline(map_x, input, ',');
                    unprojection_map_x(i, j) = ::atof(input.c_str());
                }

                // last column
                getline(map_x, input, '\n');
                unprojection_map_x(i, cols_ - 1) = ::atof(input.c_str());

                ++i;
            }

            // process Y coordinates
            i = 0;
            while (map_y.good()) {
                // fist column
                getline(map_y, input, ',');
                // break if no line left in file
                if (map_y.eof()) {
                    break;
                }
                unprojection_map_y(i, 0) = ::atof(input.c_str());

                // 2nd to 2nd last column
                for (int j = 1; j < cols_ - 1; ++j) {
                    getline(map_y, input, ',');
                    unprojection_map_y(i, j) = ::atof(input.c_str());
                }

                // last column
                getline(map_y, input, '\n');
                unprojection_map_y(i, cols_ - 1) = ::atof(input.c_str());

                ++i;
            }

            // process Z coordinates
            i = 0;
            while (map_z.good()) {
                // fist column
                getline(map_z, input, ',');
                // break if no line left in file
                if (map_z.eof()){
                    break;
                }
                unprojection_map_z(i, 0) = ::atof(input.c_str());

                // 2nd to 2nd last column
                for (int j = 1; j < cols_ - 1; ++j) {
                    getline(map_z, input, ',');
                    unprojection_map_z(i, j) = ::atof(input.c_str());
                }

                // last column
                getline(map_z, input, '\n');
                unprojection_map_z(i, cols_ - 1) = ::atof(input.c_str());

                ++i;
            }

            gs_unprojection_map_x_ = unprojection_map_x;
            gs_unprojection_map_y_ = unprojection_map_y;
            gs_unprojection_map_z_ = unprojection_map_z;

        } else {
            std::cout << "Unprojection maps for GS image not set" << std::endl;
            return_value = false;
        }

    } else {
        std::cout << "Unprojection maps for GS image not set" << std::endl;
        return_value = false;
    }
    return return_value;
}

// set the gamma value for the RS frame (used for backprojection methods)
void RsFrame::setGamma(const double gamma) {
    gamma_ = gamma;
}

// get number of rows
int RsFrame::getRows() const {
    return rows_;
}

// get number of columns
int RsFrame::getCols() const {
    return cols_;
}

// get RS image
cv::Mat RsFrame::getRsImage() {
    return image_;
}

// get GS image
cv::Mat RsFrame::getGsImage() {
    return gs_image_;
}

// get depth map corresponding to RS image
Eigen::MatrixXd RsFrame::getDepthMap() {
    return depth_map_;
}

// get corresponding 3D coordinates for each pixel of the RS image (set in backprojection)
cv::Mat RsFrame::get3dCoordinates(){
    return coordinates_3d_;
}

// get groundtruth depth map of RS image
Eigen::MatrixXd RsFrame::getGroundtruthDepthMap() {
    Eigen::Vector3d Point_world;
    Eigen::Vector3d Point_cam;
    Eigen::MatrixXd unprojection_map_z= Eigen::MatrixXd::Zero(rows_, cols_);

    for (int y=0; y<rows_; ++y) {
        for (int x=0; x<cols_; ++x) {
            // get 3D point in world coordinate system
            Point_world.x() = unprojection_map_x_(y,x);
            Point_world.y() = unprojection_map_y_(y,x);
            Point_world.z() = unprojection_map_z_(y,x);

            // transform point into camera coordinate system
            if (Point_world.norm()>0) {
                Point_cam = this->worldToCameraFrame(Point_world, y, false);
                unprojection_map_z(y,x) = Point_cam.z();
            }
        }
    }
    return unprojection_map_z;
}

// get number of scanlines (equal to number of rows)
unsigned long RsFrame::getNrScannlines() const {
    return scanlines_.size();
}

// set poses for each scanline of the RS image using synthetic data
bool RsFrame::setPoses(std::string csv_poses, std::string csv_orientation) {
    bool return_value = true;
    bool files_correct = true;
    bool nr_scanlines_correct = true;

    std::string pose_1, pose_2, pose_3;
    std::string rot_1, rot_2, rot_3, rot_4, rot_5, rot_6, rot_7, rot_8, rot_9;
    Eigen::Vector3d translation_vec;
    Eigen::Matrix3d rotation_mat;

    unsigned long nr_scanlines = scanlines_.size();
    long poses_nr_lines = 0;
    long orientation_nr_lines = 0;

    // check file names
    std::ifstream poses(csv_poses);
    std::ifstream orientation(csv_orientation);

    if (!poses) {
        std::cout << csv_poses << " Is not a valid file path for the poses file!" << std::endl;
        files_correct = false;
    }
    if (!orientation) {
        std::cout << csv_orientation << " Is not a valid file path for the orientation file!" << std::endl;
        files_correct = false;
    }

    if (files_correct) {
        // load data
        std::cout << "Loading of poses and orientation data files was successful" << std::endl;

        poses_nr_lines = std::count(std::istreambuf_iterator<char>(poses), std::istreambuf_iterator<char>(), '\n');
        orientation_nr_lines = std::count(std::istreambuf_iterator<char>(orientation),
                                          std::istreambuf_iterator<char>(), '\n');

        // reset files to start at top line
        poses.clear();
        poses.seekg(0, std::ios::beg);
        orientation.clear();
        orientation.seekg(0, std::ios::beg);

        if (poses_nr_lines != nr_scanlines) {
            std::cout << "The number of lines: " << poses_nr_lines << " in the file: " << csv_poses
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }

        if (orientation_nr_lines != nr_scanlines) {
            std::cout << "The number of lines: " << orientation_nr_lines << " in the file: " << csv_orientation
                      << " does not conform with the number of scannlines: " << nr_scanlines << "!\n";
            nr_scanlines_correct = false;
        }

        if (nr_scanlines_correct) {
            // process translation
            int i = 0;
            while (poses.good()) {
                getline(poses, pose_1, ',');
                getline(poses, pose_2, ',');
                getline(poses, pose_3, '\n');
                // break if no line is left in file
                if (poses.eof()){
                    break;
                }

                // set translation
                translation_vec << ::atof(pose_1.c_str()), ::atof(pose_2.c_str()), ::atof(pose_3.c_str());
                scanlines_[i].setTranslation(translation_vec);
                scanlines_[i].setRelativeTranslation(translation_vec);
                ++i;
            }

            // process rotation
            i = 0;
            while (orientation.good()) {
                getline(orientation, rot_1, ',');
                getline(orientation, rot_2, ',');
                getline(orientation, rot_3, ',');
                getline(orientation, rot_4, ',');
                getline(orientation, rot_5, ',');
                getline(orientation, rot_6, ',');
                getline(orientation, rot_7, ',');
                getline(orientation, rot_8, ',');
                getline(orientation, rot_9, '\n');
                // break if no line left in file
                if (orientation.eof()){
                    break;
                }

                rotation_mat << ::atof(rot_1.c_str()), ::atof(rot_2.c_str()), ::atof(rot_3.c_str()), ::atof(
                        rot_4.c_str()), ::atof(rot_5.c_str()), ::atof(rot_6.c_str()), ::atof(rot_7.c_str()), ::atof(
                        rot_8.c_str()), ::atof(rot_9.c_str());

                // set rotation
                scanlines_[i].setRotation(rotation_mat);
                scanlines_[i].setRelativeRotation(rotation_mat);
                ++i;
            }
        }
        else {
            std::cout << "NO poses and NO orientations were set" << std::endl;
            return_value = false;
        }
    }
    else {
        std::cout << "Not able to load both data files, NO poses and NO orientations were set" << std::endl;
        return_value = false;
    }
    return return_value;
}

// set the camera intrinsic parameters
void RsFrame::setIntrinsics(const Eigen::Matrix3d &intrinsics) {
    f_x_ = intrinsics(0,0);
    f_y_ = intrinsics(1,1);
    c_x_ = intrinsics(0,2);
    c_y_ = intrinsics(1,2);
}


// set depth map for GS image form unprojection map from the synthetic data (used for testing)
void RsFrame::setSyntheticDepthMapGs() {
    Eigen::Vector3d Point_world;
    Eigen::Vector3d Point_cam;

    for (int y=0; y<scanlines_.size(); ++y) {
        for (int x=0; x<cols_; ++x) {
            // get 3D point in world coordinate system
            Point_world.x() = gs_unprojection_map_x_(y,x);
            Point_world.y() = gs_unprojection_map_y_(y,x);
            Point_world.z() = gs_unprojection_map_z_(y,x);

            // transform point into camera coordinate system
            if (Point_world.norm()>0) {
                Point_cam = this->worldToCameraFrame(Point_world, 0);
                gs_depth_map_(y,x) = Point_cam.z();
            }
            else {
                gs_depth_map_(y,x) = 0;
            }
        }
    }
}

// set depth image for RS image from unprojection map from the syntehtic data (used for testing)
void RsFrame::setSyntheticDepthMapRs() {
    Eigen::Vector3d Point_world;
    Eigen::Vector3d Point_cam;
    double count_z = 0;
    int nr_z_values = 0;

    for (int y=0; y<scanlines_.size(); ++y) {
        for (int x=0; x<cols_; ++x) {
            // get 3D point in world coordinate system
            Point_world.x() = unprojection_map_x_(y,x);
            Point_world.y() = unprojection_map_y_(y,x);
            Point_world.z() = unprojection_map_z_(y,x);

            // transform point into camera coordinate system
            if (Point_world.norm()>0) {
                Point_cam = this->worldToCameraFrame(Point_world, y);
                depth_map_(y,x) = Point_cam.z();
                count_z += Point_cam.z();
                ++nr_z_values;
            }
            else {
                depth_map_(y,x) = 0;
            }
        }
    }
    std::cout << "z mean: " << count_z * 1.0 / nr_z_values << std::endl;
}

// get corresponding 3D world coordinate for a pixel in the RS image (using unprojection maps)
Eigen::Vector3d RsFrame::getUnprojectedWorldCoordinates(const Eigen::Vector2d &point) {
    Eigen::Vector3d Point;

    Point.x() = unprojection_map_x_.coeff(point.y(), point.x());
    Point.y() = unprojection_map_y_.coeff(point.y(), point.x());
    Point.z() = unprojection_map_z_.coeff(point.y(), point.x());

    return Point;
}

// transform from 3D space (in camera coordinates) to 2D image plane
Eigen::Vector2d RsFrame::spaceToPlane(const Eigen::Vector3d& Point) {
    Eigen::Vector2d point_tmp;
    Eigen::Vector2d point;

    // normalize coordinates
    point_tmp.x() = Point.x()/Point.z();
    point_tmp.y() = Point.y()/Point.z();

    // apply intrinsics
    point.x() = point_tmp.x()*f_x_ + c_x_;
    point.y() = point_tmp.y()*f_x_ + c_y_;

    return point;
}

// transform from 2D image plane to 3D space (in camera coordinate system)
// default parameter for z_value is 0, which means the depth map of RS image is used
Eigen::Vector3d RsFrame::planeToSpace(const Eigen::Vector2d& point, double z_value) {
    Eigen::Vector2d point_tmp;
    Eigen::Vector3d Point;

    // transform to normalized coordinates
    point_tmp.x() = (point.x() - c_x_) * 1.0 / f_x_;
    point_tmp.y() = (point.y() - c_y_) * 1.0 / f_y_;

    Point.x() = point_tmp.x();
    Point.y() = point_tmp.y();
    Point.z() = 1.0;

    // if default depth value, use depth map
    if (z_value==0){
       z_value = depth_map_.coeff(int(point.y()), int(point.x()));
    }

    // scale point with z value
    return z_value * Point;
}

// transform 2D plane coordinate to pixel value
Eigen::Vector2i RsFrame::coordinateToPixel(const Eigen::Vector2d point) {
    Eigen::Vector2i pixel;
    pixel.x() = floor(point.x()+0.5);
    pixel.y() = floor(point.y()+0.5);

    return pixel;
}

// tranform pixel value into 2D coordinate (int vector to double vector)
Eigen::Vector2d RsFrame::pixelToCoordinate(const Eigen::Vector2i pixel) {
    Eigen::Vector2d point;
    point.x() = 1.0*pixel.x();
    point.y() = 1.0*pixel.y();

    return point;
}

// transform 3D point in wolrd coordinates into camera coordinates
// either use relative positions or absolute positions (used for testing)
Eigen::Vector3d RsFrame::worldToCameraFrame(const Eigen::Vector3d& Point, const int scanlineNr, bool useRelative) {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    Eigen::Matrix4d P;

    // get pose of scanline
    if (useRelative) {
        R = scanlines_[scanlineNr].getRelativeRotation();
        t = scanlines_[scanlineNr].getRelativeTranslation();
    } else {
        R = scanlines_[scanlineNr].getRotation();
        t = scanlines_[scanlineNr].getTranslation();
    }

    // construct projection matrix
    P = Eigen::Matrix4d::Zero();
    P.topLeftCorner<3,3>() = R;
    P.topRightCorner<3,1>() = t;
    P(3,3) = 1.0;

    return (P*Point.homogeneous()).head<3>();
}

// transform 3D point in camera coordinates into world coordinates
// either use relative positions or absolute positions (used for testing)
Eigen::Vector3d RsFrame::cameraToWorldFrame(const Eigen::Vector3d& Point, const int scanlineNr, bool useRelative) {
    Eigen::Matrix3d R_transpose;
    Eigen::Vector3d t;
    Eigen::Matrix4d P_inverse;

    // get pose of scanline
    if (useRelative) {
        R_transpose = (scanlines_[scanlineNr].getRelativeRotation()).transpose();
        t = scanlines_[scanlineNr].getRelativeTranslation();
    } else{
        R_transpose = (scanlines_[scanlineNr].getRotation()).transpose();
        t = scanlines_[scanlineNr].getTranslation();
    }

    // construct (inverse) porjection matrix
    P_inverse = Eigen::Matrix4d::Zero();
    P_inverse.topLeftCorner<3,3>() = R_transpose;
    P_inverse.topRightCorner<3,1>() = t;
    P_inverse(0,3) = -R_transpose.row(0)*t;
    P_inverse(1,3) = -R_transpose.row(1)*t;
    P_inverse(2,3) = -R_transpose.row(2)*t;
    P_inverse(3,3) = 1.0;

    return (P_inverse*Point.homogeneous()).head<3>();
}

// calculate best matching image coordinate for given 3D point in world coordinates
// tries every scanline pose and uses the one with the smallest displacement
Eigen::Vector2d RsFrame::calculateImageCoordinatesRsFrame(const Eigen::Vector3d &Point) {
    Eigen::Vector3d Point_cam;
    Eigen::Vector2d point;
    Eigen::Matrix<double, 3, 4> projection_matrix;


    double min_diff = INFINITY;
    double diff;
    int best_row;
    for(int i=0; i<rows_; ++i) {
        // calculate 2D plane coordinate from 3D point in world coordinates
        Point_cam = this->worldToCameraFrame(Point, i);
        point = this->spaceToPlane(Point_cam);

        // displacement of scanline and y coordinate
        diff =std::abs(point.y()-double(i));

        // update if current displacement is smaller
        if (diff< min_diff) {
            min_diff = diff;
            best_row = i;
        }
    }
    // use best scanline pose
    Point_cam = this->worldToCameraFrame(Point, best_row);
    point = this->spaceToPlane(Point_cam);

    return point;
}

// set relative pose using angular and linear velocity
void RsFrame::setRelativePose(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const double k) {
    // set initial pose of first scanline
    Eigen::Vector3d P0 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
    scanlines_[0].setRelativeTranslation(P0);
    scanlines_[0].setRelativeRotation(R0);


    Eigen::Matrix3d angular_velocity_skew_matrix;
    Eigen::Matrix3d R_new;

    // transform angular velocity in skew symmetric matrix  \in so(3)
    Eigen::Vector3d omegas = angular_velocity;
    angular_velocity_skew_matrix << 0, -omegas.z(), omegas.y(),
                                    omegas.z(), 0, -omegas.x(),
                                    -omegas.y(), omegas.x(), 0;

    for (int i=1; i<scanlines_.size(); ++i) {
        // calculate beta for current scanline
        double beta_1 = (gamma_ * i/rows_ + 0.5*k*(gamma_*gamma_*i*i)/(rows_*rows_))*(2.0/(2.0+k));

        // get rotation matrix form skew symmetric matrix (approximation of exponentional mapping)
        R_new = (Eigen::Matrix3d::Identity(3,3) + beta_1*angular_velocity_skew_matrix);

        // set pose
        scanlines_[i].setRelativeRotation(R0*R_new);
        scanlines_[i].setRelativeTranslation(P0 + beta_1*linear_velocity) ;
    }

}

// use relative poses to project RS image into 3D space and back into GS image
void RsFrame::backProject() {
    cv::Mat_<cv::Vec3f> coordinates_3d(rows_, cols_);
    Eigen::Vector2d point_gs;
    Eigen::Vector3d Point_world;
    Eigen::Vector3d Point_cam;

    cv::Mat gs_image = image_.clone();
    gs_image *= 0;

    for (int y=0; y<scanlines_.size();++y) {
        for (int x=0; x<cols_; ++x) {

            if (image_.at<cv::Vec3b>(y,x) != cv::Vec3b(1,1,1)) {
                // project rs image into 3D space
                Point_cam = this->planeToSpace(Eigen::Vector2d(x,y));
                Point_world = this->cameraToWorldFrame(Point_cam, y);

                // project point cloud back into image with pose of first scanline => GS image
                Point_cam = this->worldToCameraFrame(Point_world, 0);
                point_gs = this->spaceToPlane(Point_cam);

                // store 3D point coordinates
                cv::Vec3f &point = coordinates_3d.at<cv::Vec3f>(y,x);
                point[0] = Point_world.x();
                point[1] = Point_world.y();
                point[2] = Point_world.z();

                // if point lies in image copy pixel from RS image
                if (int(point_gs.x()+0.5) >= 0 && int(point_gs.x()+0.5)<cols_ && int(point_gs.y()+0.5) >= 0 && int(point_gs.y()+0.5)<rows_ ){
                    gs_image.at<cv::Vec3b>(int(point_gs.y()+0.5), int(point_gs.x()+0.5)) = image_.at<cv::Vec3b>(y,x);
                }
            }
        }
    }
    coordinates_3d_ = coordinates_3d;
    gs_image_ = gs_image;
}

// use pose of first scanline to project RS image into 3D space and back into GS image
void RsFrame::backProjectGs() {
    cv::Mat_<cv::Vec3f> coordinates_3d(rows_, cols_);
    Eigen::Vector2d point_gs;
    Eigen::Vector3d Point_world;
    Eigen::Vector3d Point_cam;

    cv::Mat gs_image = image_.clone();
    gs_image *= 0;

    for (int y=0; y<scanlines_.size();++y) {
        for (int x=0; x<cols_; ++x) {
            if (image_.at<cv::Vec3b>(y,x) != cv::Vec3b(1,1,1)) {

                // project RS pixel into 3D space
                Point_cam = this->planeToSpace(Eigen::Vector2d(x,y));
                Point_world = this->cameraToWorldFrame(Point_cam, 0);

                // project point cloud back into GS image
                Point_cam = this->worldToCameraFrame(Point_world, 0);
                point_gs = this->spaceToPlane(Point_cam);

                // store 3D point coordinates
                cv::Vec3f &point = coordinates_3d.at<cv::Vec3f>(y,x);
                point[0] = Point_world.x();
                point[1] = Point_world.y();
                point[2] = Point_world.z();

                // if point lies in image copy pixel from RS image
                if (int(point_gs.x()+0.5) >= 0 && int(point_gs.x()+0.5)<cols_ && int(point_gs.y()+0.5) >= 0 && int(point_gs.y()+0.5)<rows_ ){
                    gs_image.at<cv::Vec3b>(int(point_gs.y()+0.5), int(point_gs.x()+0.5)) = image_.at<cv::Vec3b>(y,x);
                }
            }
        }
    }
    coordinates_3d_ = coordinates_3d;
    gs_image_ = gs_image;
}


void RsFrame::smallMotionWrapping(const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity,
                                  const double k) {
    double beta_1;
    double u;
    double v;

    Eigen::MatrixXd matrix_A(2, 3);
    Eigen::MatrixXd matrix_B(2, 3);

    double inv_depth;
    Eigen::Vector2d flow;

    int dx, dy;

    cv::Mat gs_image = image_.clone();
    gs_image *= 0;

    Eigen::Vector3d Point_cam;
    Eigen::Vector3d Point_world;
    cv::Mat_<cv::Vec3f> coordinates_3d(rows_, cols_);


    for(int y = 1; y < rows_; ++y) {
        beta_1 = (gamma_ * y / rows_ + 0.5 * k * (gamma_ * gamma_ * y * y) / (rows_ * rows_)) * (2.0 / (2.0 + k));

        for(int x = 1; x < cols_; ++x) {


            // normalize coordinates (image plane at z=1)
            u = (x-c_x_)*1.0/f_x_;
            v = (y-c_y_)*1.0/f_y_;


            // create A, B matrix
            matrix_A << -1, 0, u, 0, -1, v;
            matrix_A = matrix_A * (-1);
            matrix_B << u * v, -(1 + u * u), v, (1 + v * v), -u * v, -u;
            matrix_B = matrix_B * (-1);

            inv_depth = 1.0/depth_map_.coeff(y, x);

            // calculate flow
            flow =  beta_1 * (matrix_A * linear_velocity * inv_depth + matrix_B * angular_velocity);

            // bring back to actual size
            flow.x() *= (f_x_ * 1.0/gamma_);
            flow.y() *= (f_y_ * 1.0/gamma_);
            dx = floor(flow.x()+0.5);
            dy = floor(flow.y()+0.5);

            if (std::abs(dx) > 0 || std::abs(dy)>0) {
                if (y+dy < rows_ && y+dy >= 0 && x+dx < cols_ && x+dx >= 0 )
                    gs_image.at<cv::Vec3b>(y - dy, x - dx) = image_.at<cv::Vec3b>(y, x);
            }

            Point_cam = this->planeToSpace(Eigen::Vector2d(x,y));
            Point_world = this->cameraToWorldFrame(Point_cam, y);

            cv::Vec3f &point = coordinates_3d.at<cv::Vec3f>(y,x);
            point[0] = Point_world.x();
            point[1] = Point_world.y();
            point[2] = Point_world.z();

        }
    }
    coordinates_3d_ = coordinates_3d;
    gs_image_ = gs_image;

}

// relocate pose of each scan line such that the first scan line lies at the origin
// first scan line coincides with pose when using angular and linear velocity
void RsFrame::relocatePose() {
    // set initial pose and orientation as origin
    Eigen::Vector3d initial_pose = scanlines_[0].getTranslation();
    Eigen::Matrix3d initial_orientation = scanlines_[0].getRotation();
    Eigen::Vector3d tmp_pose;
    Eigen::Matrix3d tmp_orientation;

    for (int i = 1; i < rows_; ++i) {
        tmp_pose = scanlines_[i].getTranslation();
        scanlines_[i].setTranslation(tmp_pose - initial_pose);

        tmp_orientation = scanlines_[i].getRotation();
        scanlines_[i].setRotation(initial_orientation.inverse() * tmp_orientation);
    }
}
