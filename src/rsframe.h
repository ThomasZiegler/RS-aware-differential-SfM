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


#ifndef RS_SFM_RS_FRAME_H
#define RS_SFM_RS_FRAME_H

#include "scanline.h"
#include <Eigen/Geometry>
#include <opencv2/core.hpp>


class RsFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Method to add rolling shutter image.
     *
     * @param image (cv::Mat) RS image.
     */
    void setImage(cv::Mat image);

    /**
     * Method to add gloal shutter image.
     *
     * @param image GS image taken at position of first scanline.
     */
    void setGsImage(cv::Mat image);

    /**
     * Method to add depth map.
     *
     * @param depth_map (Eigen::MatrixXd) matrix of the same size as the RS image containing the depth values
     */
    void setDepthMap(const Eigen::MatrixXd &depth_map);

    /**
     * Method to add depth map for the GS image.
     *
     * @param depth_map (Eigen::MatrixXd) matrix of the same size as the GS image containing the depth values
     */
    void setDepthMapGs(const Eigen::MatrixXd &depth_map_gs);

    /**
     * Method to add the unprojection maps for the x, y, and z coordinate. The unprojection map represents the world
     * coordinates (in global coordinate system) of the pixel in the RS image.
     *
     * @param csv_unprojection_x (std::sting) path to the csv file storing the unprojection map for the x coordinates
     * @param csv_unprojection_y (std::sting) path to the csv file storing the unprojection map for the y coordinates
     * @param csv_unprojection_z (std::sting) path to the csv file storing the unprojection map for the z coordinates
     * @return (bool) indicator if extracting and storing of the unprojection maps succeeded.
     */
    bool setUnprojectionMapRs(const std::string csv_unprojection_x, const std::string csv_unprojection_y,
                              const std::string csv_unprojection_z);

    /**
     * Method to add the unprojection maps for the x, y, and z coordinate. The unprojection map represents the world
     * coordinates (in global coordinate system) of the pixel in the GS image.
     *
     * @param csv_unprojection_x (std::sting) path to the csv file storing the unprojection map for the x coordinates
     * @param csv_unprojection_y (std::sting) path to the csv file storing the unprojection map for the y coordinates
     * @param csv_unprojection_z (std::sting) path to the csv file storing the unprojection map for the z coordinates
     * @return (bool) indicator if extracting and storing of the unprojection maps succeeded.
     */
    bool setUnprojectionMapGs(const std::string csv_unprojection_x, const std::string csv_unprojection_y,
                              const std::string csv_unprojection_z);
    /**
     * Method to add the poses and orientations of the camera for each scanline in the rolling shutter image.
     *
     * @param poses_csv (std::string) path to the csv file storing the poses of each scanline.
     * @param orientation_csv (std::string) path to the csv file storing the orientation of each scanline.
     * @return (bool) indicator if the extracting and storing of the poses and orientations succeeded.
     */
    bool setPoses(std::string csv_poses, std::string csv_orientation);

   /**
    * Method to add the intrinsic matrix of the camera.
    *
    * @param intrinsics (3x3 matrix) containing the intrinsics of the camera.
    */
    void setIntrinsics(const Eigen::Matrix3d &intrinsics);

    /**
     * Method to add the readout time ratio
     *
     * @param gamma (double) readout time ratio
     */
    void setGamma(const double gamma);

    /**
     * Getter for the number of rows of the RS/GS image.
     *
     * @return rows (int) indicating the number of rows.
     */
    int getRows() const;

    /**
     * Getter for the number of columns of the RS/GS image.
     *
     * @return cols (int) indicating the number of columns.
     */
    int getCols() const;

    /**
     * Get RS image from RS frame
     *
     * @return image stored in RS frame
     */
    cv::Mat getRsImage();

    /**
     * Get GS image from RS frame
     *
     * @return image stored in RS frame
     */
    cv::Mat getGsImage();

   /**
    * Get depth map from RS frame.
    *
    * @return (Eigen::MatrixXd) containing the depth map.
    */
    Eigen::MatrixXd getDepthMap();

    /**
     * Get the 3d coordinates in a cv:Mat object corresponding to the RS image
     *
     * @return (cv::Mat) containing the 3D world Coordinates (X,Y,Z) of a pixel in the RS frame
     */
    cv::Mat get3dCoordinates();


    /**
     * Get the unprojection map for the z coordinate in the camera coordinate system. Which can be seen as the true depth
     * map.
     *
     * @return (Eigen::matrixXd) containing the depth map for the z coordinate.
     */
    Eigen::MatrixXd getGroundtruthDepthMap();

    /**
     * Getter for the number of scanlines
     * @return (unsigned long) indicating the number of scanlines in the RS frame. Should coincide with the number of rows.
     */
    unsigned long getNrScannlines() const;

    /**
     * Method to set the deph map for synthetic images. It uses the unprojection Z values to get the exact distance in
     * the z-direction.
     */
    void setSyntheticDepthMapRs();

    /**
     * Method to set the deph map for synthetic GS image. It uses the unprojection Z values of the GS image to get the
     * exact distance in the z-direction.
     */
    void setSyntheticDepthMapGs();

    /**
     * Method returning the 3D world coordinates for a pixel in the RS image of a synthetic picture
     *
     * @param point (2x1 vector) representing the (x, y) pixel position
     * @return (3x1 vector) representing the (X, Y, Z) coordinates in the world frame.
     */
    Eigen::Vector3d getUnprojectedWorldCoordinates(const Eigen::Vector2d &point);

    /**
     * Method returning the 2D image coordinate of the RS image for a given 3D world coordinate.
     *
     * @param point (3x1 vector) representing the 3D world coordinate.
     * @return (2x1 vector) representing the corresponding 2D image coordinate in the RS image.
     */
    Eigen::Vector2d calculateImageCoordinatesRsFrame(const Eigen::Vector3d &point);

    /**
     * Method to convert a point from 3D space in camera coordinate system into 2D image plane coordinates.
     *
     * @param Point (3x1 vector) representing the 3D point in the camera coordinate system.
     * @return (2x1 vector) representing the corresponding point in the image plane.
     */
    Eigen::Vector2d spaceToPlane(const Eigen::Vector3d& Point);

    /**
     * Method to convert a point from the 2D image plane into the a 3D space point in camera coordinate system.
     *
     * @param point (2x1 vector) representing the 2D point in the image plane.
     * @param z_value (double) Z value with which all coordinates are multiplied, if non is given (=0) the value from
     *                         stored in the depth map is used.
     * @return (3x1 vector) representing the corresponding point in the 3D space in the camera coordinate system.
     */
    Eigen::Vector3d planeToSpace(const Eigen::Vector2d& point, double z_value=0);

    /**
     * Method to convert a point in 2D image plane to pixel value.
     *
     * @param point (2x1 vector) representing point in the 2D image plane.
     * @return (2x1 vector) corresponding pixel values (x,y).
     */
    Eigen::Vector2i coordinateToPixel(const Eigen::Vector2d point);

    /**
     * Method to convert pixel values to a point in 2D image plane.
     *
     * @param pixel (2x1 vector) representing the pixel value (x,y).
     * @return (2x1 vector) corresponding point in the 2D image plane.
     */
    Eigen::Vector2d pixelToCoordinate(const Eigen::Vector2i pixel);

    /**
     * Method to convert a point in 3D space in the world coordinate system into a point in the camera coordinate system.
     * It uses the rotation and translation of the given scanline.
     *
     * @param Point (3x1 vector) representing the 3D point in the world coordinate system.
     * @param scanlineNr (int) corresponding scanline number (to get correct rotation and translation) in [0,rows-1].
     * @param useRelative (bool) flag that indicates if relative pose should be used or ground thruth.
     * @return (3x1 vector) representing the corresponding 3D point in the camera coordinate system.
     */
    Eigen::Vector3d worldToCameraFrame(const Eigen::Vector3d& Point, const int scanlineNr, bool useRelative=true);

    /**
     * Method to convert a point in 3D space in the camera coordinate system into a point in the world coordinate system.
     * It uses the rotation and translation of the given scanline.
     *
     * @param Point (3x1 vector) representing the 3D point in the camera coordinate system.
     * @param scanlineNr (int) corresponding scanline number (to get correct rotation and translation) in [0,rows-1].
     * @param useRelative (bool) flag that indicates if relative pose should be used or ground thruth.
     * @return (3x1 vector) representing the corresponding 3D point in the world coordinate system.
     */
    Eigen::Vector3d cameraToWorldFrame(const Eigen::Vector3d& Point, const int scanlineNr, bool useRelative=true);

    /**
     * Method which projects the RS image back into the GS image. The depth map has to be set beforehand!
     *
     * This method also creates the cv::Mat object storing the 3d coordinates used to create a point cloud.
     */
    void backProject();

    /**
     * Method which projects the RS image back into the GS image, however it assumes GS case (all scan line poses are
     * the same). The depth map has to be set beforehand!
     *
     * This method also creates the cv::Mat object storing the 3d coordinates used to create a point cloud.
     */
    void backProjectGs();


    void smallMotionWrapping(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const double k);

    /**
     * Method to set the relative translation and rotation for each scan-line of the RS frame.
     * @param linear_velocity (3x1 vector) containing the estimated linear velocity (due to small motion assumption this
     *        is equal to the difference in the translsation between the first scan lines of two consecutive RS frames.
     * @param anglular_velocity (3x1 vector) containing the estimated angular velocity (due to small motion assumption
     *        this is equal to the difference in the rotation between the first scan lines of two consecutive RS frames.    *
     * @param k (double) constant acceleration parameter
     */
    void setRelativePose(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& anglular_velocity, const double k);

    /**
     * Method that relocates the first scan line of the first RS frame to T=(0,0,0) and R=(1,0,0;0,1,0;0,0,1).
     */
    void relocatePose();


private:
    int rows_;
    int cols_;

    cv::Mat image_;
    cv::Mat gs_image_;

    cv::Mat coordinates_3d_;

    Eigen::MatrixXd depth_map_;
    std::vector<Scanline> scanlines_;

    Eigen::MatrixXd unprojection_map_x_;
    Eigen::MatrixXd unprojection_map_y_;
    Eigen::MatrixXd unprojection_map_z_;

    Eigen::MatrixXd gs_unprojection_map_x_;
    Eigen::MatrixXd gs_unprojection_map_y_;
    Eigen::MatrixXd gs_unprojection_map_z_;

    Eigen::MatrixXd gs_depth_map_;

    // intrinsics
    double f_x_;
    double f_y_;
    double c_x_;
    double c_y_;

    double gamma_;
};

#endif // RS_SFM_RS_FRAME_H
