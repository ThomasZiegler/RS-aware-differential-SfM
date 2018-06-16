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


#ifndef RS_SFM_CAMERA_H
#define RS_SFM_CAMERA_H

#include "rsframe.h"
#include <Eigen/Geometry>
#include <opencv2/core.hpp>


class Camera {
public:

    /**
     * Method that returns the 3x3 intrinsic matrix
     *
     * @return (3x3) matrix containing the camera intrinsics.
     */
    Eigen::Matrix3d getIntrinsics();

    /**
    * Method to add an RS frame based on real world data to the camera object.
    *
    * @param rs_image (cv::Mat) rolling shutter image, each scanline has its own camera position
    */
    void addFrameReal(cv::Mat rs_image);

    /**
     * Method to add an RS frame to the camera object. The RS frame contains several scanlines with their own pose and
     * orientation.
     *
     * @param rs_image (cv::Mat) rolling shutter image, each scanline has its own camera position
     * @param gs_image (cv::Mat) global shutter image, taken at the position of the first scanline
     * @param depth_image (cv::Mat) depth image corresponding to the global shutter image
     * @param poses_csv (std::string) path to the csv file storing the poses of each scanline
     * @param orientation_csv (std::string) path to the csv file storing the orientation of each scanline
     * @param csv_unprojection_x (std::sting) path to the csv file storing the unprojection map for the x coordinates
     * @param csv_unprojection_y (std::sting) path to the csv file storing the unprojection map for the y coordinates
     * @param csv_unprojection_z (std::sting) path to the csv file storing the unprojection map for the z coordinates
     */
    void addFrameSynthetic(cv::Mat rs_image, cv::Mat gs_image, cv::Mat depth_image, const std::string poses_csv,
                           const std::string orientation_csv, const std::string csv_unprojection_x,
                           const std::string csv_unprojection_y, const std::string csv_unprojection_z);

    /**
     * Method to add an RS frame to the camera object. The RS frame contains several scanlines with their own pose and
     * orientation.
     *
     * @param rs_image (cv::Mat) rolling shutter image, each scanline has its own camera position
     * @param gs_image (cv::Mat) global shutter image, taken at the position of the first scanline
     * @param depth_image (cv::Mat) depth image corresponding to the global shutter image
     * @param poses_csv (std::string) path to the csv file storing the poses of each scanline
     * @param orientation_csv (std::string) path to the csv file storing the orientation of each scanline
     * @param csv_unprojection_x (std::sting) path to the csv file storing the unprojection map for the x coordinates
     * @param csv_unprojection_y (std::sting) path to the csv file storing the unprojection map for the y coordinates
     * @param csv_unprojection_z (std::sting) path to the csv file storing the unprojection map for the z coordinates
     * @param csv_unprojection_x_gs (std::sting) path to the csv file storing the unprojection map of the GS image
     *        for the x coordinates
     * @param csv_unprojection_y_gs (std::sting) path to the csv file storing the unprojection map of the GS image
     *        for the y coordinates
     * @param csv_unprojection_z_gs (std::sting) path to the csv file storing the unprojection map of the GS image
     *        for the z coordinates
     */
    void addFrameSynthetic(cv::Mat rs_image, cv::Mat gs_image, cv::Mat depth_image, const std::string poses_csv,
                           const std::string orientation_csv, const std::string csv_unproject_x,
                           const std::string csv_unproject_y, const std::string csv_unproject_z,
                           const std::string csv_gs_unproject_x, const std::string csv_gs_unproject_y,
                           const std::string csv_gs_unproject_z);

    /**
     * Method to add the intrinsic matrix of the camera.
     *
     * @param source_camera (std::string) string containing the type of camera with which the images were taken.
     */
    void setIntrinsics(const std::string source_camera);

   /**
    * Method to add the intrinsic matrix of the camera from a csv file.
    *
    * @param csv_intrinsic_matrix (std::string) containing the path to the csv file containing the intrinsic matrix.
    * @param show_messages (bool) true if debug messages should be printed
    */
    bool loadIntrinsicsFromFile(const std::string csv_intrinsic_matrix, bool show_messages);

    /**
     * Method to calculate the flow in means of pixel displacement between two given RS frames. This method only works
     * with synthetic images  where the unprojection maps are given from the renderer.
     *
     * @param frameNr1 (int) number of the start RS frame <= number of added frames.
     * @param frameNr2 (int) number of the target RS frame <= number of added frames.
     * @return flow (cv::Mat) containing an cv::Point at each pixel position representing the displacement dx and dy.
     */
    cv::Mat_<cv::Point_<double>> calculateTrueFlow(const int frameNr1, const int frameNr2);

    /**
     * Generate optical flow based on Farneback or DenseFlow implementation
     *
     * @param frameNr1 Index of first frame
     * @param frameNr2 Index of second frame
     *
     * @return Matrix containing end points of flow vector for each pixel
     */
    cv::Mat_<cv::Point_<double>> calculateDeepFlow(const int frameNr1, const int frameNr2);

    /**
     * Get image showing optical flow intensity / direction as 2D color plot
     *
     * @param in_flow Flow to be shown
     *
     * @return Matrix containing image of flow
     */
    cv::Mat getImageOpticalFlow(cv::Mat_<cv::Point_<double>> flow);

    /**
     * Get frame storing image data
     *
     * @param frameNr integer describing which frame to return (1 or 2)
     *
     * @return RsFrame containing image data
     */
    RsFrame getFrame(const int frameNr);

    /**
     * Method that returns the depth map of the given frame.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     * @return (Eigen::MatrixXd) containing the depth values for each pixel.
     */
    Eigen::MatrixXd getDepthMap(const int frameNr);

    /**
     * Method that returns the depth map of the given frame.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     * @return (Eigen::MatrixXd) containing the depth values for each pixel.
     */
    Eigen::MatrixXd getGroundTruthDepthMap(const int frameNr);

   /**
    * Method to set depth image (presumably) obtained via minimal solver.
    *
    * @param frameNr (int) number of RS frame <= number of added frames.
    * @param depth_map (Eigen::Matrix) depth image.
    */
    void setDepthMap(const int frameNr, Eigen::MatrixXd depth_map);

    /**
     * Method to set the relative translation and rotation for each scan-line of the RS frame.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     * @param k (double) constant acceleration parameter
     * @param linear_velocity (3x1 vector) containing the estimated linear velocity (due to small motion assumption this
     *        is equal to the difference in the translsation between the first scan lines of two consecutive RS frames.
     * @param anglular_velocity (3x1 vector) containing the estimated angular velocity (due to small motion assumption
     *        this is equal to the difference in the rotation between the first scan lines of two consecutive RS frames.
     */
    void setPose(const int frameNr, const double k, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& anglular_velocity);

    /**
     * Method to add the readout time ratio
     *
     * @param gamma (double) readout time ratio
     */
    void setGamma(const double gamma);

    /**
     * Method which projects the RS image back into the GS image. The depth map has to be set beforehand!
     *
     * This method also creates the cv::Mat object storing the 3d coordinates used to create a point cloud.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     */
    void backProject(const int frameNr);

    /**
     * Method which projects the RS image back into the GS image while it assumes GS mode (all scane lines have the
     * same pose). The depth map has to be set beforehand!
     *
     * This method also creates the cv::Mat object storing the 3d coordinates used to create a point cloud.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     */
    void backProjectGs(const int frameNr);

    /**
     * Method which creates the .ply file for visualizing the point cloud. The used format is ASCII.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     * @param fileName (String) containing the output file name and path.
     */
    void createPointCloud(const int frameNr, const std::string fileName);

    /**
     * Method which creates an error image representing the reprojection error between ground truth 3D point in world
     * coordinates and estimated point.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     * @param max_norm (double) value which is represented as white in the error image.
     * @return image representing the error, where the intensity is the magnitude of the error between the 3D points.
     */
    cv::Mat createErrorImage(const int frameNr, const double max_norm);

    /**
     * Method which calculates the mean of the reprojection error (equal to Euclidean error) between ground truth 3D
     * point in world coordinates and estimated point.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     * @return double representing the mean reprojection error between the 3D points.
     */
    double meanReprojectionError(const int frameNr);

    /**
     * Method that sets the depth map of the given frame to the ground truth depth values
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     */

    void setSyntheticDepthMap(const int frameNr);
    /**
     * Method to smooth out the cracks in the output rectification image using interpolation.
     *
     * @param image_in (cv::Mat) image with cracks that should be interpolated.
     * @param offset (unsigned) parameter to define in what region around each pixel should be interpolated.
     * @return interpolated image.
     */
    cv::Mat interpolateCrackyImage(cv::Mat image_in, const unsigned offset);

    /**
     * Method to check if image pixel is black / below a certain threshold.
     *
     * @param point (cv::Vec3b) pixel to check.
     * @param threshold (unsigned) threshold below which a pixel is considered black (norm).
     * @return boolean (true if black, false if not black).
     */
    bool isBlackPixel(cv::Vec3b point, const unsigned threshold);

    /**
     * Method to check if area around an image pixel is covered with color pixels.
     *
     * @param image_in (cv::Mat) image to check in.
     * @param row (unsigned) row of pixel to check.
     * @param col (unsigned) col of pixel to check.
     * @param offset (unsigned) distance from pixel to check in.
     * @return boolean (true if colorful, false if black).
     */
    bool isColorfulArea(const cv::Mat image_in, const unsigned row, const unsigned col, const unsigned offset);

    /**
     * Method to interpolate area around an image pixel.
     *
     * @param image_in (cv::Mat) image to interpolate on.
     * @param row (unsigned) row of pixel to interpolate.
     * @param col (unsigned) col of pixel to interpolate.
     * @param offset (unsigned) distance from pixel to take values for interpolation from in.
     * @return Vec3b containing color of interpolated pixel.
     */
    cv::Vec3b interpolateAreaColor(const cv::Mat image_in, const unsigned row, const unsigned col, const unsigned offset);

    /**
     * Method to shift color channels of an image.
     *
     * @param image_in (cv::Mat) image to shift on.
     * @param shift_blue (double) value to shift blue channel by.
     * @param shift_green (double) value to shift green channel by.
     * @param shift_red (double) value to shift red channel by.
     * @return Mat containing the shifted image.
     */
    cv::Mat shiftChannelBGR(cv::Mat image_in, double shift_blue, double shift_green, double shift_red);

    /**
     * Method to combine two images to a meaningful Overlay.
     *
     * @param original_image (cv::Mat) image that has not been shifted.
     * @param shift_image (cv::Mat) image that was shifted somehow (but is only difference of shift).
     * @return Mat containing the overlay image.
     */
    cv::Mat createOverlayImage(cv::Mat original_image, cv::Mat shift_image);

    /**
     * Method to reconstruct the second image using the first one and the flow.
     *
     * @param flow_image (cv::Mat) image containing the optical flow.
     * @return Mat containing the reconstructed image.
     */
    cv::Mat reconstructImageFromFlow(cv::Mat_<cv::Point_<double>> flow_image);

    void smallMotionWrapping(const int frameNr, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const double k);


    /**
     * Method that prints the relative translation difference between consecutive scan lines of the given frame.
     *
     * @param frameNr (int) number of RS frame <= number of added frames.
     */
    void printRelPose(const int frameNr);

    /**
     * Method for testing the different projection method with groundtruth values
     */
    void testProjection();

    /**
     * Method which shows the flow by addin red arrows representing the flow onto the first RS image
     *
     * @param image first RS image
     * @param flow flow between first and second RS image
     * @param delta_x distance between starting points of the arrow in x direction
     * @param delta_y distance between starting points of the arrow in y direction
     * @return overlay of RS image with added arrows
     */
    cv::Mat flowArrows(const cv::Mat image, const cv::Mat_<cv::Point_<double>> flow, const int delta_x, const int delta_y);


private:
    std::vector<RsFrame> frames_;
    Eigen::Matrix3d K_;

};


#endif //RS_SFM_CAMERA_H
