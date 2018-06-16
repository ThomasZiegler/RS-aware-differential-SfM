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


#ifndef RS_SFM_SCANLINE_H
#define RS_SFM_SCANLINE_H

#include <Eigen/Geometry>

class Scanline {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Default constructor for class Scanline, initializes the camera rotation and pose with zeros
     */
    Scanline();

    /**
     * Constructor of class Scanline
     * @param rotation (3x1) vector containing the camera pose for the given scanline
     * @param translation (3x3) matrix containing the camera rotation for the given scanline
     */
    Scanline(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation);

    /**
     * Getter for the camera rotation matrix
     * @return (3x3) matrix containing the camera rotation matrix for the given scanline
     */
    const Eigen::Matrix3d &getRotation() const;

    /**
     * Getter for the camera rotation matrix
     * @return (3x3) matrix containing the camera rotation matrix for the given scanline
     */
    const Eigen::Matrix3d &getRelativeRotation() const;

    /**
     * Getter for the camera pose vector
     * @return (3x1) vector containing the camera pose vector for the given scanline
     */
    const Eigen::Vector3d &getTranslation() const;

     /**
     * Getter for the camera pose vector
     * @return (3x1) vector containing the camera pose vector for the given scanline
     */
    const Eigen::Vector3d &getRelativeTranslation() const;

    /**
     * Setter for the camera rotation matrix
     * @param rotation (3x3) matrix containing the camera rotation matrix for the given scanline
     */
    void setRotation(const Eigen::Matrix3d &rotation);

    /**
     * Setter for teh camera pose vector
     * @param translation (3x1) vector containing the camera pose vector for the given scanline
     */
    void setTranslation(const Eigen::Vector3d &translation);

    /**
     * Setter for the camera rotation matrix
     * @param rotation (3x3) matrix containing the camera rotation matrix for the given scanline
     */
    void setRelativeRotation(const Eigen::Matrix3d &rotation);

    /**
     * Setter for teh camera pose vector
     * @param translation (3x1) vector containing the camera pose vector for the given scanline
     */
    void setRelativeTranslation(const Eigen::Vector3d &translation);

private:
    Eigen::Matrix3d rotation_;
    Eigen::Vector3d translation_;

    Eigen::Matrix3d relative_rotation_;
    Eigen::Vector3d relative_translation_;

};

#endif // RS_SFM_SCANLINE_H
