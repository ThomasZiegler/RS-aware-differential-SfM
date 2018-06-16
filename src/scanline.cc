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


#include "scanline.h"
#include <Eigen/Geometry>

Scanline::Scanline() :
        rotation_(Eigen::Matrix3d::Zero()), translation_(Eigen::Vector3d::Zero()) {}

Scanline::Scanline(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) :
        rotation_(rotation), translation_(translation) {}

const Eigen::Matrix3d &Scanline::getRotation() const {
    return rotation_;
}

void Scanline::setRotation(const Eigen::Matrix3d &rotation) {
    rotation_ = rotation;
}

const Eigen::Vector3d &Scanline::getTranslation() const {
    return translation_;
}

void Scanline::setTranslation(const Eigen::Vector3d &translation) {
    translation_ = translation;
}
const Eigen::Matrix3d &Scanline::getRelativeRotation() const {
    return relative_rotation_;
}

void Scanline::setRelativeRotation(const Eigen::Matrix3d &rotation) {
    relative_rotation_ = rotation;
}

const Eigen::Vector3d &Scanline::getRelativeTranslation() const {
    return relative_translation_;
}

void Scanline::setRelativeTranslation(const Eigen::Vector3d &translation) {
    relative_translation_ = translation;
}
