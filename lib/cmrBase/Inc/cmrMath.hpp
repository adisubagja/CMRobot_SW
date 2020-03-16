/*
 * cmrMath.hpp
 *
 * Description: This file provide variable math function,
 * mainly about matrix and vector computation
 *
 * Author: Feijian.Ni
 * Data: 2020.2.13
 */

#ifndef CMRMATH_HPP_
#define CMRMATH_HPP_

#include "Eigen/Dense"

namespace cmr {

//! constant value

//！define matrix
typedef Eigen::Matrix<double, 2, 2> cmrMatrix2d;
typedef Eigen::Matrix<float, 2, 2> cmrMatrix2f;
typedef Eigen::Matrix<int, 2, 2> cmrMatrix2i;

typedef Eigen::Matrix<double, 3, 3> cmrMatrix3d;
typedef Eigen::Matrix<float, 3, 3> cmrMatrix3f;
typedef Eigen::Matrix<int, 3, 3> cmrMatrix3i;

typedef Eigen::Matrix<double, 4, 4> cmrMatrix4d;
typedef Eigen::Matrix<float, 4, 4> cmrMatrix4f;
typedef Eigen::Matrix<int, 4, 4> cmrMatrix4i;

typedef Eigen::Matrix<double, 5, 5> cmrMatrix5d;
typedef Eigen::Matrix<float, 5, 5> cmrMatrix5f;
typedef Eigen::Matrix<int, 5, 5> cmrMatrix5i;

typedef Eigen::Matrix<double, 6, 6> cmrMatrix6d;
typedef Eigen::Matrix<float, 6, 6> cmrMatrix6f;
typedef Eigen::Matrix<int, 6, 6> cmrMatrix6i;

typedef Eigen::Matrix<double, 7, 7> cmrMatrix7d;
typedef Eigen::Matrix<float, 7, 7> cmrMatrix7f;
typedef Eigen::Matrix<int, 7, 7> cmrMatrix7i;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cmrMatrixXd;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cmrMatrixXf;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cmrMatrixXi;

//! define column vector
typedef Eigen::Matrix<double, 3, 1> cmrVector3d;
typedef Eigen::Matrix<float, 3, 1> cmrVector3f;
typedef Eigen::Matrix<double, 3, 1> cmrVector3i;

typedef Eigen::Matrix<double, 6, 1> cmrVector6d;
typedef Eigen::Matrix<float, 6, 1> cmrVector6f;
typedef Eigen::Matrix<double, 6, 1> cmrVector6i;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> cmrVectorXd;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> cmrVectorXf;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> cmrVectorXi;

//! define row vecotr
typedef Eigen::Matrix<double, 1, 3> cmrRowVector3d;
typedef Eigen::Matrix<float, 1, 3> cmrRowVector3f;
typedef Eigen::Matrix<double, 1, 3> cmrRowVector3i;

typedef Eigen::Matrix<double, 1, 6> cmrRowVector6d;
typedef Eigen::Matrix<float, 1, 6> cmrRowVector6f;
typedef Eigen::Matrix<double, 1, 6> cmrRowVector6i;

typedef Eigen::Matrix<double, 1, Eigen::Dynamic> cmrRowVectorXd;
typedef Eigen::Matrix<float, 1, Eigen::Dynamic> cmrRowVectorXf;
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> cmrRowVectorXi;

//! define array
typedef Eigen::Array<double, 3, 3> cmrArray3d;
typedef Eigen::Array<float, 3, 3> cmrArray3f;
typedef Eigen::Array<double, 3, 3> cmrArray3i;

typedef Eigen::Array<double, 6, 6> cmrArray6d;
typedef Eigen::Array<float, 6, 6> cmrArray6f;
typedef Eigen::Array<double, 6, 6> cmrArray6i;

//! define quaterniond
typedef Eigen::Quaterniond cmrQuat;

//! define angle axis
typedef Eigen::AngleAxisd cmrAngleAxis;

} // namespace cmr

#endif