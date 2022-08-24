#pragma once

#include "pybind11/pybind11.h"

namespace py = pybind11;

// NUMPY
#include "pybind11/numpy.h"

// EIGEN
#include "pybind11/eigen.h"

/**
 * NumPy to Eigen behavior
 * Type                                    | Accept c-style | Accept f-style | require writable | Posted to compatible different Ref types |
 * py::EigenDRef<const Eigen::MatrixXd>    | ref            | ref            | no               | temp copy                                |
 * py::EigenDRef<Eigen::MatrixXd>          | ref            | ref            | yes              | compile time error                       |
 * Eigen::Ref<const Eigen::MatrixXd>       | ref            | copy           | no               | temp copy                                |
 * Eigen::Ref<const ERMatrix<double>>      | copy           | ref            | no               | temp copy                                |
 * Eigen::Ref<Eigen::MatrixXd>             | no             | ref            | yes              | compile time error                       |
 * Eigen::Ref<ERMatrix<double>>            | ref            | no             | yes              | compile time error                       |
 *
 * Eigen::MatrixXd::conservativeResize can be used to replace std::vector<gpis::Point<T, DIM>>
 */

#include "pybind11/functional.h"
#include "pybind11/stl.h"
#include "pybind11/iostream.h"
