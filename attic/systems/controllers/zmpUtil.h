#pragma once

#include <Eigen/Core>

#include "drake/common/drake_deprecated.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

struct DRAKE_DEPRECATED("2020-02-01",
    "Some attic controllers code is being removed.")
TVLQRData {
  // TODO(tkoolen): move into its own file
  // TODO(tkoolen): turn into class, private members
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;
  Eigen::MatrixXd Qy;
  Eigen::MatrixXd R;
  Eigen::VectorXd u0;
  Eigen::MatrixXd Q1;
  Eigen::MatrixXd R1;
  Eigen::MatrixXd N;
};

DRAKE_DEPRECATED("2020-02-01", "Some attic controllers code is being removed.")
drake::trajectories::ExponentialPlusPiecewisePolynomial<double> s1Trajectory(
    const TVLQRData& sys,
    const drake::trajectories::PiecewisePolynomial<double>& zmp_trajectory,
    const Eigen::Ref<const Eigen::MatrixXd>& S);
