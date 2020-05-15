/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TINY_INVERSE_KINEMATICS_H
#define TINY_INVERSE_KINEMATICS_H

#include "tiny_multi_body.h"
#ifdef USE_EIGEN
#include "tiny_eigen_helper.h"
#endif

template <typename Scalar, typename Utils>
struct TinyIKTarget {
  typedef ::TinyVector3<Scalar, Utils> TinyVector3;

  /**
   * Index of the link that should reach a given target.
   */
  int link_index;
  /**
   * Target position in world coordinates.
   */
  TinyVector3 position;
  /**
   * Position relative to the link which is supposed to achieve the target
   * position.
   */
  TinyVector3 body_point{
      TinyVector3(Utils::zero(), Utils::zero(), Utils::zero())};

  TinyIKTarget(int link_index, const TinyVector3& position)
      : link_index(link_index), position(position) {}
};

enum TinyIKMethod {
  IK_JAC_TRANSPOSE,
  IK_JAC_PINV,
  IK_DAMPED_LM,
};

enum TinyIKResult {
  IK_RESULT_FAILED = 0,
  IK_RESULT_CONVERGED,
  IK_RESULT_REACHED
};

/**
 * Implements methods for inverse kinematics (IK).
 * The implementation has been inspired from the Rigid Body Dynamics Library
 * (RBDL) by Martin Felis.
 */
template <typename Scalar, typename Utils,
          TinyIKMethod Method = IK_JAC_TRANSPOSE>
struct TinyInverseKinematics {
#ifdef USE_EIGEN
  // Eigen helpers not available for our other scalar types
  static_assert(Method == IK_JAC_TRANSPOSE || std::is_same_v<Scalar, double>,
                "Only Jacobian Transpose is supported as IK method for "
                "non-double scalar types.");
#else
  static_assert(
      Method == IK_JAC_TRANSPOSE || std::is_same_v<Scalar, double>,
      "USE_EIGEN must be set to use IK methods other than Jacobian Transpose.");
#endif

  static const TinyIKMethod kMethod = Method;
  typedef ::TinyIKTarget<Scalar, Utils> Target;
  typedef ::TinyMultiBody<Scalar, Utils> MultiBody;
  typedef ::TinyVector3<Scalar, Utils> Vector3;
  typedef ::TinyVectorX<Scalar, Utils> VectorX;
  typedef ::TinyMatrixXxX<Scalar, Utils> MatrixXxX;
  typedef ::TinySpatialTransform<Scalar, Utils> SpatialTransform;

  std::vector<Target> targets;

  int max_iterations{10};

  /**
   * Damping factor for the Levenberg-Marquardt method. This parameter has to
   * be chosen carefully. In case of unreachable positions, higher values (e.g
   * 0.9) can be helpful. Otherwise values of 0.0001, 0.001, 0.01, 0.1 might
   * yield good results. See the literature for best practices.
   */
  Scalar lambda{0.02};

  /**
   * Acceptable Euclidean distance between the source and target points.
   */
  Scalar target_tolerance{0.01};

  /**
   * Minimum possible change in joint coordinates until the algorithm
   * terminates.
   */
  Scalar step_tolerance{1e-8};

  /**
   * Step sized used by the Jacobian Transpose and the Jacobian Pseudo Inverse
   * method.
   */
  double alpha{5};

  /**
   * If non-empty, keep joint angles close to the reference configuration,
   * weighted by `weight_reference`.
   */
  std::vector<Scalar> q_reference;
  Scalar weight_reference{0.2};

  TinyIKResult compute(const MultiBody& mb, const std::vector<Scalar>& q_init,
                       std::vector<Scalar>& q) const {
    assert(q_init.size() == mb.dof());
    assert(q_reference.empty() || q_reference.size() == q_init.size());
    q = q_init;

    const int q_offset = mb.m_isFloating ? 7 : 0;
    const int qd_offset = mb.m_isFloating ? 6 : 0;

    MatrixXxX J(3 * targets.size(), mb.dof_qd());
    VectorX e(3 * targets.size());

    SpatialTransform base_X_world;
    std::vector<SpatialTransform> links_X_world;

    for (int iter = 0; iter < max_iterations; iter++) {
      mb.forward_kinematics_q(q, &base_X_world, &links_X_world);
      for (int k = 0; k < static_cast<int>(targets.size()); ++k) {
        const Target& target = targets[k];
        auto G = mb.point_jacobian(q, target.link_index, target.body_point);

        Vector3 actual =
            links_X_world[target.link_index].apply(target.body_point);
        for (unsigned int i = 0; i < 3; i++) {
          for (unsigned int j = 0; j < mb.dof_qd(); j++) {
            unsigned int row = k * 3 + i;
            J(row, j) = G(i, j);
          }
          e[k * 3 + i] = target.position[i] - actual[i];
        }
      }

      // abort if we are getting "close"
      if (e.length() < target_tolerance) {
        printf("Reached target close enough after %i steps.\n", iter);
        return IK_RESULT_REACHED;
      }

      VectorX delta_theta;
      if constexpr (kMethod == IK_JAC_TRANSPOSE) {
        delta_theta = J.mul_transpose(e);
      }
      if constexpr (kMethod == IK_JAC_PINV) {
        auto J_pinv = helper::pseudo_inverse(J);
        delta_theta = J_pinv * e;
      } else if constexpr (kMethod == IK_DAMPED_LM) {
        MatrixXxX JJTe_lambda2_I = J * J.transpose();
        assert(JJTe_lambda2_I.m_cols == JJTe_lambda2_I.m_rows &&
               JJTe_lambda2_I.m_cols == 3 * targets.size());
        // apply damping (damped least squares)
        for (int i = 0; i < JJTe_lambda2_I.m_cols; ++i) {
          JJTe_lambda2_I(i, i) += lambda * lambda;
        }
        Eigen::Matrix<double, Eigen::Dynamic, 1> eigen_mat =
            helper::to_eigen(JJTe_lambda2_I)
                .colPivHouseholderQr()
                .solve(helper::to_eigen(e));
        VectorX z = helper::from_eigen_v<double, DoubleUtils>(eigen_mat);
        delta_theta = J.mul_transpose(z);
      }

      Scalar sq_length = 0;
      for (int i = 0; i < mb.dof_actuated(); ++i) {
        Scalar delta = delta_theta[i + qd_offset];
        Scalar& qi = q[i + q_offset];
        qi += alpha * delta;
        if (!q_reference.empty()) {
          qi += weight_reference * (q_reference[i + q_offset] - qi);
        }
        sq_length += delta * delta;
      }

      if (sq_length < step_tolerance * step_tolerance) {
        printf("Reached convergence after %i steps.\n", iter);
        return IK_RESULT_CONVERGED;
      }
    }
    return IK_RESULT_FAILED;
  }
  bool compute(const MultiBody& mb, std::vector<Scalar>& q) const {
    return compute(mb, mb.m_q, q);
  }
};

#endif  // TINY_INVERSE_KINEMATICS_H
