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

#include "multi_body.hpp"

#ifdef USE_EIGEN
#include "math/tiny/tiny_eigen_helper.h"
#endif
#include "dynamics/kinematics.hpp"
#include "dynamics/jacobian.hpp"

namespace TINY {
    template <typename Algebra>
    struct TinyIKTarget {
        typedef typename Algebra::Vector3 Vector3;
        typedef typename Algebra::Scalar Scalar;

        /**
         * Index of the link that should reach a given target.
         */
        int link_index;
        /**
         * Target position in world coordinates.
         */
        Vector3 position;
        /**
         * Position relative to the link which is supposed to achieve the target
         * position.
         */
        Vector3 body_point{
            Vector3(Algebra::zero(), Algebra::zero(), Algebra::zero()) };

        TinyIKTarget(int link_index, const Vector3& position)
            : link_index(link_index), position(position) {}
    };

    enum TinyIKMethod {
        IK_JAC_TRANSPOSE,
        IK_JAC_PINV,
        IK_DAMPED_LM,
    };

    enum TinyIKStatus {
        IK_RESULT_FAILED = 0,
        IK_RESULT_CONVERGED,
        IK_RESULT_REACHED
    };

    template <typename Scalar>
    struct TinyIKResult
    {
        int iter;
        TinyIKStatus ik_status;
        Scalar residual;
    };
    

    /**
     * Implements methods for inverse kinematics (IK).
     * The implementation has been inspired from the Rigid Body Dynamics Library
     * (RBDL) by Martin Felis.
     */
    template <typename Algebra,
        TinyIKMethod Method = IK_JAC_TRANSPOSE>
        struct TinyInverseKinematics {
        typedef typename Algebra::Scalar Scalar;
        
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
        typedef ::TINY::TinyIKTarget<Algebra> Target;
        typedef ::tds::MultiBody<Algebra> MultiBody;
        typedef typename Algebra::Vector3 Vector3;
        typedef typename Algebra::VectorX VectorX;
        typedef typename Algebra::Matrix3 Matrix3;
        typedef typename Algebra::MatrixX MatrixXxX;
        typedef ::tds::Transform<Algebra> SpatialTransform;

        std::vector<Target> targets;

        int max_iterations{ 20 };

        /**
         * Damping factor for the Levenberg-Marquardt method. This parameter has to
         * be chosen carefully. In case of unreachable positions, higher values (e.g
         * 0.9) can be helpful. Otherwise values of 0.0001, 0.001, 0.01, 0.1 might
         * yield good results. See the literature for best practices.
         */
        Scalar lambda{ 0.02 };

        /**
         * Acceptable Euclidean distance between the source and target points.
         */
        Scalar target_tolerance{ 0.001 };

        /**
         * Minimum possible change in joint coordinates until the algorithm
         * terminates.
         */
        Scalar step_tolerance{ 1e-8 };

        /**
         * Step sized used by the Jacobian Transpose and the Jacobian Pseudo Inverse
         * method.
         */
        Scalar alpha{ 5 };

        /**
         * If non-empty, keep joint angles close to the reference configuration,
         * weighted by `weight_reference`.
         */
        VectorX q_reference;
        Scalar weight_reference{ 0.2 };
        
        TinyIKResult<Scalar> compute(const MultiBody& mb, const VectorX& q_init,
            VectorX& q) const {

            TinyIKResult<Scalar> result;
            result.residual = Algebra::fraction(-1,1);

            assert(q_init.size() == mb.dof());
            assert(q_reference.size()==0 || q_reference.size() == q_init.size());
            q = q_init;

            const int q_offset = mb.is_floating() ? 7 : 0;
            const int qd_offset = mb.is_floating() ? 6 : 0;

            MatrixXxX J(3 * targets.size(), mb.dof_qd());
            VectorX e(3 * targets.size());

            SpatialTransform base_X_world;
            std::vector<SpatialTransform> links_X_world;
            std::vector<SpatialTransform> links_X_base;
            

            for (result.iter = 0; result.iter < max_iterations; result.iter++) {
                ::tds::forward_kinematics_q< Algebra>(mb, q, &base_X_world, &links_X_world, &links_X_base);
                for (int k = 0; k < static_cast<int>(targets.size()); ++k) {
                    const Target& target = targets[k];
                    bool is_local_point = true;
                    Vector3 local_point_in_base_frame = links_X_base[target.link_index].apply(target.body_point);
                    auto G = ::tds::point_jacobian<Algebra>(mb, q, target.link_index, local_point_in_base_frame, is_local_point);

                    if (mb.is_floating())
                    {
                        Matrix3 cr;
                        Algebra::set_zero(cr);
                        Algebra::assign_block(G, cr, 0, 0);
                        G(0, 3) = Algebra::zero();
                        G(1, 4) = Algebra::zero();
                        G(2, 5) = Algebra::zero();
                    }
                    Vector3 local_actual_pos = links_X_base[target.link_index].apply(target.body_point);
                    Vector3 local_target_pos = base_X_world.apply_inverse(target.position);
                    
                    for (unsigned int i = 0; i < 3; i++) {
                        for (unsigned int j = 0; j < mb.dof_qd(); j++) {
                            unsigned int row = k * 3 + i;
                            J(row, j) = G(i, j);
                        }
                        
                        Scalar diff = local_target_pos[i] - local_actual_pos[i];
                        e[k * 3 + i] = diff;
                    }
                }

                result.residual = Algebra::norm(e);
                // abort if we are getting "close"
                if (result.residual < target_tolerance) {
                    result.ik_status = IK_RESULT_REACHED;
                    return result;
                }

                VectorX delta_theta;
                if constexpr (kMethod == IK_JAC_TRANSPOSE) {
                    delta_theta = Algebra::mul_transpose(J, e);
                }
                if constexpr (kMethod == IK_JAC_PINV) {

#ifdef USE_EIGEN
                  auto J_pinv = pseudo_inverse(J);
                  
                  delta_theta = J_pinv * e;
#endif

                } else if constexpr (kMethod == IK_DAMPED_LM) {
#ifdef USE_EIGEN
                    MatrixXxX JJTe_lambda2_I = J * J.transpose();
                    assert(JJTe_lambda2_I.m_cols == JJTe_lambda2_I.m_rows &&
                        JJTe_lambda2_I.m_cols == 3 * targets.size());
                    // apply damping (damped least squares)
                    for (int i = 0; i < JJTe_lambda2_I.m_cols; ++i) {
                        JJTe_lambda2_I(i, i) += lambda * lambda;
                    }
                    Eigen::Matrix<double, Eigen::Dynamic, 1> eigen_mat =
                        TINY::to_eigen(JJTe_lambda2_I)
                        .colPivHouseholderQr()
                        .solve(TINY::to_eigen(e));
                    VectorX z = TINY::from_eigen_v<Algebra>(eigen_mat);
                    delta_theta = J.mul_transpose(z);
#endif //USE_EIGEN
                }

                Scalar sq_length = Algebra::zero();
                for (int i = 0; i < mb.dof_actuated(); ++i) {
                    Scalar delta = delta_theta[i + qd_offset];
                    Scalar& qi = q[i + q_offset];
                    qi += alpha * delta;
                    if (Algebra::size(q_reference) != 0) {
                        qi += weight_reference * (q_reference[i + q_offset] - qi);
                    }
                    sq_length += delta * delta;
                }

                if (sq_length < step_tolerance * step_tolerance) {
                    result.ik_status = IK_RESULT_CONVERGED;
                    return result;
                }
            }
            result.ik_status = IK_RESULT_FAILED;
            return result;
        }
        TinyIKResult<Scalar> compute(const MultiBody& mb, VectorX& q) const {
            return compute(mb, mb.q_, q);
        }
    };
}; //namespace TINY

#endif  // TINY_INVERSE_KINEMATICS_H
