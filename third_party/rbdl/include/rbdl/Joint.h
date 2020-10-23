/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_JOINT_H
#define RBDL_JOINT_H

#include "rbdl/rbdl_math.h"
#include <assert.h>
#include <iostream>
#include "rbdl/Logging.h"

namespace RigidBodyDynamics {

struct Model;

/** \page joint_description Joint Modeling
 *
 * \section joint_overview Overview
 *
 * The Rigid Body Dynamics Library supports a multitude of joints:
 * revolute, planar, fixed, singularity-free spherical joints and joints
 * with multiple degrees of freedom in any combinations.
 *
 * Fixed joints do not cause any overhead in RBDL as the bodies that are
 * rigidly connected are merged into a single body. For details see \ref
 * joint_models_fixed.
 *
 * Joints with multiple degrees of freedom are emulated by default which
 * means that they are split up into multiple single degree of freedom
 * joints which results in equivalent models. This has the benefit that it
 * simplifies the required algebra and also code branching in RBDL. A
 * special case are joints with three degrees of freedom for which specific
 * joints are available that should be used for performance reasons
 * whenever possible. See \ref joint_three_dof for details.
 *
 * Joints are defined by their motion subspace. For each degree of freedom
 * a one dimensional motion subspace is specified as a Math::SpatialVector.
 * This vector follows the following convention: \f[ (r_x, r_y, r_z, t_x,
 * t_y, t_z) \f]
 *
 * To specify a planar joint with three degrees of freedom for which the
 * first two are translations in \f$x\f$ and \f$y\f$ direction and the last
 * is a rotation around \f$z\f$, the following joint definition can be
 * used:

 * \code Joint planar_joint = Joint (
 *     Math::SpatialVector (0., 0., 0., 1.,  0., 0.),
 *     Math::SpatialVector (0., 0., 0., 0., 1., 0.),
 *     Math::SpatialVector (0., 0., 1., 0., 0., 0.)
 *     );
 * \endcode

 * \note Please note that in the Rigid %Body Dynamics Library all angles
 * are specified in radians.
 *
 * \section joint_models_fixed Fixed Joints
 *
 * Fixed joints do not add an additional degree of freedom to the model.
 * When adding a body that via a fixed joint (i.e. when the type is
 * JointTypeFixed) then the dynamical parameters mass and inertia are
 * merged onto its moving parent. By doing so fixed bodies do not add
 * computational costs when performing dynamics computations.

 * To ensure a consistent API for the Kinematics such fixed bodies have a
 * different range of ids. Where as the ids start at 1 get incremented for
 * each added body, fixed bodies start at Model::fixed_body_discriminator
 * which has a default value of std::numeric_limits<unsigned int>::max() /
 * 2. This means theoretical a maximum of each 2147483646 movable and fixed
 * bodies are possible.

 * To check whether a body is connected by a fixed joint you can use the
 * function Model::IsFixedBodyId().

 * \section joint_three_dof 3-DoF Joints
 *
 * RBDL has highly efficient implementations for the following three degree
 * of freedom joints:
 * <ul>
 *   <li>\ref JointTypeTranslationXYZ which first translates along X, then
 *   Y, and finally Z.</li>
 *   <li>\ref JointTypeEulerZYX which first rotates around Z, then Y, and
 *   then X.</li>
 *   <li>\ref JointTypeEulerXYZ which first rotates around X, then Y, and
 *   then Z.</li>
 *   <li>\ref JointTypeEulerYXZ which first rotates around Y, then X, and
 *   then Z.</li>
 *   <li>\ref JointTypeEulerZXY which first rotates around Z, then X, and
 *   then Y.</li>
 *   <li>\ref JointTypeSpherical which is a singularity free joint that
 *   uses a Quaternion and the bodies angular velocity (see \ref
 *   joint_singularities for details).</li>
 * </ul>
 *
 * These joints can be created by providing the joint type as an argument
 * to the Joint constructor, e.g.:
 *
 * \code Joint joint_rot_zyx = Joint ( JointTypeEulerZYX ); \endcode
 *
 * Using 3-Dof joints is always favourable over using their emulated
 * counterparts as they are considerably faster and describe the same
 * kinematics and dynamics.

 * \section joint_floatingbase Floating-Base Joint (a.k.a. Freeflyer Joint)
 *
 * RBDL has a special joint type for floating-base systems that uses the
 * enum JointTypeFloatingBase. The first three DoF are translations along
 * X,Y, and Z. For the rotational part it uses a JointTypeSpherical joint.
 * It is internally modeled by a JointTypeTranslationXYZ and a
 * JointTypeSpherical joint. It is recommended to only use this joint for
 * the very first body added to the model.
 *
 * Positional variables are translations along X, Y, and Z, and for
 * rotations it uses Quaternions. To set/get the orientation use
 * Model::SetQuaternion () / Model::GetQuaternion() with the body id
 * returned when adding the floating base (i.e. the call to
 * Model::AddBody() or Model::AppendBody()).

 * \section joint_singularities Joint Singularities

 * Singularities in the models arise when a joint has three rotational
 * degrees of freedom and the rotations are described by Euler- or
 * Cardan-angles. The singularities present in these rotation
 * parametrizations (e.g. for ZYX Euler-angles for rotations where a 
 * +/- 90 degrees rotation around the Y-axis) may cause problems in
 * dynamics calculations, such as a rank-deficit joint-space inertia matrix
 * or exploding accelerations in the forward dynamics calculations.
 *
 * For this case RBDL has the special joint type
 * RigidBodyDynamics::JointTypeSpherical. When using this joint type the
 * model does not suffer from singularities, however this also results in
 * a change of interpretation for the values \f$\mathbf{q}, \mathbf{\dot{q}}, \mathbf{\ddot{q}}\f$, and \f$\mathbf{\tau}\f$:
 *
 * <ul>
 * <li> The values in \f$\mathbf{q}\f$ for the joint parameterizes the orientation of a joint using a
 * Quaternion \f$q_i\f$ </li>
 * <li> The values in \f$\mathbf{\dot{q}}\f$ for the joint describe the angular
 * velocity \f$\omega\f$ of the joint in body coordinates</li>
 * <li> The values in \f$\mathbf{\ddot{q}}\f$ for the joint describe the angular
 * acceleration \f$\dot{\omega}\f$ of the joint in body coordinates</li>
 * <li> The values in \f$\mathbf{\tau}\f$ for the joint describe the three couples
 * acting on the body in body coordinates that are actuated by the joint.</li>
 * </ul>

 * As a result, the dimension of the vector \f$\mathbf{q}\f$ is higher than
 * of the vector of the velocity variables. Additionally, the values in
 * \f$\mathbf{\dot{q}}\f$ are \b not the derivative of \f$q\f$ and are therefore
 * denoted by \f$\mathbf{\bar{q}}\f$ (and similarly for the joint
 * accelerations).

 * RBDL stores the Quaternions in \f$\mathbf{q}\f$ such that the 4th component of
 * the joint is appended to \f$\mathbf{q}\f$. E.g. for a model with the joints:
 * TX, Spherical, TY, Spherical, the values of \f$\mathbf{q},\mathbf{\bar{q}},\mathbf{\bar{\bar{q}}},\mathbf{\tau}\f$ are:
 *
 
 \f{eqnarray*}
        \mathbf{q} &=& ( q_{tx}, q_{q1,x}, q_{q1,y}, q_{q1,z}, q_{ty}, q_{q2,x}, q_{q2,y}, q_{q2,z}, q_{q1,w}, q_{q2,w})^T \\
  \mathbf{\bar{q}} &=& ( \dot{q}_{tx}, \omega_{1,x}, \omega_{1,y}, \omega_{1,z}, \dot{q}_{ty}, \omega_{2,x}, \omega_{2,y}, \omega_{2,z} )^T \\
  \mathbf{\bar{\bar{q}}} &=& ( \ddot{q}_{tx}, \dot{\omega}_{1,x}, \dot{\omega}_{1,y}, \dot{\omega}_{1,z}, \ddot{q}_{ty}, \dot{\omega}_{2,x}, \dot{\omega}_{2,y}, \dot{\omega}_{2,z} )^T \\
  \mathbf{\tau} &=& ( \tau_{tx}, \tau_{1,x}, \tau_{1,y}, \tau_{1,z}, \tau_{ty}, \tau_{2,x}, \tau_{2,y}, \tau_{2,z} )^T 
  \f}

  * \subsection spherical_integration Numerical Integration of Quaternions
  *
  * An additional consequence of this is, that special treatment is required
  * when numerically integrating the angular velocities. One possibility is
  * to interpret the angular velocity as an axis-angle pair scaled by the
  * timestep and use it create a quaternion that is applied to the previous
  * Quaternion. Another is to compute the quaternion rates from the angular
  * velocity. For details see James Diebel "Representing Attitude: Euler
  * Angles, Unit Quaternions, and Rotation Vectors", 2006,
  * http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.110.5134.
  *
  */

  /** \brief General types of joints
  */
  enum JointType {
    JointTypeUndefined = 0,
    JointTypeRevolute,
    JointTypePrismatic,
    JointTypeRevoluteX,
    JointTypeRevoluteY,
    JointTypeRevoluteZ,
    JointTypeSpherical, ///< 3 DoF joint using Quaternions for joint positional variables and angular velocity for joint velocity variables.
    JointTypeEulerZYX, ///< 3 DoF joint that uses Euler ZYX convention (faster than emulated multi DoF joints).
    JointTypeEulerXYZ, ///< 3 DoF joint that uses Euler XYZ convention (faster than emulated multi DoF joints).
    JointTypeEulerYXZ, ///< 3 DoF joint that uses Euler YXZ convention (faster than emulated multi DoF joints).
    JointTypeEulerZXY, ///< 3 DoF joint that uses Euler ZXY convention (faster than emulated multi DoF joints).
    JointTypeTranslationXYZ,
    JointTypeFloatingBase, ///< A 6-DoF joint for floating-base (or freeflyer) systems.
    JointTypeFixed, ///< Fixed joint which causes the inertial properties to be merged with the parent body.
    JointTypeHelical, //1 DoF joint with both rotational and translational motion
    JointType1DoF,
    JointType2DoF, ///< Emulated 2 DoF joint.
    JointType3DoF, ///< Emulated 3 DoF joint.
    JointType4DoF, ///< Emulated 4 DoF joint.
    JointType5DoF, ///< Emulated 5 DoF joint.
    JointType6DoF, ///< Emulated 6 DoF joint.
    JointTypeCustom, ///< User defined joints of varying size
  };

/** \brief Describes a joint relative to the predecessor body.
 *
 * This class contains all information required for one single joint. This
 * contains the joint type and the axis of the joint. See \ref joint_description for detailed description.
 *
 */
struct RBDL_DLLAPI Joint {
  Joint() :
    mJointAxes (NULL),
    mJointType (JointTypeUndefined),
    mDoFCount (0),
    q_index (0) {};
  Joint (JointType type) :
    mJointAxes (NULL),
    mJointType (type),
    mDoFCount (0),
    q_index (0),
    custom_joint_index(-1) {
      if (type == JointTypeRevoluteX) {
        mDoFCount = 1;
        mJointAxes = new Math::SpatialVector[mDoFCount];
        mJointAxes[0] = Math::SpatialVector (1., 0., 0., 0., 0., 0.);
      } else if (type == JointTypeRevoluteY) {
        mDoFCount = 1;
        mJointAxes = new Math::SpatialVector[mDoFCount];
        mJointAxes[0] = Math::SpatialVector (0., 1., 0., 0., 0., 0.);
      } else if (type == JointTypeRevoluteZ) {
        mDoFCount = 1;
        mJointAxes = new Math::SpatialVector[mDoFCount];
        mJointAxes[0] = Math::SpatialVector (0., 0., 1., 0., 0., 0.);
      } else if (type == JointTypeSpherical) {
        mDoFCount = 3;

        mJointAxes = new Math::SpatialVector[mDoFCount];

        mJointAxes[0] = Math::SpatialVector (0., 0., 1., 0., 0., 0.);
        mJointAxes[1] = Math::SpatialVector (0., 1., 0., 0., 0., 0.);
        mJointAxes[2] = Math::SpatialVector (1., 0., 0., 0., 0., 0.);
      } else if (type == JointTypeEulerZYX) {
        mDoFCount = 3;

        mJointAxes = new Math::SpatialVector[mDoFCount];

        mJointAxes[0] = Math::SpatialVector (0., 0., 1., 0., 0., 0.);
        mJointAxes[1] = Math::SpatialVector (0., 1., 0., 0., 0., 0.);
        mJointAxes[2] = Math::SpatialVector (1., 0., 0., 0., 0., 0.);
      } else if (type == JointTypeEulerXYZ) {
        mDoFCount = 3;

        mJointAxes = new Math::SpatialVector[mDoFCount];

        mJointAxes[0] = Math::SpatialVector (1., 0., 0., 0., 0., 0.);
        mJointAxes[1] = Math::SpatialVector (0., 1., 0., 0., 0., 0.);
        mJointAxes[2] = Math::SpatialVector (0., 0., 1., 0., 0., 0.);
      } else if (type == JointTypeEulerYXZ) {
        mDoFCount = 3;

        mJointAxes = new Math::SpatialVector[mDoFCount];

        mJointAxes[0] = Math::SpatialVector (0., 1., 0., 0., 0., 0.);
        mJointAxes[1] = Math::SpatialVector (1., 0., 0., 0., 0., 0.);
        mJointAxes[2] = Math::SpatialVector (0., 0., 1., 0., 0., 0.);
      } else if (type == JointTypeEulerZXY) {
        mDoFCount = 3;

        mJointAxes = new Math::SpatialVector[mDoFCount];

        mJointAxes[0] = Math::SpatialVector (0., 0., 1., 0., 0., 0.);
        mJointAxes[1] = Math::SpatialVector (1., 0., 0., 0., 0., 0.);
        mJointAxes[2] = Math::SpatialVector (0., 1., 0., 0., 0., 0.);
      } else if (type == JointTypeTranslationXYZ) {
        mDoFCount = 3;

        mJointAxes = new Math::SpatialVector[mDoFCount];

        mJointAxes[0] = Math::SpatialVector (0., 0., 0., 1., 0., 0.);
        mJointAxes[1] = Math::SpatialVector (0., 0., 0., 0., 1., 0.);
        mJointAxes[2] = Math::SpatialVector (0., 0., 0., 0., 0., 1.);
      } else if (type >= JointType1DoF && type <= JointType6DoF) {
        // create a joint and allocate memory for it.
        // Warning: the memory does not get initialized by this function!
        mDoFCount = type - JointType1DoF + 1;
        mJointAxes = new Math::SpatialVector[mDoFCount];
  std::cerr << "Warning: uninitalized vector" << std::endl;
      } else if (type == JointTypeCustom) {
        //This constructor cannot be used for a JointTypeCustom because
        //we have no idea what mDoFCount is.
        std::cerr << "Error: Invalid use of Joint constructor Joint(JointType"
                  << " type). Only allowed when type != JointTypeCustom" 
                  << std::endl;
        assert(0);
        abort();                  
      } else if (type != JointTypeFixed && type != JointTypeFloatingBase) {
        std::cerr << "Error: Invalid use of Joint constructor Joint(JointType type). Only allowed when type == JointTypeFixed or JointTypeSpherical." << std::endl;
        assert (0);
        abort();
      }
    }
    Joint (JointType type, int degreesOfFreedom) :
      mJointAxes (NULL),
      mJointType (type),
      mDoFCount (0),
      q_index (0),
      custom_joint_index(-1) {
     if (type == JointTypeCustom) {        
        mDoFCount   = degreesOfFreedom;
        mJointAxes  = new Math::SpatialVector[mDoFCount];
        for(unsigned int i=0; i<mDoFCount;++i){
          mJointAxes[i] = Math::SpatialVector (0., 0., 0., 0., 0., 0.);
        }        
      } else {
        std::cerr << "Error: Invalid use of Joint constructor Joint(JointType"
                  << " type, int degreesOfFreedom). Only allowed when "
                  << "type  == JointTypeCustom." 
                  << std::endl;
        assert (0);
        abort();
      }
    }  
  Joint (const Joint &joint) :
    mJointType (joint.mJointType),
    mDoFCount (joint.mDoFCount),
    q_index (joint.q_index),
    custom_joint_index(joint.custom_joint_index) {
      mJointAxes = new Math::SpatialVector[mDoFCount];

      for (unsigned int i = 0; i < mDoFCount; i++)
        mJointAxes[i] = joint.mJointAxes[i];
    };
  Joint& operator= (const Joint &joint) {
    if (this != &joint) {
      if (mDoFCount > 0) {
        assert (mJointAxes);
        delete[] mJointAxes;
      }
      mJointType = joint.mJointType;
      mDoFCount = joint.mDoFCount;
      custom_joint_index = joint.custom_joint_index;
      mJointAxes = new Math::SpatialVector[mDoFCount];

      for (unsigned int i = 0; i < mDoFCount; i++)
        mJointAxes[i] = joint.mJointAxes[i];

      q_index = joint.q_index;
    }
    return *this;
  }
  ~Joint() {
    if (mJointAxes) {
      assert (mJointAxes);
      delete[] mJointAxes;
      mJointAxes = NULL;
      mDoFCount = 0;
      custom_joint_index = -1;
    }
  }

  /** \brief Constructs a joint from the given cartesian parameters.
   *
   * This constructor creates all the required spatial values for the given
   * cartesian parameters.
   *
   * \param joint_type whether the joint is revolute or prismatic
   * \param joint_axis the axis of rotation or translation
   */
  Joint (
      const JointType joint_type,
      const Math::Vector3d &joint_axis
      ) {
    mDoFCount = 1;
    mJointAxes = new Math::SpatialVector[mDoFCount];

    // Some assertions, as we concentrate on simple cases

    // Only rotation around the Z-axis
    assert ( joint_type == JointTypeRevolute || joint_type == JointTypePrismatic );

    mJointType = joint_type;

    if (joint_type == JointTypeRevolute) {
      // make sure we have a unit axis
      mJointAxes[0].set (
          joint_axis[0],
          joint_axis[1], 
          joint_axis[2], 
          0., 0., 0.
          );

    } else if (joint_type == JointTypePrismatic) {
      // make sure we have a unit axis
      assert (joint_axis.squaredNorm() == 1.);

      mJointAxes[0].set (
          0., 0., 0.,
          joint_axis[0],
          joint_axis[1],
          joint_axis[2]
          );
    }
  }

  /** \brief Constructs a 1 DoF joint with the given motion subspaces.
   *
   * The motion subspaces are of the format:
   * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
   *
   * \param axis_0 Motion subspace for axis 0
   */
  Joint (
      const Math::SpatialVector &axis_0
      ) {
    mDoFCount = 1;
    mJointAxes = new Math::SpatialVector[mDoFCount];
    mJointAxes[0] = Math::SpatialVector (axis_0);
    if (axis_0 == Math::SpatialVector(1., 0., 0., 0., 0., 0.)) {
      mJointType = JointTypeRevoluteX;
    } else if (axis_0 == Math::SpatialVector(0., 1., 0., 0., 0., 0.)) {
      mJointType = JointTypeRevoluteY;
    } else if (axis_0 == Math::SpatialVector(0., 0., 1., 0., 0., 0.)) {
      mJointType = JointTypeRevoluteZ;
    } else if (axis_0[0] == 0 &&
         axis_0[1] == 0 &&
         axis_0[2] == 0) {
      mJointType = JointTypePrismatic;
    } else {
      mJointType = JointTypeHelical;
    }
    validate_spatial_axis (mJointAxes[0]);
  }

  /** \brief Constructs a 2 DoF joint with the given motion subspaces.
   *
   * The motion subspaces are of the format:
   * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
   *
   * \note So far only pure rotations or pure translations are supported.
   *
   * \param axis_0 Motion subspace for axis 0
   * \param axis_1 Motion subspace for axis 1
   */
  Joint (
      const Math::SpatialVector &axis_0,
      const Math::SpatialVector &axis_1
      ) {
    mJointType = JointType2DoF;
    mDoFCount = 2;

    mJointAxes = new Math::SpatialVector[mDoFCount];
    mJointAxes[0] = axis_0;
    mJointAxes[1] = axis_1;

    validate_spatial_axis (mJointAxes[0]);
    validate_spatial_axis (mJointAxes[1]);
  }

  /** \brief Constructs a 3 DoF joint with the given motion subspaces.
   *
   * The motion subspaces are of the format:
   * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
   *
   * \note So far only pure rotations or pure translations are supported.
   *
   * \param axis_0 Motion subspace for axis 0
   * \param axis_1 Motion subspace for axis 1
   * \param axis_2 Motion subspace for axis 2
   */
  Joint (
      const Math::SpatialVector &axis_0,
      const Math::SpatialVector &axis_1,
      const Math::SpatialVector &axis_2
      ) {
    mJointType = JointType3DoF;
    mDoFCount = 3;

    mJointAxes = new Math::SpatialVector[mDoFCount];

    mJointAxes[0] = axis_0;
    mJointAxes[1] = axis_1;
    mJointAxes[2] = axis_2;

    validate_spatial_axis (mJointAxes[0]);
    validate_spatial_axis (mJointAxes[1]);
    validate_spatial_axis (mJointAxes[2]);
  }

  /** \brief Constructs a 4 DoF joint with the given motion subspaces.
   *
   * The motion subspaces are of the format:
   * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
   *
   * \note So far only pure rotations or pure translations are supported.
   *
   * \param axis_0 Motion subspace for axis 0
   * \param axis_1 Motion subspace for axis 1
   * \param axis_2 Motion subspace for axis 2
   * \param axis_3 Motion subspace for axis 3
   */
  Joint (
      const Math::SpatialVector &axis_0,
      const Math::SpatialVector &axis_1,
      const Math::SpatialVector &axis_2,
      const Math::SpatialVector &axis_3
      ) {
    mJointType = JointType4DoF;
    mDoFCount = 4;

    mJointAxes = new Math::SpatialVector[mDoFCount];

    mJointAxes[0] = axis_0;
    mJointAxes[1] = axis_1;
    mJointAxes[2] = axis_2;
    mJointAxes[3] = axis_3;

    validate_spatial_axis (mJointAxes[0]);
    validate_spatial_axis (mJointAxes[1]);
    validate_spatial_axis (mJointAxes[2]);
    validate_spatial_axis (mJointAxes[3]);
  }

  /** \brief Constructs a 5 DoF joint with the given motion subspaces.
   *
   * The motion subspaces are of the format:
   * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
   *
   * \note So far only pure rotations or pure translations are supported.
   *
   * \param axis_0 Motion subspace for axis 0
   * \param axis_1 Motion subspace for axis 1
   * \param axis_2 Motion subspace for axis 2
   * \param axis_3 Motion subspace for axis 3
   * \param axis_4 Motion subspace for axis 4
   */
  Joint (
      const Math::SpatialVector &axis_0,
      const Math::SpatialVector &axis_1,
      const Math::SpatialVector &axis_2,
      const Math::SpatialVector &axis_3,
      const Math::SpatialVector &axis_4
      ) {
    mJointType = JointType5DoF;
    mDoFCount = 5;

    mJointAxes = new Math::SpatialVector[mDoFCount];

    mJointAxes[0] = axis_0;
    mJointAxes[1] = axis_1;
    mJointAxes[2] = axis_2;
    mJointAxes[3] = axis_3;
    mJointAxes[4] = axis_4;

    validate_spatial_axis (mJointAxes[0]);
    validate_spatial_axis (mJointAxes[1]);
    validate_spatial_axis (mJointAxes[2]);
    validate_spatial_axis (mJointAxes[3]);
    validate_spatial_axis (mJointAxes[4]);
  }

  /** \brief Constructs a 6 DoF joint with the given motion subspaces.
   *
   * The motion subspaces are of the format:
   * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
   *
   * \note So far only pure rotations or pure translations are supported.
   *
   * \param axis_0 Motion subspace for axis 0
   * \param axis_1 Motion subspace for axis 1
   * \param axis_2 Motion subspace for axis 2
   * \param axis_3 Motion subspace for axis 3
   * \param axis_4 Motion subspace for axis 4
   * \param axis_5 Motion subspace for axis 5
   */
  Joint (
      const Math::SpatialVector &axis_0,
      const Math::SpatialVector &axis_1,
      const Math::SpatialVector &axis_2,
      const Math::SpatialVector &axis_3,
      const Math::SpatialVector &axis_4,
      const Math::SpatialVector &axis_5
      ) {
    mJointType = JointType6DoF;
    mDoFCount = 6;

    mJointAxes = new Math::SpatialVector[mDoFCount];

    mJointAxes[0] = axis_0;
    mJointAxes[1] = axis_1;
    mJointAxes[2] = axis_2;
    mJointAxes[3] = axis_3;
    mJointAxes[4] = axis_4;
    mJointAxes[5] = axis_5;

    validate_spatial_axis (mJointAxes[0]);
    validate_spatial_axis (mJointAxes[1]);
    validate_spatial_axis (mJointAxes[2]);
    validate_spatial_axis (mJointAxes[3]);
    validate_spatial_axis (mJointAxes[4]);
    validate_spatial_axis (mJointAxes[5]);
  }

  /** \brief Checks whether we have pure rotational or translational axis.
   *
   * This function is mainly used to print out warnings when specifying an
   * axis that might not be intended.
   */
  bool validate_spatial_axis (Math::SpatialVector &axis) {

    bool axis_rotational = false;
    bool axis_translational = false;

    Math::Vector3d rotation (axis[0], axis[1], axis[2]);
    Math::Vector3d translation (axis[3], axis[4], axis[5]);

    if (fabs(rotation.norm()) > 1.0e-8)
      axis_rotational = true;

    if (fabs(translation.norm()) > 1.0e-8)
      axis_translational = true;

    if(axis_rotational && rotation.norm() - 1.0 > 1.0e-8) {
      std::cerr << "Warning: joint rotation axis is not unit!" << std::endl;
    }

    if(axis_translational && translation.norm() - 1.0 > 1.0e-8) {
      std::cerr << "Warning: joint translation axis is not unit!" << std::endl;
    }
    
    return axis_rotational != axis_translational;
  }

  /// \brief The spatial axes of the joint
  Math::SpatialVector* mJointAxes;
  /// \brief Type of joint 
  JointType mJointType;
  /// \brief Number of degrees of freedom of the joint. Note: CustomJoints
  // have here a value of 0 and their actual numbers of degrees of freedom
  // can be obtained using the CustomJoint structure.
  unsigned int mDoFCount;
  unsigned int q_index;
  int custom_joint_index;
};

/** \brief Computes all variables for a joint model
 *
 *  By appropriate modification of this function all types of joints can be
 *  modeled. See RBDA Section 4.4 for details.
 *
 * \param model    the rigid body model
 * \param joint_id the id of the joint we are interested in. This will be used to determine the type of joint and also the entries of \f[ q, \dot{q} \f].
 * \param q        joint state variables
 * \param qdot     joint velocity variables
 */
RBDL_DLLAPI
void jcalc (
    Model &model,
    unsigned int joint_id,
    const Math::VectorNd &q,
    const Math::VectorNd &qdot
    );

RBDL_DLLAPI
Math::SpatialTransform jcalc_XJ (
    Model &model,
    unsigned int joint_id,
    const Math::VectorNd &q);

RBDL_DLLAPI
void jcalc_X_lambda_S (
    Model &model,
    unsigned int joint_id,
    const Math::VectorNd &q
    );

struct RBDL_DLLAPI CustomJoint {
  CustomJoint() 
  { }
  virtual ~CustomJoint() {};

  virtual void jcalc (Model &model,
      unsigned int joint_id,
      const Math::VectorNd &q,
      const Math::VectorNd &qdot
      ) = 0;
  virtual void jcalc_X_lambda_S (Model &model,
      unsigned int joint_id,
      const Math::VectorNd &q
      ) = 0;

  unsigned int mDoFCount;
  Math::SpatialTransform XJ;
  Math::MatrixNd S;
  Math::MatrixNd U;
  Math::MatrixNd Dinv;
  Math::VectorNd u;
  Math::VectorNd d_u;
};

}

/* RBDL_JOINT_H */
#endif
