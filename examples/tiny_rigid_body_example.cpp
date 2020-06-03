// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "TinyRigidBodyExample.h"

#include <string.h>

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "BulletCollision/CollisionDispatch/btBoxBoxDetector.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "stb_image/stb_image.h"
#define stdvector btAlignedObjectArray
#include "TinyRBD.h"

#define SPHERE_RADIUS 1
static int globalStep = 0;

static Fix64Scalar gRestitution = Fix64Scalar::zero();

#define SCE_PFX_CLAMP(v, a, b) Fix64Max(a, Fix64Min(v, b))

/// returns true if we found a pair of closest points
void ComputeClosestPointsPlaneSphereFixedPoint(const TinyRBDPlane& planeWorld,
                                               const TinyRBDSphere& sphere,
                                               const TinyRBDPose& spherePose,
                                               TinyRBDContactPoint& pointOut) {
  // B3_PROFILE("ComputeClosestPointsPlaneSphere");
  Fix64Vector3 spherePosWorld = spherePose.m_position;
  Fix64Scalar t =
      -(spherePosWorld.dot(-planeWorld.m_normal) + planeWorld.m_planeConstant);
  Fix64Vector3 intersectionPoint = spherePosWorld + t * -planeWorld.m_normal;
  Fix64Scalar distance = t - sphere.m_radius;
  pointOut.m_distance = distance;

  Fix64Vector3 pointAWorld = intersectionPoint;
  Fix64Vector3 pointBWorld =
      spherePosWorld + sphere.m_radius * -planeWorld.m_normal;

  pointOut.m_localPointA = pointAWorld;
  // pointOut.m_localPointA = sphereAPose.inverseTransformPoint(pointAWorld);
  pointOut.m_localPointB = spherePose.inverseTransformPoint(pointBWorld);
  pointOut.m_normalOnB = -planeWorld.m_normal;
}

void ComputeClosestPointsSphereSphere(const TinyRBDSphere& sphereA,
                                      const TinyRBDPose& sphereAPose,
                                      const TinyRBDSphere& sphereB,
                                      const TinyRBDPose& sphereBPose,
                                      TinyRBDContactPoint& pointOut,
                                      Fix64Scalar distanceSqrThreshold) {
  Fix64Vector3 diff = sphereBPose.m_position - sphereAPose.m_position;
  Fix64Scalar len = diff.length();
  pointOut.m_distance = len - (sphereA.m_radius + sphereB.m_radius);
  pointOut.m_normalOnB = Fix64Vector3::create(
      Fix64Scalar::one(), Fix64Scalar::zero(), Fix64Scalar::zero());

  if (len > Fix64Scalar::epsilon()) {
    pointOut.m_normalOnB = (Fix64Scalar::one() / len) * diff;
  } else {
    pointOut.m_normalOnB = Fix64Vector3::makeIdentity();
  }
  Fix64Vector3 pointAWorld =
      (sphereAPose.m_position - sphereA.m_radius * pointOut.m_normalOnB);
  Fix64Vector3 pointBWorld =
      pointAWorld - (pointOut.m_distance * pointOut.m_normalOnB);

  pointOut.m_localPointA = sphereAPose.inverseTransformPoint(pointAWorld);
  pointOut.m_localPointB = sphereBPose.inverseTransformPoint(pointBWorld);

  btAssert(!pointOut.m_distance.isInvalidValue()) pointOut.m_normalOnB =
      -pointOut.m_normalOnB;
  pointOut.m_distance = pointOut.m_distance;
}

inline Fix64Scalar VertexBFaceATest(Fix64Vector3& ptsVec, Fix64Scalar& t0,
                                    Fix64Scalar& t1, const Fix64Vector3& hA,
                                    const Fix64Vector3& offsetAB) {
  // compute center of sphere in box's coordinate system

  Fix64Vector3 cptsVec = Fix64Vector3(offsetAB);

  // compute the parameters of the point on the face

  t0 = cptsVec.getX();
  t1 = cptsVec.getY();

  if (t0 > hA.getX())
    t0 = hA.getX();
  else if (t0 < -hA.getX())
    t0 = -hA.getX();
  if (t1 > hA.getY())
    t1 = hA.getY();
  else if (t1 < -hA.getY())
    t1 = -hA.getY();

  cptsVec.m_x -= t0;
  cptsVec.m_y -= t1;

  ptsVec = Fix64Vector3(cptsVec);

  return Fix64Vector3::dot2(ptsVec, ptsVec);
}

Fix64Scalar pfxContactBoxSphere(Fix64Vector3& normal, Fix64Vector3& pointA,
                                Fix64Vector3& pointB, const TinyRBDBox& boxA,
                                const TinyRBDPose& transformA,
                                const TinyRBDSphere& sphereB,
                                const TinyRBDPose& transformB,
                                Fix64Scalar distanceThreshold) {
  Fix64Vector3 ident[3] = {
      Fix64Vector3::create(Fix64Scalar::one(), Fix64Scalar::zero(),
                           Fix64Scalar::zero()),
      Fix64Vector3::create(Fix64Scalar::zero(), Fix64Scalar::one(),
                           Fix64Scalar::zero()),
      Fix64Vector3::create(Fix64Scalar::zero(), Fix64Scalar::zero(),
                           Fix64Scalar::one()),
  };
  // offsetAB is vector from A's center to B's center, in A's coordinate system
  Fix64Vector3 translationB = transformB.m_position;
  Fix64Vector3 offsetAB = transpose(Fix64Matrix3x3(transformA.m_orientation)) *
                          (translationB - transformA.m_position);

  // find separating axis with largest gap between objects

  Fix64Vector3 axisA;
  int faceDimA;
  Fix64Scalar maxGap;

  Fix64Vector3 gapsA = absPerElem(offsetAB) - boxA.m_halfExtents -
                       Fix64Vector3::create(sphereB.m_radius, sphereB.m_radius,
                                            sphereB.m_radius);
  Fix64Vector3 signsA = copySignPerElem(
      Fix64Vector3::create(Fix64Scalar::one(), Fix64Scalar::one(),
                           Fix64Scalar::one()),
      offsetAB);

  {
    Fix64Scalar gap = gapsA.getX();

    if (gap > distanceThreshold) {
      return gap;
    }

    maxGap = gap;
    faceDimA = 0;
    axisA = mulPerElem(ident[0], signsA);

    if (gap > maxGap) {
      maxGap = gap;
      faceDimA = 0;
      axisA = mulPerElem(ident[0], signsA);
    }

    gap = gapsA.getY();

    if (gap > distanceThreshold) {
      return gap;
    }

    if (gap > maxGap) {
      maxGap = gap;
      faceDimA = 1;
      axisA = mulPerElem(ident[1], signsA);
    }

    gap = gapsA.getZ();

    if (gap > distanceThreshold) {
      return gap;
    }

    if (gap > maxGap) {
      maxGap = gap;
      faceDimA = 2;
      axisA = mulPerElem(ident[2], signsA);
    }
  }
  int dimA[3];
  dimA[2] = faceDimA;
  dimA[0] = (faceDimA + 1) % 3;
  dimA[1] = (faceDimA + 2) % 3;

  Fix64Matrix3x3 apermCol;

  apermCol.setCol0(ident[dimA[0]]);
  apermCol.setCol1(ident[dimA[1]]);
  apermCol.setCol2(ident[dimA[2]]);

  Fix64Matrix3x3 apermRow = transpose(apermCol);

  // permute vectors

  Fix64Vector3 halfA_perm = apermRow * boxA.m_halfExtents;
  Fix64Vector3 offsetAB_perm = apermRow * offsetAB;
  Fix64Vector3 signsA_perm = apermRow * signsA;

  // compute the vector between the center of the box face and the sphere center

  Fix64Scalar signA2 = signsA_perm.getZ();
  Fix64Scalar scaleA2 = halfA_perm.getZ() * signA2;
  offsetAB_perm.setZ(offsetAB_perm.getZ() - scaleA2);

  // find point on face closest to sphere center

  Fix64Scalar t0, t1;
  Fix64Scalar minDistSqr;
  Fix64Vector3 closestPtsVec_perm;
  Fix64Vector3 localPointA_perm;

  minDistSqr = VertexBFaceATest(closestPtsVec_perm, t0, t1,
                                Fix64Vector3(halfA_perm), offsetAB_perm);
  localPointA_perm = Fix64Vector3::create(t0, t1, scaleA2);
  // compute normal
  bool centerInside =
      (signA2 * closestPtsVec_perm.getZ() < Fix64Scalar::zero());

  Fix64Scalar lenSqrTol = Fix64Scalar::epsilon();  // :fromScalar(1.0e-30f);

  if (centerInside || (minDistSqr < lenSqrTol)) {
    normal = Fix64Matrix3x3(transformA.m_orientation) * axisA;
  } else {
    Fix64Vector3 closestPtsVec = apermCol * closestPtsVec_perm;
    normal =
        Fix64Matrix3x3(transformA.m_orientation) *
        (closestPtsVec * (Fix64Scalar::one() / Fix64Scalar::sqrt(minDistSqr)));
  }
  // compute box point
  pointA = Fix64Vector3(apermCol * Fix64Vector3(localPointA_perm));
  // compute sphere point
  pointB = Fix64Vector3(transpose(transformB.m_orientation) *
                        (-normal * sphereB.m_radius));
  // return distance
  if (centerInside) {
    return -Fix64Scalar::sqrt(minDistSqr) - sphereB.m_radius;
  } else {
    return Fix64Scalar::sqrt(minDistSqr) - sphereB.m_radius;
  }
}

void ComputeClosestPointsBoxSphere(const TinyRBDBox& boxA,
                                   const TinyRBDPose& boxAPose,
                                   const TinyRBDSphere& sphereB,
                                   const TinyRBDPose& sphereBPose,
                                   TinyRBDContactPoint& pointOut) {
  // B3_PROFILE("ComputeClosestPointsBoxSphere");
  // pointOut.m_normalOnB

  // Fix64Vector3& normal,Fix64Vector3 &pointA,Fix64Vector3 &pointB,

  Fix64Scalar distanceThreshold = Fix64Scalar::epsilon();

  Fix64Scalar dist = pfxContactBoxSphere(
      pointOut.m_normalOnB, pointOut.m_localPointA, pointOut.m_localPointB,
      boxA, boxAPose, sphereB, sphereBPose, distanceThreshold);
  pointOut.m_normalOnB = -pointOut.m_normalOnB;
  pointOut.m_distance = dist;
}

// ComputeClosestPointsBoxBox

// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.

#define dDOTpq(a, b, p, q) \
  ((a)[0] * (b)[0] + (a)[p] * (b)[q] + (a)[2 * (p)] * (b)[2 * (q)])

/*PURE_INLINE btScalar dDOT   (const btScalar *a, const btScalar *b) { return
dDOTpq(a,b,1,1); } PURE_INLINE btScalar dDOT13 (const btScalar *a, const
btScalar *b) { return dDOTpq(a,b,1,3); } PURE_INLINE btScalar dDOT31 (const
btScalar *a, const btScalar *b) { return dDOTpq(a,b,3,1); } PURE_INLINE btScalar
dDOT33 (const btScalar *a, const btScalar *b) { return dDOTpq(a,b,3,3); }
*/
static Fix64Scalar dDOT(const Fix64Scalar* a, const Fix64Scalar* b) {
  return dDOTpq(a, b, 1, 1);
}
static Fix64Scalar dDOT44(const Fix64Scalar* a, const Fix64Scalar* b) {
  return dDOTpq(a, b, 4, 4);
}
static Fix64Scalar dDOT41(const Fix64Scalar* a, const Fix64Scalar* b) {
  return dDOTpq(a, b, 4, 1);
}
static Fix64Scalar dDOT14(const Fix64Scalar* a, const Fix64Scalar* b) {
  return dDOTpq(a, b, 1, 4);
}

#define dMULTIPLYOP1_331(A, op, B, C) \
  {                                   \
    (A)[0] op dDOT41((B), (C));       \
    (A)[1] op dDOT41((B + 1), (C));   \
    (A)[2] op dDOT41((B + 2), (C));   \
  }

// todo: check if this is colum or row!
#define dMULTIPLYOP0_331(A, op, B, C)                                          \
  {                                                                            \
    (A)[0] op Fix64Vector3::dot2(Fix64Vector3::create(B[0], B[1], B[2]), (C)); \
    (A)[1] op Fix64Vector3::dot2(Fix64Vector3::create(B[4], B[5], B[6]), (C)); \
    (A)[2] op Fix64Vector3::dot2(Fix64Vector3::create(B[8], B[9], B[10]),      \
                                 (C));                                         \
  }

#define dMULTIPLY1_331(A, B, C) dMULTIPLYOP1_331(A, =, B, C)
#define dMULTIPLY0_331(A, B, C) dMULTIPLYOP0_331(A, =, B, C)

typedef Fix64Scalar dMatrix3[4 * 3];
typedef Fix64Scalar dVector3[4];

void dLineClosestApproach(const Fix64Vector3& pa, const Fix64Vector3& ua,
                          const Fix64Vector3& pb, const Fix64Vector3& ub,
                          Fix64Scalar* alpha, Fix64Scalar* beta);
void dLineClosestApproach(const Fix64Vector3& pa, const Fix64Vector3& ua,
                          const Fix64Vector3& pb, const Fix64Vector3& ub,
                          Fix64Scalar* alpha, Fix64Scalar* beta) {
  Fix64Vector3 p =
      Fix64Vector3::create(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
  Fix64Scalar uaub = Fix64Vector3::dot2(ua, ub);
  Fix64Scalar q1 = Fix64Vector3::dot2(ua, p);
  Fix64Scalar q2 = -Fix64Vector3::dot2(ub, p);
  Fix64Scalar d = Fix64Scalar::one() - uaub * uaub;
  if (d <= Fix64Scalar::fromScalar(0.0001f)) {
    // @@@ this needs to be made more robust
    *alpha = Fix64Scalar::zero();
    *beta = Fix64Scalar::zero();
  } else {
    d = Fix64Scalar::one() / d;
    *alpha = (q1 + uaub * q2) * d;
    *beta = (uaub * q1 + q2) * d;
  }
}

// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).

static int intersectRectQuad2(Fix64Scalar h[2], Fix64Scalar p[8],
                              Fix64Scalar ret[16]) {
  // q (and r) contain nq (and nr) coordinate points for the current (and
  // chopped) polygons
  int nq = 4, nr = 0;
  Fix64Scalar buffer[16];
  Fix64Scalar* q = p;
  Fix64Scalar* r = ret;
  for (int dir = 0; dir <= 1; dir++) {
    // direction notation: xy[0] = x axis, xy[1] = y axis
    for (Fix64Scalar sign = -Fix64Scalar::one(); sign <= Fix64Scalar::one();
         sign = sign + Fix64Scalar::two()) {
      // chop q along the line xy[dir] = sign*h[dir]
      Fix64Scalar* pq = q;
      Fix64Scalar* pr = r;
      nr = 0;
      for (int i = nq; i > 0; i--) {
        // go through all points in q and all lines between adjacent points
        if (sign * pq[dir] < h[dir]) {
          // this point is inside the chopping line
          pr[0] = pq[0];
          pr[1] = pq[1];
          pr += 2;
          nr++;
          if (nr & 8) {
            q = r;
            goto done;
          }
        }
        Fix64Scalar* nextq = (i > 1) ? pq + 2 : q;
        if ((sign * pq[dir] < h[dir]) ^ (sign * nextq[dir] < h[dir])) {
          // this line crosses the chopping line
          pr[1 - dir] = pq[1 - dir] + (nextq[1 - dir] - pq[1 - dir]) /
                                          (nextq[dir] - pq[dir]) *
                                          (sign * h[dir] - pq[dir]);
          pr[dir] = sign * h[dir];
          pr += 2;
          nr++;
          if (nr & 8) {
            q = r;
            goto done;
          }
        }
        pq += 2;
      }
      q = r;
      r = (q == ret) ? buffer : ret;
      nq = nr;
    }
  }
done:
  if (q != ret) memcpy(ret, q, nr * 2 * sizeof(Fix64Scalar));
  return nr;
}

#define M__PI 3.14159265f

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].
#ifdef 0
static void Fix64cullPoints(int n, Fix64Scalar p[], int m, int i0, int iret[]) {
  // compute the centroid of the polygon in cx,cy
  int i, j;
  Fix64Scalar a, cx, cy, q;
  if (n == 1) {
    cx = p[0];
    cy = p[1];
  } else if (n == 2) {
    cx = Fix64Scalar::fromScalar(0.5) * (p[0] + p[2]);
    cy = Fix64Scalar::fromScalar(0.5) * (p[1] + p[3]);
  } else {
    a = Fix64Scalar::zero();
    cx = Fix64Scalar::zero();
    cy = Fix64Scalar::zero();
    for (i = 0; i < (n - 1); i++) {
      q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
      a += q;
      cx += q * (p[i * 2] + p[i * 2 + 2]);
      cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
    }
    q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
    if (Fix64Abs(a + q) > Fix64Scalar::epsilon()) {
      a = Fix64Scalar::one() / (Fix64Scalar::fromScalar(3.0) * (a + q));
    } else {
      a = Fix64Scalar::maxValue();
    }
    cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
    cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
  }

  // compute the angle of each point w.r.t. the centroid
  btScalar A[8];
  for (i = 0; i < n; i++) A[i] = btAtan2(p[i * 2 + 1] - cy, p[i * 2] - cx);

  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  int avail[8];
  for (i = 0; i < n; i++) avail[i] = 1;
  avail[i0] = 0;
  iret[0] = i0;
  iret++;
  for (j = 1; j < m; j++) {
    a = btScalar(j) * (2 * M__PI / m) + A[i0];
    if (a > M__PI) a -= 2 * M__PI;
    btScalar maxdiff = 1e9, diff;

    *iret = i0;  // iret is not allowed to keep this value, but it sometimes
                 // does, when diff=#QNAN0

    for (i = 0; i < n; i++) {
      if (avail[i]) {
        diff = btFabs(A[i] - a);
        if (diff > M__PI) diff = 2 * M__PI - diff;
        if (diff < maxdiff) {
          maxdiff = diff;
          *iret = i;
        }
      }
    }
#if defined(DEBUG) || defined(_DEBUG)
    btAssert(*iret != i0);  // ensure iret got set
#endif
    avail[*iret] = 0;
    iret++;
  }
}
#endif
struct LWContactPool {
  btAlignedObjectArray<TinyRBDContactPoint> m_contacts;

  void addContact(const TinyRBDContactPoint& contactPoint) {
    if (contactPoint.m_distance < Fix64Scalar::epsilon()) {
      m_contacts.push_back(contactPoint);
    }
  }
};

#if 0
void cullPoints2(int n, Fix64Scalar p[], int m, int i0, int iret[]) {
  // compute the centroid of the polygon in cx,cy
  int i;
  Fix64Scalar j;
  Fix64Scalar a, cx, cy, q;
  if (n == 1) {
    cx = p[0];
    cy = p[1];
  } else if (n == 2) {
    cx = Fix64Scalar::half() * (p[0] + p[2]);
    cy = Fix64Scalar::half() * (p[1] + p[3]);
  } else {
    a = Fix64Scalar::zero();
    cx = Fix64Scalar::zero();
    cy = Fix64Scalar::zero();
    for (i = 0; i < (n - 1); i++) {
      q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
      a += q;
      cx += q * (p[i * 2] + p[i * 2 + 2]);
      cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
    }
    q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
    if (Fix64Abs(a + q) > Fix64Scalar::epsilon()) {
      a = Fix64Scalar::one() / (Fix64Scalar::three() * (a + q));
    } else {
      a = Fix64Scalar::largeValue();  // BT_LARGE_FLOAT;
    }
    cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
    cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
  }

  // compute the angle of each point w.r.t. the centroid
  Fix64Scalar A[8];
  for (i = 0; i < n; i++) A[i] = btAtan2(p[i * 2 + 1] - cy, p[i * 2] - cx);

  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  int avail[8];
  for (i = 0; i < n; i++) avail[i] = 1;
  avail[i0] = 0;
  iret[0] = i0;
  iret++;
  for (j = Fix64Scalar::one(); j < m; j++) {
    a = Fix64Scalar(j) * (Fix64Scalar::twopi() / m) + A[i0];
    if (a > Fix64Scalar::pi()) a -= Fix64Scalar::twopi();
    Fix64Scalar maxdiff = Fix64Scalar::maxValue(), diff;

    *iret = i0;  // iret is not allowed to keep this value, but it sometimes
                 // does, when diff=#QNAN0

    for (i = 0; i < n; i++) {
      if (avail[i]) {
        diff = Fix64Abs(A[i] - a);
        if (diff > Fix64Scalar::pi()) diff = Fix64Scalar::twopi() - diff;
        if (diff < maxdiff) {
          maxdiff = diff;
          *iret = i;
        }
      }
    }
#if defined(DEBUG) || defined(_DEBUG)
    btAssert(*iret != i0);  // ensure iret got set
#endif
    avail[*iret] = 0;
    iret++;
  }
}
#endif
int Fix64BoxBox2(const TinyRBDPose& boxAPose, const Fix64Vector3& side1,
                 const TinyRBDPose& boxBPose, const Fix64Vector3& side2,
                 Fix64Vector3& normal1, Fix64Scalar* depth, int* return_code,
                 int maxc, LWContactPool& pool, int bodyA,
                 int bodyB)  //, TinyRBDContactPoint* points)
{
  const Fix64Vector3& p1 = boxAPose.m_position;
  Fix64Matrix3x3 R1mat = boxAPose.m_orientation;
  const Fix64Vector3& p2 = boxBPose.m_position;
  Fix64Matrix3x3 R2mat = boxBPose.m_orientation;

#if 0
  return 0;
#else
  Fix64Scalar normal[3] = {normal1.getX(), normal1.getY(), normal1.getZ()};

  const Fix64Scalar p1Array[3] = {p1.getX(), p1.getY(), p1.getZ()};
  const Fix64Scalar p2Array[3] = {p2.getX(), p2.getY(), p2.getZ()};

  const Fix64Scalar fudge_factor =
      Fix64Scalar::fromRawInt64(4509715456);  // :::fromScalar(1.05);
  Fix64Vector3 p__, pp, normalC = Fix64Vector3::makeIdentity();

  const Fix64Scalar* normalR = 0;

  Fix64Scalar A[3], B[3], R11, R12, R13, R21, R22, R23, R31, R32, R33, Q11, Q12,
      Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
  int i, j, invert_normal, code;

  int cnum = 0;  // number of penetrating contact points found

  // get vector from centers of box 1 to box 2, relative to box 1
  p__ = p2 - p1;

  dMatrix3 R1;
  dMatrix3 R2;

  dVector3 pp2;
  dVector3 p = {p__[0], p__[1], p__[2]};

  for (int j = 0; j < 3; j++) {
    R1[0 + 4 * j] = R1mat[j].getX();
    R2[0 + 4 * j] = R2mat[j].getX();

    R1[1 + 4 * j] = R1mat[j].getY();
    R2[1 + 4 * j] = R2mat[j].getY();

    R1[2 + 4 * j] = R1mat[j].getZ();
    R2[2 + 4 * j] = R2mat[j].getZ();
  }

  dMULTIPLYOP1_331(pp2, =, R1, p);
  Fix64Vector3 bla = R1mat.transposeTimes(p__);

  dMULTIPLY1_331(pp, R1, p);  // get pp = p relative to body 1

  // get side lengths / 2
  A[0] = side1[0] * Fix64Scalar::half();
  A[1] = side1[1] * Fix64Scalar::half();
  A[2] = side1[2] * Fix64Scalar::half();
  B[0] = side2[0] * Fix64Scalar::half();
  B[1] = side2[1] * Fix64Scalar::half();
  B[2] = side2[2] * Fix64Scalar::half();

  // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  R11 = dDOT44(R1 + 0, R2 + 0);
  R12 = dDOT44(R1 + 0, R2 + 1);
  R13 = dDOT44(R1 + 0, R2 + 2);
  R21 = dDOT44(R1 + 1, R2 + 0);
  R22 = dDOT44(R1 + 1, R2 + 1);
  R23 = dDOT44(R1 + 1, R2 + 2);
  R31 = dDOT44(R1 + 2, R2 + 0);
  R32 = dDOT44(R1 + 2, R2 + 1);
  R33 = dDOT44(R1 + 2, R2 + 2);

  Q11 = Fix64Abs(R11);
  Q12 = Fix64Abs(R12);
  Q13 = Fix64Abs(R13);
  Q21 = Fix64Abs(R21);
  Q22 = Fix64Abs(R22);
  Q23 = Fix64Abs(R23);
  Q31 = Fix64Abs(R31);
  Q32 = Fix64Abs(R32);
  Q33 = Fix64Abs(R33);

  // for all 15 possible separating axes:
  //   * see if the axis separates the boxes. if so, return 0.
  //   * find the depth of the penetration along the separating axis (s2)
  //   * if this is the largest depth so far, record it.
  // the normal vector will be set to the separating axis with the smallest
  // depth. note: normalR is set to point to a column of R1 or R2 if that is
  // the smallest depth normal so far. otherwise normalR is 0 and normalC is
  // set to a vector relative to body 1. invert_normal is 1 if the sign of
  // the normal should be flipped.

#define TST(expr1, expr2, norm, cc)                  \
  s2 = Fix64Abs(expr1) - (expr2);                    \
  if (s2 > Fix64Scalar::zero()) return 0;            \
  if (s2 > s) {                                      \
    s = s2;                                          \
    normalR = norm;                                  \
    invert_normal = ((expr1) < Fix64Scalar::zero()); \
    code = (cc);                                     \
  }

  s = Fix64Scalar::minValue();
  invert_normal = 0;
  code = 0;

  // separating axis = u1,u2,u3
  TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1 + 0, 1);
  TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1 + 1, 2);
  TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1 + 2, 3);

  // separating axis = v1,v2,v3
  TST(dDOT41(R2 + 0, p), (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), R2 + 0,
      4);
  TST(dDOT41(R2 + 1, p), (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), R2 + 1,
      5);
  TST(dDOT41(R2 + 2, p), (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), R2 + 2,
      6);

  // note: cross product axes need to be scaled when s is computed.
  // normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1, expr2, n1, n2, n3, cc)                         \
  s2 = Fix64Abs(expr1) - (expr2);                                 \
  if (s2 > Fix64Scalar::epsilon()) return 0;                      \
  l = Fix64Scalar::sqrt((n1) * (n1) + (n2) * (n2) + (n3) * (n3)); \
  if (l > Fix64Scalar::epsilon()) {                               \
    s2 = s2 / l;                                                  \
    if (s2 * fudge_factor > s) {                                  \
      s = s2;                                                     \
      normalR = 0;                                                \
      normalC[0] = (n1) / l;                                      \
      normalC[1] = (n2) / l;                                      \
      normalC[2] = (n3) / l;                                      \
      invert_normal = ((expr1) < Fix64Scalar::zero());            \
      code = (cc);                                                \
    }                                                             \
  }

  // btScalar fudge2 (1.0e-5f);
  Fix64Scalar fudge2 = Fix64Scalar::fudge2();

  Q11 += fudge2;
  Q12 += fudge2;
  Q13 += fudge2;

  Q21 += fudge2;
  Q22 += fudge2;
  Q23 += fudge2;

  Q31 += fudge2;
  Q32 += fudge2;
  Q33 += fudge2;

  // separating axis = u1 x (v1,v2,v3)
  // TST(,,Fix64Scalar::zero(),-R31,R21,7);

  {
    Fix64Scalar expr1 = pp[2] * R21 - pp[1] * R31;
    Fix64Scalar expr2 = (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12);
    Fix64Scalar n1 = Fix64Scalar::zero();
    Fix64Scalar n2 = -R31;
    Fix64Scalar n3 = R21;
    int cc = 7;

    s2 = Fix64Abs(expr1) - (expr2);
    if (s2 > Fix64Scalar::epsilon()) return 0;
    l = Fix64Scalar::sqrt((n1) * (n1) + (n2) * (n2) + (n3) * (n3));
    if (l > Fix64Scalar::epsilon()) {
      s2 = s2 / l;
      if (s2 * fudge_factor > s) {
        s = s2;
        normalR = 0;
        normalC[0] = (n1) / l;
        normalC[1] = (n2) / l;
        normalC[2] = (n3) / l;
        invert_normal = ((expr1) < Fix64Scalar::zero());
        code = (cc);
      }
    }
  }

  // separating axis = u1 x (v1,v2,v3)
  TST(pp[2] * R21 - pp[1] * R31,
      (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), Fix64Scalar::zero(),
      -R31, R21, 7);
  TST(pp[2] * R22 - pp[1] * R32,
      (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), Fix64Scalar::zero(),
      -R32, R22, 8);
  TST(pp[2] * R23 - pp[1] * R33,
      (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), Fix64Scalar::zero(),
      -R33, R23, 9);

  // separating axis = u2 x (v1,v2,v3)
  TST(pp[0] * R31 - pp[2] * R11,
      (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22), R31,
      Fix64Scalar::zero(), -R11, 10);
  TST(pp[0] * R32 - pp[2] * R12,
      (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21), R32,
      Fix64Scalar::zero(), -R12, 11);
  TST(pp[0] * R33 - pp[2] * R13,
      (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21), R33,
      Fix64Scalar::zero(), -R13, 12);

  // separating axis = u3 x (v1,v2,v3)
  TST(pp[1] * R11 - pp[0] * R21,
      (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11,
      Fix64Scalar::zero(), 13);
  TST(pp[1] * R12 - pp[0] * R22,
      (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12,
      Fix64Scalar::zero(), 14);
  TST(pp[1] * R13 - pp[0] * R23,
      (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13,
      Fix64Scalar::zero(), 15);

#undef TST

  if (!code) return 0;

  // if we get to this point, the boxes interpenetrate. compute the normal
  // in global coordinates.
  if (normalR) {
    normal[0] = normalR[0];
    normal[1] = normalR[4];
    normal[2] = normalR[8];
  } else {
    dMULTIPLY0_331(normal, R1, normalC);
  }
  if (invert_normal) {
    normal[0] = -normal[0];
    normal[1] = -normal[1];
    normal[2] = -normal[2];
  }
  *depth = -s;

  // compute contact point(s)

  if (code > 6) {
    // an edge from box 1 touches an edge from box 2.
    // find a point pa on the intersecting edge of box 1
    Fix64Vector3 pa;
    Fix64Scalar sign;
    for (i = 0; i < 3; i++) pa[i] = p1[i];
    for (j = 0; j < 3; j++) {
      sign = (dDOT14(normal, R1 + j) > Fix64Scalar::zero())
                 ? Fix64Scalar::one()
                 : -Fix64Scalar::one();
      for (i = 0; i < 3; i++) pa[i] += sign * A[j] * R1[i * 4 + j];
    }

    // find a point pb on the intersecting edge of box 2
    Fix64Vector3 pb;
    for (i = 0; i < 3; i++) pb[i] = p2[i];
    for (j = 0; j < 3; j++) {
      sign = (dDOT14(normal, R2 + j) > Fix64Scalar::zero())
                 ? -Fix64Scalar::one()
                 : Fix64Scalar::one();
      for (i = 0; i < 3; i++) pb[i] += sign * B[j] * R2[i * 4 + j];
    }

    Fix64Scalar alpha, beta;
    Fix64Vector3 ua, ub;
    for (i = 0; i < 3; i++) ua[i] = R1[((code)-7) / 3 + i * 4];
    for (i = 0; i < 3; i++) ub[i] = R2[((code)-7) % 3 + i * 4];

    dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
    for (i = 0; i < 3; i++) pa[i] += ua[i] * alpha;
    for (i = 0; i < 3; i++) pb[i] += ub[i] * beta;

    {
      // contact[0].pos[i] = btScalar(0.5)*(pa[i]+pb[i]);
      // contact[0].depth = *depth;
      // Fix64Vector3 pointInWorld;

#ifdef USE_CENTER_POINT
      for (i = 0; i < 3; i++) pointInWorld[i] = (pa[i] + pb[i]) * btScalar(0.5);
      output.addContactPoint(-normal, pointInWorld, -*depth);
#else
      // output.addContactPoint(-normal,pb,-*depth);

      TinyRBDContactPoint pt;
      pt.m_bodyA = bodyA;
      pt.m_bodyB = bodyB;
      pt.m_distance = -*depth;
      pt.m_localPointA = boxAPose.inverseTransformPoint(pa);
      pt.m_localPointB = boxBPose.inverseTransformPoint(pb);
      pt.m_normalOnB.setValue(-normal[0], -normal[1], -normal[2]);
      pool.addContact(pt);

#endif  //
      *return_code = code;
    }
    return 1;
  }

  // okay, we have a face-something intersection (because the separating
  // axis is perpendicular to a face). define face 'a' to be the reference
  // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
  // the incident face (the closest face of the other box).

  const Fix64Scalar *Ra, *Rb, *pa, *pb, *Sa, *Sb;
  if (code <= 3) {
    Ra = R1;
    Rb = R2;
    pa = p1Array;
    pb = p2Array;
    Sa = A;
    Sb = B;
  } else {
    Ra = R2;
    Rb = R1;
    pa = p2Array;
    pb = p1Array;
    Sa = B;
    Sb = A;
  }

  // nr = normal vector of reference face dotted with axes of incident box.
  // anr = absolute values of nr.
  Fix64Vector3 normal2, nr, anr;
  if (code <= 3) {
    normal2[0] = normal[0];
    normal2[1] = normal[1];
    normal2[2] = normal[2];
  } else {
    normal2[0] = -normal[0];
    normal2[1] = -normal[1];
    normal2[2] = -normal[2];
  }
  dMULTIPLY1_331(nr, Rb, normal2.m_vec);
  anr[0] = Fix64Abs(nr[0]);
  anr[1] = Fix64Abs(nr[1]);
  anr[2] = Fix64Abs(nr[2]);

  // find the largest compontent of anr: this corresponds to the normal
  // for the indident face. the other axis numbers of the indicent face
  // are stored in a1,a2.
  int lanr, a1, a2;
  if (anr[1] > anr[0]) {
    if (anr[1] > anr[2]) {
      a1 = 0;
      lanr = 1;
      a2 = 2;
    } else {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  } else {
    if (anr[0] > anr[2]) {
      lanr = 0;
      a1 = 1;
      a2 = 2;
    } else {
      a1 = 0;
      a2 = 1;
      lanr = 2;
    }
  }

  // compute center point of incident face, in reference-face coordinates
  Fix64Vector3 center;
  if (nr[lanr] < Fix64Scalar::zero()) {
    for (i = 0; i < 3; i++)
      center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i * 4 + lanr];
  } else {
    for (i = 0; i < 3; i++)
      center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i * 4 + lanr];
  }

  // find the normal and non-normal axis numbers of the reference box
  int codeN, code1, code2;
  if (code <= 3)
    codeN = code - 1;
  else
    codeN = code - 4;
  if (codeN == 0) {
    code1 = 1;
    code2 = 2;
  } else if (codeN == 1) {
    code1 = 0;
    code2 = 2;
  } else {
    code1 = 0;
    code2 = 1;
  }

  // find the four corners of the incident face, in reference-face coordinates
  Fix64Scalar quad[8];  // 2D coordinate of incident face (x,y pairs)
  Fix64Scalar c1, c2, m11, m12, m21, m22;
  c1 = dDOT14(center.m_vec, Ra + code1);
  c2 = dDOT14(center.m_vec, Ra + code2);
  // optimize this? - we have already computed this data above, but it is not
  // stored in an easy-to-index format. for now it's quicker just to recompute
  // the four dot products.
  m11 = dDOT44(Ra + code1, Rb + a1);
  m12 = dDOT44(Ra + code1, Rb + a2);
  m21 = dDOT44(Ra + code2, Rb + a1);
  m22 = dDOT44(Ra + code2, Rb + a2);
  {
    Fix64Scalar k1 = m11 * Sb[a1];
    Fix64Scalar k2 = m21 * Sb[a1];
    Fix64Scalar k3 = m12 * Sb[a2];
    Fix64Scalar k4 = m22 * Sb[a2];
    quad[0] = c1 - k1 - k3;
    quad[1] = c2 - k2 - k4;
    quad[2] = c1 - k1 + k3;
    quad[3] = c2 - k2 + k4;
    quad[4] = c1 + k1 + k3;
    quad[5] = c2 + k2 + k4;
    quad[6] = c1 + k1 - k3;
    quad[7] = c2 + k2 - k4;
  }

  // find the size of the reference face
  Fix64Scalar rect[2];
  rect[0] = Sa[code1];
  rect[1] = Sa[code2];

  // intersect the incident and reference faces
  Fix64Scalar ret[16];
  int n = intersectRectQuad2(rect, quad, ret);
  if (n < 1) return 0;  // this should never happen

  // convert the intersection points into reference-face coordinates,
  // and compute the contact position and depth for each point. only keep
  // those points that have a positive (penetrating) depth. delete points in
  // the 'ret' array as necessary so that 'point' and 'ret' correspond.
  Fix64Scalar point[3 * 8];  // penetrating contact points
  Fix64Scalar dep[8];        // depths for those points
  Fix64Scalar det1 = Fix64Scalar::one() / (m11 * m22 - m12 * m21);
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;

  for (j = 0; j < n; j++) {
    Fix64Scalar k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
    Fix64Scalar k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
    for (i = 0; i < 3; i++)
      point[cnum * 3 + i] =
          center[i] + k1 * Rb[i * 4 + a1] + k2 * Rb[i * 4 + a2];
    dep[cnum] = Sa[codeN] - dDOT(normal2.m_vec, point + cnum * 3);
    if (dep[cnum] >= Fix64Scalar::zero()) {
      ret[cnum * 2] = ret[j * 2];
      ret[cnum * 2 + 1] = ret[j * 2 + 1];
      cnum++;
    }
  }
  if (cnum < 1) return 0;  // this should never happen

#if 1

  // we can't generate more contacts than we actually have
  if (maxc > cnum) maxc = cnum;
  if (maxc < 1) maxc = 1;

  if (cnum <= maxc) {
    if (code < 4) {
      // we have less contacts than we need, so we use them all
      for (j = 0; j < cnum; j++) {
        Fix64Vector3 pointInWorldB;
        for (i = 0; i < 3; i++) pointInWorldB[i] = point[j * 3 + i] + pa[i];

        Fix64Vector3 pointInWorldA;
        for (i = 0; i < 3; i++)
          pointInWorldA[i] = point[j * 3 + i] + pa[i] - normal[i] * dep[j];

        TinyRBDContactPoint pt;
        pt.m_bodyA = bodyA;
        pt.m_bodyB = bodyB;
        pt.m_distance = -dep[j];
        pt.m_localPointA = boxAPose.inverseTransformPoint(pointInWorldA);
        pt.m_localPointB = boxBPose.inverseTransformPoint(pointInWorldB);
        pt.m_normalOnB.setValue(-normal[0], -normal[1], -normal[2]);
        pool.addContact(pt);
      }
    } else {
      // we have less contacts than we need, so we use them all
      for (j = 0; j < cnum; j++) {
        Fix64Vector3 pointInWorldB;
        for (i = 0; i < 3; i++)
          pointInWorldB[i] = point[j * 3 + i] + pa[i] - normal[i] * dep[j];

        Fix64Vector3 pointInWorldA;
        for (i = 0; i < 3; i++) pointInWorldA[i] = point[j * 3 + i] + pa[i];

        TinyRBDContactPoint pt;
        pt.m_bodyA = bodyA;
        pt.m_bodyB = bodyB;
        pt.m_distance = -dep[j];
        pt.m_localPointA = boxAPose.inverseTransformPoint(pointInWorldA);
        pt.m_localPointB = boxBPose.inverseTransformPoint(pointInWorldB);
        pt.m_normalOnB.setValue(-normal[0], -normal[1], -normal[2]);
        pool.addContact(pt);
      }
    }
  } else {
    btAssert(0);
#if 0
    // we have more contacts than are wanted, some of them must be culled.
    // find the deepest point, it is always the first contact.
    int i1 = 0;
    Fix64Scalar maxdepth = dep[0];
    for (i=1; i<cnum; i++) {
      if (dep[i] > maxdepth) {
	maxdepth = dep[i];
	i1 = i;
      }
    }

    int iret[8];
    cullPoints2 (cnum,ret,maxc,i1,iret);

    for (j=0; j < maxc; j++) {
//      dContactGeom *con = CONTACT(contact,skip*j);
  //    for (i=0; i<3; i++) con->pos[i] = point[iret[j]*3+i] + pa[i];
    //  con->depth = dep[iret[j]];

		Fix64Vector3 posInWorld;
		for (i=0; i<3; i++) 
			posInWorld[i] = point[iret[j]*3+i] + pa[i];
		if (code<4) 
	   {
			output.addContactPoint(-normal,posInWorld,-dep[iret[j]]);


		} else
		{
			output.addContactPoint(-normal,posInWorld-normal*dep[iret[j]],-dep[iret[j]]);



		}
    }
	cnum = maxc;
#endif
    cnum = 0;
  }

#endif

  *return_code = code;

  return cnum;
#endif
}

#if 0
struct MyContactAdder : public btDiscreteCollisionDetectorInterface::Result
{
	
	LWContactPool& m_contactPool;
	btTransform m_transA;
	btTransform m_transB;

	int m_bodyA;
	int m_bodyB;

	MyContactAdder(LWContactPool& contactPool, int bodyA, int bodyB, const btTransform& transA,const btTransform& transB)
		:m_contactPool(contactPool),
		m_transA(transA),
		m_transB(transB),
		m_bodyA(bodyA),
		m_bodyB(bodyB)
	{
	}

	virtual void setShapeIdentifiersA(int partId0,int index0)
	{
	}
	virtual void setShapeIdentifiersB(int partId1,int index1)
	{
	}
	virtual void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth)
	{
		//printf("added!\n");
		TinyRBDContactPoint pt;
		pt.m_bodyA = m_bodyA;
		pt.m_bodyB = m_bodyB;
		pt.m_distance = Fix64Scalar::fromScalar(depth);
		pt.m_normalOnB.setValue(Fix64Scalar::fromScalar(normalOnBInWorld[0]),Fix64Scalar::fromScalar(normalOnBInWorld[1]),Fix64Scalar::fromScalar(normalOnBInWorld[2]));
		btVector3 localPtA = m_transA.invXform(pointInWorld+normalOnBInWorld*depth);
		pt.m_localPointA.setValue(Fix64Scalar::fromScalar(localPtA[0]),Fix64Scalar::fromScalar(localPtA[1]),Fix64Scalar::fromScalar(localPtA[2]));
		btVector3 localPtB = m_transB.invXform(pointInWorld);
		pt.m_localPointB.setValue(Fix64Scalar::fromScalar(localPtB[0]),Fix64Scalar::fromScalar(localPtB[1]),Fix64Scalar::fromScalar(localPtB[2]));
		m_contactPool.addContact(pt);
	}
};
#endif

void ComputeClosestPointsBoxBox(const TinyRBDBox& boxA,
                                const TinyRBDPose& boxAPose,
                                const TinyRBDBox& boxB,
                                const TinyRBDPose& boxBPose,
                                LWContactPool& contactPool, int bodyA,
                                int bodyB) {
#if 0
	btVector3 halfExtentsA(boxA.m_halfExtents.getX().getScalar(),
		boxA.m_halfExtents.getY().getScalar(),
		boxA.m_halfExtents.getZ().getScalar());

	btVector3 halfExtentsB(boxB.m_halfExtents.getX().getScalar(),
		boxB.m_halfExtents.getY().getScalar(),
		boxB.m_halfExtents.getZ().getScalar());
	
	btBoxShape boxA_(halfExtentsA);
	btBoxShape boxB_(halfExtentsB);
	
	btBoxBoxDetector det(&boxA_,&boxB_);

	btDiscreteCollisionDetectorInterface::ClosestPointInput input;
	input.m_maximumDistanceSquared = SIMD_EPSILON;
	input.m_transformA.setOrigin(btVector3(boxAPose.m_position.getX().getScalar(),
		boxAPose.m_position.getY().getScalar(),
		boxAPose.m_position.getZ().getScalar()));

	input.m_transformA.setRotation(btQuaternion(boxAPose.m_orientation.getX().getScalar(),
		boxAPose.m_orientation.getY().getScalar(),
		boxAPose.m_orientation.getZ().getScalar(),
		boxAPose.m_orientation.getW().getScalar()));

	input.m_transformB.setOrigin(btVector3(boxBPose.m_position.getX().getScalar(),
		boxBPose.m_position.getY().getScalar(),
		boxBPose.m_position.getZ().getScalar()));

	input.m_transformB.setRotation(btQuaternion(boxBPose.m_orientation.getX().getScalar(),
		boxBPose.m_orientation.getY().getScalar(),
		boxBPose.m_orientation.getZ().getScalar(),
		boxBPose.m_orientation.getW().getScalar()));

	
	MyContactAdder output(contactPool,bodyA, bodyB, input.m_transformA, input.m_transformB);

	

	det.getClosestPoints(input, output, 0);

#else

  // B3_PROFILE("ComputeClosestPointsBoxBox");
  Fix64Scalar distanceThreshold = Fix64Scalar::maxValue();

  Fix64Vector3 normal;
  Fix64Scalar depth;
  int return_code;

  int maxc = 12;
  //	TinyRBDContactPoint points[4];

  int num =
      Fix64BoxBox2(boxAPose, boxA.m_halfExtents * Fix64Scalar::two(), boxBPose,
                   boxB.m_halfExtents * Fix64Scalar::two(), normal, &depth,
                   &return_code, maxc, contactPool, bodyA, bodyB);

#endif
}

enum LWRIGIDBODY_FLAGS {
  LWFLAG_USE_QUATERNION_DERIVATIVE = 1,

};
struct LWRigidBody {
  BT_DECLARE_ALIGNED_ALLOCATOR();

  TinyRBDPose m_worldPose;
  Fix64Vector3 m_linearVelocity;
  Fix64Vector3 m_angularVelocity;

  Fix64Vector3 m_localInertia;
  Fix64Scalar m_invMass;
  Fix64Matrix3x3 m_invInertiaTensorWorld;

  Fix64Vector3 m_totalForce;
  Fix64Vector3 m_totalTorque;

  void computeInvInertiaTensorWorld() {
    Fix64Vector3 invInertiaLocal;
    invInertiaLocal.setValue(m_localInertia.m_x != Fix64Scalar::zero()
                                 ? Fix64Scalar::one() / m_localInertia.m_x
                                 : Fix64Scalar::zero(),
                             m_localInertia.m_y != Fix64Scalar::zero()
                                 ? Fix64Scalar::one() / m_localInertia.m_y
                                 : Fix64Scalar::zero(),
                             m_localInertia.m_z != Fix64Scalar::zero()
                                 ? Fix64Scalar::one() / m_localInertia.m_z
                                 : Fix64Scalar::zero());
    Fix64Matrix3x3 m(m_worldPose.m_orientation);
    m_invInertiaTensorWorld = m.scaled(invInertiaLocal) * m.transposed();
  }

  int m_graphicsIndex;
  TinyRBDCollisionShape m_collisionShape;

  LWRIGIDBODY_FLAGS m_flags;

  LWRigidBody()
      : m_linearVelocity(Fix64Vector3::makeIdentity()),
        m_angularVelocity(Fix64Vector3::makeIdentity()),

        m_invInertiaTensorWorld(Fix64Matrix3x3::getIdentity()),
        m_flags(LWFLAG_USE_QUATERNION_DERIVATIVE) {}

  const Fix64Vector3& getPosition() const { return m_worldPose.m_position; }

  Fix64Vector3 getVelocity(const Fix64Vector3& relPos) const {
    return m_linearVelocity + m_angularVelocity.cross(relPos);
  }

  void applyImpulse(const Fix64Vector3& impulse, const Fix64Vector3& rel_pos) {
    m_linearVelocity += m_invMass * impulse;
    Fix64Vector3 torqueImpulse = rel_pos.cross(impulse);
    // printf("m_angularVelocity=%f,%f,%f\n",
    // m_angularVelocity.getX().getScalar(),m_angularVelocity.getY().getScalar(),m_angularVelocity.getZ().getScalar());
    m_angularVelocity += m_invInertiaTensorWorld * torqueImpulse;
    // printf("m_angularVelocity=%f,%f,%f\n",
    // m_angularVelocity.getX().getScalar(),m_angularVelocity.getY().getScalar(),m_angularVelocity.getZ().getScalar());
  }

  void clearForces() {
    m_totalForce = Fix64Vector3::create(
        Fix64Scalar::zero(), Fix64Scalar::zero(), Fix64Scalar::zero());
    m_totalTorque = Fix64Vector3::create(
        Fix64Scalar::zero(), Fix64Scalar::zero(), Fix64Scalar::zero());
  }

  void applyForceImpulse(Fix64Scalar deltaTime) {
    m_linearVelocity += m_totalForce * m_invMass * deltaTime;
    m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * deltaTime;
  }

  void applyGravity(const Fix64Vector3& gravityAcceleration) {
    Fix64Scalar mass = Fix64Scalar::zero();

    if (!m_invMass.isZero()) {
      mass = Fix64Scalar::one() / this->m_invMass;
    }

    Fix64Vector3 gravityForce = mass * gravityAcceleration;
    applyCentralForce(gravityForce);
  }

  void applyCentralForce(const Fix64Vector3& force) { m_totalForce += force; }

  void integrateVelocity(Fix64Scalar deltaTime) {
    TinyRBDPose newPose;
    newPose.m_position = m_worldPose.m_position + m_linearVelocity * deltaTime;

    // printf("m_angularVelocity=%f,%f,%f\n",
    // m_angularVelocity.getX().getScalar(),m_angularVelocity.getY().getScalar(),m_angularVelocity.getZ().getScalar());
    if (m_flags & LWFLAG_USE_QUATERNION_DERIVATIVE) {
      newPose.m_orientation = m_worldPose.m_orientation;
      newPose.m_orientation += (m_angularVelocity * newPose.m_orientation) *
                               (deltaTime * Fix64Scalar::half());
      newPose.m_orientation.normalize();
      m_worldPose = newPose;
    } else {
      // Exponential map
      // google for "Practical Parameterization of Rotations Using the
      // Exponential Map", F. Sebastian Grassia

      // btQuaternion q_w = [ sin(|w|*dt/2) * w/|w| , cos(|w|*dt/2)]
      // btQuaternion q_new =  q_w * q_old;

      Fix64Vector3 axis;
      Fix64Scalar fAngle = m_angularVelocity.length();
      // limit the angular motion
      const Fix64Scalar angularMotionThreshold =
          Fix64Scalar::half() * Fix64Scalar::halfPi();

      if (fAngle * deltaTime > angularMotionThreshold) {
        fAngle = angularMotionThreshold / deltaTime;
      }

      if (fAngle < Fix64Scalar::contactSlop()) {
        // use Taylor's expansions of sync function
        axis =
            m_angularVelocity *
            (Fix64Scalar::half() * deltaTime -
             (deltaTime * deltaTime * deltaTime) *
                 (Fix64Scalar::fromScalar(0.020833333333)) * fAngle * fAngle);
      } else {
        // sync(fAngle) = sin(c*fAngle)/t
        axis = m_angularVelocity *
               (Fix64Sin(Fix64Scalar::half() * fAngle * deltaTime) / fAngle);
      }
      Fix64Quaternion dorn = Fix64Quaternion::create(
          axis.m_x, axis.m_y, axis.m_z,
          Fix64Cos(fAngle * deltaTime * Fix64Scalar::half()));
      Fix64Quaternion orn0 = m_worldPose.m_orientation;

      Fix64Quaternion predictedOrn = dorn * orn0;
      predictedOrn.normalize();
      m_worldPose.m_orientation = predictedOrn;
    }
  }

  /*	void	stepSimulation(double deltaTime)
          {
                  integrateVelocity(Fix64Scalar::fromScalar(deltaTime));
          }
          */
};

struct Fix64SolverBody {
  Fix64SolverBody() {
    m_deltaLinearVelocity = Fix64Vector3::makeIdentity();
    m_deltaAngularVelocity = Fix64Vector3::makeIdentity();
  }

  Fix64Vector3 m_deltaLinearVelocity;
  Fix64Vector3 m_deltaAngularVelocity;
  Fix64Quaternion m_orientation;
  Fix64Matrix3x3 m_inertiaInv;
  Fix64Scalar m_massInv;
};

struct Fix64SetupSolverBodiesParam {
  LWRigidBody* m_bodies;
  Fix64SolverBody* m_solverBodies;
  int m_numRigidBodies;
};

void pfxSetupSolverBodies(Fix64SetupSolverBodiesParam& param) {
  LWRigidBody* bodies = param.m_bodies;
  Fix64SolverBody* solverBodies = param.m_solverBodies;
  int numRigidBodies = param.m_numRigidBodies;

  for (int i = 0; i < numRigidBodies; i++) {
    LWRigidBody& body = bodies[i];
    Fix64SolverBody& solverBody = solverBodies[i];

    solverBody.m_orientation = body.m_worldPose.m_orientation;
    solverBody.m_deltaLinearVelocity = Fix64Vector3::makeIdentity();
    solverBody.m_deltaAngularVelocity = Fix64Vector3::makeIdentity();

    if (!body.m_invMass.isZero()) {
      Fix64Matrix3x3 ori(solverBody.m_orientation);
      solverBody.m_massInv = body.m_invMass;
      body.computeInvInertiaTensorWorld();
      solverBody.m_inertiaInv = body.m_invInertiaTensorWorld;
    } else {
      solverBody.m_massInv = Fix64Scalar::zero();
      solverBody.m_inertiaInv = Fix64Matrix3x3::zero();
    }
  }
}

struct Fix64ConstraintRow {
  Fix64Vector3 m_normal;
  Fix64Scalar m_rhs;
  Fix64Scalar m_jacDiagInv;
  Fix64Scalar m_lowerLimit;
  Fix64Scalar m_upperLimit;
  Fix64Scalar m_accumImpulse;
};

static void pfxGetPlaneSpace(const Fix64Vector3& n, Fix64Vector3& fptr,
                             Fix64Vector3& q) {
  if (Fix64Abs(n.getZ()) > Fix64Scalar::sqrt(Fix64Scalar::half())) {
    // choose fptr in y-z plane
    Fix64Scalar a = n.getY() * n.getY() + n.getZ() * n.getZ();
    Fix64Scalar k = Fix64Scalar::one() / Fix64Scalar::sqrt(a);
    fptr =
        Fix64Vector3::create(Fix64Scalar::zero(), -n.getZ() * k, n.getY() * k);
    // set q = n x fptr
    q = Fix64Vector3::create(a * k, -n.getX() * fptr.getZ(),
                             n.getX() * fptr.getY());
  } else {
    // choose fptr in x-y plane
    Fix64Scalar a = n.getX() * n.getX() + n.getY() * n.getY();
    Fix64Scalar k = Fix64Scalar::one() / Fix64Scalar::sqrt(a);
    fptr =
        Fix64Vector3::create(-n.getY() * k, n.getX() * k, Fix64Scalar::zero());
    // set q = n x fptr
    q = Fix64Vector3::create(-n.getZ() * fptr.getY(), n.getZ() * fptr.getX(),
                             a * k);
  }

  /*printf("n=%f,%f,%f\nfptr=%f,%f,%f\nq=%f,%f,%f\n",n.getX().getScalar(),n.getY().getScalar(),n.getZ().getScalar(),
          fptr.getX().getScalar(),fptr.getY().getScalar(),fptr.getZ().getScalar(),
          q.getX().getScalar(),q.getY().getScalar(),q.getZ().getScalar());
          */
}

//#define SCE_PFX_CONTACT_SLOP 0.001f

void pfxSetupContactConstraint(
    Fix64ConstraintRow& constraintResponse,
    Fix64ConstraintRow& constraintFriction1,
    Fix64ConstraintRow& constraintFriction2, Fix64Scalar penetrationDepth,
    Fix64Scalar restitution, Fix64Scalar friction,
    const Fix64Vector3& contactNormal, const Fix64Vector3& contactPointA,
    const Fix64Vector3& contactPointB, const LWRigidBody& bodyA,
    const LWRigidBody& bodyB, Fix64SolverBody& solverBodyA,
    Fix64SolverBody& solverBodyB, Fix64Scalar separateBias,
    Fix64Scalar timeStep) {
  Fix64Vector3 rA = Fix64QuatRotate(solverBodyA.m_orientation, contactPointA);
  Fix64Vector3 rB = Fix64QuatRotate(solverBodyB.m_orientation, contactPointB);

  Fix64Scalar massInvA = solverBodyA.m_massInv;
  Fix64Scalar massInvB = solverBodyB.m_massInv;
  Fix64Matrix3x3 inertiaInvA = solverBodyA.m_inertiaInv;
  Fix64Matrix3x3 inertiaInvB = solverBodyB.m_inertiaInv;

  Fix64Matrix3x3 sc = Fix64Matrix3x3::scale(Fix64Vector3::create(
      massInvA + massInvB, massInvA + massInvB, massInvA + massInvB));

  Fix64Matrix3x3 K = sc - crossMatrix(rA) * inertiaInvA * crossMatrix(rA) -
                     crossMatrix(rB) * inertiaInvB * crossMatrix(rB);

  Fix64Vector3 vA = bodyA.m_linearVelocity +
                    Fix64Vector3::cross2(bodyA.m_angularVelocity, rA);
  Fix64Vector3 vB = bodyB.m_linearVelocity +
                    Fix64Vector3::cross2(bodyB.m_angularVelocity, rB);
  Fix64Vector3 vAB = vA - vB;

  Fix64Vector3 tangent1, tangent2;
  pfxGetPlaneSpace(contactNormal, tangent1, tangent2);

  // Contact Constraint
  {
    Fix64Vector3 normal = contactNormal;
    //	printf("normal=%f,%f,%f\n", contactNormal.getX().getScalar(),
    // contactNormal.getY().getScalar(), contactNormal.getZ().getScalar());
    Fix64Scalar denom = Fix64Vector3::dot2(K * normal, normal);

    constraintResponse.m_rhs =
        -(Fix64Scalar::one() + restitution) *
        Fix64Vector3::dot2(vAB,
                           normal);  // velocity error
                                     //	printf("rhs=%f\n",
                                     // constraintResponse.m_rhs.getScalar());

    Fix64Scalar s = separateBias * penetrationDepth;
    // printf("penetrationDepth=%f\n", s.getScalar());

    // printf("denom=%f\n", denom.getScalar());

    constraintResponse.m_rhs =
        constraintResponse.m_rhs -
        (separateBias *
         Fix64Min(Fix64Scalar::zero(),
                  penetrationDepth + Fix64Scalar::contactSlop())) /
            timeStep;  // position error
                       // printf("rhs=%f\n",
                       // constraintResponse.m_rhs.getScalar());

    constraintResponse.m_rhs = constraintResponse.m_rhs / denom;
    // printf("rhs=%f\n", constraintResponse.m_rhs.getScalar());
    constraintResponse.m_jacDiagInv = Fix64Scalar::one() / denom;
    constraintResponse.m_lowerLimit = Fix64Scalar::zero();
    constraintResponse.m_upperLimit = Fix64Scalar::maxValue();
    constraintResponse.m_normal = normal;
  }

  // Friction Constraint 1
  {
    Fix64Vector3 normal = tangent1;
    Fix64Scalar denom = Fix64Vector3::dot2(K * normal, normal);
    constraintFriction1.m_jacDiagInv = Fix64Scalar::one() / denom;
    constraintFriction1.m_rhs = -Fix64Vector3::dot2(vAB, normal);
    constraintFriction1.m_rhs =
        constraintFriction1.m_rhs * constraintFriction1.m_jacDiagInv;
    constraintFriction1.m_lowerLimit = Fix64Scalar::zero();
    constraintFriction1.m_upperLimit = Fix64Scalar::maxValue();
    constraintFriction1.m_normal = normal;
  }

  // Friction Constraint 2
  {
    Fix64Vector3 normal = tangent2;

    Fix64Scalar denom = Fix64Vector3::dot2(K * normal, normal);

    constraintFriction2.m_jacDiagInv = Fix64Scalar::one() / denom;
    constraintFriction2.m_rhs = -Fix64Vector3::dot2(vAB, normal);
    constraintFriction2.m_rhs =
        constraintFriction2.m_rhs * constraintFriction2.m_jacDiagInv;
    constraintFriction2.m_lowerLimit = Fix64Scalar::zero();
    constraintFriction2.m_upperLimit = Fix64Scalar::maxValue();
    constraintFriction2.m_normal = normal;
  }
}

static void pfxSolveLinearConstraintRow(
    Fix64ConstraintRow& constraint, Fix64Vector3& deltaLinearVelocityA,
    Fix64Vector3& deltaAngularVelocityA, Fix64Scalar massInvA,
    const Fix64Matrix3x3& inertiaInvA, const Fix64Vector3& rA,
    Fix64Vector3& deltaLinearVelocityB, Fix64Vector3& deltaAngularVelocityB,
    Fix64Scalar massInvB, const Fix64Matrix3x3& inertiaInvB,
    const Fix64Vector3& rB) {
  B3_PROFILE("pfxSolveLinearConstraintRow");
  Fix64Scalar deltaImpulse = constraint.m_rhs;
  Fix64Vector3 dVA =
      deltaLinearVelocityA + Fix64Vector3::cross2(deltaAngularVelocityA, rA);
  Fix64Vector3 dVB =
      deltaLinearVelocityB + Fix64Vector3::cross2(deltaAngularVelocityB, rB);
  deltaImpulse =
      deltaImpulse - constraint.m_jacDiagInv *
                         Fix64Vector3::dot2(constraint.m_normal, dVA - dVB);
  Fix64Scalar oldImpulse = constraint.m_accumImpulse;
  constraint.m_accumImpulse =
      SCE_PFX_CLAMP(oldImpulse + deltaImpulse, constraint.m_lowerLimit,
                    constraint.m_upperLimit);
  deltaImpulse = constraint.m_accumImpulse - oldImpulse;
  deltaLinearVelocityA += deltaImpulse * massInvA * constraint.m_normal;
  Fix64Vector3 crossVecA = Fix64Vector3::cross2(rA, constraint.m_normal);
  deltaAngularVelocityA += deltaImpulse * (inertiaInvA * crossVecA);

  deltaLinearVelocityB =
      deltaLinearVelocityB - deltaImpulse * massInvB * constraint.m_normal;
  Fix64Vector3 crossVecB = Fix64Vector3::cross2(rB, constraint.m_normal);
  deltaAngularVelocityB =
      deltaAngularVelocityB - deltaImpulse * (inertiaInvB * crossVecB);
}

inline const Fix64Vector3 rotate(const Fix64Quaternion& quat,
                                 const Fix64Vector3& vec) {
  Fix64Scalar tmpX, tmpY, tmpZ, tmpW;
  tmpX = (((quat.getW() * vec.getX()) + (quat.getY() * vec.getZ())) -
          (quat.getZ() * vec.getY()));
  tmpY = (((quat.getW() * vec.getY()) + (quat.getZ() * vec.getX())) -
          (quat.getX() * vec.getZ()));
  tmpZ = (((quat.getW() * vec.getZ()) + (quat.getX() * vec.getY())) -
          (quat.getY() * vec.getX()));
  tmpW = (((quat.getX() * vec.getX()) + (quat.getY() * vec.getY())) +
          (quat.getZ() * vec.getZ()));
  return Fix64Vector3::create(
      ((((tmpW * quat.getX()) + (tmpX * quat.getW())) - (tmpY * quat.getZ())) +
       (tmpZ * quat.getY())),
      ((((tmpW * quat.getY()) + (tmpY * quat.getW())) - (tmpZ * quat.getX())) +
       (tmpX * quat.getZ())),
      ((((tmpW * quat.getZ()) + (tmpZ * quat.getW())) - (tmpX * quat.getY())) +
       (tmpY * quat.getX())));
}

void pfxSolveContactConstraint(Fix64ConstraintRow& constraintResponse,
                               Fix64ConstraintRow& constraintFriction1,
                               Fix64ConstraintRow& constraintFriction2,
                               const Fix64Vector3& contactPointA,
                               const Fix64Vector3& contactPointB,
                               Fix64SolverBody& solverBodyA,
                               Fix64SolverBody& solverBodyB,
                               Fix64Scalar friction) {
  Fix64Vector3 rA, rB;
  {
    B3_PROFILE("rotate");
    rA = rotate(solverBodyA.m_orientation, contactPointA);
    rB = rotate(solverBodyB.m_orientation, contactPointB);
  }
  pfxSolveLinearConstraintRow(
      constraintResponse, solverBodyA.m_deltaLinearVelocity,
      solverBodyA.m_deltaAngularVelocity, solverBodyA.m_massInv,
      solverBodyA.m_inertiaInv, rA, solverBodyB.m_deltaLinearVelocity,
      solverBodyB.m_deltaAngularVelocity, solverBodyB.m_massInv,
      solverBodyB.m_inertiaInv, rB);

  Fix64Scalar mf = friction * Fix64Abs(constraintResponse.m_accumImpulse);
  constraintFriction1.m_lowerLimit = -mf;
  constraintFriction1.m_upperLimit = mf;
  constraintFriction2.m_lowerLimit = -mf;
  constraintFriction2.m_upperLimit = mf;

  pfxSolveLinearConstraintRow(
      constraintFriction1, solverBodyA.m_deltaLinearVelocity,
      solverBodyA.m_deltaAngularVelocity, solverBodyA.m_massInv,
      solverBodyA.m_inertiaInv, rA, solverBodyB.m_deltaLinearVelocity,
      solverBodyB.m_deltaAngularVelocity, solverBodyB.m_massInv,
      solverBodyB.m_inertiaInv, rB);

  pfxSolveLinearConstraintRow(
      constraintFriction2, solverBodyA.m_deltaLinearVelocity,
      solverBodyA.m_deltaAngularVelocity, solverBodyA.m_massInv,
      solverBodyA.m_inertiaInv, rA, solverBodyB.m_deltaLinearVelocity,
      solverBodyB.m_deltaAngularVelocity, solverBodyB.m_massInv,
      solverBodyB.m_inertiaInv, rB);
}

Fix64Scalar resolveCollision(LWRigidBody& bodyA, LWRigidBody& bodyB,
                             TinyRBDContactPoint& contactPoint) {
  b3Assert(contactPoint.m_distance <= Fix64Scalar::zero());

  Fix64Scalar appliedImpulse = Fix64Scalar::zero();

  Fix64Vector3 pointAWorld =
      bodyA.m_worldPose.transformPoint(contactPoint.m_localPointA);
  Fix64Vector3 pointALocal =
      bodyA.m_worldPose.inverseTransformPoint(pointAWorld);

  Fix64Vector3 pointBWorld =
      bodyB.m_worldPose.transformPoint(contactPoint.m_localPointB);
  Fix64Vector3 pointBLocal =
      bodyB.m_worldPose.inverseTransformPoint(pointBWorld);

  Fix64Vector3 rel_pos1 = pointAWorld - bodyA.getPosition();
  Fix64Vector3 rel_pos2 = pointBWorld - bodyB.getPosition();

  Fix64Scalar rel_vel = contactPoint.m_normalOnB.dot(
      bodyA.getVelocity(rel_pos1) - bodyB.getVelocity(rel_pos2));
  btScalar rv = rel_vel.getScalar();
  if (rel_vel < -Fix64Scalar::epsilon()) {
    // printf("rel_vel=%f\n", rel_vel.getScalar());
    Fix64Vector3 temp1 = bodyA.m_invInertiaTensorWorld *
                         rel_pos1.cross(contactPoint.m_normalOnB);
    Fix64Vector3 temp2 = bodyB.m_invInertiaTensorWorld *
                         rel_pos2.cross(contactPoint.m_normalOnB);

    Fix64Scalar impulse = -(Fix64Scalar::one() + gRestitution) * rel_vel /
                          (bodyA.m_invMass + bodyB.m_invMass +
                           contactPoint.m_normalOnB.dot(temp1.cross(rel_pos1) +
                                                        temp2.cross(rel_pos2)));

    Fix64Vector3 impulse_vector = contactPoint.m_normalOnB * impulse;
    // b3Printf("impulse = %f\n", impulse.getScalar());
    appliedImpulse = impulse;
    bodyA.applyImpulse(impulse_vector, rel_pos1);
    bodyB.applyImpulse(-impulse_vector, rel_pos2);
  }
  return appliedImpulse;
}

class TinyRigidBodyExample : public CommonExampleInterface {
  CommonGraphicsApp* m_app;
  GUIHelperInterface* m_guiHelper;
  int m_tutorialIndex;

  stdvector<LWRigidBody> m_bodies;

  int addRigidBody(Fix64Scalar mass, const Fix64Vector3& localInertiaDiag,
                   const Fix64Vector3& posWorld,
                   const Fix64Quaternion& ornWorld,
                   const TinyRBDCollisionShape& collisionShape) {
    int index = m_bodies.size();

    m_bodies.expand();

    m_bodies[index].m_invMass =
        !mass.isZero() ? Fix64Scalar::one() / mass : Fix64Scalar::zero();
    m_bodies[index].m_collisionShape = collisionShape;

    m_bodies[index].m_localInertia = localInertiaDiag;
    m_bodies[index].m_worldPose.m_position = posWorld;
    m_bodies[index].m_worldPose.m_orientation = ornWorld;

    m_bodies[index].computeInvInertiaTensorWorld();
    return index;
  }

  int createCheckeredTexture(int red, int green, int blue) {
    if (m_textureIndex2 >= 0) return m_textureIndex2;

    int texWidth = 1024;
    int texHeight = 1024;
    btAlignedObjectArray<unsigned char> texels;
    texels.resize(texWidth * texHeight * 3);
    for (int i = 0; i < texWidth * texHeight * 3; i++) texels[i] = 255;

    for (int i = 0; i < texWidth; i++) {
      for (int j = 0; j < texHeight; j++) {
        int a = i < texWidth / 2 ? 1 : 0;
        int b = j < texWidth / 2 ? 1 : 0;

        if (a == b) {
          texels[(i + j * texWidth) * 3 + 0] = red;
          texels[(i + j * texWidth) * 3 + 1] = green;
          texels[(i + j * texWidth) * 3 + 2] = blue;
          //					texels[(i+j*texWidth)*4+3] =
          // 255;
        }
        /*else
        {
                texels[i*3+0+j*texWidth] = 255;
                texels[i*3+1+j*texWidth] = 255;
                texels[i*3+2+j*texWidth] = 255;
        }
        */
      }
    }

    int texId =
        m_app->m_renderer->registerTexture(&texels[0], texWidth, texHeight);
    m_textureIndex2 = texId;
    return texId;
  }

  int addSphereRigidBody(Fix64Scalar mass, Fix64Scalar radius,
                         const Fix64Vector3& posWorld) {
    Fix64Quaternion ornWorld = Fix64Quaternion::makeIdentity();
    TinyRBDCollisionShape collisionShape;
    collisionShape.m_type = TINYRBD_SPHERE_TYPE;
    collisionShape.m_sphere.m_radius = radius;
    Fix64Vector3 localInertiaDiag;
    collisionShape.m_sphere.computeLocalInertia(mass, localInertiaDiag);
    int index = addRigidBody(mass, localInertiaDiag, posWorld, ornWorld,
                             collisionShape);

    // int boxId = m_app->registerGraphicsUnitSphereShape>registerCubeShape(1,1,
    // 0.01);
    int textureIndex = createCheckeredTexture(1, 0, 1);

    if (m_sphereTransparent < 0)
      m_sphereTransparent =
          m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_HIGH, textureIndex);

    // m_app->registerGraphicsUnitSphereShape()
    // int sphereOpaque= m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_HIGH,
    // textureIndex);

    double pos[4] = {posWorld.m_x.getScalar(), posWorld.m_y.getScalar(),
                     posWorld.m_z.getScalar(), 1};
    double orn[4] = {0, 0, 0, 1};
    double color[4] = {1, 1, 1, 1};
    double scaling[3] = {2. * radius.getScalar(), 2. * radius.getScalar(),
                         2. * radius.getScalar()};
    m_bodies[index].m_graphicsIndex =
        m_app->m_renderer->registerGraphicsInstance(m_sphereTransparent, pos,
                                                    orn, color, scaling);
    return index;
  }

  int addBoxRigidBody(Fix64Scalar mass, const Fix64Vector3& halfExtents,
                      const Fix64Vector3& posWorld) {
    Fix64Quaternion ornWorld = Fix64Quaternion::makeIdentity();
    TinyRBDCollisionShape collisionShape;
    collisionShape.m_type = TINYRBD_BOX_TYPE;
    collisionShape.m_box.m_halfExtents = halfExtents;
    Fix64Vector3 localInertiaDiag;
    collisionShape.m_box.computeLocalInertia(mass, localInertiaDiag);
    int index = addRigidBody(mass, localInertiaDiag, posWorld, ornWorld,
                             collisionShape);

    // int boxId = m_app->registerGraphicsUnitSphereShape>registerCubeShape(1,1,
    // 0.01);
    int textureIndex = createCheckeredTexture(1, 0, 1);

    int sphereTransparent = m_app->registerCubeShape(
        halfExtents.getX().getScalar(), halfExtents.getY().getScalar(),
        halfExtents.getZ().getScalar(), textureIndex, 1. / 4.);
    // m_app->registerGraphicsUnitSphereShape()
    // int sphereOpaque= m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_HIGH,
    // textureIndex);

    double pos[4] = {posWorld.m_x.getScalar(), posWorld.m_y.getScalar(),
                     posWorld.m_z.getScalar(), 1};
    double orn[4] = {0, 0, 0, 1};
    double color[4] = {.4, .4, 1, 1};
    double scaling[3] = {1, 1, 1};
    m_bodies[index].m_graphicsIndex =
        m_app->m_renderer->registerGraphicsInstance(sphereTransparent, pos, orn,
                                                    color, scaling);
    return index;
  }

  TimeSeriesCanvas* m_timeSeriesCanvas0;
  TimeSeriesCanvas* m_timeSeriesCanvas1;

  stdvector<TinyRBDContactPoint> m_contactPoints;
  int m_textureIndex;
  int m_textureIndex2;
  int m_sphereTransparent;
  int m_stage;
  int m_counter;

 public:
  TinyRigidBodyExample(GUIHelperInterface* guiHelper, int tutorialIndex)
      : m_app(guiHelper->getAppInterface()),
        m_guiHelper(guiHelper),
        m_tutorialIndex(tutorialIndex),
        m_timeSeriesCanvas0(0),
        m_timeSeriesCanvas1(0),
        m_stage(0),
        m_counter(0) {
    m_app->setUpAxis(1);

    {
      m_textureIndex = -1;
      m_textureIndex2 = -1;
      m_sphereTransparent = -1;

      if (1) {
        int width, height, n;

        const char* filename = "data/checker_blue.png";
        const unsigned char* image = 0;

        const char* prefix[] = {"./", "../", "../../", "../../../",
                                "../../../../"};
        int numprefix = sizeof(prefix) / sizeof(const char*);

        for (int i = 0; !image && i < numprefix; i++) {
          char relativeFileName[1024];
          sprintf(relativeFileName, "%s%s", prefix[i], filename);
          image = stbi_load(relativeFileName, &width, &height, &n, 3);
        }

        b3Assert(image);
        if (image) {
          m_textureIndex =
              m_app->m_renderer->registerTexture(image, width, height);
        }
      }

      if (0) {
        int ss = 0;  // 1073468572;//1074468572;
        Fix64Scalar maxdiff = Fix64Scalar::zero();
        Fix64Scalar maxdiffsigned = Fix64Scalar::zero();

        int count = 10000000;
        int failed = false;

        for (ss = 26000000;; ss++) {
          Fix64Scalar s2 = Fix64Scalar::fromScalar(ss);
          Fix64Scalar s = Fix64Scalar::sqrt(s2);
          if (s < Fix64Scalar::zero()) {
            if (!failed) {
              failed = true;
              printf("sqrt fail at %d\n", ss);
              // ss=1069999999, maxdiff=0.000008
              // sqrt fail at 1073741824
            }
            btAssert(0);
          }
          Fix64Scalar s22 = s * s;
          Fix64Scalar sdiff = s22 - s2;
          if (Fix64Abs(sdiff) > maxdiff) {
            maxdiff = Fix64Abs(sdiff);
            maxdiffsigned = sdiff;
          }
          count--;
          if (count == 0) {
            count = 10000000;
            printf("ss=%d, maxdiff=%f, maxdiffsigned=%f\n", ss,
                   maxdiff.getScalar(), maxdiffsigned.getScalar());
          }
        }
      }
      if (1) {
        Fix64Scalar mass = Fix64Scalar::zero();
        Fix64Vector3 localInertiaDiag = Fix64Vector3::makeIdentity();
        double pos[4] = {0, -2, 0, 1};
        double halfExtents[4] = {150, 2, 150, 1};
        double orn[4] = {0, 0, 0, 1};

        Fix64Vector3 posWorld = Fix64Vector3::create(
            Fix64Scalar::zero(), -Fix64Scalar::two(), Fix64Scalar::zero());
        Fix64Quaternion ornWorld = Fix64Quaternion::makeIdentity();
        TinyRBDCollisionShape collisionShape;
#if 1
        collisionShape.m_type = TINYRBD_PLANE_TYPE;
        collisionShape.m_plane.m_normal = Fix64Vector3::create(
            Fix64Scalar::zero(), Fix64Scalar::one(), Fix64Scalar::zero());
        collisionShape.m_plane.m_planeConstant = Fix64Scalar::zero();
#else
        collisionShape.m_type = TINYRBD_BOX_TYPE;
        collisionShape.m_box.m_halfExtents =
            Fix64Vector3::create(Fix64Scalar::fromScalar(halfExtents[0]),
                                 Fix64Scalar::fromScalar(halfExtents[1]),
                                 Fix64Scalar::fromScalar(halfExtents[2]));
#endif
        int index = addRigidBody(mass, localInertiaDiag, posWorld, ornWorld,
                                 collisionShape);

        int boxId =
            m_app->registerCubeShape(halfExtents[0], halfExtents[1],
                                     halfExtents[2], m_textureIndex, 150);

        double color[4] = {1, 1, 1, 1};
        double scaling[3] = {1, 1, 1};
        m_bodies[index].m_graphicsIndex =
            m_app->m_renderer->registerGraphicsInstance(boxId, pos, orn, color,
                                                        scaling);
      }

      // int addRigidBody(Fix64Scalar mass, const Fix64Vector3&
      // localInertiaDiag, const Fix64Vector3& posWorld, const Fix64Quaternion&
      // ornWorld, const TinyRBDCollisionShape& collisionShape)

      //            int boxId = m_app->registerCubeShape(1,1,1,textureIndex);
      // int sphereTransparent =
      // m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_HIGH,
      // m_textureIndex); int sphereOpaque=
      // m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_HIGH,
      // m_textureIndex);

      double scaling[3] = {SPHERE_RADIUS, SPHERE_RADIUS, SPHERE_RADIUS};
      if (0) {
        double pos[4] = {5, 1, 5, 1};
        double orn[4] = {0, 0, 0, 1};

        Fix64Scalar radius = Fix64Scalar::one();
        Fix64Scalar mass = Fix64Scalar::one();

        int i = addSphereRigidBody(
            mass, radius,
            Fix64Vector3::create(Fix64Scalar::fromScalar(pos[0]),
                                 Fix64Scalar::fromScalar(pos[1]),
                                 Fix64Scalar::fromScalar(pos[2])));
      }
      for (Fix64Scalar e = Fix64Scalar::zero(); e < Fix64Scalar::three();
           e = e + Fix64Scalar::one()) {
        for (Fix64Scalar q = Fix64Scalar::zero(); q < Fix64Scalar::three();
             q = q + Fix64Scalar::one()) {
          // int gfxShape = sphereOpaque;
          double color[4] = {1, 1, 1, 1};
          // double pos[4] = { 2, 1+q * 2, e*2, 1};//+q*.01, 1 };
          Fix64Scalar pos[4] = {
              Fix64Scalar::two(), Fix64Scalar::two() + q * Fix64Scalar::three(),
              e * Fix64Scalar::two() + q * Fix64Scalar::half(),
              Fix64Scalar::one()};
          double orn[4] = {0, 0, 0, 1};

          Fix64Scalar radius = Fix64Scalar::fromScalar(1);
          // Fix64Scalar mass = q? Fix64Scalar::fromScalar(1) :
          // Fix64Scalar::zero();
          Fix64Scalar mass = Fix64Scalar::one();

          if (1)  // q&1)
          {
            int i = addSphereRigidBody(
                mass, radius, Fix64Vector3::create(pos[0], pos[1], pos[2]));
          } else {
            Fix64Vector3 halfExtents = Fix64Vector3::create(
                Fix64Scalar::fromScalar(1), Fix64Scalar::fromScalar(1),
                Fix64Scalar::fromScalar(1));

            int i =
                addBoxRigidBody(mass, halfExtents,
                                Fix64Vector3::create(pos[0], pos[1], pos[2]));
          }
        }

        // m_bodies[i].m_graphicsIndex =
        // m_app->m_renderer->registerGraphicsInstance(gfxShape,pos,
        // orn,color,scaling);
        // m_app->m_renderer->writeSingleInstanceTransformToCPU(pos, orn,
        // m_bodies[i].m_graphicsIndex);
      }
    }

    m_app->m_renderer->writeTransforms();
  }
  virtual ~TinyRigidBodyExample() {
    delete m_timeSeriesCanvas0;
    delete m_timeSeriesCanvas1;

    m_timeSeriesCanvas0 = 0;
    m_timeSeriesCanvas1 = 0;
  }

  virtual void initPhysics() {}
  virtual void exitPhysics() {}

  void tutorial1Update(float deltaTime);
  void tutorial2Update(float deltaTime);
  void tutorialCollisionUpdate(float deltaTime, TinyRBDContactPoint& contact);
  void tutorialSolveContactConstraintUpdate(float deltaTime,
                                            TinyRBDContactPoint& contact);

  virtual void stepSimulation(float deltaTime2) {
    B3_PROFILE("stepSimulation");

    // Sleep(25);
    Fix64Scalar deltaTimeFP =
        Fix64Scalar::oneOver60();  // 0.016f;//1./60.;//240.;
    static int step = 0;
    printf("stepSimulation(%d)\n", step++);

    Fix64Vector3 gravityAcceleration = Fix64Vector3::create(
        Fix64Scalar::zero(), -Fix64Scalar::ten(), Fix64Scalar::zero());
    // Fix64Vector3 gravityAcceleration =
    // Fix64Vector3::create(Fix64Scalar::zero(),Fix64Scalar::zero(),
    // Fix64Scalar::zero()); Fix64Vector3 gravityAcceleration =
    // Fix64Vector3::create(Fix64Scalar::fromScalar(1),Fix64Scalar::fromScalar(-10),
    // Fix64Scalar::zero() );

    {
      B3_PROFILE("applyGravity");
      for (int i = 0; i < m_bodies.size(); i++) {
        m_bodies[i].clearForces();
        m_bodies[i].applyGravity(gravityAcceleration);
        m_bodies[i].applyForceImpulse(deltaTimeFP);
      }
    }

    {
      {
        B3_PROFILE("clear");
        m_contactPoints.clear();
      }
      {
        B3_PROFILE("computeInvInertiaTensorWorld");
        for (int i = 0; i < m_bodies.size(); i++) {
          m_bodies[i].computeInvInertiaTensorWorld();
        }
      }

      {
        B3_PROFILE("StepSimInner");

        btAlignedObjectArray<Fix64SolverBody> solverBodies;
        solverBodies.resize(m_bodies.size());

        {
          B3_PROFILE("reset deltaLinearVelocity");
          for (int i = 0; i < solverBodies.size(); i++) {
            solverBodies[i].m_deltaLinearVelocity =
                Fix64Vector3::makeIdentity();
            solverBodies[i].m_deltaAngularVelocity =
                Fix64Vector3::makeIdentity();
          }
        }

        Fix64SetupSolverBodiesParam param;
        param.m_bodies = &m_bodies[0];
        param.m_solverBodies = &solverBodies[0];
        param.m_numRigidBodies = solverBodies.size();
        pfxSetupSolverBodies(param);
        LWContactPool contactPool;

        contactPool.m_contacts.reserve(param.m_numRigidBodies *
                                       param.m_numRigidBodies);

        {
          B3_PROFILE("collision detection");
          // naive bruteforce broadphase' and narrowphase collision detection
          for (int a = 0; a < m_bodies.size(); a++) {
            for (int b = a + 1; b < m_bodies.size(); b++) {
              if (m_bodies[a].m_collisionShape.m_type == TINYRBD_PLANE_TYPE) {
                if (!m_bodies[b].m_invMass.isZero() &&
                    m_bodies[b].m_collisionShape.m_type ==
                        TINYRBD_SPHERE_TYPE) {
                  TinyRBDContactPoint contactPoint;
                  contactPoint.m_bodyA = a;
                  contactPoint.m_bodyB = b;

                  contactPoint.m_distance = Fix64Scalar::zero();
                  ComputeClosestPointsPlaneSphereFixedPoint(
                      m_bodies[a].m_collisionShape.m_plane,
                      m_bodies[b].m_collisionShape.m_sphere,
                      m_bodies[b].m_worldPose, contactPoint);

                  if (contactPoint.m_distance.getScalar() < 0) {
                    contactPool.addContact(contactPoint);
                  }
                }
              }

              if (m_bodies[a].m_collisionShape.m_type == TINYRBD_BOX_TYPE) {
                if (!m_bodies[b].m_invMass.isZero() &&
                    m_bodies[b].m_collisionShape.m_type ==
                        TINYRBD_SPHERE_TYPE) {
                  TinyRBDContactPoint contactPoint;
                  contactPoint.m_bodyA = a;
                  contactPoint.m_bodyB = b;

                  contactPoint.m_distance = Fix64Scalar::zero();
                  ComputeClosestPointsBoxSphere(
                      m_bodies[a].m_collisionShape.m_box,
                      m_bodies[a].m_worldPose,
                      m_bodies[b].m_collisionShape.m_sphere,
                      m_bodies[b].m_worldPose, contactPoint);

                  if (contactPoint.m_distance.getScalar() < 0) {
                    contactPool.addContact(contactPoint);
                  }
                }
              }

              if (m_bodies[a].m_collisionShape.m_type == TINYRBD_SPHERE_TYPE) {
                if (!m_bodies[b].m_invMass.isZero() &&
                    m_bodies[b].m_collisionShape.m_type == TINYRBD_BOX_TYPE) {
                  TinyRBDContactPoint contactPoint;
                  contactPoint.m_bodyA = b;
                  contactPoint.m_bodyB = a;

                  contactPoint.m_distance = Fix64Scalar::zero();
                  ComputeClosestPointsBoxSphere(
                      m_bodies[b].m_collisionShape.m_box,
                      m_bodies[b].m_worldPose,
                      m_bodies[a].m_collisionShape.m_sphere,
                      m_bodies[a].m_worldPose, contactPoint);

                  if (contactPoint.m_distance.getScalar() < 0) {
                    contactPool.addContact(contactPoint);
                  }
                }
              }

              if (m_bodies[a].m_collisionShape.m_type == TINYRBD_BOX_TYPE) {
                if (!m_bodies[b].m_invMass.isZero() &&
                    m_bodies[b].m_collisionShape.m_type == TINYRBD_BOX_TYPE) {
                  B3_PROFILE("ComputeClosestPointsBoxBox");
                  ComputeClosestPointsBoxBox(m_bodies[a].m_collisionShape.m_box,
                                             m_bodies[a].m_worldPose,
                                             m_bodies[b].m_collisionShape.m_box,
                                             m_bodies[b].m_worldPose,
                                             contactPool, a, b);
                }
              }

              if (1)
                if (m_bodies[a].m_collisionShape.m_type ==
                    TINYRBD_SPHERE_TYPE) {
                  if (m_bodies[b].m_collisionShape.m_type ==
                      TINYRBD_SPHERE_TYPE) {
                    TinyRBDContactPoint contactPoint;
                    contactPoint.m_bodyA = a;
                    contactPoint.m_bodyB = b;

                    contactPoint.m_distance = Fix64Scalar::maxValue();
                    ComputeClosestPointsSphereSphere(
                        m_bodies[a].m_collisionShape.m_sphere,
                        m_bodies[a].m_worldPose,
                        m_bodies[b].m_collisionShape.m_sphere,
                        m_bodies[b].m_worldPose, contactPoint,
                        Fix64Scalar::zero());

                    if (contactPoint.m_distance < Fix64Scalar::epsilon()) {
                      contactPool.addContact(contactPoint);
                    }
                  }
                }
            }
          }
        }
        globalStep++;
        // setup contact constraints

        for (int iter = 0; iter < 10; iter++) {
          for (int i = 0; i < contactPool.m_contacts.size(); i++) {
            resolveCollision(m_bodies[contactPool.m_contacts.at(i).m_bodyA],
                             m_bodies[contactPool.m_contacts.at(i).m_bodyB],
                             contactPool.m_contacts.at(i));
          }
        }

#if 0
				btAlignedObjectArray<Fix64ConstraintRow> constraintRows;
				constraintRows.resize(contactPool.m_contacts.size()*3);
                Fix64Scalar friction = Fix64Scalar::six()/Fix64Scalar::ten();//Fix64Scalar::fromScalar(0.6);
                Fix64Scalar separateBias = Fix64Scalar::one()/Fix64Scalar::ten();//:fromScalar(0.1);

				{
					B3_PROFILE("SetupContactConstraint");
					for (int i = 0; i < contactPool.m_contacts.size(); i++)
					{
						pfxSetupContactConstraint(constraintRows[i*3], constraintRows[i*3+1], constraintRows[i*3+2],
							contactPool.m_contacts[i].m_distance, gRestitution, friction,
							contactPool.m_contacts[i].m_normalOnB,
							contactPool.m_contacts[i].m_localPointA,
							contactPool.m_contacts[i].m_localPointB,
							m_bodies[contactPool.m_contacts[i].m_bodyA],
							m_bodies[contactPool.m_contacts[i].m_bodyB],
							solverBodies[contactPool.m_contacts[i].m_bodyA],
							solverBodies[contactPool.m_contacts[i].m_bodyB], separateBias, deltaTimeFP);

					}
				}

				{

					B3_PROFILE("SolveContactConstraint");
					//printf("constraintRows.size=%d\n", constraintRows.size());
					//solve constraints
					int maxIter = 10;
					for (int iter = 0; iter < maxIter; iter++)
					{
						for (int i = 0; i < constraintRows.size()/3; i++)
						{
							pfxSolveContactConstraint(constraintRows[i*3], constraintRows[i*3+1],constraintRows[i*3+2],
									contactPool.m_contacts[i].m_localPointA,
									contactPool.m_contacts[i].m_localPointB,
									solverBodies[contactPool.m_contacts[i].m_bodyA],
									solverBodies[contactPool.m_contacts[i].m_bodyB],
									friction);
						}
					}
				}

				{
					B3_PROFILE("Add deltaLinearVelocity");
					for (int i = 0; i<m_bodies.size(); i++)
					{
						m_bodies[i].m_linearVelocity += solverBodies[i].m_deltaLinearVelocity;
						m_bodies[i].m_angularVelocity += solverBodies[i].m_deltaAngularVelocity;
					}
				}

#endif
      }
    }

    if (m_timeSeriesCanvas0) m_timeSeriesCanvas0->nextTick();

    if (m_timeSeriesCanvas1) m_timeSeriesCanvas1->nextTick();

    Fix64Scalar eps = Fix64Scalar::epsilon();
    // integrate
    if (1) {
      B3_PROFILE("integrateVelocity");

      for (int i = 0; i < m_bodies.size(); i++) {
        m_bodies[i].integrateVelocity(deltaTimeFP);
      }
    }
    {
      B3_PROFILE("sync transforms");
      for (int i = 0; i < m_bodies.size(); i++) {
        m_bodies[i].integrateVelocity(deltaTimeFP);

        // printf("body (%d)=%lld,%lld,%lld\n",
        // i,m_bodies[i].m_worldPose.m_position[0].m_rawValue1,
        //      m_bodies[i].m_worldPose.m_position[1].m_rawValue1,
        //     m_bodies[i].m_worldPose.m_position[2].m_rawValue1);

        double pos[4] = {m_bodies[i].m_worldPose.m_position.getX().getScalar(),
                         m_bodies[i].m_worldPose.m_position.getY().getScalar(),
                         m_bodies[i].m_worldPose.m_position.getZ().getScalar(),
                         1};
        double orn[4] = {
            m_bodies[i].m_worldPose.m_orientation.getX().getScalar(),
            m_bodies[i].m_worldPose.m_orientation.getY().getScalar(),
            m_bodies[i].m_worldPose.m_orientation.getZ().getScalar(),
            m_bodies[i].m_worldPose.m_orientation.getW().getScalar()};

        m_app->m_renderer->writeSingleInstanceTransformToCPU(
            pos, orn, m_bodies[i].m_graphicsIndex);
      }
    }

    m_app->m_renderer->writeTransforms();
  }
  virtual void renderScene() {
    m_app->m_renderer->renderScene();
    m_app->drawText3D("X", 1, 0, 0, 1);
    m_app->drawText3D("Y", 0, 1, 0, 1);
    m_app->drawText3D("Z", 0, 0, 1, 1);

    for (int i = 0; i < m_contactPoints.size(); i++) {
      const TinyRBDContactPoint& contact = m_contactPoints[i];
      double color[4] = {1, 1, 0, 1};
      float lineWidth = 3;
      if (contact.m_distance.getScalar() < 0) {
        color[0] = 1;
        color[1] = 0;
        color[2] = 0;
      }
      // m_app->m_renderer->drawLine(contact.m_ptOnAWorld,contact.m_ptOnBWorld,color,lineWidth);
    }
  }

  virtual void physicsDebugDraw(int debugDrawFlags) {}
  virtual bool mouseMoveCallback(float x, float y) { return false; }
  virtual bool mouseButtonCallback(int button, int state, float x, float y) {
    return false;
  }
  virtual bool keyboardCallback(int key, int state) { return false; }

  virtual void resetCamera() {
    float dist = 28;
    float pitch = -32;
    float yaw = 40;
    float targetPos[3] = {0, 0, 0};
    if (m_app->m_renderer && m_app->m_renderer->getActiveCamera()) {
      m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
      m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
      m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
      m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(
          targetPos[0], targetPos[1], targetPos[2]);
    }
  }
};

void TinyRigidBodyExample::tutorial2Update(float deltaTime) {}
void TinyRigidBodyExample::tutorial1Update(float deltaTime) {
  for (int i = 0; i < m_bodies.size(); i++) {
    if (m_bodies[i].m_invMass.isZero()) continue;

    switch (m_stage) {
      case 0: {
        m_bodies[i].m_angularVelocity = Fix64MakeVector3(0, 0, 0);
        m_bodies[i].m_linearVelocity = Fix64MakeVector3(1, 0, 0);
        break;
      }
      case 1: {
        m_bodies[i].m_linearVelocity = Fix64MakeVector3(-1, 0, 0);
        break;
      }
      case 2: {
        m_bodies[i].m_linearVelocity = Fix64MakeVector3(0, 1, 0);
        break;
      }
      case 3: {
        m_bodies[i].m_linearVelocity = Fix64MakeVector3(0, -1, 0);
        break;
      }
      case 4: {
        m_bodies[i].m_linearVelocity = Fix64MakeVector3(0, 0, 1);
        break;
      }
      case 5: {
        m_bodies[i].m_linearVelocity = Fix64MakeVector3(0, 0, -1);
        break;
      }
      case 6: {
        m_bodies[i].m_linearVelocity = Fix64MakeVector3(0, 0, 0);
        m_bodies[i].m_angularVelocity = Fix64MakeVector3(1, 0, 0);
        break;
      }
      case 7: {
        m_bodies[i].m_angularVelocity = Fix64MakeVector3(-1, 0, 0);
        break;
      }
      case 8: {
        m_bodies[i].m_angularVelocity = Fix64MakeVector3(0, 1, 0);
        break;
      }
      case 9: {
        m_bodies[i].m_angularVelocity = Fix64MakeVector3(0, -1, 0);
        break;
      }
      case 10: {
        m_bodies[i].m_angularVelocity = Fix64MakeVector3(0, 0, 1);
        break;
      }
      case 11: {
        m_bodies[i].m_angularVelocity = Fix64MakeVector3(0, 0, -1);
        break;
      }
      default: { m_bodies[i].m_angularVelocity = Fix64MakeVector3(0, 0, 0); }
    };
  }

  m_counter++;
  if (m_counter > 60) {
    m_counter = 0;
    m_stage++;
    if (m_stage > 11) m_stage = 0;
    // b3Printf("Stage = %d\n",m_stage);
    // b3Printf("linVel =
    // %f,%f,%f\n",m_bodies[0]->m_linearVelocity.x,m_bodies[0]->m_linearVelocity.y,m_bodies[0]->m_linearVelocity.z);
    // b3Printf("angVel =
    // %f,%f,%f\n",m_bodies[0]->m_angularVelocity.x,m_bodies[0]->m_angularVelocity.y,m_bodies[0]->m_angularVelocity.z);
  }
}

void TinyRigidBodyExample::tutorialSolveContactConstraintUpdate(
    float deltaTime, TinyRBDContactPoint& contact) {}

void TinyRigidBodyExample::tutorialCollisionUpdate(
    float deltaTime, TinyRBDContactPoint& contact) {
  return;
}

class CommonExampleInterface* TinyRigidBodyExampleCreateFunc(
    struct CommonExampleOptions& options) {
  return new TinyRigidBodyExample(options.m_guiHelper, options.m_option);
}
