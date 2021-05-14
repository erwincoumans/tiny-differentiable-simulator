/**
 * Note: The code in this folder is referenced from the following source
 * It is largely modified to be integerated into the rest of the code base
 */

/////////////////////////////////////////////////////////////////////////////////////////////
//	FileName:	MarchingCubes.cpp
//	Author	:	Michael Y. Polyakov
//	email	:	myp@andrew.cmu.edu  or  mikepolyakov@hotmail.com
//	website	:	www.angelfire.com/linux/myp
//	date	:	July 2002
//
//	Description:	'Straight' and Recursive Marching Cubes Algorithms
//				Normal vectors are defined for each vertex as a
//gradients 				For function definitions see MarchingCubes.h 				For a tutorial on
//Marching Cubes please visit www.angelfire.com/myp/linux
//
//	Please email me with any suggestion/bugs.
/////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <functional>
#include <unordered_map>
#include <vector>
#include <array>

#include "mc_table.hpp"


#define VAL_ID 3

namespace MC {

// Yizhou: Helper class to construct a vector with 4 values
// The last value is aliased as "val" in line with the rest of the code
template <typename Algebra>
struct Vector4 {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Index = typename Algebra::Index;

  Vector3 vec;
  Scalar val;

  Vector4() : vec(), val(0) {}

  Vector4(Scalar x, Scalar y, Scalar z, Scalar w) : vec(x, y, z), val(w) {}

  Vector4(const Vector4 &rhs) : vec(rhs.vec), val(rhs.val) {}

  Vector4(const Vector3 &in_v, Scalar w) : vec(in_v), val(w) {}

  Vector4 &operator=(const Vector4 &rhs) {
    vec = rhs.vec;
    val = rhs.val;
    return *this;
  }

  Scalar operator[](Index i) {
    if (i == 3) {
      return val;
    } else {
      return vec[i];
    }
  }

  Scalar x() const { return vec.x(); }
  Scalar y() const { return vec.y(); }
  Scalar z() const { return vec.z(); }
  Scalar getVal() const { return val; }

  void setX(Scalar nx) { vec.setX(nx); }
  void setY(Scalar ny) { vec.setY(ny); }
  void setZ(Scalar nz) { vec.setZ(nz); }
  void setVal(Scalar nval) { val = nval; }

  Vector4 operator+(const Vector4 &rhs) {
    return Vector4(vec + rhs.vec, val + rhs.val);
  }

  Vector4 operator-(const Vector4 &rhs) {
    return Vector4(vec - rhs.vec, val - rhs.val);
  }

  Vector4 operator*(Scalar rhs) { return Vector4(vec * rhs, val * rhs); }

  Vector4 operator/(Scalar rhs) { return Vector4(vec / rhs, val / rhs); }
};

// Yizhou: Templated Triangle structure using Algebra
template <typename Algebra>
struct RenderTriangle {
  typename Algebra::Vector3 p[3];
  typename Algebra::Vector3 norm[3];
};

template <typename Algebra>
struct RenderVertex {
  typename Algebra::Vector3 p;
  typename Algebra::Vector3 norm;
};

// Struct to store the triangle using the indexed vertices
typedef std::array<int, 3> IndexTriangle;

// Double hash table to store the vertex index on each edge
// There is at most one vertex on each edge of each cube
// The vertex on edge with vertices (i, j) can be accessed
// by T[i][j] or T[j][i]
typedef std::unordered_map<int, std::unordered_map<int, int>> VertexTable;

// Struct to store the entire shape
// struct MCShape {
//   std::vector<Vertex> vertices;
//   std::vector<IndexTriangle> index_triangles;
// };

template <typename Algebra>
struct MCShape {
  std::vector<RenderVertex<Algebra>> vertices;
  std::vector<IndexTriangle> index_triangles;
};

// does Linear Interpolation between points p1 and p2 (they already contain
// their computed values)
// mpVector LinearInterp(mp4Vector p1, mp4Vector p2, float value);
template <typename Algebra>
typename Algebra::Vector3 LinearInterp(const Vector4<Algebra> &p1,
                                       const Vector4<Algebra> &p2,
                                       float value) {
  typename Algebra::Vector3 p1_v = p1.vec;
  typename Algebra::Vector3 p2_v = p2.vec;
  typename Algebra::Vector3 p;
  if (abs(Algebra::to_double(p1.getVal()) - Algebra::to_double(p2.getVal())) >
      0.00001) {
    typename Algebra::Vector3 diff = p2_v - p1_v;
    diff *= 1 / (p2.val - p1.val);
    diff *= (value - p1.val);
    p = p1_v + diff;
  } else {
    p = p1_v;
  }
  return p;
}

////////////////////////////////////////////////////////////////////////////////////////
// POINTERS TO FUNCTIONS
// pointer to function which computes the value at point p
// typedef float (*FORMULA)(mpVector);
template <typename Algebra>
using FORMULA =
    std::function<typename Algebra::Scalar(typename Algebra::Vector3)>;
////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
///// MARCHING CUBES ALGORITHM /////
/////////////////////////////////////////////////////////////////////////////////////////

// 'STRAIGHT' Marching Cubes Algorithm
// //////////////////////////////////////////////////
// takes number of cells (ncellsX, ncellsY, ncellsZ) to subdivide on each axis
// minValue used to pass into LinearInterp
// gradFactor for each axis (multiplies each component of gradient vector by
// 1/(2*gradFactor) ).
//		Should be set to the length of a side (or close to it)
// array of length (ncellsX+1)(ncellsY+1)(ncellsZ+1) of mp4Vector points
// containing coordinates and values
// returns pointer to triangle array and the number of triangles in numTriangles
// note: array of points is first taken on z axis, then y and then x. So for
// example, if you iterate through it in a
//       for loop, have indexes i, j, k for x, y, z respectively, then to get
//       the point you will have to make the
//		 following index: i*(ncellsY+1)*(ncellsZ+1) + j*(ncellsZ+1) + k
//. 		Also, the array starts at the minimum on all axes.
// TRIANGLE* MarchingCubes(int ncellsX, int ncellsY, int ncellsZ, float
// gradFactorX, float gradFactorY, float gradFactorZ, 										float minValue, mp4Vector
// * points, int &numTriangles);

// Yizhou: Modified function for the core MarchingCubes algorithm
// MCShape MarchingCubes(int ncellsX, int ncellsY, int ncellsZ,
// 						float gradFactorX, float gradFactorY, float
// gradFactorZ, 						float minValue, mp4Vector * points);
template <typename Algebra>
MCShape<Algebra> MarchingCubes(int ncellsX, int ncellsY, int ncellsZ,
                               float gradFactorX, float gradFactorY,
                               float gradFactorZ, float minValue,
                               typename Algebra::Vector3 *points);
/////////////////////////////////////////////////////////////////////////////////////////

// Yizhou: STRAIGHT implementation with point building
// MCShape MarchingCubes(float mcMinX, float mcMaxX, float mcMinY, float mcMaxY,
// float mcMinZ, float mcMaxZ,
//                     int ncellsX, int ncellsY, int ncellsZ, float gradFactorX,
//                     float gradFactorY, float gradFactorZ, float minValue,
//                     FORMULA formula);

template <typename Algebra>
MCShape<Algebra> MarchingCubes(float mcMinX, float mcMaxX, float mcMinY,
                               float mcMaxY, float mcMinZ, float mcMaxZ,
                               int ncellsX, int ncellsY, int ncellsZ,
                               float gradFactorX, float gradFactorY,
                               float gradFactorZ, float minValue,
                               FORMULA<Algebra> formula);

///////////////////////////////////////////////////////////////////////////
// END of definitions ////
///////////////////////////////////////////////////////////////////////////

// Macros used to compute gradient vector on each vertex of a cube
// argument should be the name of array of vertices
// can be verts or *verts if done by reference
#define CALC_GRAD_VERT_0(verts)                               \
  Vector4(points[ind - YtimeZ][VAL_ID] - (verts[1])[VAL_ID],  \
          points[ind - pointsZ][VAL_ID] - (verts[4])[VAL_ID], \
          points[ind - 1][VAL_ID] - (verts[3])[VAL_ID], (verts[0])[VAL_ID]);
#define CALC_GRAD_VERT_1(verts)                                        \
  Vector4((verts[0])[VAL_ID] - points[ind + 2 * YtimeZ][VAL_ID],       \
          points[ind + YtimeZ - pointsZ][VAL_ID] - (verts[5])[VAL_ID], \
          points[ind + YtimeZ - 1][VAL_ID] - (verts[2])[VAL_ID],       \
          (verts[1])[VAL_ID]);
#define CALC_GRAD_VERT_2(verts)                                        \
  Vector4((verts[3])[VAL_ID] - points[ind + 2 * YtimeZ + 1][VAL_ID],   \
          points[ind + YtimeZ - ncellsZ][VAL_ID] - (verts[6])[VAL_ID], \
          (verts[1])[VAL_ID] - points[ind + YtimeZ + 2][VAL_ID],       \
          (verts[2])[VAL_ID]);
#define CALC_GRAD_VERT_3(verts)                                  \
  Vector4(points[ind - YtimeZ + 1][VAL_ID] - (verts[2])[VAL_ID], \
          points[ind - ncellsZ][VAL_ID] - (verts[7])[VAL_ID],    \
          (verts[0])[VAL_ID] - points[ind + 2][VAL_ID], (verts[3])[VAL_ID]);
#define CALC_GRAD_VERT_4(verts)                                            \
  Vector4(points[ind - YtimeZ + ncellsZ + 1][VAL_ID] - (verts[5])[VAL_ID], \
          (verts[0])[VAL_ID] - points[ind + 2 * pointsZ][VAL_ID],          \
          points[ind + ncellsZ][VAL_ID] - (verts[7])[VAL_ID],              \
          (verts[4])[VAL_ID]);
#define CALC_GRAD_VERT_5(verts)                                                \
  Vector4((verts[4])[VAL_ID] - points[ind + 2 * YtimeZ + ncellsZ + 1][VAL_ID], \
          (verts[1])[VAL_ID] - points[ind + YtimeZ + 2 * pointsZ][VAL_ID],     \
          points[ind + YtimeZ + ncellsZ][VAL_ID] - (verts[6])[VAL_ID],         \
          (verts[5])[VAL_ID]);
#define CALC_GRAD_VERT_6(verts)                                                \
  Vector4((verts[7])[VAL_ID] - points[ind + 2 * YtimeZ + ncellsZ + 2][VAL_ID], \
          (verts[2])[VAL_ID] - points[ind + YtimeZ + 2 * ncellsZ + 3][VAL_ID], \
          (verts[5])[VAL_ID] - points[ind + YtimeZ + ncellsZ + 3][VAL_ID],     \
          (verts[6])[VAL_ID]);
#define CALC_GRAD_VERT_7(verts)                                            \
  Vector4(points[ind - YtimeZ + ncellsZ + 2][VAL_ID] - (verts[6])[VAL_ID], \
          (verts[3])[VAL_ID] - points[ind + 2 * ncellsZ + 3][VAL_ID],      \
          (verts[4])[VAL_ID] - points[ind + ncellsZ + 3][VAL_ID],          \
          (verts[7])[VAL_ID]);

// Yizhou: Helper macro function to update the vertex lookup table and vertex
// list
#define UPDATE_INDEX(edge_id, x, y)                                       \
  {                                                                       \
    int idx = vert_id[x];                                                 \
    int idy = vert_id[y];                                                 \
    int id_v;                                                             \
    if (VT.find(idx) == VT.end() || VT[idx].find(idy) == VT[idx].end()) { \
      RenderVertex<Algebra> &new_vertex = vertices[vert_cnt];             \
      new_vertex.p = intVerts[edge_id];                                   \
      new_vertex.norm = grads[edge_id];                                   \
      id_v = vert_cnt;                                                    \
      VT[idx][idy] = VT[idy][idx] = id_v;                                 \
      vert_cnt++;                                                         \
    } else {                                                              \
      id_v = VT[idx][idy];                                                \
    }                                                                     \
    intVerts_id[edge_id] = id_v;                                          \
  }

template <typename Algebra>
MCShape<Algebra> MarchingCubes(int ncellsX, int ncellsY, int ncellsZ,
                               float gradFactorX, float gradFactorY,
                               float gradFactorZ, float minValue,
                               Vector4<Algebra> *points) {
  using Vector3 = typename Algebra::Vector3;
  using Vector4 = Vector4<Algebra>;

  // This should be enough space
  // If not, change 3 to 4 to reserve more space
  std::vector<IndexTriangle> triangles(3 * ncellsX * ncellsY * ncellsZ,
                                       {0, 0, 0});
  size_t numTriangles = 0;

  int pointsZ = ncellsZ + 1;  // initialize global variable (for extra speed)
  int YtimeZ = (ncellsY + 1) * pointsZ;
  int lastX = ncellsX;  // left from older version
  int lastY = ncellsY;
  int lastZ = ncellsZ;

  Vector4 *verts[8];  // vertices of a cube (array of pointers for extra speed)
  Vector3 intVerts[12];  // linearly interpolated vertices on each edge
  int cubeIndex;         // shows which vertices are outside/inside
  int edgeIndex;         // index returned by edgeTable[cubeIndex]
  Vector4 gradVerts[8];  // gradients at each vertex of a cube
  Vector3 grads[12];     // linearly interpolated gradients on each edge
  int indGrad;           // shows which gradients already have been computed
  int ind, ni, nj;       // ind: index of vertex 0
  // factor by which corresponding coordinates of gradient vectors are scaled
  Vector3 factor(1.0 / (2.0 * gradFactorX), 1.0 / (2.0 * gradFactorY),
                 1.0 / (2.0 * gradFactorZ));

  // Yizhou: Added data structures to index each vertex
  std::vector<RenderVertex<Algebra>> vertices(3 * ncellsX * ncellsY * ncellsZ);
  int intVerts_id[12];
  size_t vert_cnt = 0;
  VertexTable VT;
  int vert_id[8];

  // MAIN LOOP: goes through all the points
  for (int i = 0; i < lastX; i++) {  // x axis
    ni = i * YtimeZ;
    for (int j = 0; j < lastY; j++) {  // y axis
      nj = j * pointsZ;
      for (int k = 0; k < lastZ; k++, ind++)  // z axis
      {
        // initialize vertices
        ind = ni + nj + k;
        verts[0] = &points[ind];
        verts[1] = &points[ind + YtimeZ];
        verts[4] = &points[ind + pointsZ];
        verts[5] = &points[ind + YtimeZ + pointsZ];
        verts[2] = &points[ind + YtimeZ + 1];
        verts[3] = &points[ind + 1];
        verts[6] = &points[ind + YtimeZ + pointsZ + 1];
        verts[7] = &points[ind + pointsZ + 1];

        vert_id[0] = ind;
        vert_id[1] = ind + YtimeZ;
        vert_id[4] = ind + pointsZ;
        vert_id[5] = ind + YtimeZ + pointsZ;
        vert_id[2] = ind + YtimeZ + 1;
        vert_id[3] = ind + 1;
        vert_id[6] = ind + YtimeZ + pointsZ + 1;
        vert_id[7] = ind + pointsZ + 1;

        // get the index
        cubeIndex = int(0);
        for (int n = 0; n < 8; n++)
          if (verts[n]->val <= minValue) cubeIndex |= (1 << n);

        // check if its completely inside or outside
        if (!edgeTable[cubeIndex]) continue;

        indGrad = int(0);
        edgeIndex = edgeTable[cubeIndex];

        if (edgeIndex & 1) {
          intVerts[0] = LinearInterp(*verts[0], *verts[1], minValue);
          if (i != 0 && j != 0 && k != 0)
            gradVerts[0] = CALC_GRAD_VERT_0(*verts) else gradVerts[0] =
                Vector4(1.0, 1.0, 1.0, 1.0);
          if (i != lastX - 1 && j != 0 && k != 0)
            gradVerts[1] = CALC_GRAD_VERT_1(*verts) else gradVerts[1] =
                Vector4(1.0, 1.0, 1.0, 1.0);
          indGrad |= 3;
          grads[0] = LinearInterp(gradVerts[0], gradVerts[1], minValue);
          grads[0][0] *= factor[0];
          grads[0][1] *= factor[1];
          grads[0][2] *= factor[2];

          UPDATE_INDEX(0, 0, 1);
        }
        if (edgeIndex & 2) {
          intVerts[1] = LinearInterp(*verts[1], *verts[2], minValue);
          if (!(indGrad & 2)) {
            if (i != lastX - 1 && j != 0 && k != 0)
              gradVerts[1] = CALC_GRAD_VERT_1(*verts) else gradVerts[1] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 2;
          }
          if (i != lastX - 1 && j != 0 && k != 0)
            gradVerts[2] = CALC_GRAD_VERT_2(*verts) else gradVerts[2] =
                Vector4(1.0, 1.0, 1.0, 1.0);
          indGrad |= 4;
          grads[1] = LinearInterp(gradVerts[1], gradVerts[2], minValue);
          grads[1][0] *= factor[0];
          grads[1][1] *= factor[1];
          grads[1][2] *= factor[2];

          UPDATE_INDEX(1, 1, 2);
        }
        if (edgeIndex & 4) {
          intVerts[2] = LinearInterp(*verts[2], *verts[3], minValue);
          if (!(indGrad & 4)) {
            if (i != lastX - 1 && j != 0 && k != 0)
              gradVerts[2] = CALC_GRAD_VERT_2(*verts) else gradVerts[2] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 4;
          }
          if (i != 0 && j != 0 && k != lastZ - 1)
            gradVerts[3] = CALC_GRAD_VERT_3(*verts) else gradVerts[3] =
                Vector4(1.0, 1.0, 1.0, 1.0);
          indGrad |= 8;
          grads[2] = LinearInterp(gradVerts[2], gradVerts[3], minValue);
          grads[2][0] *= factor[0];
          grads[2][1] *= factor[1];
          grads[2][2] *= factor[2];

          UPDATE_INDEX(2, 2, 3);
        }
        if (edgeIndex & 8) {
          intVerts[3] = LinearInterp(*verts[3], *verts[0], minValue);
          if (!(indGrad & 8)) {
            if (i != 0 && j != 0 && k != lastZ - 1)
              gradVerts[3] = CALC_GRAD_VERT_3(*verts) else gradVerts[3] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 8;
          }
          if (!(indGrad & 1)) {
            if (i != 0 && j != 0 && k != 0)
              gradVerts[0] = CALC_GRAD_VERT_0(*verts) else gradVerts[0] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 1;
          }
          grads[3] = LinearInterp(gradVerts[3], gradVerts[0], minValue);
          grads[3][0] *= factor[0];
          grads[3][1] *= factor[1];
          grads[3][2] *= factor[2];

          UPDATE_INDEX(3, 3, 0);
        }
        if (edgeIndex & 16) {
          intVerts[4] = LinearInterp(*verts[4], *verts[5], minValue);

          if (i != 0 && j != lastY - 1 && k != 0)
            gradVerts[4] = CALC_GRAD_VERT_4(*verts) else gradVerts[4] =
                Vector4(1.0, 1.0, 1.0, 1.0);

          if (i != lastX - 1 && j != lastY - 1 && k != 0)
            gradVerts[5] = CALC_GRAD_VERT_5(*verts) else gradVerts[5] =
                Vector4(1.0, 1.0, 1.0, 1.0);

          indGrad |= 48;
          grads[4] = LinearInterp(gradVerts[4], gradVerts[5], minValue);
          grads[4][0] *= factor[0];
          grads[4][1] *= factor[1];
          grads[4][2] *= factor[2];

          UPDATE_INDEX(4, 4, 5);
        }
        if (edgeIndex & 32) {
          intVerts[5] = LinearInterp(*verts[5], *verts[6], minValue);
          if (!(indGrad & 32)) {
            if (i != lastX - 1 && j != lastY - 1 && k != 0)
              gradVerts[5] = CALC_GRAD_VERT_5(*verts) else gradVerts[5] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 32;
          }

          if (i != lastX - 1 && j != lastY - 1 && k != lastZ - 1)
            gradVerts[6] = CALC_GRAD_VERT_6(*verts) else gradVerts[6] =
                Vector4(1.0, 1.0, 1.0, 1.0);
          indGrad |= 64;
          grads[5] = LinearInterp(gradVerts[5], gradVerts[6], minValue);
          grads[5][0] *= factor[0];
          grads[5][1] *= factor[1];
          grads[5][2] *= factor[2];

          UPDATE_INDEX(5, 5, 6);
        }
        if (edgeIndex & 64) {
          intVerts[6] = LinearInterp(*verts[6], *verts[7], minValue);
          if (!(indGrad & 64)) {
            if (i != lastX - 1 && j != lastY - 1 && k != lastZ - 1)
              gradVerts[6] = CALC_GRAD_VERT_6(*verts) else gradVerts[6] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 64;
          }

          if (i != 0 && j != lastY - 1 && k != lastZ - 1)
            gradVerts[7] = CALC_GRAD_VERT_7(*verts) else gradVerts[7] =
                Vector4(1.0, 1.0, 1.0, 1.0);
          indGrad |= 128;
          grads[6] = LinearInterp(gradVerts[6], gradVerts[7], minValue);
          grads[6][0] *= factor[0];
          grads[6][1] *= factor[1];
          grads[6][2] *= factor[2];

          UPDATE_INDEX(6, 6, 7);
        }
        if (edgeIndex & 128) {
          intVerts[7] = LinearInterp(*verts[7], *verts[4], minValue);
          if (!(indGrad & 128)) {
            if (i != 0 && j != lastY - 1 && k != lastZ - 1)
              gradVerts[7] = CALC_GRAD_VERT_7(*verts) else gradVerts[7] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 128;
          }
          if (!(indGrad & 16)) {
            if (i != 0 && j != lastY - 1 && k != 0)
              gradVerts[4] = CALC_GRAD_VERT_4(*verts) else gradVerts[4] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 16;
          }
          grads[7] = LinearInterp(gradVerts[7], gradVerts[4], minValue);
          grads[7][0] *= factor[0];
          grads[7][1] *= factor[1];
          grads[7][2] *= factor[2];

          UPDATE_INDEX(7, 7, 4);
        }
        if (edgeIndex & 256) {
          intVerts[8] = LinearInterp(*verts[0], *verts[4], minValue);
          if (!(indGrad & 1)) {
            if (i != 0 && j != 0 && k != 0)
              gradVerts[0] = CALC_GRAD_VERT_0(*verts) else gradVerts[0] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 1;
          }
          if (!(indGrad & 16)) {
            if (i != 0 && j != lastY - 1 && k != 0)
              gradVerts[4] = CALC_GRAD_VERT_4(*verts) else gradVerts[4] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 16;
          }
          grads[8] = LinearInterp(gradVerts[0], gradVerts[4], minValue);
          grads[8][0] *= factor[0];
          grads[8][1] *= factor[1];
          grads[8][2] *= factor[2];

          UPDATE_INDEX(8, 0, 4);
        }
        if (edgeIndex & 512) {
          intVerts[9] = LinearInterp(*verts[1], *verts[5], minValue);
          if (!(indGrad & 2)) {
            if (i != lastX - 1 && j != 0 && k != 0)
              gradVerts[1] = CALC_GRAD_VERT_1(*verts) else gradVerts[1] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 2;
          }
          if (!(indGrad & 32)) {
            if (i != lastX - 1 && j != lastY - 1 && k != 0)
              gradVerts[5] = CALC_GRAD_VERT_5(*verts) else gradVerts[5] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 32;
          }
          grads[9] = LinearInterp(gradVerts[1], gradVerts[5], minValue);
          grads[9][0] *= factor[0];
          grads[9][1] *= factor[1];
          grads[9][2] *= factor[2];

          UPDATE_INDEX(9, 1, 5);
        }
        if (edgeIndex & 1024) {
          intVerts[10] = LinearInterp(*verts[2], *verts[6], minValue);
          if (!(indGrad & 4)) {
            if (i != lastX - 1 && j != 0 && k != 0)
              gradVerts[2] = CALC_GRAD_VERT_2(*verts) else gradVerts[5] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 4;
          }
          if (!(indGrad & 64)) {
            if (i != lastX - 1 && j != lastY - 1 && k != lastZ - 1)
              gradVerts[6] = CALC_GRAD_VERT_6(*verts) else gradVerts[6] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 64;
          }
          grads[10] = LinearInterp(gradVerts[2], gradVerts[6], minValue);
          grads[10][0] *= factor[0];
          grads[10][1] *= factor[1];
          grads[10][2] *= factor[2];
          UPDATE_INDEX(10, 2, 6);
        }
        if (edgeIndex & 2048) {
          intVerts[11] = LinearInterp(*verts[3], *verts[7], minValue);
          if (!(indGrad & 8)) {
            if (i != 0 && j != 0 && k != lastZ - 1)
              gradVerts[3] = CALC_GRAD_VERT_3(*verts) else gradVerts[3] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 8;
          }
          if (!(indGrad & 128)) {
            if (i != 0 && j != lastY - 1 && k != lastZ - 1)
              gradVerts[7] = CALC_GRAD_VERT_7(*verts) else gradVerts[7] =
                  Vector4(1.0, 1.0, 1.0, 1.0);
            indGrad |= 128;
          }
          grads[11] = LinearInterp(gradVerts[3], gradVerts[7], minValue);
          grads[11][0] *= factor[0];
          grads[11][1] *= factor[1];
          grads[11][2] *= factor[2];

          UPDATE_INDEX(11, 3, 7);
        }

        // now build the triangles using triTable
        for (int n = 0; triTable[cubeIndex][n] != -1; n += 3) {
          int index[3] = {triTable[cubeIndex][n], triTable[cubeIndex][n + 1],
                          triTable[cubeIndex][n + 2]};
          for (int h = 0; h < 3;
               h++) {  // copy vertices and normals into triangles array
            // triangles[numTriangles].p[h] = intVerts[index[h]];
            // triangles[numTriangles].norm[h] = grads[index[h]];
            triangles[numTriangles][h] = intVerts_id[index[h]];
          }
          numTriangles++;  // one more triangle has been added
        }
      }  // END OF FOR LOOP ON Z AXIS
    }    // END OF FOR LOOP ON Y AXIS
  }      // END OF FOR LOOP ON X AXIS

  // Yizhou: Enclose the final result in MCShape
  triangles.resize(numTriangles);
  vertices.resize(vert_cnt);
  MCShape<Algebra> result;
  result.index_triangles = triangles;
  result.vertices = vertices;

  return result;
}

// Implementation including the initialization of the vertices
template <typename Algebra>
MCShape<Algebra> MarchingCubes(float mcMinX, float mcMaxX, float mcMinY,
                               float mcMaxY, float mcMinZ, float mcMaxZ,
                               int ncellsX, int ncellsY, int ncellsZ,
                               float gradFactorX, float gradFactorY,
                               float gradFactorZ, float minValue,
                               FORMULA<Algebra> formula) {
  using Vector3 = typename Algebra::Vector3;
  using Vector4 = Vector4<Algebra>;
  Vector4 *mcDataPoints =
      new Vector4[(ncellsX + 1) * (ncellsY + 1) * (ncellsZ + 1)];
  Vector3 stepSize((mcMaxX - mcMinX) / ncellsX, (mcMaxY - mcMinY) / ncellsY,
                   (mcMaxZ - mcMinZ) / ncellsZ);

  int YtimesZ = (ncellsY + 1) * (ncellsZ + 1);  // for extra speed
  for (int i = 0; i < ncellsX + 1; i++) {
    int ni = i * YtimesZ;  // for speed
    float vertX = mcMinX + i * stepSize.x();
    for (int j = 0; j < ncellsY + 1; j++) {
      int nj = j * (ncellsZ + 1);  // for speed
      float vertY = mcMinY + j * stepSize.y();
      for (int k = 0; k < ncellsZ + 1; k++) {
        Vector4 vert(vertX, vertY, mcMinZ + k * stepSize.z(), 0);
        vert.val = formula(vert.vec);
        mcDataPoints[ni + nj + k] = vert;
      }
    }
  }

  // Run the optimized Marching Cubes on the data
  return MarchingCubes(ncellsX, ncellsY, ncellsZ, gradFactorX, gradFactorY,
                       gradFactorZ, minValue, mcDataPoints);
}

}  // namespace MC

#undef VAL_ID