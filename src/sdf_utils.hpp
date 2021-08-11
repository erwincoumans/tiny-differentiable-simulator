#pragma once

#include <functional>


namespace tds {
  


struct MarchingCubesConfig {
  float minX;
  float maxX;
  float minY;
  float maxY;
  float minZ;
  float maxZ;

  int ncellsX;
  int ncellsY;
  int ncellsZ;

  float gradX;
  float gradY;
  float gradZ;

  float minValue = 0;

  bool compute_normals = true;
};

} // namespace tds