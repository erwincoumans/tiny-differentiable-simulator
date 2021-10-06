#ifndef TINY_VERTEX_FORMAT_H
#define TINY_VERTEX_FORMAT_H
struct GfxVertexFormat0 {
  float x, y, z, w;
  float unused0, unused1, unused2, unused3;
  float u, v;
};

struct GfxVertexFormat1 {
  float x, y, z, w;
  float nx, ny, nz;
  float u, v;
};
#endif //TINY_VERTEX_FORMAT_H