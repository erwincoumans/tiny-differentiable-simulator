#ifndef TINY_MESH_UTILS_H
#define TINY_MESH_UTILS_H
#include "tiny_min_max.h"
#include <float.h>

#include "math/tiny/tiny_float_utils.h"
#include "tiny_obj_loader.h"

class TinyMeshUtils {
public:
	static void extract_shape(
		const tinyobj::attrib_t& attribute,
		const tinyobj::shape_t& shape,
		const std::vector<tinyobj::material_t>& materials,
		std::vector<int>& indices,
		std::vector<GfxVertexFormat1>& vertices,
		int& texture_index)
	{
		texture_index = -1;
		bool flatShading = false;

		{
			int faceCount = shape.mesh.indices.size();

			for (int f = 0; f < faceCount; f += 3)
			{
				::TINY::TinyVector3f normal(0, 1, 0);
				int vtxBaseIndex = vertices.size();

				if (f < 0 && f >= int(shape.mesh.indices.size()))
				{
					continue;
				}

				GfxVertexFormat1 vtx0;
				tinyobj::index_t v_0 = shape.mesh.indices[f];
				vtx0.x = attribute.vertices[3 * v_0.vertex_index];
				vtx0.y = attribute.vertices[3 * v_0.vertex_index + 1];
				vtx0.z = attribute.vertices[3 * v_0.vertex_index + 2];
				vtx0.w = 0.f;

				if (attribute.texcoords.size())
				{
					int uv0Index = 2 * v_0.texcoord_index;
					int uv1Index = 2 * v_0.texcoord_index + 1;
					if (uv0Index >= 0 && uv1Index >= 0 && (uv0Index < int(attribute.texcoords.size()) && (uv1Index < attribute.texcoords.size())))
					{
						vtx0.u = attribute.texcoords[uv0Index];
						vtx0.v = attribute.texcoords[uv1Index];
					}
					else
					{
						//	b3Warning("obj texture coordinate out-of-range!");
						vtx0.u = 0;
						vtx0.v = 0;
					}
				}
				else
				{
					vtx0.u = 0.5;
					vtx0.v = 0.5;
				}

				GfxVertexFormat1 vtx1;
				tinyobj::index_t v_1 = shape.mesh.indices[f + 1];
				vtx1.x = attribute.vertices[3 * v_1.vertex_index];
				vtx1.y = attribute.vertices[3 * v_1.vertex_index + 1];
				vtx1.z = attribute.vertices[3 * v_1.vertex_index + 2];
				vtx1.w = 0.f;

				if (attribute.texcoords.size())
				{
					int uv0Index = 2 * v_1.texcoord_index;
					int uv1Index = 2 * v_1.texcoord_index + 1;
					if (uv0Index >= 0 && uv1Index >= 0 && (uv0Index < attribute.texcoords.size()) && (uv1Index < attribute.texcoords.size()))
					{
						vtx1.u = attribute.texcoords[uv0Index];
						vtx1.v = attribute.texcoords[uv1Index];
					}
					else
					{
						//	b3Warning("obj texture coordinate out-of-range!");
						vtx1.u = 0;
						vtx1.v = 0;
					}
				}
				else
				{
					vtx1.u = 0.5f;
					vtx1.v = 0.5f;
				}

				GfxVertexFormat1 vtx2;
				tinyobj::index_t v_2 = shape.mesh.indices[f + 2];
				vtx2.x = attribute.vertices[3 * v_2.vertex_index];
				vtx2.y = attribute.vertices[3 * v_2.vertex_index + 1];
				vtx2.z = attribute.vertices[3 * v_2.vertex_index + 2];
				vtx2.w = 0.f;
				if (attribute.texcoords.size())
				{
					int uv0Index = 2 * v_2.texcoord_index;
					int uv1Index = 2 * v_2.texcoord_index + 1;

					if (uv0Index >= 0 && uv1Index >= 0 && (uv0Index < attribute.texcoords.size()) && (uv1Index < attribute.texcoords.size()))
					{
						vtx2.u = attribute.texcoords[uv0Index];
						vtx2.v = attribute.texcoords[uv1Index];
					}
					else
					{
						//b3Warning("obj texture coordinate out-of-range!");
						vtx2.u = 0;
						vtx2.v = 0;
					}
				}
				else
				{
					vtx2.u = 0.5;
					vtx2.v = 0.5;
				}

				::TINY::TinyVector3f v0(vtx0.x, vtx0.y, vtx0.z);
				::TINY::TinyVector3f v1(vtx1.x, vtx1.y, vtx1.z);
				::TINY::TinyVector3f v2(vtx2.x, vtx2.y, vtx2.z);

				unsigned int maxIndex = 0;
				unsigned n0Index = shape.mesh.indices[f].normal_index;
				unsigned n1Index = shape.mesh.indices[f + 1].normal_index;
				unsigned n2Index = shape.mesh.indices[f + 2].normal_index;

				maxIndex = TinyMax(maxIndex, 3 * n0Index + 0);
				maxIndex = TinyMax(maxIndex, 3 * n0Index + 1);
				maxIndex = TinyMax(maxIndex, 3 * n0Index + 2);
				maxIndex = TinyMax(maxIndex, 3 * n1Index + 0);
				maxIndex = TinyMax(maxIndex, 3 * n1Index + 1);
				maxIndex = TinyMax(maxIndex, 3 * n1Index + 2);
				maxIndex = TinyMax(maxIndex, 3 * n2Index + 0);
				maxIndex = TinyMax(maxIndex, 3 * n2Index + 1);
				maxIndex = TinyMax(maxIndex, 3 * n2Index + 2);

				bool hasNormals = (attribute.normals.size() && maxIndex < attribute.normals.size());

				if (flatShading || !hasNormals)
				{
					normal = (v1 - v0).cross(v2 - v0);
					float len2 = normal.length_squared();
					//skip degenerate triangles
					if (len2 > FLT_EPSILON)
					{
						normal.normalize();
					}
					else
					{
						normal.setValue(0, 0, 0);
					}
					vtx0.nx = normal[0];
					vtx0.ny = normal[1];
					vtx0.nz = normal[2];
					vtx1.nx = normal[0];
					vtx1.ny = normal[1];
					vtx1.nz = normal[2];
					vtx2.nx = normal[0];
					vtx2.ny = normal[1];
					vtx2.nz = normal[2];
				}
				else
				{
					vtx0.nx = attribute.normals[3 * n0Index + 0];
					vtx0.ny = attribute.normals[3 * n0Index + 1];
					vtx0.nz = attribute.normals[3 * n0Index + 2];
					vtx1.nx = attribute.normals[3 * n1Index + 0];
					vtx1.ny = attribute.normals[3 * n1Index + 1];
					vtx1.nz = attribute.normals[3 * n1Index + 2];
					vtx2.nx = attribute.normals[3 * n2Index + 0];
					vtx2.ny = attribute.normals[3 * n2Index + 1];
					vtx2.nz = attribute.normals[3 * n2Index + 2];
				}
				vertices.push_back(vtx0);
				vertices.push_back(vtx1);
				vertices.push_back(vtx2);
				indices.push_back(vtxBaseIndex);
				indices.push_back(vtxBaseIndex + 1);
				indices.push_back(vtxBaseIndex + 2);
			}
		}
	}
};


#endif //TINY_MESH_UTILS_H
