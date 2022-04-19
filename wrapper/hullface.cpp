#include "qhullWrapper/hull/hullface.h"
#include "qhullWrapper/hull/meshconvex.h"
#include "trimesh2/TriMesh.h"
#include "trimesh2/XForm.h"
#include "trimesh2/quaternion.h"

#include "mmesh/util/dumplicate.h"

#include <numeric>

namespace qhullWrapper
{
	class MeshVertex
	{
	public:
		trimesh::point p;
		std::vector<uint32_t> connected_faces;

		MeshVertex(trimesh::point p) : p(p) { connected_faces.reserve(8); }
	};

	class MeshFace
	{
	public:
		int vertex_index[3] = { -1 };
		int connected_face_index[3];
		bool bUnique = true;
		bool bAdd = false;

		int index = 0;  //just for sort
	};

	class MeshTest
	{
	public:
		MeshTest() {};
		~MeshTest() {};
		std::vector<MeshVertex> vertices;
		std::vector<MeshFace> faces;
	};

	class FPoint3
	{
	public:
		float x, y, z;
		FPoint3() {}
		FPoint3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
		FPoint3(const trimesh::point& p) : x(p.x * .001), y(p.y * .001), z(p.z * .001) {}
		float vSize2() const
		{
			return x * x + y * y + z * z;
		}

		float vSize() const
		{
			return sqrt(vSize2());
		}
		bool operator==(FPoint3& p) const { return x == p.x && y == p.y && z == p.z; }
		FPoint3 operator/(const float f) const { return FPoint3(x / f, y / f, z / f); }
		FPoint3 operator*(const float f) const { return FPoint3(x * f, y * f, z * f); }
		FPoint3& operator *= (const float f) { x *= f; y *= f; z *= f; return *this; }
		FPoint3 cross(const FPoint3& p) const
		{
			return FPoint3(
				y * p.z - z * p.y,
				z * p.x - x * p.z,
				x * p.y - y * p.x);
		}
		double operator*(FPoint3 lhs)
		{
			return lhs.x * x + lhs.y * y + lhs.z * z;
		}
	};

	int getFaceIdxWithPoints(MeshTest& ameshtest, int idx0, int idx1, int notFaceIdx, int notFaceVertexIdx)
	{
		std::vector<int> candidateFaces;
		for (int f : ameshtest.vertices[idx0].connected_faces)
		{
			if (f == notFaceIdx)
			{
				continue;
			}
			if (ameshtest.faces[f].vertex_index[0] == idx1
				|| ameshtest.faces[f].vertex_index[1] == idx1
				|| ameshtest.faces[f].vertex_index[2] == idx1
				)  candidateFaces.push_back(f);

		}

		if (candidateFaces.size() == 0)
		{
			// has_disconnected_faces = true;
			return -1;
		}
		if (candidateFaces.size() == 1) { return candidateFaces[0]; }


		if (candidateFaces.size() % 2 == 0)
		{
			//cura::logDebug("Warning! Edge with uneven number of faces connecting it!(%i)\n", candidateFaces.size() + 1);
			//has_disconnected_faces = true;
		}

		FPoint3 vn = ameshtest.vertices[idx1].p - ameshtest.vertices[idx0].p;
		FPoint3 n = vn / vn.vSize(); // the normal of the plane in which all normals of faces connected to the edge lie => the normalized normal
		FPoint3 v0 = ameshtest.vertices[idx1].p - ameshtest.vertices[idx0].p;

		// the normals below are abnormally directed! : these normals all point counterclockwise (viewed from idx1 to idx0) from the face, irrespective of the direction of the face.
		FPoint3 n0 = FPoint3(ameshtest.vertices[notFaceVertexIdx].p - ameshtest.vertices[idx0].p).cross(v0);

		if (n0.vSize() <= 0)
		{
			//cura::logDebug("Face %i has zero area!", notFaceIdx);
		}

		double smallestAngle = 1000; // more then 2 PI (impossible angle)
		int bestIdx = -1;

		for (int candidateFace : candidateFaces)
		{
			int candidateVertex;
			{// find third vertex belonging to the face (besides idx0 and idx1)
				for (candidateVertex = 0; candidateVertex < 3; candidateVertex++)
					if (ameshtest.faces[candidateFace].vertex_index[candidateVertex] != idx0 && ameshtest.faces[candidateFace].vertex_index[candidateVertex] != idx1)
						break;
			}

			FPoint3 v1 = ameshtest.vertices[ameshtest.faces[candidateFace].vertex_index[candidateVertex]].p - ameshtest.vertices[idx0].p;
			FPoint3 n1 = v0.cross(v1);

			double dot = n0 * n1;
			double det = n * n0.cross(n1);
			double angle = std::atan2(det, dot);
#define  M_PI 3.1415926
			if (angle < 0) angle += 2 * M_PI; // 0 <= angle < 2* M_PI

			if (angle == 0)
			{
				//cura::logDebug("Overlapping faces: face %i and face %i.\n", notFaceIdx, candidateFace);
				//has_overlapping_faces = true;
			}
			if (angle < smallestAngle)
			{
				smallestAngle = angle;
				bestIdx = candidateFace;
			}
		}
		if (bestIdx < 0)
		{
			//cura::logDebug("Couldn't find face connected to face %i.\n", notFaceIdx);
			//has_disconnected_faces = true;
		}
		return bestIdx;
	};

	void trimesh2meshtest(trimesh::TriMesh* currentMesh, MeshTest& ameshTest)
	{
		for (trimesh::point apoint : currentMesh->vertices)
		{
			ameshTest.vertices.push_back(apoint);
		}

		for (unsigned int i = 0; i < currentMesh->faces.size(); i++)
		{
			trimesh::TriMesh::Face& face = currentMesh->faces[i];
			ameshTest.faces.emplace_back();
			ameshTest.faces[i].vertex_index[0] = face.at(0);
			ameshTest.faces[i].vertex_index[1] = face.at(1);
			ameshTest.faces[i].vertex_index[2] = face.at(2);

			ameshTest.vertices[face.at(0)].connected_faces.push_back(i);
			ameshTest.vertices[face.at(1)].connected_faces.push_back(i);
			ameshTest.vertices[face.at(2)].connected_faces.push_back(i);
		}

		for (unsigned int i = 0; i < currentMesh->faces.size(); i++)
		{
			trimesh::TriMesh::Face& face = currentMesh->faces[i];
			// faces are connected via the outside
			ameshTest.faces[i].connected_face_index[0] = getFaceIdxWithPoints(ameshTest, face.at(0), face.at(1), i, face.at(2));
			ameshTest.faces[i].connected_face_index[1] = getFaceIdxWithPoints(ameshTest, face.at(1), face.at(2), i, face.at(0));
			ameshTest.faces[i].connected_face_index[2] = getFaceIdxWithPoints(ameshTest, face.at(2), face.at(0), i, face.at(1));
		}
	};

	trimesh::fxform fromQuaterianEX(const trimesh::quaternion& q)
	{
		float x2 = q.xp * q.xp;
		float y2 = q.yp * q.yp;
		float z2 = q.zp * q.zp;
		float xy = q.xp * q.yp;
		float xz = q.xp * q.zp;
		float yz = q.yp * q.zp;
		float wx = q.wp * q.xp;
		float wy = q.wp * q.yp;
		float wz = q.wp * q.zp;


		// This calculation would be a lot more complicated for non-unit length quaternions
		// Note: The constructor of Matrix4 expects the Matrix in column-major format like expected by
		//   OpenGL
		return trimesh::fxform(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f,
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f,
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
	}

	double det(trimesh::point& p0, trimesh::point& p1, trimesh::point& p2)
	{
		trimesh::vec3 a = p0 - p1;
		trimesh::vec3 b = p1 - p2;
		return sqrt(pow((a.y * b.z - a.z * b.y), 2) + pow((a.z * b.x - a.x * b.z), 2)
			+ pow((a.x * b.y - a.y * b.x), 2)) / 2.0f;
	}

	double  getArea(std::vector<trimesh::point>& inVertices)
	{
		double sum = 0.0f;
		for (int n = 1; n < inVertices.size() - 1; n++)
		{
			sum += det(inVertices[0], inVertices[n], inVertices[n + 1]);
		}
		return abs(sum);
	}
	trimesh::point normalized(trimesh::point& apoint)
	{
		float  divisor = sqrt(apoint.x * apoint.x + apoint.y * apoint.y + apoint.z * apoint.z);
		return apoint / divisor;
	};

	std::vector<trimesh::TriMesh*> hullFacesFromMesh(trimesh::TriMesh* mesh)
	{
		std::vector<trimesh::TriMesh*> meshes;
		if (mesh)
		{
			trimesh::TriMesh* convex = qhullWrapper::convex_hull_3d(mesh);
			mmesh::weldingMesh(convex);
			meshes = hullFacesFromConvexMesh(convex);
		}
		return meshes;
	}

	void getNormalFace(MeshTest& meshT, trimesh::point& Nnormal,int n, trimesh::TriMesh* aface)
	{
		for (int k = 0; k < 3; k++)
		{
			int faceIndex = meshT.faces[n].connected_face_index[k];
			if (meshT.faces[faceIndex].bAdd)
			{
				continue;//排除已经添加过的face
			}
			int index0 = meshT.faces[faceIndex].vertex_index[0];
			int index1 = meshT.faces[faceIndex].vertex_index[1];
			int index2 = meshT.faces[faceIndex].vertex_index[2];
			trimesh::point thisNormal = trimesh::normalized(trimesh::trinorm(meshT.vertices[index0].p, meshT.vertices[index1].p, meshT.vertices[index2].p));
			if (std::abs(Nnormal.at(0) - thisNormal.at(0)) < 0.01
				&& std::abs(Nnormal.at(1) - thisNormal.at(1)) < 0.01
				&& std::abs(Nnormal.at(2) - thisNormal.at(2)) < 0.01)
			{
				aface->vertices.push_back(meshT.vertices[index0].p);
				aface->vertices.push_back(meshT.vertices[index1].p);
				aface->vertices.push_back(meshT.vertices[index2].p);
				meshT.faces[faceIndex].bAdd = true;
				getNormalFace(meshT, Nnormal, faceIndex, aface);
			}
		}
	}

	std::vector<trimesh::TriMesh*> hullFacesFromConvexMesh(trimesh::TriMesh* mesh)
	{
		std::vector<trimesh::TriMesh*> meshes;

		if (!mesh)
			return meshes;

		MeshTest meshT;
		trimesh2meshtest(mesh, meshT);

		//按面积对meshT 进行排序
		int faceNum = (int)meshT.faces.size();
		if (faceNum <= 0)
			return meshes;

		//zenggui
		std::vector<float> areas(faceNum, 0.0f);
		std::vector<trimesh::vec3> normals(faceNum);
		for (int i = 0; i < faceNum; ++i)
		{
			MeshFace& currentMeshFace = meshT.faces[i];
			currentMeshFace.index = i;
			int currentVertIndex0 = currentMeshFace.vertex_index[0];
			int currentVertIndex1 = currentMeshFace.vertex_index[1];
			int currentVertIndex2 = currentMeshFace.vertex_index[2];
			areas.at(i) = det(meshT.vertices[currentVertIndex0].p, meshT.vertices[currentVertIndex1].p, meshT.vertices[currentVertIndex2].p);

			normals.at(i) = trimesh::normalized(trimesh::trinorm(meshT.vertices[currentVertIndex0].p,
				meshT.vertices[currentVertIndex1].p, meshT.vertices[currentVertIndex2].p));
		}

		std::sort(meshT.faces.begin(), meshT.faces.end(), [&meshT, areas](const MeshFace& face1, const MeshFace& face2)->bool {
			return areas[face1.index] > areas[face2.index];
			});

		for (int n = 0; n < faceNum; n++)
		{
			MeshFace& faceN = meshT.faces[n];
			if (faceN.bUnique == false)
			{
				continue;
			}
		
			trimesh::TriMesh* aface = new trimesh::TriMesh();
			trimesh::point Nnormal = normals.at(faceN.index); 
			for (int m = n + 1; m < faceNum; m++)
			{
				MeshFace& faceM = meshT.faces[m];
				trimesh::point Mnormal = normals.at(faceM.index);
				if (std::abs(Nnormal.at(0) - Mnormal.at(0)) < 0.005
					&& std::abs(Nnormal.at(1) - Mnormal.at(1)) < 0.005
					&& std::abs(Nnormal.at(2) - Mnormal.at(2)) < 0.005)
				{
					aface->vertices.push_back(meshT.vertices[faceM.vertex_index[0]].p);
					aface->vertices.push_back(meshT.vertices[faceM.vertex_index[1]].p);
					aface->vertices.push_back(meshT.vertices[faceM.vertex_index[2]].p);
					faceM.bUnique = false;
					faceM.bAdd = true;
				}
			}
		
			if (faceN.bUnique)
			{
				aface->normals.push_back(Nnormal);
				aface->vertices.push_back(meshT.vertices[faceN.vertex_index[0]].p);
				aface->vertices.push_back(meshT.vertices[faceN.vertex_index[1]].p);
				aface->vertices.push_back(meshT.vertices[faceN.vertex_index[2]].p);
				faceN.bAdd = true;
				getNormalFace(meshT, Nnormal, n, aface); //递归 合并 共法线的相邻面
				meshes.push_back(aface);
			}
			else
			{
				delete aface;
			}
			if (meshes.size() > 500)
			{
				break;
			}
		}
		//wang wenbin
		//for (int n = 1; n < meshT.faces.size(); n++)
		//{
		//	int preIndex = n - 1;
		//	MeshFace currentMeshFace = meshT.faces[n];
		//	int currentVertIndex0 = meshT.faces[n].vertex_index[0];
		//	int currentVertIndex1 = meshT.faces[n].vertex_index[1];
		//	int currentVertIndex2 = meshT.faces[n].vertex_index[2];
		//	int currentArea = det(meshT.vertices[currentVertIndex0].p, meshT.vertices[currentVertIndex1].p, meshT.vertices[currentVertIndex2].p);
		//	while (preIndex >= 0 && det(meshT.vertices[meshT.faces[preIndex].vertex_index[0]].p,
		//		meshT.vertices[meshT.faces[preIndex].vertex_index[1]].p,
		//		meshT.vertices[meshT.faces[preIndex].vertex_index[2]].p) <
		//		currentArea)
		//	{
		//		meshT.faces[preIndex + 1] = meshT.faces[preIndex];
		//		preIndex--;
		//	}
		//	meshT.faces[preIndex + 1] = currentMeshFace;
		//}

		//获取面积最大并且不共法线的500个三角面，并合并和其共法线的面
		//for (int n = 0; n < faceNum; n++)
		//{
		//	if (meshT.faces[n].bUnique==false)
		//	{
		//		continue;
		//	}
		//	trimesh::TriMesh* aface = new trimesh::TriMesh();
		//	int index0 = meshT.faces[n].vertex_index[0];
		//	int index1 = meshT.faces[n].vertex_index[1];
		//	int index2 = meshT.faces[n].vertex_index[2];
		//	trimesh::point Nnormal = trimesh::normalized(trimesh::trinorm(meshT.vertices[index0].p, meshT.vertices[index1].p, meshT.vertices[index2].p));
		//	for (int m = n + 1; m < faceNum; m++)
		//	{
		//		int index0 = meshT.faces[m].vertex_index[0];
		//		int index1 = meshT.faces[m].vertex_index[1];
		//		int index2 = meshT.faces[m].vertex_index[2];
		//		trimesh::point Mnormal = trimesh::normalized(trimesh::trinorm(meshT.vertices[index0].p, meshT.vertices[index1].p, meshT.vertices[index2].p));
		//		if (std::abs(Nnormal.at(0) - Mnormal.at(0)) < 0.005
		//			&& std::abs(Nnormal.at(1) - Mnormal.at(1)) < 0.005
		//			&& std::abs(Nnormal.at(2) - Mnormal.at(2)) < 0.005)
		//		{
		//			aface->vertices.push_back(meshT.vertices[meshT.faces[m].vertex_index[0]].p);
		//			aface->vertices.push_back(meshT.vertices[meshT.faces[m].vertex_index[1]].p);
		//			aface->vertices.push_back(meshT.vertices[meshT.faces[m].vertex_index[2]].p);
		//			meshT.faces[m].bUnique = false;
		//			meshT.faces[m].bAdd = true;
		//		}
		//	}
		//
		//	if (meshT.faces[n].bUnique)
		//	{
		//		aface->normals.push_back(Nnormal);
		//		aface->vertices.push_back(meshT.vertices[meshT.faces[n].vertex_index[0]].p);
		//		aface->vertices.push_back(meshT.vertices[meshT.faces[n].vertex_index[1]].p);
		//		aface->vertices.push_back(meshT.vertices[meshT.faces[n].vertex_index[2]].p);
		//		meshT.faces[n].bAdd = true;
		//		getNormalFace(meshT, Nnormal, n, aface); //递归 合并 共法线的相邻面
		//		meshes.push_back(aface);
		//	}
		//	if (meshes.size() > 500)
		//	{
		//		break;
		//	}
		//}

		//按面积对polygon 进行排序，只保留面积最大的20个polygon:
		std::sort(meshes.rbegin(), meshes.rend(), [](trimesh::TriMesh* a, trimesh::TriMesh* b)
			{
				return getArea(a->vertices) < getArea(b->vertices);
			});
		meshes.resize(std::min((int)meshes.size(), 20));

		//开始遍历所有的多边形，将点转换到xy平面:
		for (unsigned int polygon_id = 0; polygon_id < meshes.size(); ++polygon_id)
		{
			trimesh::TriMesh*& currentMesh = meshes[polygon_id];
			trimesh::vec3 normal = currentMesh->normals[0];

			//将面轻微抬起，防止与模型的面重叠
			for (trimesh::point& apoint : currentMesh->vertices)
			{
				apoint += normal * 0.1;
			}

			//绕z和y旋转，使平面变平
			const trimesh::vec3 XYnormal(0.0f, 0.0f, 1.0f);
			trimesh::quaternion q = q.rotationTo(normal, XYnormal);
			trimesh::fxform xf = fromQuaterian(q);
			for (trimesh::point& apoint : currentMesh->vertices)
			{
				apoint = (xf * apoint);
			}
			currentMesh = qhullWrapper::convex_hull_2d(currentMesh);
			currentMesh->normals.push_back(normal);
			std::vector<trimesh::point>& polygon = currentMesh->vertices;

			//检查多边形的内角，并丢弃角小于以下阈值的多边形
			static constexpr double PI = 3.141592653589793238;
			bool discard = false;
			const double angle_threshold = ::cos(10.0 * (double)PI / 180.0);
			for (unsigned int i = 0; i < polygon.size(); ++i)
			{
				const trimesh::point& prec = polygon[(i == 0) ? polygon.size() - 1 : i - 1];
				const trimesh::point& curr = polygon[i];
				const trimesh::point& next = polygon[(i == polygon.size() - 1) ? 0 : i + 1];
				if (normalized(prec - curr).dot(normalized(next - curr)) > angle_threshold)
				{
					discard = true;
					break;
				}
			}
			if (discard)
			{
				meshes.erase(meshes.begin() + (polygon_id--));
				continue;
			}

			//将多边形内缩一点，防止碰到模型的边缘:
			trimesh::point centroid = std::accumulate(polygon.begin(), polygon.end(), trimesh::point(0.0, 0.0, 0.0));
			centroid /= (double)polygon.size();
			for (auto& vertex : polygon)
			{
				vertex = 0.9f * vertex + 0.1f * centroid;
			}

			//多边形现在是简单和凸的，我们将使尖角变圆，这样看起来更好
			//该算法取一个顶点，计算各自边的中间值，然后移动顶点
			//接近平均水平(由“aggressivity”控制)。重复k次。
			const unsigned int k = 10; // number of iterations
			const float aggressivity = 0.2f;  // agressivity
			const unsigned int N = polygon.size();
			std::vector<std::pair<unsigned int, unsigned int>> neighbours;
			if (k != 0)
			{
				std::vector<trimesh::point> points_out(2 * k * N); // vector long enough to store the future vertices
				for (unsigned int j = 0; j < N; ++j) {
					points_out[j * 2 * k] = polygon[j];
					neighbours.push_back(std::make_pair((int)(j * 2 * k - k) < 0 ? (N - 1) * 2 * k + k : j * 2 * k - k, j * 2 * k + k));
				}

				for (unsigned int i = 0; i < k; ++i) {
					// Calculate middle of each edge so that neighbours points to something useful:
					for (unsigned int j = 0; j < N; ++j)
						if (i == 0)
							points_out[j * 2 * k + k] = 0.5f * (points_out[j * 2 * k] + points_out[j == N - 1 ? 0 : (j + 1) * 2 * k]);
						else {
							float r = 0.2 + 0.3 / (k - 1) * i; // the neighbours are not always taken in the middle
							points_out[neighbours[j].first] = r * points_out[j * 2 * k] + (1 - r) * points_out[neighbours[j].first - 1];
							points_out[neighbours[j].second] = r * points_out[j * 2 * k] + (1 - r) * points_out[neighbours[j].second + 1];
						}
					// Now we have a triangle and valid neighbours, we can do an iteration:
					for (unsigned int j = 0; j < N; ++j)
						points_out[2 * k * j] = (1 - aggressivity) * points_out[2 * k * j] +
						aggressivity * 0.5f * (points_out[neighbours[j].first] + points_out[neighbours[j].second]);

					for (auto& n : neighbours) {
						++n.first;
						--n.second;
					}
				}
				polygon = points_out; // replace the coarse polygon with the smooth one that we just created
			}
			//通过逆矩阵转换回三维坐标
			for (trimesh::point& apoint : meshes[polygon_id]->vertices)
			{
				apoint = trimesh::inv(xf) * apoint;
			}
		}

		//三角化
		for (trimesh::TriMesh* amesh : meshes)
		{
			std::vector<trimesh::point> apoints = amesh->vertices;
			amesh->vertices.clear();
			if (apoints.size() < 3)
			{
				continue;
			}
			//修正法线
			trimesh::vec3 originNormal = amesh->normals[0];
			trimesh::vec3 newNormal = trimesh::normalized(trimesh::trinorm(apoints[0], apoints[1], apoints[2]));
			bool isReverse = false;
			if ((originNormal DOT newNormal)<0)
			{
				isReverse = true;
			}

			for (int n = 2; n < apoints.size(); n++)
			{
				int index = amesh->vertices.size();
				amesh->vertices.push_back(apoints[0]);
				amesh->vertices.push_back(apoints[n - 1]);
				amesh->vertices.push_back(apoints[n]);
				if (isReverse)
				{
					amesh->faces.push_back(trimesh::TriMesh::Face(index+2, index + 1, index));
				}
				else
				{
					amesh->faces.push_back(trimesh::TriMesh::Face(index, index + 1, index + 2));
				}
			}
		}
		return meshes;
	}
}