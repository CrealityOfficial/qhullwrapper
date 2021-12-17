#include "qhullWrapper/hull/meshconvex.h"
#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"

#include "trimesh2/TriMesh.h"
#include "ccglobal/tracer.h"

using namespace orgQhull;
namespace qhullWrapper
{
	trimesh::point calTriNormal(trimesh::point ver1, trimesh::point ver2, trimesh::point ver3)
	{
		double temp1[3], temp2[3], normal[3];
		double length = 0.0;
		temp1[0] = ver2[0] - ver1[0];
		temp1[1] = ver2[1] - ver1[1];
		temp1[2] = ver2[2] - ver1[2];
		temp2[0] = ver3[0] - ver2[0];
		temp2[1] = ver3[1] - ver2[1];
		temp2[2] = ver3[2] - ver2[2];
		//计算法线
		normal[0] = temp1[1] * temp2[2] - temp1[2] * temp2[1];
		normal[1] = -(temp1[0] * temp2[2] - temp1[2] * temp2[0]);
		normal[2] = temp1[0] * temp2[1] - temp1[1] * temp2[0];
		//法线单位化
		length = sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
		if (length == 0.0f) { length = 1.0f; }
		normal[0] /= length;
		normal[1] /= length;
		normal[2] /= length;
		trimesh::point e_normal(normal[0], normal[1], normal[2]);
		return e_normal;
	}

	void calculateConvex(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
	{
		if (!mesh || mesh->vertices.size() == 0)
			return;

		size_t vertexSize = mesh->vertices.size();
		std::vector<bool> added(vertexSize, false);
		try
		{
			Qhull q("", 3, vertexSize, (float*)mesh->vertices.data(), "");
			QhullFacetList facets = q.facetList();

			mesh->flags.clear();
			auto fadd = [&mesh, &added](int id) {
				if (!added.at(id))
				{
					mesh->flags.push_back(id);
					added.at(id) = true;
				}
			};
			for (QhullFacet f : facets)
			{
				if (!f.isGood())
				{
					// ignore facet
				}
				else if (!f.isTopOrient() && f.isSimplicial())
				{ /* orient the vertices like option 'o' */
					QhullVertexSet vs = f.vertices();
					fadd(vs[1].point().id());
					fadd(vs[0].point().id());
					for (int i = 2; i < (int)vs.size(); ++i)
					{
						fadd(vs[i].point().id());
					}
				}
				else
				{  /* note: for non-simplicial facets, this code does not duplicate option 'o', see qh_facet3vertex and qh_printfacetNvertex_nonsimplicial */
					for (QhullVertex vertex : f.vertices())
					{
						QhullPoint p = vertex.point();
						fadd(p.id());
					}
				}
			}
		}
		catch (QhullError& e) {
		}
	}

	trimesh::TriMesh* convex_hull_3d(trimesh::TriMesh* inMesh)
	{
		orgQhull::Qhull qhull;
		qhull.disableOutputStream(); // we want qhull to be quiet
		std::vector<realT> src_vertices;

		if (inMesh->vertices.size() == 0)
			return nullptr;

		trimesh::TriMesh* outMesh = new trimesh::TriMesh();
		{
			qhull.runQhull("", 3, (int)inMesh->vertices.size(), (const realT*)(inMesh->vertices.front().data()), "Qt");
		}

		// Let's collect results:
		auto facet_list = qhull.facetList().toStdVector();
		for (const orgQhull::QhullFacet& facet : facet_list)// iterate through facets
		{
			orgQhull::QhullVertexSet vertices = facet.vertices();
			for (int i = 0; i < 3; ++i)// iterate through facet's vertices
			{
				orgQhull::QhullPoint p = vertices[i].point();
				const auto* coords = p.coordinates();
				outMesh->vertices.emplace_back(coords[0], coords[1], coords[2]);
			}
			auto Qhullcoord = facet.hyperplane().coordinates();
			trimesh::vec a(Qhullcoord[0], Qhullcoord[1], Qhullcoord[2]);

			unsigned int size = (unsigned int)outMesh->vertices.size();
			trimesh::vec b = calTriNormal(outMesh->vertices[size - 3], outMesh->vertices[size - 2], outMesh->vertices[size - 1]);

			float d = trimesh::dot(a, b);
			if (d > 0)
			{
				outMesh->faces.emplace_back(size - 3, size - 2, size - 1);
			}
			else
			{
				outMesh->faces.emplace_back(size - 1, size - 2, size - 3);
			}
		}
		return outMesh;
	}

	void convex_hull_2d(trimesh::TriMesh& inMesh)
	{
		if (inMesh.vertices.size() < 3)
		{
			return ;
		}

		std::vector<trimesh::point> outVertices;
		std::sort(inMesh.vertices.begin(), inMesh.vertices.end());

		outVertices.resize(3 * inMesh.vertices.size());

		// Build lower hull
		int hullNum = 0;
		for (int i = 0; i < inMesh.vertices.size(); ++i)
		{
			trimesh::point currentPoint = inMesh.vertices[i];
			while (hullNum >= 2)
			{
				trimesh::point hull_1 = outVertices[hullNum - 1];
				trimesh::point hull_2 = outVertices[hullNum - 2];

				double num = (double)(hull_2.x - hull_1.x) * (double)(currentPoint.y - hull_1.y) -
					(double)(hull_2.y - hull_1.y) * (double)(currentPoint.x - hull_1.x);
				if (num <= 0)
				{
					--hullNum;
				}
				else
				{
					break;
				}
			}
			outVertices[hullNum++] = inMesh.vertices[i];
		}

		// Build upper hull
		for (int i = inMesh.vertices.size() - 2, t = hullNum + 1; i >= 0; --i)
		{
			trimesh::point currentPoint = inMesh.vertices[i];
			while (hullNum >= t)
			{
				trimesh::point hull_1 = outVertices[hullNum - 1];
				trimesh::point hull_2 = outVertices[hullNum - 2];

				double num = (double)(hull_2.x - hull_1.x) * (double)(currentPoint.y - hull_1.y) -
					(double)(hull_2.y - hull_1.y) * (double)(currentPoint.x - hull_1.x);
				if (num <= 0)
				{
					--hullNum;
				}
				else
				{
					break;
				}
			}
			outVertices[hullNum++] = inMesh.vertices[i];
		}

		outVertices.resize(hullNum);
		if (outVertices.front() == outVertices.back())
		{
			outVertices.pop_back();
		}
		inMesh.vertices = outVertices;
	}
}