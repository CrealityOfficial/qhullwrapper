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
}