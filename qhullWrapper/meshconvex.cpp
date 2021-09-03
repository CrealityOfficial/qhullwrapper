#include "meshconvex.h"
#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"

namespace qhullWrapper
{
	using namespace orgQhull;
	class MeshConvexCalculatorImpl : public MeshConvexCalculator
	{
	public:
		MeshConvexCalculatorImpl()
		{
		}

		~MeshConvexCalculatorImpl()
		{
		}

		void setInput(float* vertexInput, unsigned vertexSize, std::function<void(float)> func) override
		{
			if (!vertexInput || vertexSize == 0)
				return;

			std::vector<bool> added(vertexSize, false);
			try
			{
				Qhull q("", 3, vertexSize, vertexInput, "");
				QhullFacetList facets = q.facetList();

				auto fadd = [this, &added](int id) {
					if (!added.at(id))
					{
						flags.push_back(id);
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

		int convexVertexCount() override
		{
			return (int)flags.size();
		}

		void fillConvexIndex(unsigned int* indicesBuffer, int bufferSize) override
		{
			int size = (int)flags.size();
			for (int i = 0; i < bufferSize; ++i)
				if (i < size)
					indicesBuffer[i] = flags.at(i);
		}

	protected:
		std::vector<unsigned> flags;
	};

	MeshConvexCalculator* createConvexCalculator()
	{
		return new MeshConvexCalculatorImpl();
	}

	void deleteConvexCalculator(MeshConvexCalculator* calculator)
	{
		delete calculator;
	}
}