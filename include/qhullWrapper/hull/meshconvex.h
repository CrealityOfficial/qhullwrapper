#ifndef QHULLWRAPPER_MESHCONVEX_1627200674710_H
#define QHULLWRAPPER_MESHCONVEX_1627200674710_H
#include "qhullWrapper/interface.h"
#include <functional>

namespace trimesh
{
	class TriMesh;
}

namespace ccglobal
{
	class Tracer;
}

namespace qhullWrapper
{
	QHULLWRAPPER_API void calculateConvex(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer);

	QHULLWRAPPER_API trimesh::TriMesh* convex_hull_3d(trimesh::TriMesh* inMesh);
}

#endif // QHULLWRAPPER_MESHCONVEX_1627200674710_H