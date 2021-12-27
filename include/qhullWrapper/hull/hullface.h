#ifndef QHULLWRAPPER_HULLFACE_1640345352031_H
#define QHULLWRAPPER_HULLFACE_1640345352031_H
#include "qhullWrapper/interface.h"
#include <functional>
#include <vector>

namespace trimesh
{
	class TriMesh;
}

namespace qhullWrapper
{
	QHULLWRAPPER_API std::vector<trimesh::TriMesh*> hullFacesFromConvexMesh(trimesh::TriMesh* mesh);
	QHULLWRAPPER_API std::vector<trimesh::TriMesh*> hullFacesFromMesh(trimesh::TriMesh* mesh);
}

#endif // QHULLWRAPPER_HULLFACE_1640345352031_H