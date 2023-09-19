#ifndef QHULLWRAPPER_HULLFACE_1640345352031_H
#define QHULLWRAPPER_HULLFACE_1640345352031_H
#include "qhullWrapper/interface.h"
#include "trimesh2/TriMesh.h"
#include <functional>
#include <memory>
#include <vector>

typedef std::shared_ptr<trimesh::TriMesh> HMeshPtr;

namespace trimesh
{
	class TriMesh;
}

namespace qhullWrapper
{
	QHULLWRAPPER_API std::vector<trimesh::TriMesh*> hullFacesFromConvexMesh(trimesh::TriMesh* mesh);
	QHULLWRAPPER_API std::vector<trimesh::TriMesh*> hullFacesFromMesh(trimesh::TriMesh* mesh);
    struct HullFace {
        HMeshPtr mesh = std::make_shared<trimesh::TriMesh>();
        trimesh::vec3 normal;
        float hullarea = 0.0f;
    };

    QHULLWRAPPER_API void hullFacesFromConvexMesh(trimesh::TriMesh* convexMesh, std::vector<HullFace>& hullFaces, float thresholdNormal = 0.995);
}

#endif // QHULLWRAPPER_HULLFACE_1640345352031_H