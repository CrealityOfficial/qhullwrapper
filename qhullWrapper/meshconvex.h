#ifndef QHULLWRAPPER_MESHCONVEX_1627200674710_H
#define QHULLWRAPPER_MESHCONVEX_1627200674710_H
#include "qhullWrapper/interface.h"
#include <functional>

namespace qhullWrapper
{
	class MeshConvexCalculator
	{
	public:
		virtual ~MeshConvexCalculator() {}

		virtual void setInput(float* vertex, unsigned vertexSize, std::function<void(float)> func) = 0;
		virtual int convexVertexCount() = 0;
		virtual void fillConvexIndex(unsigned int* indicesBuffer, int bufferSize) = 0;
	};

	class Progress
	{
	public:
		virtual ~Progress() {}
		virtual void progress(float r) = 0;
	};

	QHULL_WRAPPER_API MeshConvexCalculator* createConvexCalculator();
	QHULL_WRAPPER_API void deleteConvexCalculator(MeshConvexCalculator* calculator);
}

#endif // QHULLWRAPPER_MESHCONVEX_1627200674710_H