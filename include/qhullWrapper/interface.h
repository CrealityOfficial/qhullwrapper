#ifndef QHULL_WRAPPER_INTERFACE
#define QHULL_WRAPPER_INTERFACE
#include "ccglobal/export.h"

#ifdef QHULLWRAPPER_DLL
	#define QHULLWRAPPER_API CC_DECLARE_EXPORT
#else
	#define QHULLWRAPPER_API CC_DECLARE_IMPORT
#endif

#endif // QHULL_WRAPPER_INTERFACE