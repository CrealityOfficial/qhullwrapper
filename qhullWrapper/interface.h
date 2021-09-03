#ifndef QHULL_WRAPPER_INTERFACE
#define QHULL_WRAPPER_INTERFACE

#ifdef WIN32
	#ifdef QHULL_WRAPPER_DLL
		#define QHULL_WRAPPER_API __declspec(dllexport)
	#else
		#define QHULL_WRAPPER_API __declspec(dllimport)
	#endif
#else
	#ifdef QHULL_WRAPPER_DLL
		#define QHULL_WRAPPER_API __attribute__((visibility("default")))
	#else
		#define QHULL_WRAPPER_API
	#endif
#endif

#endif // QHULL_WRAPPER_INTERFACE