__find_qhull()

set(qhullWrapperSrc qhullWrapper/interface.h
					qhullWrapper/meshconvex.h
					qhullWrapper/meshconvex.cpp
					)
					
if(STATIC_QHULL_WRAPPER)
	__add_real_target(qhullWrapper lib SOURCE ${qhullWrapperSrc}
								   LIB qhullstatic_r qhullcpp
								   INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
								   FOLDER qhull
								   )
else()
	__add_real_target(qhullWrapper dll SOURCE ${qhullWrapperSrc}
								   LIB qhullstatic_r qhullcpp
								   DEF QHULL_WRAPPER_DLL
								   INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
								   FOLDER qhull
								   )
endif()
