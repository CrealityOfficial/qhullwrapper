__assert_target(trimesh2)

set(SRCS)
set(LIBS trimesh2)
set(INCS ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(INERTFACES ${CMAKE_CURRENT_SOURCE_DIR}/include/)

include(qhull.cmake)
include(wrapper.cmake)

__add_platform_library(qhullWrapper)
					
if(CC_INSTALL_QHULLWRAPPER)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/include/ qhullWrapper)
endif()
