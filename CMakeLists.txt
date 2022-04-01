__assert_target(trimesh2)
__assert_target(mmesh)

set(SRCS)
set(LIBS trimesh2 mmesh)
set(INCS ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(INTERFACES ${CMAKE_CURRENT_SOURCE_DIR}/include/)

include(qhull.cmake)
include(wrapper.cmake)

__add_platform_library(qhullWrapper)
					
if(CC_INSTALL_QHULLWRAPPER)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/include/ qhullWrapper)
endif()
