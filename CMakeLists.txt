cmake_minimum_required(VERSION 3.16.0)

# If building under ESP-IDF (IDF_PATH set and project.cmake present), include
# the ESP-IDF project helper. Otherwise fall back to a normal CMake project
# so desktop CI and local builds work without the ESP-IDF toolchain.
if(DEFINED ENV{IDF_PATH} AND EXISTS "$ENV{IDF_PATH}/tools/cmake/project.cmake")
	include($ENV{IDF_PATH}/tools/cmake/project.cmake)
	project(DOPE)
else()
	project(DOPE)
	# If the repository provides a local tools/cmake/project.cmake, include it
	# (optional helper for non-ESP builds).
	if(EXISTS "${CMAKE_SOURCE_DIR}/tools/cmake/project.cmake")
		include("${CMAKE_SOURCE_DIR}/tools/cmake/project.cmake")
	endif()
endif()
