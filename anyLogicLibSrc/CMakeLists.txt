#生成PathPlanning的lib

#添加文件
set(anylib_file_list 
	${PROJECT_SOURCE_DIR}/anyLogicLibSrc/cpp_CppPathPlanning.h
	${PROJECT_SOURCE_DIR}/anyLogicLibSrc/cpp_CppPathPlanning.cpp	
	${PROJECT_SOURCE_DIR}/taskAllocationSrc/centralizedMRTA.h
	${PROJECT_SOURCE_DIR}/taskAllocationSrc/centralizedMRTA.cpp
	
#	${PROJECT_SOURCE_DIR}/taskAllocationSrc/test_MRTA.cpp
	)
	

include_directories (${PROJECT_SOURCE_DIR}/bgeoSrc)
include_directories (${PROJECT_SOURCE_DIR}/taskAllocationSrc)
include_directories(${PROJECT_SOURCE_DIR}/src)
 	

include_directories(../include)
find_package(Boost 1.65.0)

if(Boost_FOUND)

include_directories(${Boost_INCLUDE_DIRS})

message(${Boost_INCLUDE_DIRS})

endif()

#add_executable(test_MRTA ${MRTA_file_list})

add_library(anyLogicLib SHARED ${anylib_file_list})
#设置输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
#设置输出名称
set_target_properties(anyLogicLib PROPERTIES OUTPUT_NAME "libAnyPlanning")


if (UNIX)
    target_compile_options(pathplan PRIVATE -std=c++11)
    target_compile_options(test_pathplan PRIVATE -std=c++11)
    message(STATUS "Using -std=c++11")
endif()
