#生成PathPlanning的lib

#添加文件
set(MRTA_file_list 
	${PROJECT_SOURCE_DIR}/bgeoSrc/bgeometry.h
	${PROJECT_SOURCE_DIR}/bgeoSrc/bgeometry.cpp
	${PROJECT_SOURCE_DIR}/taskAllocationSrc/centralizedMRTA.h
	${PROJECT_SOURCE_DIR}/taskAllocationSrc/centralizedMRTA.cpp
	${PROJECT_SOURCE_DIR}/taskAllocationSrc/test_MRTA.cpp
	)
	

include_directories (${PROJECT_SOURCE_DIR}/bgeoSrc)	
include_directories(../include)
find_package(Boost 1.65.0)

if(Boost_FOUND)

include_directories(${Boost_INCLUDE_DIRS})

message(${Boost_INCLUDE_DIRS})

endif()

add_executable(test_MRTA ${MRTA_file_list})

if (UNIX)
    target_compile_options(pathplan PRIVATE -std=c++11)
    target_compile_options(test_pathplan PRIVATE -std=c++11)
    message(STATUS "Using -std=c++11")
endif()
