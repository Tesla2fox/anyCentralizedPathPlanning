﻿#生成PathPlanning的lib

#添加文件
set(PP_file_list 
	${PROJECT_SOURCE_DIR}/bgeoSrc/bgeometry.h
	${PROJECT_SOURCE_DIR}/bgeoSrc/bgeometry.cpp
	#iostream
	${PROJECT_SOURCE_DIR}/ioSrc/ssconfig.hpp
	${PROJECT_SOURCE_DIR}/ioSrc/Gwrite.hpp
	${PROJECT_SOURCE_DIR}/ioSrc/Gwrite.cpp
	
	#test_exe
	${PROJECT_SOURCE_DIR}/pathPlanningSrc/test_PathPlanning.cpp
	)
	

include_directories (${PROJECT_SOURCE_DIR}/bgeoSrc)	
include_directories(../include)
find_package(Boost 1.65.0)

if(Boost_FOUND)

include_directories(${Boost_INCLUDE_DIRS})

message(${Boost_INCLUDE_DIRS})

endif()

add_executable(test_pathPlanning ${PP_file_list})

if (UNIX)
    target_compile_options(pathplan PRIVATE -std=c++11)
    target_compile_options(test_pathplan PRIVATE -std=c++11)
    message(STATUS "Using -std=c++11")
endif()
