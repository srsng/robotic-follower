cmake_minimum_required(VERSION 3.14)
project(pcl_processing)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖项
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

# 包含目录
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include
    src
)

# 编译选项
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)

# 链接库
link_directories(
    ${OpenCV_LIBS_DIR}
    ${PCL_LIBRARY_DIRS}
)

# ===========================================
# 源文件
# ===========================================

# 核心处理库
set(CORE_SRCS
    src/depth_to_pointcloud.cpp
    src/density_calculator.cpp
)

# 滤波器
set(FILTER_SRCS
    src/filters/voxel_filter.cpp
    src/filters/statistical_filter.cpp
    src/filters/passthrough_filter.cpp
    src/filters/radius_filter.cpp
)

# 分割模块
set(SEGMENTATION_SRCS
    src/segmentation/plane_segmentation.cpp
    src/segmentation/euclidean_clustering.cpp
)

# ===========================================
# 库构建
# ===========================================

# 创建静态库
add_library(pcl_processing_core STATIC
    ${CORE_SRCS}
    ${FILTER_SRCS}
    ${SEGMENTATION_SRCS}
)

# 添加依赖
target_link_libraries(pcl_processing_core
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)

# PCL 定义
target_compile_definitions(pcl_processing_core PRIVATE
    ${PCL_DEFINITIONS}
)

# ===========================================
# 示例程序
# ===========================================

if(BUILD_EXAMPLES)
    # 测试程序
    add_executable(test_filters
        examples/test_filters.cpp
    )
    target_link_libraries(test_filters
        pcl_processing_core
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
    )

    # 传统处理管道示例
    add_executable(traditional_pipeline_demo
        examples/traditional_pipeline_demo.cpp
    )
    target_link_libraries(traditional_pipeline_demo
        pcl_processing_core
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
    )
endif()

# ===========================================
# 安装
# ===========================================

install(TARGETS pcl_processing_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/pcl_processing/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

# 安装示例
if(BUILD_EXAMPLES)
    install(TARGETS test_filters traditional_pipeline_demo
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
endif()

# ===========================================
# 打印配置信息
# ===========================================

message(STATUS "")
message(STATUS "pcl_processing Configuration:")
message(STATUS "  OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  PCL version: ${PCL_VERSION}")
message(STATUS "  Boost version: ${Boost_VERSION}")
message(STATUS "  Build examples: ${BUILD_EXAMPLES}")
message(STATUS "  Build Python bindings: ${BUILD_PYTHON_BINDINGS}")
message(STATUS "")
