# CMake 最低版本需要求
cmake_minimum_required (VERSION 3.8)

# 如果支持，请为 MSVC 编译器启用热重载。
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

# 项目名称
project ("lunar_recon")

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv4")

find_package(OpenCV REQUIRED)
find_package(apriltag)

# 头文件搜索路径
include_directories(include)

# 将源代码添加到此项目的可执行文件。
add_executable (lunar_recon src/main.cpp)
target_link_libraries(lunar_recon PRIVATE
	${OpenCV_LIBS}
	apriltag::apriltag
)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET lunar_recon PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
