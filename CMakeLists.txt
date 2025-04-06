cmake_minimum_required(VERSION 3.15)

# 设置 Release 模式
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 添加 O2 优化选项
if(MSVC)
  add_compile_options("/O2")
else()
  add_compile_options("-O2")
endif()

# 添加CUDA语言支持
project(Moer-lite LANGUAGES CXX CUDA)

# 设置CUDA编译器选项
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 70 75 80)  # 根据你的GPU架构调整，例如：70/75/80等

# 添加CUDA相关编译选项
if(NOT MSVC)
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --expt-relaxed-constexpr --expt-extended-lambda")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr --expt-extended-lambda")
endif()

if(MSVC)
  add_compile_options("/MP")
  add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
  add_definitions(-D_USE_MATH_DEFINES)
endif(MSVC)

if(WIN32)
  add_definitions(-DUNICODE -D_UNICODE)
  add_definitions(-DNOMINMAX)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/target/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/target/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/target/bin)

add_subdirectory(./externals)

add_executable(Moer
  src/main.cpp
  src/CoreLayer/Debug/Debug.cpp

  src/CoreLayer/Math/Transform.cpp

  src/FunctionLayer/Acceleration/AABB.cpp
  src/FunctionLayer/Acceleration/Acceleration.cpp
  src/FunctionLayer/Acceleration/EmbreeBVH.cpp
  src/FunctionLayer/Acceleration/Linear.cpp
  src/FunctionLayer/Acceleration/Octree.cpp
  src/FunctionLayer/Acceleration/BVH.cpp
  src/FunctionLayer/Acceleration/LBVH.cpp

  src/FunctionLayer/Integrator/NormalIntegrator.cpp
  src/FunctionLayer/Integrator/DirectIntegrator.cpp
  src/FunctionLayer/Integrator/WhittedIntegrator.cpp
  src/FunctionLayer/Camera/Camera.cpp
  src/FunctionLayer/Camera/Pinhole.cpp
  src/FunctionLayer/Scene/Scene.cpp
  src/FunctionLayer/Light/SpotLight.cpp
  src/FunctionLayer/Light/AreaLight.cpp
  src/FunctionLayer/Light/EnvironmentLight.cpp
  src/FunctionLayer/Shape/Shape.cpp
  src/FunctionLayer/Shape/Triangle.cpp
  src/FunctionLayer/Shape/Cube.cpp
  src/FunctionLayer/Shape/Sphere.cpp
  src/FunctionLayer/Shape/Parallelogram.cpp
  src/FunctionLayer/Texture/NormalTexture.cpp
  src/FunctionLayer/Texture/ImageTexture.cpp
  src/FunctionLayer/Texture/Texture.cpp
  src/FunctionLayer/Texture/Mipmap.cpp
  src/FunctionLayer/Material/Matte.cpp
  src/FunctionLayer/Material/Mirror.cpp
  src/FunctionLayer/Material/Material.cpp
  src/FunctionLayer/Sampler/IndependentSampler.cpp
  src/ResourceLayer/Image.cpp
  src/ResourceLayer/Mesh.cpp
  src/ResourceLayer/FileUtil.cpp
  src/FunctionLayer/Shape/Disk.cpp
  src/FunctionLayer/Shape/Cone.cpp
  src/FunctionLayer/Shape/Cylinder.cpp 
  src/FunctionLayer/Integrator/UVIntegrator.cpp)

# 启用CUDA支持
set_target_properties(Moer PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

# 针对CUDA代码禁用一些警告
if(CMAKE_CUDA_COMPILER_ID STREQUAL "NVIDIA")
  target_compile_options(Moer PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--disable-warnings>)
endif()

target_include_directories(Moer PUBLIC ./externals)
target_include_directories(Moer PUBLIC ./src)
target_include_directories(Moer PUBLIC ./externals/json/include)
target_include_directories(Moer PUBLIC ./externals/embree/include)
target_include_directories(Moer PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

if(WIN32)
  set(LIBS_PATH ${PROJECT_SOURCE_DIR}/libs/win)
  set(EMBREE_DLL ${LIBS_PATH}/embree3.dll ${LIBS_PATH}/tbb12.dll)
endif()

if(UNIX AND NOT APPLE)
  set(LIBS_PATH ${PROJECT_SOURCE_DIR}/libs/linux)
  set(EMBREE_DLL ${LIBS_PATH}/libembree3.so.3 ${LIBS_PATH}/libtbb.so.12.2)
endif()

if(APPLE)
  set(LIBS_PATH ${PROJECT_SOURCE_DIR}/libs/macos)
  set(EMBREE_DLL ${LIBS_PATH}/libembree3.3.dylib ${LIBS_PATH}/libtbb.12.2.dylib)
endif()


target_link_directories(Moer PUBLIC ${LIBS_PATH})
target_link_directories(Moer PUBLIC ${PROJECT_SOURCE_DIR}/target/lib)

add_custom_command(TARGET Moer POST_BUILD
  DEPENDS ${LIBS_PATH}/embree3.dll
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${EMBREE_DLL} $<TARGET_FILE_DIR:Moer>)

target_link_libraries(Moer PUBLIC embree3)
target_link_libraries(Moer PUBLIC tinyobjloader)
target_link_libraries(Moer PUBLIC cuda cudart)

if(APPLE)
  execute_process(
    COMMAND uname -m
    RESULT_VARIABLE result
    OUTPUT_VARIABLE MOER_OSX_NATIVE_ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(${MOER_OSX_NATIVE_ARCHITECTURE} STREQUAL "arm64")
    set_target_properties(Moer PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
  endif()
endif()