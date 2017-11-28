cmake_minimum_required( VERSION 2.8 )
project( mmd_ar )

set( CMAKE_BUILD_TYPE Release )
set(CMAKE_CXX_STANDARD 14)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(Eigen REQUIRED)
IF (Eigen_FOUND)
  message([message] Eigen_VER:${EIGEN_VERSION})
  include_directories(${EIGEN_INCLUDE_DIRS})
  link_directories(${EIGEN_LIBRARY_DIRS})
ENDIF()

find_package(OpenMP)
find_package(GLUT)
find_package(OpenGL)
find_package(Bullet REQUIRED)
set(Pangolin_DIR "${PROJECT_SOURCE_DIR}/third_party/Pangolin/build/src")
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message("[Pngolin include dir] ${Pangolin_INCLUDE_DIRS}")
find_package(Pangolin REQUIRED)

set(OpenCV_DIR "~/.local/OpenCV33/share/OpenCV")
find_package(OpenCV REQUIRED)
IF (OpenCV_FOUND)
  message([message] OpenCV_VER:${OpenCV_VERSION})
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
  add_definitions(${OpenCV_DEFINITIONS})
ENDIF()

# Saba dependencies
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/src)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/viewer)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/gl3w/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/gli/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/glm/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/stb/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/spdlog/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/tinyobjloader/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/imgui/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/picojson/include)
include_directories(BEFORE ${PROJECT_SOURCE_DIR}/third_party/saba/external/lua)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/sol2/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/saba/external/tinyxfileloader/include)
include_directories(${BULLET_INCLUDE_DIRS})

SET(CMAKE_C_FLAGS_RELEASE "-O3  -Wall -Wextra -Wunused-variable  -g -fPIC -msse2 -msse3 -msse4 -ffast-math")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wunused-variable -g -fPIC -msse2 -msse3 -msse4 -ffast-math")

add_executable( 
  mmd_ar
  main.cc
  viewer.cc
  utils.cc
  frame.cc
  tracking.cc
  simulator.cc
  )

target_link_libraries( 
  mmd_ar
  ${OpenCV_LIBS}
  -pthread 
  -lpthread
  -lgomp 
  -fopenmp  
  ${GLUT_LIBRARY} 
  ${OPENGL_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/external/lua/liblua.a
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/external/fcpp/libfcpp.a
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/viewer/libSabaViewer.a
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/src/libSaba.a
  ${BULLET_LIBRARIES}
  )

add_executable( 
  mmd_ar_simulator
  main2.cc
  viewer.cc
  utils.cc
  frame.cc
  tracking.cc
  simulator.cc
  )

target_link_libraries( 
  mmd_ar_simulator
  ${OpenCV_LIBS}
  -pthread 
  -lpthread
  -lgomp 
  -fopenmp  
  ${GLUT_LIBRARY} 
  ${OPENGL_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/external/lua/liblua.a
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/external/fcpp/libfcpp.a
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/viewer/libSabaViewer.a
  ${PROJECT_SOURCE_DIR}/third_party/saba/build/src/libSaba.a
  ${BULLET_LIBRARIES}
  )

  add_executable( 
    mmd_ar_marker
    main3.cc
    viewer.cc
    utils.cc
    frame.cc
    tracking.cc
    simulator.cc
    )
  
  target_link_libraries( 
    mmd_ar_marker
    ${OpenCV_LIBS}
    -pthread 
    -lpthread
    -lgomp 
    -fopenmp  
    ${GLUT_LIBRARY} 
    ${OPENGL_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/third_party/saba/build/external/lua/liblua.a
    ${PROJECT_SOURCE_DIR}/third_party/saba/build/external/fcpp/libfcpp.a
    ${PROJECT_SOURCE_DIR}/third_party/saba/build/viewer/libSabaViewer.a
    ${PROJECT_SOURCE_DIR}/third_party/saba/build/src/libSaba.a
    ${BULLET_LIBRARIES}
    )