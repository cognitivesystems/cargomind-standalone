cmake_minimum_required(VERSION 2.8.3)
project(system_gui)

# compiler flags - compile with warning and debugging support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing")

find_package(Bullet REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL QtNetwork)
set(QT_USE_QTMAIN TRUE)
include(${QT_USE_FILE})
find_package(Coin)
find_package(OpenGL)
find_package(SoQt)
find_package(TinyXML REQUIRED)

include_directories(
  ${QT_INCLUDES}
  ${COIN_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${SOQT_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIR}
  ${TinyXML_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/include
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/resources
  /usr/include/jsoncpp
)

LINK_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/lib

)

include("${CMAKE_SOURCE_DIR}/cmake/qt_wrap_ui.cmake")

QT4_WRAP_UI_LOCAL(MOC_UI_SRCS
 src/mainwindow.ui
 src/binpackinggui.ui
 src/listitem.ui
)

QT4_WRAP_CPP(MOC_SRCS
 src/mainwindow.h
 src/binpackinggui.h
 src/myslider.h
# src/defs.h
 src/listitem.h
# src/BppROS.h
 src/sginterface.h
 src/WorldStateGenerator.h
# src/wsg_utils.h
# src/SGJSONParser.h
# src/BlobStoreHelper.h
# src/bullet/Actor.h
)

SET (HDRS
 src/mainwindow.h
 src/binpackinggui.h
 src/myslider.h
 src/defs.h
 src/listitem.h
 src/BppROS.h
 src/sginterface.h
 src/WorldStateGenerator.h
 src/wsg_utils.h
 src/SGJSONParser.h
 src/BlobStoreHelper.h

)

SET (SRCS
 src/mainwindow.cpp
 src/binpackinggui.cpp
 src/myslider.cpp
 src/bppinterface.cpp
 src/system_gui_node.cpp
 src/listitem.cpp
 src/BppROS.cpp
 src/sginterface.cpp
 src/WorldStateGenerator.cpp
 src/wsg_utils.cpp
 src/SGJSONParser.cpp
 src/BlobStoreHelper.cpp
)

file(GLOB bpa_SRC "src/bpa/*.h" "src/bpa/*.cpp"  "src/bullet/*.h" "src/bullet/*.cpp")

## Declare a C++ executable
add_executable(system_gui_node ${MOC_SRCS} ${MOC_UI_SRCS} ${HDRS} ${SRCS} ${bpa_SRC})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(system_gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(system_gui_node
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${COIN_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${SOQT_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${TinyXML_LIBRARIES}
  curl
  jsoncpp
  gui_lib
  pthread
)

