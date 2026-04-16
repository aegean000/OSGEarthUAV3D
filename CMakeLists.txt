cmake_minimum_required(VERSION 3.19)
project(project1 LANGUAGES CXX)

# --- 1. 显式设置路径 (统一使用正斜杠 /) ---
set(OSG_DIR "D:/OSG/osg-install")
set(OSGEARTH_DIR "D:/OSG/QT-osgearth/osgearth-install")

# --- 2. 查找 Qt6 组件 ---
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# --- 3. 包含头文件目录 ---
include_directories(
    "${OSG_DIR}/include"
    "${OSGEARTH_DIR}/include"
)

# --- 4. 包含库文件目录 ---
link_directories(
    "${OSG_DIR}/lib"
    "${OSGEARTH_DIR}/lib"
)

# --- 5. 定义可执行程序 ---
add_executable(project1
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    QOSGWidget.h QOSGWidget.cpp
)

# --- 6. 链接库文件 ---
# 注意：如果编译报 LNK1104 错误，请检查 lib 目录下对应的文件名
target_link_libraries(project1
    PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGLWidgets
    OpenThreads.lib
    osg.lib
    osgDB.lib
    osgViewer.lib
    osgGA.lib
    osgUtil.lib
    osgText.lib
    osgEarth.lib
)