cmake_minimum_required(VERSION 3.19)
project(em-shield-designer LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets)
# Add Eigen include path (header-only library, no compilation needed)
include_directories(${CMAKE_SOURCE_DIR}/external/eigen/eigen-5.0.0)

qt_standard_project_setup()

qt_add_executable(em-shield-designer
    WIN32 MACOSX_BUNDLE
    src/main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    include/core/PhysicsConstants.h
    include/core/BranchTemplate.h
    include/core/TL_EmptyCavity.h
    include/core/SRC_VoltageSource.h
    include/core/LOAD_Impedance.h
    include/core/MNASolver.h
    src/core/MNASolver.cpp
)

target_link_libraries(em-shield-designer
    PRIVATE
        Qt::Core
        Qt::Widgets
)

include(GNUInstallDirs)

install(TARGETS em-shield-designer
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET em-shield-designer
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
