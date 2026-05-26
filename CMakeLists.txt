cmake_minimum_required(VERSION 3.19)
project(em-shield-designer LANGUAGES CXX)

# ── C++ standard ─────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ─────────────────────────────────────────────────────────────

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets PrintSupport LinguistTools)

# Add Eigen include path (header-only library, no compilation needed)
include_directories(${CMAKE_SOURCE_DIR}/external/eigen/eigen-5.0.0)

add_definitions(-DDEBUG_APERTURE)

qt_standard_project_setup()

# ── Windows resource script (embeds .ico into .exe) ──────────
# Only compiled on Windows. On other platforms the variable is
# empty and no .rc file is added to the source list.
set(APP_WIN_RESOURCE "")
if(WIN32)
    set(APP_WIN_RESOURCE "${CMAKE_SOURCE_DIR}/resources/app.rc")
endif()
# ─────────────────────────────────────────────────────────────

qt_add_executable(em-shield-designer
    WIN32 MACOSX_BUNDLE
    src/main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    qcustomplot.h
    qcustomplot.cpp
    include/core/PhysicsConstants.h
    include/core/BranchTemplate.h
    include/core/TL_EmptyCavity.h
    include/core/SRC_VoltageSource.h
    include/core/LOAD_Impedance.h
    include/core/MNASolver.h
    src/core/MNASolver.cpp
    include/core/AP_SlotAperture.h
    include/core/TL_DielectricCavity.h
    include/core/AP_SlotWithCover.h
    include/core/CircuitGenerator.h
    SectionItem.h
    CircuitCanvas.h
    PropertyPanel.h
    StartupWindow.h
    CircuitBuilderWindow.h
    resources/resources.qrc
    ${APP_WIN_RESOURCE}
    Styles.h
    StackLayerPanel.h
    MessageDialog.h
)

# ── Internationalisation (P3a) ───────────────────────────────
# Compile translations/*.ts -> *.qm and embed the .qm into the
# executable under the :/i18n/ resource prefix (loaded by main.cpp).
#
# NOTE: qt_add_translations() / qt6_add_lrelease() are deliberately
# NOT used. Their internal _qt_internal_expose_source_file_to_ide step
# passes the target's binary directory to set_source_files_properties
# UNQUOTED, which breaks on any path containing a space — and this
# project lives under "C:/Users/Salif Kante/...". We instead use the
# lower-level qt6_add_translation() (lrelease only, no IDE exposure,
# space-safe) and embed the generated .qm via a .qrc compiled directly
# with rcc. Same outcome: .qm under build/, embedded at :/i18n/.
# Source language is English; the Russian catalogue is filled in P3d.
set(EMSHIELD_TS_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/translations/em-shield-designer_en.ts"
    "${CMAKE_CURRENT_SOURCE_DIR}/translations/em-shield-designer_ru.ts"
)
qt6_add_translation(EMSHIELD_QM_FILES ${EMSHIELD_TS_FILES})

# Build a .qrc that embeds every compiled .qm under the :/i18n/ prefix.
set(EMSHIELD_QM_QRC "${CMAKE_CURRENT_BINARY_DIR}/em-shield-designer_i18n.qrc")
set(EMSHIELD_QRC_ENTRIES "")
foreach(_qm IN LISTS EMSHIELD_QM_FILES)
    get_filename_component(_qm_name "${_qm}" NAME)
    string(APPEND EMSHIELD_QRC_ENTRIES
        "        <file alias=\"${_qm_name}\">${_qm}</file>\n")
endforeach()
file(WRITE "${EMSHIELD_QM_QRC}"
    "<!DOCTYPE RCC><RCC version=\"1.0\">\n"
    "    <qresource prefix=\"/i18n\">\n"
    "${EMSHIELD_QRC_ENTRIES}"
    "    </qresource>\n"
    "</RCC>\n")

# Compile the generated .qrc with rcc into a .cpp added to the target.
# Explicit add_custom_command (not a Qt resource macro) so no spaced-path
# IDE-exposure helper is ever invoked. DEPENDS on the .qm chains build
# order after lrelease.
set(EMSHIELD_QM_RCC_CPP "${CMAKE_CURRENT_BINARY_DIR}/em-shield-designer_i18n_qrc.cpp")
add_custom_command(
    OUTPUT "${EMSHIELD_QM_RCC_CPP}"
    COMMAND ${QT_CMAKE_EXPORT_NAMESPACE}::rcc
            --name em_shield_i18n
            --output "${EMSHIELD_QM_RCC_CPP}"
            "${EMSHIELD_QM_QRC}"
    DEPENDS ${EMSHIELD_QM_FILES} "${EMSHIELD_QM_QRC}"
    VERBATIM
)
target_sources(em-shield-designer PRIVATE "${EMSHIELD_QM_RCC_CPP}")

# [P3b] update_translations — re-run lupdate to refresh the Russian .ts
# from the tr()/translate() calls in the sources. Invoke manually:
#   cmake --build build --target update_translations
# Uses lupdate directly (NOT qt_add_lupdate, which would re-trigger the
# spaced-path IDE-exposure bug). en.ts is intentionally NOT passed:
# English is the source language and is never loaded, so it stays empty.
set(EMSHIELD_I18N_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/StartupWindow.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/CircuitBuilderWindow.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/PropertyPanel.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/StackLayerPanel.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/SectionItem.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/CircuitCanvas.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/MessageDialog.h"
)
add_custom_target(update_translations
    COMMAND ${QT_CMAKE_EXPORT_NAMESPACE}::lupdate
            ${EMSHIELD_I18N_SOURCES}
            -ts "${CMAKE_CURRENT_SOURCE_DIR}/translations/em-shield-designer_ru.ts"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Updating Russian translation catalogue (lupdate)"
    VERBATIM
)
# ─────────────────────────────────────────────────────────────

# macOS bundle icon file name (ignored on other platforms).
# Harmless to leave in; has no effect on Windows builds.
set_target_properties(em-shield-designer PROPERTIES
    MACOSX_BUNDLE_ICON_FILE "emshield.icns"
)

target_link_libraries(em-shield-designer
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt6::PrintSupport
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
