cmake_minimum_required(VERSION 3.13)
project(ccusb VERSION 1.0.0 LANGUAGES C)

option(CCUSB_BUILD_TOOLS "Build the ccusb command-line tool" ON)
option(CCUSB_BUILD_TESTS "Build the unit tests" ON)
option(BUILD_SHARED_LIBS "Build libccusb as a shared library" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
include(GNUInstallDirs)

# ---- libusb-1.0 ------------------------------------------------------
# Linux/macOS: pkg-config.  Windows: set LIBUSB_ROOT (headers in
# include/libusb-1.0, import library in lib or a VS20xx/MS64 subfolder),
# or use vcpkg (libusb) which also provides pkg-config files.
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND AND NOT LIBUSB_ROOT AND NOT DEFINED ENV{LIBUSB_ROOT})
  pkg_check_modules(LIBUSB IMPORTED_TARGET libusb-1.0)
endif()
if(LIBUSB_FOUND)
  set(CCUSB_LIBUSB PkgConfig::LIBUSB)
else()
  find_path(LIBUSB_INCLUDE_DIR libusb.h
    HINTS ${LIBUSB_ROOT} ENV LIBUSB_ROOT
    PATH_SUFFIXES include/libusb-1.0 libusb-1.0 include)
  find_library(LIBUSB_LIBRARY NAMES usb-1.0 libusb-1.0
    HINTS ${LIBUSB_ROOT} ENV LIBUSB_ROOT
    PATH_SUFFIXES lib MinGW64/dll MinGW64/static VS2022/MS64/dll VS2019/MS64/dll MS64/dll)
  if(NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
    message(FATAL_ERROR "libusb-1.0 not found. Install libusb-1.0-0-dev "
      "(Debian/Ubuntu), libusb1-devel (Fedora), 'brew install libusb', or set "
      "LIBUSB_ROOT on Windows.")
  endif()
  add_library(ccusb_libusb INTERFACE)
  target_include_directories(ccusb_libusb INTERFACE ${LIBUSB_INCLUDE_DIR})
  target_link_libraries(ccusb_libusb INTERFACE ${LIBUSB_LIBRARY})
  set(CCUSB_LIBUSB ccusb_libusb)
endif()

# ---- library ---------------------------------------------------------
add_library(ccusb
  src/ccusb.c
  src/usb.c
  src/sim.c
  src/platform.c
  src/xxusb_compat.c)
target_include_directories(ccusb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(ccusb PRIVATE ${CCUSB_LIBUSB})
target_compile_definitions(ccusb PRIVATE CCUSB_BUILDING)
if(NOT BUILD_SHARED_LIBS)
  target_compile_definitions(ccusb PUBLIC CCUSB_STATIC)
endif()
set_target_properties(ccusb PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  C_VISIBILITY_PRESET hidden)
if(MSVC)
  target_compile_options(ccusb PRIVATE /W3 /D_CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(ccusb PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

install(TARGETS ccusb EXPORT ccusbTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES include/ccusb/ccusb.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ccusb)
install(FILES include/libxxusb.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT ccusbTargets FILE ccusbConfig.cmake NAMESPACE ccusb::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ccusb)

configure_file(ccusb.pc.in ${CMAKE_CURRENT_BINARY_DIR}/ccusb.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ccusb.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Optional: copy the built library into python/ccusb so that
# "pip install ./python" bundles it.
option(CCUSB_PYTHON_BUNDLE "Copy libccusb into the Python package after build" OFF)
if(CCUSB_PYTHON_BUNDLE)
  add_custom_command(TARGET ccusb POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ccusb>
            ${CMAKE_CURRENT_SOURCE_DIR}/python/ccusb/$<TARGET_FILE_NAME:ccusb>)
endif()

# ---- tools -----------------------------------------------------------
if(CCUSB_BUILD_TOOLS)
  add_executable(ccusb_cli tools/ccusb_cli.c)
  # same base name as the library: keep the debug symbol files apart
  set_target_properties(ccusb_cli PROPERTIES OUTPUT_NAME ccusb PDB_NAME ccusb_cli)
  target_link_libraries(ccusb_cli PRIVATE ccusb)
  if(MSVC)
    target_compile_options(ccusb_cli PRIVATE /D_CRT_SECURE_NO_WARNINGS)
  endif()
  install(TARGETS ccusb_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# ---- tests -----------------------------------------------------------
if(CCUSB_BUILD_TESTS)
  enable_testing()
  add_executable(test_protocol tests/test_protocol.c)
  target_link_libraries(test_protocol PRIVATE ccusb)
  add_test(NAME protocol COMMAND test_protocol)

  add_executable(test_simulator tests/test_simulator.c)
  target_link_libraries(test_simulator PRIVATE ccusb)
  add_test(NAME simulator COMMAND test_simulator)

  add_executable(test_legacy tests/test_legacy.c)
  target_link_libraries(test_legacy PRIVATE ccusb)
  add_test(NAME legacy COMMAND test_legacy)
endif()
