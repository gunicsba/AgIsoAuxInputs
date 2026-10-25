# Custom component file instead of PlatformIO's auto-generated one.
#
# REQUIRES framework-arduinoespressif32: the auto-generated CMakeLists.txt
# PlatformIO would otherwise produce wires this dependency in automatically;
# a custom CMakeLists.txt has to declare it explicitly, or app_main()
# (defined in that component, calling Arduino's setup()/loop()) never gets
# linked in ("undefined reference to app_main").
#
# The object pool is compiled in as a plain C byte array
# (object_pool/aux_n_pool_data.h) rather than linker-embedded via
# target_add_binary_data()/board_build.embed_files - both turned out to be
# fragile (mis-pathed / silently not firing) under this mixed
# arduino+espidf build, so the array is far simpler and just works.
set(APP_SRCS "aux_buttons.cpp" "lights.cpp" "main.cpp" "settings.cpp" "debug_log.cpp" "canlink_bridge.cpp" "wifi_setup.cpp" "object_pool_adapt.cpp")
set(APP_INCLUDES ".")

# joystick_seeder: the seeder controller's core and ISOBUS app as a second
# working set (SEEDER_DIR is set by tools/seeder_combo.py). Its own ESP
# layer (I/O, web, OTA) stays out; storage.cpp keeps its settings in NVS.
if(DEFINED ENV{SEEDER_DIR})
  set(SEEDER_DIR "$ENV{SEEDER_DIR}")
  file(GLOB SEEDER_CORE_SRCS "${SEEDER_DIR}/src/core/*.cpp")
  list(APPEND APP_SRCS ${SEEDER_CORE_SRCS}
       "${SEEDER_DIR}/src/app/seeder_app.cpp"
       "${SEEDER_DIR}/src/app/tc_link.cpp"
       "${SEEDER_DIR}/src/app/vt_ui.cpp"
       "${SEEDER_DIR}/src/esp/storage.cpp")
  list(APPEND APP_INCLUDES "${SEEDER_DIR}/include" "${SEEDER_DIR}/src/app" "${SEEDER_DIR}/src/esp")
endif()

idf_component_register(SRCS ${APP_SRCS} INCLUDE_DIRS ${APP_INCLUDES}
                        REQUIRES framework-arduinoespressif32)

if(DEFINED ENV{SEEDER_DIR})
  # The seeder is written for ESP-IDF 5 (C++17); this IDF 4.4 build defaults to gnu++11
  target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
endif()
