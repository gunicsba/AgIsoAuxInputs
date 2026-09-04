# Only used when a PlatformIO env builds with framework=espidf (currently
# just env:joystick_aux_n). Explicitly lists the one source file to build,
# since ESP-IDF's default auto-generated CMakeLists.txt globs *every* .cpp
# under src/ - which would pull in this repo's other (mutually exclusive)
# ECU firmware files, e.g. main.cpp's own setup()/loop(), causing duplicate
# symbols. Plain-Arduino-framework envs are unaffected by this file.
#
# The object pool is compiled in as a plain C byte array
# (object_pool/aux_n_pool_data.h) rather than linker-embedded via
# target_add_binary_data()/board_build.embed_files - both turned out to be
# fragile (mis-pathed / silently not firing) under this mixed
# arduino+espidf build, so the array is far simpler and just works.
# REQUIRES framework-arduinoespressif32: the auto-generated CMakeLists.txt
# PlatformIO would otherwise produce wires this dependency in automatically;
# a custom CMakeLists.txt has to declare it explicitly, or app_main()
# (defined in that component, calling Arduino's setup()/loop()) never gets
# linked in ("undefined reference to app_main").
idf_component_register(SRCS "joystick_aux_n.cpp" INCLUDE_DIRS "."
                        REQUIRES framework-arduinoespressif32)
