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
idf_component_register(SRCS "main.cpp" INCLUDE_DIRS "."
                        REQUIRES framework-arduinoespressif32)
