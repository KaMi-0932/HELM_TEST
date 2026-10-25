# CMakeLists.txt for Chassis

# Add module to include path
target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Auto-include source files
file(GLOB MODULE_CHASSIS_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

target_sources(xr PRIVATE ${MODULE_CHASSIS_SRC})
