# CMakeLists.txt for Chassis

# Add module to include path
target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Auto-include source files
file(GLOB MODULE_CHASSIS_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

target_sources(xr PRIVATE ${MODULE_CHASSIS_SRC})

# Single-precision DAQP; Helm binds the ordinary QP workspace to fixed arrays.
file(GLOB HELM_DAQP_SRC "${CMAKE_CURRENT_LIST_DIR}/third_party/daqp/src/*.c")
target_sources(xr PRIVATE ${HELM_DAQP_SRC})
target_include_directories(xr PRIVATE "${CMAKE_CURRENT_LIST_DIR}/third_party/daqp/include")
set_source_files_properties(${HELM_DAQP_SRC} PROPERTIES
    COMPILE_DEFINITIONS "DAQP_SINGLE_PRECISION;DAQP_NO_SOFT_WEIGHTS"
)
