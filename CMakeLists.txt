# Define the library
add_library(serialcom
    src/serialcom.cpp
)

# Include directories
target_include_directories(serialcom PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
