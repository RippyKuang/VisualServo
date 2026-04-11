find_package(OpenCV REQUIRED)

include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}"
		            )

add_library(vsc STATIC imgProcess.cpp)

target_link_libraries(vsc ${OpenCV_LIBS})