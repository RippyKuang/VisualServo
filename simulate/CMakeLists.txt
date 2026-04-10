find_package(mujoco CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}"
		            )

add_library(simulate STATIC simulate.cpp robot.cpp window.cpp resolvePlugin.cpp)

target_link_libraries(simulate mujoco::mujoco glfw3 ${OpenCV_LIBS})