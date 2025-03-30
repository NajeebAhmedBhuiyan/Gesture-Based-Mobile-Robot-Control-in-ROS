
cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

# Enable additional warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the required ament_cmake package
find_package(ament_cmake REQUIRED)

# Install the necessary directories for runtime
install(
    DIRECTORY launch model parameters scripts
    DESTINATION share/${PROJECT_NAME}
)


# Configure testing tools if testing is enabled
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# Declare this project as an ament package
ament_package()
