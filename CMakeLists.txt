cmake_minimum_required(VERSION 3.10)

project(Control LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_STANDARD_REQUIRED YES)

add_subdirectory(src/lib/BMI160)
add_subdirectory(src/lib/BMP280)
add_subdirectory(src/lib/BMP580)
