cmake_minimum_required(VERSION 2.8)

file(GLOB bv32_files
     "src/*.c"
)

target_sources(app PRIVATE
        ${bv32_files}
)

target_include_directories(app PRIVATE "include")

