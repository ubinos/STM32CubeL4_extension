if(INCLUDE__STM32CUBEL4_EXTENSION)

    get_filename_component(_tmp_source_dir "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
    file(GLOB_RECURSE _tmp_sources
        "${_tmp_source_dir}/*.c"
        "${_tmp_source_dir}/*.cpp"
        "${_tmp_source_dir}/*.S"
        "${_tmp_source_dir}/*.s")
    set(PROJECT_SOURCES ${PROJECT_SOURCES} ${_tmp_sources})

endif(INCLUDE__STM32CUBEL4_EXTENSION)

