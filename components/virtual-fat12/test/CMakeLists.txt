add_executable(vfat-test test_vfat.cpp)
target_link_libraries(vfat-test PRIVATE virtual-fat12::virtual-fat12)
add_test(NAME vfat COMMAND vfat-test)
