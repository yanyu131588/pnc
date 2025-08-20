
set(LIB_NAME "comm")

file(GLOB_RECURSE CPP_FILES  src/*.cpp)

add_library(${LIB_NAME} STATIC ${CPP_FILES}) #STATIC

target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${LIB_NAME}  utils)

install(TARGETS ${LIB_NAME} DESTINATION ${INSTALL_LIB_DIR})
