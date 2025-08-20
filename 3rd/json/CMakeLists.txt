set(LIB_NAME "json")

message(STATUS "[${LIB_NAME}] Building....")

file(GLOB_RECURSE CPP_FILES  src/*.c)

add_library(${LIB_NAME} STATIC ${CPP_FILES})

target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS ${LIB_NAME} DESTINATION ${INSTALL_LIB_DIR})



