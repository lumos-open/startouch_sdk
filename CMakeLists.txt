cmake_minimum_required(VERSION 3.16)
project(user_app VERSION 1.0.0)


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED CONFIG)
find_package(Threads REQUIRED)


# if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#     message(STATUS "Target architecture is ARM")
#     set(DEPEND_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5_src/libarx_r5_src-arm64.so")
# else ()
#     set(DEPEND_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5_src/libarx_r5_src.so")
# endif ()

# target_include_directories(kinematic_solver PUBLIC lib/)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPEND_LIB} pybind11::module)
# target_link_libraries(kinematic_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libkinematic_solver.so pybind11::module)

# # 设置安装路径
# set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/api/" CACHE PATH "Installation Directory" FORCE)

# install(DIRECTORY lib/ DESTINATION ./)

# install(TARGETS ${PROJECT_NAME}
#         LIBRARY DESTINATION ./${PROJECT_NAME}/
#         ARCHIVE DESTINATION ./${PROJECT_NAME}/)
# install(TARGETS kinematic_solver
#         LIBRARY DESTINATION ./${PROJECT_NAME}/
#         ARCHIVE DESTINATION ./${PROJECT_NAME}/)


# Select the runtime library to link against.
#
# ARM builds are not ABI-stable across Ubuntu releases on this project, so keep
# versioned ARM runtime snapshots alongside the generic fallback and pick the
# best match for the current host OS.
set(OS_VERSION "")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    execute_process(
        COMMAND lsb_release -rs
        OUTPUT_VARIABLE OS_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    file(STRINGS "/etc/os-release" OS_RELEASE_CONTENT)
    foreach(LINE ${OS_RELEASE_CONTENT})
        if(LINE MATCHES "VERSION_ID=")
            string(REGEX REPLACE "VERSION_ID=\"?([0-9.]*)\"?" "\\1" OS_VERSION ${LINE})
        endif()
    endforeach()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    if(OS_VERSION VERSION_GREATER_EQUAL "24.04" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm24")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm24")
    elseif(OS_VERSION VERSION_GREATER_EQUAL "22.04" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm22")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm22")
    elseif(OS_VERSION VERSION_GREATER_EQUAL "20.04" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm20")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm20")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm64")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.arm64")
    else()
        message(FATAL_ERROR "No ARM runtime library found under src/ for host OS ${OS_VERSION}")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(OS_VERSION VERSION_GREATER_EQUAL "24.04")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.24")
    elseif(OS_VERSION VERSION_GREATER_EQUAL "22.04")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.22")
    elseif(OS_VERSION VERSION_GREATER_EQUAL "20.04")
        set(DEPEND_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so.20")
    else()
        message(FATAL_ERROR "Unsupported OS version: ${OS_VERSION}. Supported versions: 20.04, 22.04, 24.04 and above")
    endif()
endif()

set(DEPEND_LIB_DEST "${CMAKE_CURRENT_SOURCE_DIR}/src/libstartouch.so")

if(NOT DEPEND_LIB_SRC STREQUAL DEPEND_LIB_DEST)
    configure_file(
        ${DEPEND_LIB_SRC}
        ${DEPEND_LIB_DEST}
        COPYONLY
    )
endif()

# 创建用户的可执行文件
# add_executable(user_app main.cpp)
# target_include_directories(user_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
#                     ${EIGEN3_INCLUDE_DIR}
#                                         )
# target_link_libraries(user_app PUBLIC ${DEPEND_LIB_DEST} ${PINOCCHIO_LIBRARIES} )

#[[
###生成.so文件  pybind.cpp 这个是带有皮诺曹和初始值的###
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/src)
pybind11_add_module(startouch pybind.cpp)
# add_executable(controller_pybind examples/controller_pybind.cpp)
target_include_directories(startouch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
                    ${PINOCCHIO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                                        )
target_link_libraries(startouch PRIVATE ${DEPEND_LIB_DEST} ${PINOCCHIO_LIBRARIES} )
]]


###生成.so文件
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/interface_py)
# pybind11_add_module(startouch pybind_wopin.cpp)  #这里是最后把参数辨识参数融进去的
pybind11_add_module(startouch ${CMAKE_CURRENT_SOURCE_DIR}/src/pybind_0825testfast.cpp) #这个是需要额外的辨识参数

target_include_directories(startouch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(startouch PRIVATE ${DEPEND_LIB_DEST} Eigen3::Eigen Threads::Threads)
set_target_properties(startouch PROPERTIES
    BUILD_RPATH "$ORIGIN;$ORIGIN/../src"
    INSTALL_RPATH "$ORIGIN"
)

install(TARGETS startouch LIBRARY DESTINATION .)
install(FILES
    ${DEPEND_LIB_DEST}
    ${CMAKE_CURRENT_SOURCE_DIR}/interface_py/startouchclass.py
    ${CMAKE_CURRENT_SOURCE_DIR}/interface_py/fastumi_startouch_client.py
    ${CMAKE_CURRENT_SOURCE_DIR}/interface_py/pi0_rollout_clinet.py
    DESTINATION .
)
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/interface_py/openpi_client
    DESTINATION .
    PATTERN "__pycache__" EXCLUDE
)
