<<<<<<< HEAD
# Install script for directory: /home/merce/ros2_ws/src/spacenav-test
=======
# Install script for directory: /home/merce/Documents/GitHub/spacenav-test
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/spacenav-test/spacenav-test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/spacenav-test/spacenav-test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/spacenav-test/spacenav-test"
         RPATH "")
  endif()
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/spacenav-test" TYPE EXECUTABLE FILES "/home/merce/ros2_ws/src/spacenav-test/build/spacenav-test")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/spacenav-test" TYPE EXECUTABLE FILES "/home/merce/Documents/GitHub/spacenav-test/build/spacenav-test")
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/spacenav-test/spacenav-test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/spacenav-test/spacenav-test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/spacenav-test/spacenav-test"
<<<<<<< HEAD
         OLD_RPATH "/opt/ros/humble/lib:/home/merce/repos/yarp-devices-ros2/ros2_interfaces_ws/install/yarp_control_msgs/lib:/usr/local/lib:"
=======
         OLD_RPATH "/opt/ros/humble/lib:/usr/local/lib:"
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/spacenav-test/spacenav-test")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  include("/home/merce/ros2_ws/src/spacenav-test/build/CMakeFiles/spacenav-test.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/spacenav-test")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/spacenav-test")
=======
  include("/home/merce/Documents/GitHub/spacenav-test/build/CMakeFiles/spacenav-test.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/spacenav-test")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/spacenav-test")
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test/environment" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_environment_hooks/ament_prefix_path.dsv")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test/environment" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_environment_hooks/ament_prefix_path.dsv")
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test/environment" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_index/share/ament_index/resource_index/packages/spacenav-test")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test/environment" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_index/share/ament_index/resource_index/packages/spacenav-test")
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_core/spacenav-testConfig.cmake"
    "/home/merce/ros2_ws/src/spacenav-test/build/ament_cmake_core/spacenav-testConfig-version.cmake"
=======
    "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_core/spacenav-testConfig.cmake"
    "/home/merce/Documents/GitHub/spacenav-test/build/ament_cmake_core/spacenav-testConfig-version.cmake"
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/ros2_ws/src/spacenav-test/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spacenav-test" TYPE FILE FILES "/home/merce/Documents/GitHub/spacenav-test/package.xml")
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/merce/ros2_ws/src/spacenav-test/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/merce/Documents/GitHub/spacenav-test/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> a595ea93ac26c0d6da7f29cba2bae734478470e9
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
