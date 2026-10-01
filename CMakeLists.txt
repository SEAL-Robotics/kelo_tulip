cmake_minimum_required(VERSION 3.5.1)
project(kelo_tulip)

if(POLICY CMP0148) 
	cmake_policy(SET CMP0148 OLD) 
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS thread)

if(WIN32)
	set(INCLUDE_DIRS
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/win32
		include/kelo_tulip/soem/oshw/win32
		${WINPCAP_INCLUDE_DIRS}
	)
else(WIN32)
	set(INCLUDE_DIRS
		include
		include/kelo_tulip/soem
		include/kelo_tulip/soem/osal
		include/kelo_tulip/soem/osal/linux
		include/kelo_tulip/soem/oshw/linux
	)
endif(WIN32)

set(msg_files
	"msg/KeloDriveInput.msg"
	"msg/KeloDrivesInput.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	DEPENDENCIES std_msgs
)

include_directories(
	${INCLUDE_DIRS}
)

# Optionally apply cap_sys_nice,cap_net_raw+ep to the installed binary and add
# an ld.so.conf entry, both with sudo. Off by default: a plain build must never
# run sudo or write to /etc. Enable with -DUSE_SETCAP=ON, or set the
# capabilities by hand after installing (see README).
OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" OFF)
IF("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND sudo)				# default = sudo
ELSE("$ENV{SUDO_COMMAND}" STREQUAL "")
	SET(SUDO_COMMAND $ENV{SUDO_COMMAND})
ENDIF("$ENV{SUDO_COMMAND}" STREQUAL "")

add_subdirectory(src/soem)

add_library(tulip_velocity_controller
	src/Utils.cpp
	src/VelocityPlatformController.cpp
)

ament_target_dependencies(tulip_velocity_controller
	rclcpp
	geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
)

add_library(tulip_library
	src/EtherCATMaster.cpp
	src/EtherCATModule.cpp
	src/EtherCATModuleROS.cpp
	src/PlatformDriver.cpp
	src/PlatformDriverROS.cpp
	src/ParameterValidation.cpp
	src/EscCounters.cpp
	src/DumpPolicy.cpp
	src/EthercatBlackBox.cpp
	src/EscDiagnostics.cpp
	src/RtLog.cpp
	src/EscDiagnosticsRos.cpp
	src/modules/RobileMasterBattery.cpp
	src/modules/RobileMasterBatteryROS.cpp
)

ament_target_dependencies(tulip_library
	rclcpp
	geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	diagnostic_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
	Boost
)

target_link_libraries(tulip_library
	soem
	tulip_velocity_controller
)

if ("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "humble")
	rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
	target_link_libraries(tulip_library "${cpp_typesupport_target}")
else ("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "humble")
	rosidl_target_interfaces(tulip_library ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif ("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "humble")

add_executable(platform_driver
	src/TulipMain.cpp
)

target_link_libraries(platform_driver   
	${Boost_LIBRARIES}
	tulip_library
	pthread
)

# IF(USE_SETCAP)
# 	add_custom_command(
# 		TARGET platform_driver 
# 		POST_BUILD
# 		COMMAND ${CMAKE_COMMAND} -E echo "Set capabilities of platform_driver"
# 		COMMAND ${SUDO_COMMAND} setcap cap_sys_nice,cap_net_raw+ep $<TARGET_FILE:platform_driver>
# 	)
# endif(USE_SETCAP)

install(TARGETS
	tulip_library
	tulip_velocity_controller
	soem
	EXPORT export_${PROJECT_NAME}
	DESTINATION lib
)

install(TARGETS
	platform_driver
	DESTINATION lib/${PROJECT_NAME}
)

IF(USE_SETCAP)
set (t_cmd setcap)
set (t_arg cap_sys_nice,cap_net_raw+ep)
set (t_file ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/platform_driver)
set (t_ld_path /etc/ld.so.conf.d)
set (t_ld_name ld_kelo_tulip.conf)
install( CODE 
    "
message(NOTICE \"POST INSTALL\")
execute_process(COMMAND getcap ${t_file} RESULT_VARIABLE t_res OUTPUT_VARIABLE t_cap)
#message(NOTICE \"getcap1 => \${t_cap}\")
if(t_cap STREQUAL \"\")
	message(NOTICE \"sudo ${t_cmd} ${t_arg} ${t_file}\")
	execute_process(COMMAND sudo ${t_cmd} ${t_arg} ${t_file} )
	execute_process(COMMAND getcap ${t_file} RESULT_VARIABLE t_res OUTPUT_VARIABLE t_cap)
	#message(NOTICE \"getcap2 => \${t_cap}\")
else()	
	message(NOTICE \"${t_file} already has capabilities set, no action.\")
endif()
if(EXISTS ${t_ld_path}/${t_ld_name})
	message(NOTICE \"${t_ld_path}/${t_ld_name} already exists, no action.\")
else()
	file(WRITE ${t_ld_name} \"/opt/ros/\$ENV{ROS_DISTRO}/lib\n\${CMAKE_INSTALL_PREFIX}/lib\n\")
	execute_process(COMMAND sudo cp ${t_ld_name} ${t_ld_path} )
	message(NOTICE \"${t_ld_name} generated and copied to ${t_ld_path}\")
	execute_process(COMMAND sudo ldconfig )
	message(NOTICE \"sudo ldconfig excecuted.\")
endif()
    "
)
# TODO : find solution when sudo requires password (request is not shown)
# To mitigate this, cache sudo credentials before colcon build (f.e. sudo ls)
ENDIF(USE_SETCAP)

install(DIRECTORY 
	include/
	DESTINATION include
)

install(DIRECTORY
	config
	launch
	DESTINATION share/${PROJECT_NAME}/
)

#install(
	#TARGETS tulip_library tulip_velocity_controller
	#EXPORT export_${PROJECT_NAME}
	#LIBRARY DESTINATION lib
	#ARCHIVE DESTINATION lib
	#RUNTIME DESTINATION bin
#)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_dependencies(Boost)
ament_export_dependencies(std_msgs)
ament_export_dependencies(diagnostic_msgs)
ament_export_libraries(tulip_library tulip_velocity_controller soem)

if(BUILD_TESTING)
	find_package(ament_cmake_gtest REQUIRED)
	ament_add_gtest(test_command_watchdog test/test_command_watchdog.cpp)
	ament_add_gtest(test_platform_driver_ros_watchdog test/test_platform_driver_ros_watchdog.cpp)
	ament_add_gtest(test_driver_liveness test/test_driver_liveness.cpp)
	ament_add_gtest(test_wheel_recovery test/test_wheel_recovery.cpp)
	ament_add_gtest(test_odometry_freshness test/test_odometry_freshness.cpp)
	ament_add_gtest(test_wheel_recovery_machine test/test_wheel_recovery_machine.cpp)
	ament_add_gtest(test_esc_counters test/test_esc_counters.cpp)
	target_link_libraries(test_esc_counters tulip_library)
	ament_add_gtest(test_dump_policy test/test_dump_policy.cpp)
	target_link_libraries(test_dump_policy tulip_library)
	ament_add_gtest(test_seqlock_ring test/test_seqlock_ring.cpp)
	target_link_libraries(test_seqlock_ring pthread)
	ament_add_gtest(test_ethercat_blackbox test/test_ethercat_blackbox.cpp)
	target_link_libraries(test_ethercat_blackbox tulip_library)
	ament_add_gtest(test_platform_driver_recovery test/test_platform_driver_recovery.cpp)
	target_link_libraries(test_platform_driver_recovery tulip_library)
	ament_add_gtest(test_soem_close test/test_soem_close.cpp)
	target_link_libraries(test_soem_close tulip_library)
	ament_add_gtest(test_rt_log test/test_rt_log.cpp)
	target_link_libraries(test_rt_log tulip_library)
	ament_add_gtest(test_esc_diagnostics_ros test/test_esc_diagnostics_ros.cpp)
	target_link_libraries(test_esc_diagnostics_ros tulip_library)
	ament_add_gtest(test_current_shaping test/test_current_shaping.cpp)
	ament_add_gtest(test_velocity_platform_controller test/test_velocity_platform_controller.cpp)
	target_link_libraries(test_velocity_platform_controller tulip_velocity_controller)
	target_link_libraries(test_platform_driver_ros_watchdog tulip_library)
	ament_add_gtest(test_ramp_timing test/test_ramp_timing.cpp)
	ament_add_gtest(test_stop_sequence test/test_stop_sequence.cpp)
	ament_add_gtest(test_velocity_command test/test_velocity_command.cpp)
	target_link_libraries(test_velocity_command pthread)
	ament_add_gtest(test_parameter_validation test/test_parameter_validation.cpp)
	target_link_libraries(test_parameter_validation tulip_library)
	ament_add_gtest(test_platform_driver_stop test/test_platform_driver_stop.cpp)
	target_link_libraries(test_platform_driver_stop tulip_library)

	# Tests never join a live robot's DDS network: a shell on a robot, or a
	# laptop on its LAN, may carry that robot's domain and peer list. Every
	# test registered in this directory tree gets its own domain and
	# localhost-only discovery, whatever the caller's environment says.
	function(kelo_tulip_isolate_ros_tests dir)
		get_property(_tests DIRECTORY "${dir}" PROPERTY TESTS)
		if(_tests)
			set_tests_properties(${_tests} DIRECTORY "${dir}" PROPERTIES ENVIRONMENT_MODIFICATION
				"ROS_DOMAIN_ID=set:98;ROS_AUTOMATIC_DISCOVERY_RANGE=set:LOCALHOST;CYCLONEDDS_URI=unset:;DDS_LOCAL_IP=unset:;ROS_STATIC_PEERS=unset:;ROS_LOCALHOST_ONLY=unset:")
		endif()
		get_property(_subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
		foreach(_sub IN LISTS _subdirs)
			kelo_tulip_isolate_ros_tests("${_sub}")
		endforeach()
	endfunction()
	kelo_tulip_isolate_ros_tests("${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()
