/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Walter Nowak
 * Sebastian Blumenthal
 * Dharmin Bakaraniya
 * Nico Huebel
 * Arthur Ketels
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "kelo_tulip/DriverLiveness.h"
#include "kelo_tulip/EscDiagnosticsRos.h"
#include "kelo_tulip/EtherCATMaster.h"
#include "kelo_tulip/PlatformDriverROS.h"
#include "kelo_tulip/modules/RobileMasterBatteryROS.h"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cstdlib>

// create and configure one module
kelo::EtherCATModuleROS* createModule(rclcpp::Node::SharedPtr nh, std::string moduleType, std::string moduleName, std::string configTag) {
	kelo::EtherCATModuleROS* module = NULL;	
	if (moduleType == "robile_master_battery") {
		module = new kelo::RobileMasterBatteryROS();
	} else if (moduleType == "platform_driver") {
		module = new kelo::PlatformDriverROS();
	} else {
		std::cout << "Unknown module type: " << moduleType << std::endl;
		return NULL;
	}

	if (!module) {
		std::cout << "Module " << moduleName << " could not be created." << std::endl;
		return NULL;
	}

	// Initialize module and if not possible, delete it.
	if (!module->init(nh, configTag)) {
		std::cout << "Failed to initialize module " << moduleName << "." << std::endl;
		delete module;
		return NULL;
	}
	
	return module;
}

// Where the black box dumps go when the launch names no directory. With no home
// there is none: a shared, guessable directory is worse than no dumps.
std::string defaultDumpDir() {
	const char* rosHome = std::getenv("ROS_HOME");
	const char* home = std::getenv("HOME");
	if (rosHome && *rosHome)
		return std::string(rosHome) + "/kelo_tulip/ethercat_dumps";
	if (home && *home)
		return std::string(home) + "/.ros/kelo_tulip/ethercat_dumps";
	return "";
}

kelo::BlackBoxConfig readBlackBoxConfig(rclcpp::Node::SharedPtr nh) {
	kelo::BlackBoxConfig config;
	config.dir = nh->get_parameter("blackbox.dir").as_string();
	if (config.dir.empty())
		config.dir = defaultDumpDir();
	config.historyS = nh->get_parameter("blackbox.history_s").as_double();
	config.postTriggerS = nh->get_parameter("blackbox.post_trigger_s").as_double();
	config.minIntervalS = nh->get_parameter("blackbox.min_interval_s").as_double();
	config.maxFiles = static_cast<std::size_t>(std::max<int64_t>(0, nh->get_parameter("blackbox.max_files").as_int()));
	config.maxTotalBytes =
		static_cast<std::uint64_t>(std::max<int64_t>(0, nh->get_parameter("blackbox.max_total_mb").as_int())) *
		1024ull * 1024ull;
	return kelo::sanitizeBlackBoxConfig(config);
}

/*
// step through all modules
void stepModules(const rclcpp::TimerEvent&) {
		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
}
*/

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto nh = rclcpp::Node::make_shared("platform_driver");

	nh->declare_parameter("modules.list", std::vector<std::string>{}); 
	nh->declare_parameter("start_retry_delay", 0);
	nh->declare_parameter("robile_master_battery_ethercat_number", 0);
	nh->declare_parameter("device", "");
	nh->declare_parameter("enable_ethercat_recovery", false);
	// Last seconds of the 1 kHz loop, dumped to CSV on a communication error.
	nh->declare_parameter("blackbox.enabled", true);
	nh->declare_parameter("blackbox.dir", "");
	nh->declare_parameter("blackbox.history_s", 2.0);
	nh->declare_parameter("blackbox.post_trigger_s", 0.5);
	nh->declare_parameter("blackbox.min_interval_s", 10.0);
	nh->declare_parameter("blackbox.max_files", 100);
	nh->declare_parameter("blackbox.max_total_mb", 256);
	// ESC error counters per slave; 0 turns the read off.
	nh->declare_parameter("ethercat_diagnostics_period_s", 5.0);

	std::vector<kelo::EtherCATModuleROS*> rosModules;

	// create modules by iterating through struct in config
	std::string configModulesTag = "modules";
	std::vector<std::string> moduleList = nh->get_parameter(configModulesTag + ".list").as_string_array();
	for (unsigned int i = 0; i < moduleList.size(); i++) {
		nh->declare_parameter(configModulesTag + "." + moduleList[i] + ".type", "");
		std::string moduleName = moduleList[i];
		std::string moduleType = nh->get_parameter(configModulesTag + "." + moduleList[i] + ".type").as_string();
		std::string configTag = configModulesTag + "." + moduleName + ".";
		kelo::EtherCATModuleROS* module = createModule(nh, moduleType, moduleName, configTag);
		
		if (!module)
			return -1;
			
		rosModules.push_back(module);
	}

	// legacy config mode for master battery
	int robileMasterBatteryEthercatNumber = nh->get_parameter("robile_master_battery_ethercat_number").as_int();
	if (robileMasterBatteryEthercatNumber > 0) {	
		kelo::EtherCATModuleROS* module = new kelo::RobileMasterBatteryROS();
		if (!module || !module->init(nh, ""))
			return -1;
			
		rosModules.push_back(module);		
	}

	// collect EtherCAT modules
	std::vector<kelo::EtherCATModule*> etherCATmodules;
	for (size_t i = 0; i < rosModules.size(); i++)
		etherCATmodules.push_back(rosModules[i]->getEtherCATModule());

	// create and configure EtherCAT master
	std::string device = nh->get_parameter("device").as_string();
	int delayRetry = nh->get_parameter("start_retry_delay").as_int();
	bool enableEthercatRecovery = nh->get_parameter("enable_ethercat_recovery").as_bool();

	kelo::EtherCATMaster* master = new kelo::EtherCATMaster(device, etherCATmodules);
	if (!master) {
		std::cout << "Failed to create EtherCAT master." << std::endl;
		return -1;		
	}

	if (nh->get_parameter("blackbox.enabled").as_bool()) {
		const kelo::BlackBoxConfig blackBoxConfig = readBlackBoxConfig(nh);
		if (blackBoxConfig.dir.empty()) {
			RCLCPP_ERROR(nh->get_logger(), "No blackbox.dir, ROS_HOME or HOME: EtherCAT black box is off");
		} else {
			master->enableBlackBox(blackBoxConfig);
			RCLCPP_INFO(nh->get_logger(), "EtherCAT black box dumps to %s", blackBoxConfig.dir.c_str());
		}
	}
	master->enableEscDiagnostics(nh->get_parameter("ethercat_diagnostics_period_s").as_double());
	kelo::EscDiagnosticsPublisher escPublisher(nh, [master] { return master->escSnapshot(); });

	// initialize EtherCAT
	while (!master->initEthercat()) {
		if (delayRetry == 0) {
			RCLCPP_ERROR(nh->get_logger(), "Failed to initialize EtherCAT");
			return -1;
		}
		RCLCPP_ERROR(nh->get_logger(), "Failed to initialize EtherCAT, will retry in %d s.", delayRetry);
		std::this_thread::sleep_for(std::chrono::seconds(delayRetry));
	}
	
	// ROS main loop
	rclcpp::Rate rate(20.0f); // hz
	int exitStatus = 0;
	while (rclcpp::ok()) {
		const bool reinitAttempted = enableEthercatRecovery && master->needsReinit();
		const bool reinitSucceeded = reinitAttempted && master->reinitializeEthercat();
		const kelo::LoopVerdict verdict =
			kelo::checkEthercatLiveness(reinitAttempted, reinitSucceeded, master->hasStopped());
		if (verdict != kelo::LoopVerdict::Continue) {
			RCLCPP_FATAL(nh->get_logger(), "%s; exiting so the stack is restarted.",
				verdict == kelo::LoopVerdict::ReinitFailed ?
				"EtherCAT reinitialization failed" : "EtherCAT communication has stopped");
			exitStatus = kelo::exitCode(verdict);
			break;
		}

		rclcpp::spin_some(nh);		
		escPublisher.publishIfNew();
		
		for (size_t i = 0; i < rosModules.size(); i++)
			rosModules[i]->step();
		
		rate.sleep();
	}

	// The master's loop steps the modules, so it stops (and flushes the black
	// box) before they go.
	delete master;

	for (size_t i = 0; i < rosModules.size(); i++)
		delete rosModules[i];
	
	rclcpp::shutdown();
	return exitStatus;
}

