#pragma once
// LIBRARIES
// C++ Standard Libraries
#include <iostream>
#include <fstream>
#include <string>

// Third Pary Libraries
#include <Eigen/Eigen>

// Custom Libraries
#include <log_gpis/logGPIS.hpp>
#include <filesystem>

void saveWorld(std::string world_name, std::string path_data_folder, LogGPIS& log_gpis);
void loadWorld(std::string world_name, std::string path_data_folder, LogGPIS& log_gpis);

void saveMatrix(std::string filename, const Eigen::MatrixXd& matrix);
void loadMatrix(std::string filename, Eigen::MatrixXd& matrix);

void buildWorld(std::string world_name, LogGPIS& log_gpis);
void buildWorld_lgpis_test_1(LogGPIS& log_gpis);
void buildWorld_casy_scenario_1(LogGPIS& log_gpis);
