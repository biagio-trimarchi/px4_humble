#include <simulation_manager/world_manager.hpp>

void saveWorld(std::string world_name, std::string path_data_folder, LogGPIS& log_gpis) {
	if (!std::filesystem::exists(path_data_folder + world_name)) {
		std::filesystem::create_directory(path_data_folder + world_name);
	}

	std::string path_K_matrix = path_data_folder + (world_name + "/") + "K.txt";
	std::string path_data_x_matrix = path_data_folder + (world_name + "/") + "data_x.txt";
	std::string path_data_y_matrix = path_data_folder + (world_name + "/") + "data_y.txt";

	saveMatrix(path_K_matrix, log_gpis.getK());
	saveMatrix(path_data_x_matrix, log_gpis.getDataX());
	saveMatrix(path_data_y_matrix, log_gpis.getDataY());
}

void loadWorld(std::string world_name, std::string path_data_folder, LogGPIS& log_gpis) {
	std::string path_K_matrix = path_data_folder + (world_name + "/") + "K.txt";
	std::string path_data_x_matrix = path_data_folder + (world_name + "/") + "data_x.txt";
	std::string path_data_y_matrix = path_data_folder + (world_name + "/") + "data_y.txt";
	
	Eigen::MatrixXd K;
	Eigen::MatrixXd data_x;
	Eigen::MatrixXd data_y;

	loadMatrix(path_K_matrix, K);
	loadMatrix(path_data_x_matrix, data_x);
	loadMatrix(path_data_y_matrix, data_y);
	log_gpis.loadData(K, data_x, data_y);
}

void saveMatrix(std::string filename, const Eigen::MatrixXd& matrix) {
	int rows_number = matrix.rows();
	int columns_number = matrix.cols();

	std::ofstream outputFile(filename);
	if (!outputFile.is_open()) {
		std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
		return;
	}

	// Write each entry of the matrix, rowwise
	outputFile << rows_number;
	outputFile << ", " << columns_number;

	for (int row = 0; row < rows_number; row++) {
		for (int column = 0; column < columns_number; column ++) {
			outputFile << ", " << matrix(row, column);
		}
	}
	outputFile << std::endl;
	
	// Close file
	outputFile.close();
}

void loadMatrix(std::string filename, Eigen::MatrixXd& matrix) {
	int rows_number;
	int columns_number;

	std::ifstream inputFile(filename);
	if (!inputFile.is_open()){
		std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
		return;
	}

	// Get number of rows and columns
	inputFile >> rows_number;     // Rows number
	inputFile.ignore();           // Ignore comma
	inputFile >> columns_number;  // Columns number
	
	// Allocate matrix
	if (rows_number > 0 && columns_number > 0) {
		matrix = Eigen::MatrixXd(rows_number, columns_number);
	}

	// Read matrix row wise
	for (int row = 0; row < rows_number; row++) {
		for (int column = 0; column < columns_number; column ++) {
			inputFile.ignore();                // Ignore comma
			inputFile >> matrix(row, column);  // Store value
		}
	}

	inputFile.close();	
}

void buildWorld(std::string world_name, LogGPIS& lgpis) {
	if (world_name == "lgpis_test_1") {
		buildWorld_lgpis_test_1(lgpis);	
	} else {
		std::cout << "[ERROR] : Unknown world name";
	}
}

void buildWorld_lgpis_test_1(LogGPIS& log_gpis) {
		// Add ground
		for (double x = -5.0; x < 5.1; x += 0.5) {
			for (double y = -5.0; y < 5.1; y += 0.5) {
				Eigen::Vector3d point(x, y, -0.5);
				log_gpis.add_sample(point);
			}
		}

		// Add Box 1
		for (double z = 0.0; z < 1.1; z += 0.2) {
			for (double s = 0.0; s < 2.0*4.0 + 2.0*2.0 + 0.1; s+=0.2) {
				if (s < 4.0) {
					Eigen::Vector3d point(-1.0 - s, -3.0, z);
					log_gpis.add_sample(point);
				} else if (s < 6.0) {
					Eigen::Vector3d point(-5.0, -3.0 - (s - 4.0), z);
					log_gpis.add_sample(point);
				} else if (s < 10.0) {
					Eigen::Vector3d point(-5.0 + (s - 6.0), -5.0, z);
					log_gpis.add_sample(point);
				} else {
					Eigen::Vector3d point(-3.0, -5.0 + (s - 10.0), z);
					log_gpis.add_sample(point);
				}
			}
		}
		for (double x = -5.0; x < -1.1; x += 0.5) {
			for (double y = -5.0; y < -3.1; y += 0.5) {
				Eigen::Vector3d point(x, y, 1.0);
				log_gpis.add_sample(point);
			}
		}

		// Add Sphere 1
		Eigen::Vector3d sphere_center(3.0, -3.0, 1.0);
		double sphere_radius = 1.0;
		for (double psi = 0.0; psi < M_PI+0.1; psi += 0.2){
			for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
				Eigen::Vector3d point(sin(psi) * cos(theta),
				                      sin(psi) * sin(theta),
				                      cos(psi)
				                     );
				point = sphere_center + sphere_radius * point;
				log_gpis.add_sample(point);
			}
		}
		
		// Add Cylinder 1
		Eigen::Vector3d cylinder_center_ground(4.0, 4.0, 0.0);
		double cylinder_radius = 1.0;
		for (double z = 0.0; z < 3.1; z += 0.2){
			for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
				Eigen::Vector3d point(cylinder_radius * cos(theta), 
				                      cylinder_radius * sin(theta), 
				                      z);
				point = cylinder_center_ground + cylinder_radius * point;
				log_gpis.add_sample(point);
			}
		}

		// Add Cylinder 2
		cylinder_center_ground = Eigen::Vector3d(-3.0, 2.0, 0.0);
		cylinder_radius = 1.0;
		for (double z = 0.0; z < 3.1; z += 0.2){
			for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
				Eigen::Vector3d point(cylinder_radius * cos(theta), 
				                      cylinder_radius * sin(theta), 
				                      z);
				point = cylinder_center_ground + cylinder_radius * point;
				log_gpis.add_sample(point);
			}
		}
 
		// Add Cylinder 3
		cylinder_center_ground = Eigen::Vector3d(-3.0, -2.0, 0.0);
		cylinder_radius = 0.5;
		for (double z = 0.0; z < 3.1; z += 0.2){
			for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
				Eigen::Vector3d point(cylinder_radius * cos(theta), 
				                      cylinder_radius * sin(theta), 
				                      z);
				point = cylinder_center_ground + cylinder_radius * point;
				log_gpis.add_sample(point);
			}
		}

		std::cout <<  "LogGPIS training..." << std::endl;
		log_gpis.train();
		std::cout <<  "LogGPIS training finished!" << std::endl;
}
