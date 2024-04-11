#include <simulation_manager/world_manager.hpp>

void saveWorld(std::string world_name, std::string path_data_folder,
               LogGPIS& log_gpis) {

	if (!std::filesystem::exists(path_data_folder + world_name)) {
		std::filesystem::create_directory(path_data_folder + world_name);
	}

	std::string path_K_matrix = path_data_folder + (world_name + "/") + "K.txt";
	std::string path_alpha_matrix = path_data_folder + (world_name + "/") + "alpha.txt";
	std::string path_data_x_matrix = path_data_folder + (world_name + "/") + "data_x.txt";
	std::string path_data_y_matrix = path_data_folder + (world_name + "/") + "data_y.txt";

	saveMatrix(path_K_matrix, log_gpis.getK());
	saveMatrix(path_alpha_matrix, log_gpis.getAlpha());
	saveMatrix(path_data_x_matrix, log_gpis.getDataX());
	saveMatrix(path_data_y_matrix, log_gpis.getDataY());
}

void saveWorld(std::string world_name, std::string path_data_folder,
               std::vector<std::shared_ptr<LogGPIS>> log_gpis) {

	if (!std::filesystem::exists(path_data_folder + world_name)) {
		std::filesystem::create_directory(path_data_folder + world_name);
	}

	for (unsigned int i = 0; i < log_gpis.size(); ++i) {
		std::string path_K_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "K.txt";
		std::string path_alpha_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "alpha.txt";
		std::string path_data_x_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "data_x.txt";
		std::string path_data_y_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "data_y.txt";

		saveMatrix(path_K_matrix, log_gpis[i]->getK());
		saveMatrix(path_alpha_matrix, log_gpis[i]->getAlpha());
		saveMatrix(path_data_x_matrix, log_gpis[i]->getDataX());
		saveMatrix(path_data_y_matrix, log_gpis[i]->getDataY());
	}
}


void loadWorld(std::string world_name, std::string path_data_folder,
               LogGPIS& log_gpis) {

	std::string path_K_matrix = path_data_folder + (world_name + "/") + "K.txt";
	std::string path_alpha_matrix = path_data_folder + (world_name + "/") + "alpha.txt";
	std::string path_data_x_matrix = path_data_folder + (world_name + "/") + "data_x.txt";
	std::string path_data_y_matrix = path_data_folder + (world_name + "/") + "data_y.txt";
	
	Eigen::MatrixXd K;
	Eigen::MatrixXd alpha;
	Eigen::MatrixXd data_x;
	Eigen::MatrixXd data_y;

	loadMatrix(path_K_matrix, K);
	loadMatrix(path_alpha_matrix, alpha);
	loadMatrix(path_data_x_matrix, data_x);
	loadMatrix(path_data_y_matrix, data_y);
	log_gpis.loadData(K, alpha, data_x, data_y);
}

void loadWorld(std::string world_name, std::string path_data_folder, 
               std::vector<std::shared_ptr<LogGPIS>>& log_gpis,
               double gp_lambda_whittle,
               double gp_resolution,
               double gp_error_variance) {

	// Count number of objects
	int count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(path_data_folder + world_name)) {
		if (entry.is_regular_file()) {
			count++;
		}
	}

	// Initialize LogGpis
	for (int i = 0; i < count/4; ++i) {
		log_gpis.push_back(std::make_shared<LogGPIS>(gp_lambda_whittle, gp_resolution, gp_error_variance));
	}

	// Load data
	for (unsigned int i = 0; i < log_gpis.size(); ++i) {
		std::string path_K_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "K.txt";
		std::string path_alpha_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "alpha.txt";
		std::string path_data_x_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "data_x.txt";
		std::string path_data_y_matrix = path_data_folder + (world_name + "/Object_") + (std::to_string(i) +  "_") + "data_y.txt";
		
		Eigen::MatrixXd K;
		Eigen::MatrixXd alpha;
		Eigen::MatrixXd data_x;
		Eigen::MatrixXd data_y;

		loadMatrix(path_K_matrix, K);
		loadMatrix(path_alpha_matrix, alpha);
		loadMatrix(path_data_x_matrix, data_x);
		loadMatrix(path_data_y_matrix, data_y);
		log_gpis[i]->loadData(K, alpha, data_x, data_y);
	}
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
	} else if (world_name == "casy_scenario_1") {
		buildWorld_casy_scenario_1(lgpis);
	} 
	else {
		std::cout << "[ERROR] : Unknown world name";
	}
}

void buildWorld(std::string world_name, 
                std::vector<std::shared_ptr<LogGPIS>>& log_gpis,
                double gp_lambda_whittle,
                double gp_resolution,
                double gp_error_variance) {
	if (world_name == "chamber_1") {
		buildWorld_chamber_1(log_gpis,
                         gp_lambda_whittle,
                         gp_resolution,
                         gp_error_variance);
							
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
				point = cylinder_center_ground + point;
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
				point = cylinder_center_ground + point;
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
				point = cylinder_center_ground + point;
				log_gpis.add_sample(point);
			}
		}

		std::cout <<  "LogGPIS training..."; 
		log_gpis.train();
		std::cout <<  " finished!" << std::endl;
}

void buildWorld_casy_scenario_1(LogGPIS& log_gpis) {
		// Add ground
		// for (double x = -3.5; x < 3.6; x += 0.5) {
			// for (double y = -3.5; y < 3.6; y += 0.5) {
				// Eigen::Vector3d point(x, y, -0.5);
				// log_gpis.add_sample(point);
			// }
		// }

		// Add roof
		for (double x = -3.5; x < 3.6; x += 0.5) {
			for (double y = -3.5; y < 3.6; y += 0.5) {
				Eigen::Vector3d point(x, y, 3.0);
				log_gpis.add_sample(point);
			}
		}

		// Add north wall
		for (double y = -3.5; y < 3.6; y += 0.5) {
			for (double z = -0.5; z < 3.6; z += 0.5) {
				Eigen::Vector3d point(2.5, y, z);
				log_gpis.add_sample(point);
			}
		}

		// Add south wall
		for (double y = -3.5; y < 3.6; y += 0.5) {
			for (double z = -0.5; z < 3.6; z += 0.5) {
				Eigen::Vector3d point(-2.5, y, z);
				log_gpis.add_sample(point);
			}
		}

		// Add east wall
		for (double x = -3.5; x < 3.6; x += 0.5) {
			for (double z = -0.5; z < 3.6; z += 0.5) {
				Eigen::Vector3d point(x, 2.5, z);
				log_gpis.add_sample(point);
			}
		}

		// Add east wall
		for (double x = -3.5; x < 3.6; x += 0.5) {
			for (double z = -0.5; z < 3.6; z += 0.5) {
				Eigen::Vector3d point(x, -2.5, z);
				log_gpis.add_sample(point);
			}
		}

		// Add Sphere 1
		Eigen::Vector3d sphere_center(2.0, -2.0, 0.5);
		double sphere_radius = 0.5;
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

		// Add Sphere 2
		sphere_center = Eigen::Vector3d(2.0, 0.0, 0.5);
		sphere_radius = 0.5;
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
	Eigen::Vector3d cylinder_center_ground(0.0, 0.0, 0.0);
	double cylinder_radius = 0.75;
	for (double z = 0.0; z < 3.1; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis.add_sample(point);
		}
	}

	std::cout <<  "LogGPIS training..."; 
	log_gpis.train();
	std::cout <<  " finished!" << std::endl;
}

void buildWorld_chamber_1(std::vector<std::shared_ptr<LogGPIS>>& log_gpis,
                          double gp_lambda_whittle,
                          double gp_resolution,
                          double gp_error_variance) {
	for (int i = 0; i < 16; ++i) {
		log_gpis.push_back(std::make_shared<LogGPIS>(gp_lambda_whittle, gp_resolution, gp_error_variance));
	}
	double cylinder_radius = 0.7;
	Eigen::Vector3d cylinder_center_ground(0.0, 0.0, 0.0);
	double cylinder_length = 4.0;
	
	// Add Cylinder 1 (TODO REPLACE WITH WALL)
	cylinder_center_ground = Eigen::Vector3d(5.0, 0.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[0]->add_sample(point);
		}
	}
	
	// Add Cylinder 1
	cylinder_center_ground = Eigen::Vector3d(5.0, 0.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[1]->add_sample(point);
		}
	}

	// Add Cylinder 2
	cylinder_center_ground = Eigen::Vector3d(8.0, 3.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[2]->add_sample(point);
		}
	}

	// Add Cylinder 3
	cylinder_center_ground = Eigen::Vector3d(10.0, -3.5, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[3]->add_sample(point);
		}
	}

	// Add Cylinder 4
	cylinder_center_ground = Eigen::Vector3d(11.0, 3.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[4]->add_sample(point);
		}
	}

	// Add Cylinder 5
	cylinder_center_ground = Eigen::Vector3d(13.0, -1.5, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[5]->add_sample(point);
		}
	}

	// Add Cylinder 6
	cylinder_center_ground = Eigen::Vector3d(15.0, 3.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[6]->add_sample(point);
		}
	}

	// Add Cylinder 7
	cylinder_center_ground = Eigen::Vector3d(16.0, 0.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[7]->add_sample(point);
		}
	}

	// Add Cylinder 8
	cylinder_center_ground = Eigen::Vector3d(17.5, -3.5, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[8]->add_sample(point);
		}
	}

	// Add Cylinder 9
	cylinder_center_ground = Eigen::Vector3d(19.0, 4.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[9]->add_sample(point);
		}
	}

	// Add Cylinder 10 
	cylinder_center_ground = Eigen::Vector3d(20.0, 0.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[10]->add_sample(point);
		}
	}

	// Add Cylinder 11 
	cylinder_center_ground = Eigen::Vector3d(22.5, 2.5, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[11]->add_sample(point);
		}
	}

	// Add Cylinder 12 
	cylinder_center_ground = Eigen::Vector3d(22.5, -3.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[12]->add_sample(point);
		}
	}

	// Add Cylinder 13 
	cylinder_center_ground = Eigen::Vector3d(25.0, 4.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[13]->add_sample(point);
		}
	}

	// Add Cylinder 14 
	cylinder_center_ground = Eigen::Vector3d(25.0, 0.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[14]->add_sample(point);
		}
	}

	// Add Cylinder 15 
	cylinder_center_ground = Eigen::Vector3d(25.0, -4.0, 0.0);
	for (double z = 0.0; z < cylinder_length; z += 0.2){
		for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
			Eigen::Vector3d point(cylinder_radius * cos(theta), 
			                      cylinder_radius * sin(theta), 
			                      z);
			point = cylinder_center_ground + point;
			log_gpis[15]->add_sample(point);
		}
	}
	
	for (unsigned int i = 0; i < log_gpis.size(); i++) {
		std::cout <<  "LogGPIS " << i + 1 << " training...";
		log_gpis[i]->train();
		std::cout <<  " finished!" << std::endl;
	}
}

double distanceFromCylinder(Eigen::Vector3d position, Eigen::Vector3d center, double radius, double height) {
	// Auxiliary variables for computations
	Eigen::Vector2d position_xy = Eigen::Vector2d(position.x(), position.y());
	Eigen::Vector2d center_xy = Eigen::Vector2d(center.x(), center.y());
	double xy_distance = (position_xy - center_xy).norm();
	double theta = std::atan2(position_xy.y(), position_xy.x());
	double z_distance = position.z() - center.z();


	// Straight over the cylinder
	if (xy_distance < radius && z_distance > height/2.0) {
		return z_distance - height/2.0;
	}

	// Over the cylinder, but out of infinite projection
	if (xy_distance > radius && z_distance > height/2.0) {
		Eigen::Vector3d boundary_point = Eigen::Vector3d(
		  center.x() + radius * std::cos(theta),
		  center.y() + radius * std::sin(theta),
			center.z() + height/2.0
		);

		return (position - boundary_point).norm();
	}

	// Inside the cylinder 
	if (xy_distance < radius && z_distance < height / 2.0 && z_distance > -height / 2.0) {
		double distance_from_top = height/2.0 - z_distance;
		double distance_from_bottom = -height/2.0 - z_distance;
		double unsigned_distance = std::min(std::abs(distance_from_top), std::abs(distance_from_bottom));
		unsigned_distance = std::min(std::abs(xy_distance), std::abs(unsigned_distance));
		return 0.0;
	}
	
	// Outside the cylinder
	if (xy_distance > radius && z_distance < height / 2.0 && z_distance > -height / 2.0) {
	}

	// Straight below the cylinder
	if (xy_distance < radius && z_distance < -height / 2.0) {
		return z_distance + height / 2.0;
	}

	// Below the cylinder, but outside of infinite projection
	if (xy_distance > radius && z_distance < -height / 2.0) {
		Eigen::Vector3d boundary_point = Eigen::Vector3d(
		  center.x() + radius * std::cos(theta),
		  center.y() + radius * std::sin(theta),
			center.z() - height/2.0
		);

		return (position - boundary_point).norm();
	}

	// Something went wrong
	return 0.0;
}

double distanceFromBox(Eigen::Vector3d point, Eigen::Vector3d center, Eigen::Vector3d sizes){
	// Find projection of the box
	Eigen::Vector3d projection_on_box;
	projection_on_box.x() = std::max(center.x() - sizes.x() / 2.0, std::min(point.x(), center.x() + sizes.x() / 2.0));
	projection_on_box.y() = std::max(center.y() - sizes.y() / 2.0, std::min(point.y(), center.y() + sizes.y() / 2.0));
	projection_on_box.z() = std::max(center.z() - sizes.z() / 2.0, std::min(point.z(), center.z() + sizes.z() / 2.0));

	// Compute distance between given point and projection on the box
	return (point - projection_on_box).norm();
}

double distanceFromSphere(Eigen::Vector3d position, Eigen::Vector3d center, double radius) {
	return std::max((position - center).norm() - radius, 0.0);
}
