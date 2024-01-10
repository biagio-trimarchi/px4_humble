#include <simple_qps/program_interface.hpp>

// CLASS
TestQP::TestQP() {}
TestQP::~TestQP() {}

void TestQP::initialize() {
	H << 2.0, 0.0 , 0.0, 4.0;
	g << 0.0, 0.0;
	A << 0.0, 0.0;
	lbA[0] = -10.0;
	ubA[0] = 10.0;
	lb << -10.0, -10.0;
	ub << 10.0, 10.0;
}

void TestQP::run() {
	nWSR = 10;

	std::cout << "H: " << H(0) << " " << H(1) << " " << H(2) << " " << H(3) << std::endl;
	std::cout << "g: " << g(0) << " " << g(1) << std::endl;
	std::cout << "A: " << A(0) << " " << A(1) << std::endl;
	std::cout << "lbA: " << lbA[0] << std::endl;
	std::cout << "ubA: " << ubA[0] << std::endl;
	std::cout << "lb: " << lb(0) << " " << lb(1) << std::endl;
	std::cout << "ub: " << ub(0) << " " << ub(1) << std::endl;
	std::cout << "nWSR: " << nWSR << std::endl;

	auto start = std::chrono::high_resolution_clock::now();
	auto qp = qpOASES::QProblemB(2);
	qp.init(H.data(), g.data(), lb.data(), ub.data(), nWSR);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	qp.getPrimalSolution(x_star.data());
	std::cout << "Solution: " << x_star(0) << " " << x_star(1) << std::endl;
	std::cout << "Execution time: " << duration.count() << " ms" << std::endl;

	nWSR = 10;

	std::cout << "H: " << H(0) << " " << H(1) << " " << H(2) << " " << H(3) << std::endl;
	std::cout << "g: " << g(0) << " " << g(1) << std::endl;
	std::cout << "A: " << A(0) << " " << A(1) << std::endl;
	std::cout << "lbA: " << lbA[0] << std::endl;
	std::cout << "ubA: " << ubA[0] << std::endl;
	std::cout << "lb: " << lb(0) << " " << lb(1) << std::endl;
	std::cout << "ub: " << ub(0) << " " << ub(1) << std::endl;
	std::cout << "nWSR: " << nWSR << std::endl;

	start = std::chrono::high_resolution_clock::now();
	auto qp_2 = qpOASES::QProblemB(2);
	qp_2.init(H.data(), g.data(), lb.data(), ub.data(), nWSR);
	stop = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	qp_2.getPrimalSolution(x_star.data());
	std::cout << "Solution: " << x_star(0) << " " << x_star(1) << std::endl;
	std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
}

// MAIN
int main(int argc, char** argv) {
	TestQP program;
	program.initialize();
	program.run();
	program.run();
}
