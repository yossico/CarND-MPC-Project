#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

using CppAD::AD;

// TODO: Set the timestep length and duration - set as default (10,0.1) to predisct 1 second into the future
const size_t N = NUMBER_OF_STEPS; // (20 timestamps)
const double dt = 0.1; //0.1 second between timestamps

// This value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// LF was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double LF = 2.67;
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 50;//was 100 in walkthough try to be a bit safer

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC. `fg` is a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and the Solver function below.
	  //reference state cost function as show in class
	  fg[0] = 0;
	  for (int i = 0; i < N; i++) {
		  fg[0] += W_CTE * CppAD::pow(vars[cte_start + i] - REF_CTE, 2);     // weighting the CTE
		  fg[0] += W_EPSI * CppAD::pow(vars[epsi_start + i] - REF_EPSI, 2);  // weighting the angle
		  fg[0] += W_V * CppAD::pow(vars[v_start + i] - REF_V, 2);           //weighting the speed difference
	  }

	  // Minimize the use of actuators.
	  for (int i = 0; i < N - 1; i++) {
		  fg[0] += W_DELTA * CppAD::pow(vars[delta_start + i], 2);    
		  fg[0] += W_A * CppAD::pow(vars[a_start + i], 2);
	  }

	  // Minimize the value gap between sequential actuations.
	  for (int i = 0; i < N - 2; i++) {
		  fg[0] += W_DDELTA * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2); //weighting steering differences
		  fg[0] += W_DA * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);  //weighting acceleration diffrences
	  }

	  fg[1 + x_start] = vars[x_start];
	  fg[1 + y_start] = vars[y_start];
	  fg[1 + psi_start] = vars[psi_start];
	  fg[1 + v_start] = vars[v_start];
	  fg[1 + cte_start] = vars[cte_start];
	  fg[1 + epsi_start] = vars[epsi_start];
	  

	  for (int i = 0; i < N - 1; i++) {
		  //use only CppAD vars according to solution of section 9 of MPC lesson	
		  // The state at time t+1 .
		  AD<double> x1 = vars[x_start + i + 1];
		  AD<double> y1 = vars[y_start + i + 1];
		  AD<double> psi1 = vars[psi_start + i + 1];
		  AD<double> v1 = vars[v_start + i + 1];
		  AD<double> cte1 = vars[cte_start + i + 1];
		  AD<double> epsi1 = vars[epsi_start + i + 1];

		  // The state at time t.
		  AD<double> x0 = vars[x_start + i];
		  AD<double> y0 = vars[y_start + i];
		  AD<double> psi0 = vars[psi_start + i];
		  AD<double> v0 = vars[v_start + i];
		  AD<double> cte0 = vars[cte_start + i];
		  AD<double> epsi0 = vars[epsi_start + i];

		  // actuators at time t.
		  AD<double> delta0 = vars[delta_start + i];
		  AD<double> a0 = vars[a_start + i];

		  
		  AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
		  AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

		  fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
		  fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
		  fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / LF * DT);
		  fg[2 + v_start + i] = v1 - (v0 + a0 * DT);
		  fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * DT));
		  fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0/LF * DT);
	  }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;

  typedef CPPAD_TESTVECTOR(double) Dvector;
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 6 element vector, the actuators is a 2 element vector and there are N timesteps,
  // The number of variables is: 6 * N + 2 * (N-1)  since we dont use actuators values for t=Ndt
  size_t n_vars = N * 6 + (N - 1) * 2; // 
  size_t n_constraints = N * 6; // Set the number of constraints
  std::cout << "state \n " << state <<"coeffs \n" << coeffs << std::endl;

  // Initial value of the independent variables. SHOULD BE 0 besides initial state.
  Dvector vars(n_vars); //init the vector which hold the entire state and prediction (state + actuators)Xtimesteps to 0
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // TODO: Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  for (int i = 0; i < delta_start; i++) {
	  vars_lowerbound[i] = -BOUND;
	  vars_upperbound[i] = BOUND;
  }
  for (int i = delta_start; i < a_start; i++) {
	  vars_lowerbound[i] = -DED25RAD;
	  vars_upperbound[i] = DED25RAD;
  }
  // Acceleration/decceleration upper and lower limits
  for (int i = a_start; i < n_vars; i++) {
	  vars_lowerbound[i] = -MAXTHR;
	  vars_upperbound[i] = MAXTHR;
  }
  // Lower and upper limits for the constraints Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
	  constraints_lowerbound[i] = 0;
	  constraints_upperbound[i] = 0;
  }


  //set the values to initial state value using constraints 
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage of sparse routines, this makes the computation MUCH FASTER. If you can 
  // uncomment 1 of these and see if it makes a difference or not but if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true forward\n";
  options += "Sparse  true reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
	  options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  this->mpc_x = {};
  this->mpc_y = {};
  for (int i = 0; i < N; i++) {
	  this->mpc_x.push_back(solution.x[x_start + i]);
	  this->mpc_y.push_back(solution.x[y_start + i]);
  }
  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  // TODO: Return the first actuator values. The variables can be accessed with `solution.x[i]`.
  return result;
  
}
