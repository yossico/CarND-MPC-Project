#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define DT 0.1 // time step duration dt in s 

using namespace std;

#define NUMBER_OF_STEPS 20
#define LF 2.67
#define REF_CTE 0
#define REF_EPSI 0
#define REF_V  30 

// Set weights parameters for the cost function
#define W_CTE 30
#define W_EPSI 10
#define W_V  20
#define W_DELTA 100000
#define W_A 20
#define W_DDELTA 0.01
#define W_DA 10


// Set lower and upper limits for variables.
#define DED25RAD 0.436332 // 25 deg in rad, used as delta bound
#define MAXTHR 1.0 // Maximal a value
#define BOUND 1.0e3 // Bound value for other variables

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> mpc_x;
  vector<double> mpc_y;
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
