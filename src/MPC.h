#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Set lower and upper limits for variables.
#define DED25RAD 0.436332 // 25 deg in rad, used as delta bound
#define MAXTHR 1.0 // Maximal a value
#define BOUND 1.0e19 // Bound value for other variables
#define NUMBER_OF_STEPS 10


// Set weights parameters for the cost function
#define W_CTE 8.4
#define W_EPSI 0.32
#define W_V 0.261
#define W_DELTA 600000
#define W_A 17.1
#define W_DDELTA 0.01
#define W_DA 0.00001

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
