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
