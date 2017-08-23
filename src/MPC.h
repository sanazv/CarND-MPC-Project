#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct ResultVectors {
    vector<double> delta;
    vector<double> a;
    vector<double> x;
    vector<double> y;
};

class MPC {
 public:
  MPC();
    
    double prev_delta;
    double prev_a;
    

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  ResultVectors Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    
    int getLatencyIndex();
};

#endif /* MPC_H */
