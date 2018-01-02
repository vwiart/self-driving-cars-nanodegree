#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 20;
double dt = 0.1;

const double Lf = 2.67;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 50;

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
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
	  fg[0] = 0;
	  for (unsigned int t = 0; t < N; t++) {
	  	fg[0] += 2 * CppAD::pow(vars[cte_start + t], 2);
	  	fg[0] += 1500 * CppAD::pow(vars[epsi_start + t], 2);
	  	fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
	  }
	
	  for (unsigned int t = 0; t < N - 1; t++) {
	  	fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
	  	fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
	  }
	
	  for (unsigned int t = 0; t < N - 2; t++) {
	  	fg[0] += 500 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	  	fg[0] += 500 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	  }
	
	  fg[1 + x_start] = vars[x_start];
	  fg[1 + y_start] = vars[y_start];
	  fg[1 + psi_start] = vars[psi_start];
	  fg[1 + v_start]  = vars[v_start];
	  fg[1 + cte_start]  = vars[cte_start];
	  fg[1 + epsi_start] = vars[epsi_start];
	
	  for (unsigned int t = 1; t < N; t++) {
      AD<double> x0 = vars[x_start + t - 1];
		  AD<double> y0 = vars[y_start + t - 1];
		  AD<double> psi0 = vars[psi_start + t - 1];
		  AD<double> v0 = vars[v_start + t - 1];
		  AD<double> cte0 = vars[cte_start + t - 1];
		  AD<double> epsi0 = vars[epsi_start + t - 1];

		  AD<double> x1 = vars[x_start + t];
		  AD<double> y1 = vars[y_start + t];
		  AD<double> psi1 = vars[psi_start + t];
		  AD<double> v1 = vars[v_start + t];
		  AD<double> cte1 = vars[cte_start + t];
		  AD<double> epsi1 = vars[epsi_start + t];		  
		
		  AD<double> delta0 = vars[delta_start + t - 1];
		  AD<double> a0 = vars[a_start + t - 1];
		
		  AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
		  AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);
		
		  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
		  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
		  fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
	  }
  }
};

typedef CPPAD_TESTVECTOR(double) Dvector;

class Model {
  public:
    void init(Eigen::VectorXd state) {
      this->x = state[0];
      this->y = state[1];
      this->psi = state[2];
      this->v = state[3];
      this->cte = state[4];
      this->epsi = state[5];
    }
    void updt_constraints(Dvector& vec) {
      vec[x_start] = this->x;
      vec[y_start] = this->y;
      vec[psi_start] = this->psi;
      vec[v_start] = this->v;
      vec[cte_start] = this->cte;
      vec[epsi_start] = this->epsi;
    }
  private:
    double x, y, psi, v, cte, epsi;
};

MPC::MPC() {
  this->n_constraints = N * 6;
  this->n_vars = N * 6 + (N - 1) * 2;
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;  

  Model model;
  model.init(state);  
  
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    if (i  < delta_start) {
      vars_lowerbound[i] = -1.0e19;
		  vars_upperbound[i] = 1.0e19;
    } else if (i < a_start) {
      // between [-25;25] in radian
      vars_lowerbound[i] = -0.436332;
		  vars_upperbound[i] = 0.436332;
    } else {
      // between [-1;1]
      vars_lowerbound[i] = -1.0;
		  vars_upperbound[i] = 1.0;
    }
  }
  
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  model.updt_constraints(constraints_lowerbound);
  model.updt_constraints(constraints_upperbound);
  
  FG_eval fg_eval(coeffs);

  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  
  for (unsigned int i = 0; i < N; i++) {
	  result.push_back(solution.x[x_start + i]);
	  result.push_back(solution.x[y_start + i]);
  }
  
  return result;
}
