//
//  optimal_balance_solver.h
//  PhoneSim
//
//  Created by Yuan on 11/11/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __PhoneSim__optimal_balance_solver__
#define __PhoneSim__optimal_balance_solver__

#include "../solver_base.h"
#include "cplex_adapter.h"
#include "cplex_balance_adapter.h"

namespace mobile_sensing_sim {
  
  class OptimalBalanceSolver : public SolverBase {
  public:
    OptimalBalanceSolver() : use_milp_(false) {}
    Result Solve(const Scenario& scen);
    const GraphConverter& GetGraphConverter() {
      return gc_;
    }
    void SetMILP(bool status) {
      use_milp_ = status;
    }
  private:
    bool use_milp_;
    CplexBalanceAdapter cplex_adapter_;
    GraphConverter gc_;
  };
}

#endif /* defined(__PhoneSim__optimal_balance_solver__) */
