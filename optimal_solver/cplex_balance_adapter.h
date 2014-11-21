//
//  cplex_balance_adapter.h
//  PhoneSim
//
//  Created by Yuan on 11/11/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __PhoneSim__cplex_balance_adapter__
#define __PhoneSim__cplex_balance_adapter__

#include <vector>
#include <ilcplex/cplex.h>
#include "cplex_adapter_base.h"
#include "graph_converter.h"

namespace mobile_sensing_sim {
  struct BalanceOption{
    bool sensing;
    bool communication;
    bool upload;
    
    BalanceOption() :
    sensing(true),
    communication(true),
    upload(true) {}
  };
  
  class CplexBalanceAdapter : public CplexAdapterBase {
  public:
    CplexBalanceAdapter() : lp_(NULL), use_milp_(false){}
    void SetMILP(bool status) {
      use_milp_ = status;
    }
    bool Solve(const Graph &g, const Scenario& scen, const BalanceOption &bo, Solution &s);
  private:
    int AddBalanceConstraints(const Graph &g, const Scenario &scen, const BalanceOption &bo);
    void Reset();
    
    CPXLPptr  lp_;
    bool use_milp_;
  };
}

#endif /* defined(__PhoneSim__cplex_balance_adapter__) */
