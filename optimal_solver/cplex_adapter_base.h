//
//  cplex_adapter_base.h
//  PhoneSim
//
//  Created by Yuan on 11/4/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __PhoneSim__cplex_adapter_base__
#define __PhoneSim__cplex_adapter_base__

#include <vector>
#include <ilcplex/cplex.h>
#include "graph_converter.h"
#include "../solution.h"

namespace mobile_sensing_sim {
  class CplexAdapterBase {
  public:
    CplexAdapterBase() : env_(NULL), net_(NULL) {}
    virtual bool Solve(const Graph &g, Solution &s) { return true; }
  protected:
    virtual bool CreateNetworkProblem(const Graph &g);
    int BuildNetwork(const Graph& g);
    virtual void Reset();
    
    int status_;
    
    CPXENVptr env_;
    CPXNETptr net_;
  };
}

#endif /* defined(__PhoneSim__cplex_adapter_base__) */
