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

namespace mobile_sensing_sim {
  struct Solution {
    Solution() : is_valid(false), obj(0.0) {}
    void Clear() {
      edge_values.clear();
      edge_costs.clear();
    }
    bool is_valid;
    double obj;
    int solution_status;
    int edge_count;
    int vertex_count;
    std::vector<double> edge_values;
    std::vector<double> edge_costs;
  };
  
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
