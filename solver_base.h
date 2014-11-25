//
//  solver_base.h
//  PhoneSim
//
//  Created by Yuan on 7/9/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__solver_base__
#define __MobileSensingSim__solver_base__

#include "scenario_generator/scenario_generator.h"
#include "milp_base.h"

namespace mobile_sensing_sim {
  struct Cost {
    enum CostType {
      SENSING = 0,
      COMM,
      UPLOAD
    };
    
    Cost() : costs_(3) {
      std::fill(costs_.begin(), costs_.end(), 0.0);
    }
    double& operator[](CostType ctype) {
      return costs_[static_cast<int>(ctype)];
    }
    double operator[](CostType ctype) const{
      return costs_[static_cast<int>(ctype)];
    }
  private:
    std::vector<double> costs_;
  };
  
  struct Result {
    Result(const int phone_count) : phone_cost(phone_count), is_valid(false), all_cost(0.0), solution_status(-1), is_optimal(false){}
    std::vector<Cost> phone_cost;
    Cost total_cost;
    double all_cost;
    bool is_valid;
    int solution_status;
    bool is_optimal;
    void AddCost(const int phoneid, const double cost, Cost::CostType ctype) {
      total_cost[ctype] += cost;
      assert(phoneid < phone_cost.size());
      phone_cost[phoneid][ctype] += cost;
      all_cost += cost;
    }
    
    double PhoneCost(int phoneid) const {
      Cost c = phone_cost[phoneid];
      return c[Cost::SENSING] + c[Cost::COMM] + c[Cost::UPLOAD];
    }
    
    double MaxPhoneCost() const {
      double max = 0;
      for (int i = 0; i < phone_cost.size(); ++i) {
        double cost = PhoneCost(i);
        if (cost > max) {
          max = cost;
        }
      }
      return max;
    }
  };
  
  class SolverBase : public MilpBase {
  public:
    virtual Result Solve(const Scenario& scen) = 0;
  };
}

#endif /* defined(__MobileSensingSim__solver_base__) */
