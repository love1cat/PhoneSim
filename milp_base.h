//
//  milp_base.h
//  PhoneSim
//
//  Created by Yuan on 11/21/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef PhoneSim_milp_base_h
#define PhoneSim_milp_base_h

namespace mobile_sensing_sim {
  // Hack class used to simplified MILP switch in many classes.
  // Although some classes do not need this switch...
  class MilpBase {
  public:
    MilpBase() : use_milp_(false){}
    void SetMILP(bool status) {
      use_milp_ = status;
    }
    bool UseMILP() {
      return use_milp_;
    }
  private:
    bool use_milp_;
  };
}

#endif
