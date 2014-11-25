//
//  stat.h
//  MobileSensingSim
//
//  Created by Yuan on 6/7/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef MobileSensingSim_stat_h
#define MobileSensingSim_stat_h

#include <vector>
#include <numeric>

namespace mobile_sensing_sim {
  class Statistics {
  public:
    void AddValue(double val) { v_.push_back(val); }
    void Clear() { v_.clear(); }
    double Mean() const {
      if (v_.empty()) {
        return 0.0;
      }
      double sum = std::accumulate(v_.begin(), v_.end(), 0.0);
      return sum / v_.size();
    }
    
    double Max() const {
      std::vector<double>::const_iterator cit = std::max_element(v_.begin(), v_.end());
      if (cit != v_.end()) {
        return *cit;
      }
      
      return 0.0;
    }
    
    double Min() const {
      std::vector<double>::const_iterator cit = std::min_element(v_.begin(), v_.end());
      if (cit != v_.end()) {
        return *cit;
      }
      
      return 0.0;
    }
    
    double Variance() const {
      double sq_sum = std::inner_product(v_.begin(), v_.end(), v_.begin(), 0.0);
      double mean = Mean();
      return sq_sum / v_.size() - mean * mean;
    }
  private:
    std::vector<double> v_;
  };
}

#endif
