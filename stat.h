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
		void AddValue(double value) {
			data_.push_back(value);
		}
		
		double GetMean() {
			if (data_.empty()) {
				return 0.0;
			}
			double sum = std::accumulate(data_.begin(), data_.end(), 0.0);
			return sum / data_.size();
		}
	private:
		std::vector<double> data_;
	};
	
	struct AlgorithmStatistics {
		Statistics totalcost_stat;
		Statistics sensingcost_stat;
		Statistics commcost_stat;
		Statistics uploadcost_stat;
	};
}

#endif
