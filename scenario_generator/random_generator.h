//
//  random_generator.h
//  MobileSensingSim
//
//  Created by Yuan on 5/2/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__random_generator__
#define __MobileSensingSim__random_generator__

#include <boost/random/mersenne_twister.hpp>
#include "boost/random/discrete_distribution.hpp"

namespace mobile_sensing_sim {
	class RandomGenerator {
	public:
		static boost::mt19937& get_gen() {
			return rng_;
		}
		
		static void set_seed(int seed) {
			rng_.seed(seed);
		}
	private:
		static boost::mt19937 rng_;
	};
}

#endif /* defined(__MobileSensingSim__random_generator__) */
