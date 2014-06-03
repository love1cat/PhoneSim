//
//  simlog.cpp
//  MobileSensingSim
//
//  Created by Yuan on 5/1/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include "simlog.h"

namespace mobile_sensing_sim {
	bool SimLog::IsLog = true;
    std::string SimLog::file_name_ = "invalid.txt";

	SimLog log("./sim_log.txt");
	SimLog hlog("./heuristic_alg_log.txt");
	SimLog olog("./optimal_alg_log.txt");
	SimLog nlog("./naive_alg_log.txt");
}
