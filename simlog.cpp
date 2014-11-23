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
  SimLog hlog("./heur_log.txt");
  SimLog hdlog("./heur_dyn_log.txt");
  SimLog ahlog("./aggheur_log.txt");
  SimLog olog("./opt_log.txt");
  SimLog nlog("./naive_log.txt");
  SimLog oblog("./opt_balance_log.txt");
}
