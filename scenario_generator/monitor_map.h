//
//  monitor_map.h
//  MobileSensingSim
//
//  Created by Yuan on 5/1/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__monitor_map__
#define __MobileSensingSim__monitor_map__

#include <iostream>
#include "area_map.h"
#include <boost/shared_ptr.hpp>

namespace mobile_sensing_sim {
	class MonitorMap;
	typedef boost::shared_ptr<MonitorMap> MonitorMapPtr;
	
	class MonitorMap {
	public:
		MonitorMap() {}
		MonitorMap(const std::vector<Point>& monitor_points, const AreaMap& area_map)
		: monitor_points_(monitor_points), area_map_(area_map) {}
		
		std::vector<Point> monitor_points_;
		AreaMap area_map_;
	};
}

#endif /* defined(__MobileSensingSim__monitor_map__) */
