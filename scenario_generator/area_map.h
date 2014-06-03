//
//  area_map.h
//  MobileSensingSim
//
//  Created by Yuan on 4/15/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__area_map__
#define __MobileSensingSim__area_map__

#include <utility>
#include <vector>

namespace mobile_sensing_sim {
	struct Point {
		double x;
		double y;
		Point(double newx, double newy) : x(newx), y(newy) {}
		Point() : x(-1), y(-1) {}
		static double DistanceSquare(const Point& p1, const Point& p2) {
			return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
		}
	};
	
	class AreaMap {
	public:
		AreaMap(){}
		AreaMap(const std::vector<Point>& entry_points, const std::vector<Point>& intersect_points, double length, double width)
		: entry_points_(entry_points), intersect_points_(intersect_points), length_(length), width_(width) {}
		
		bool IsOutOfBound(const Point& pt) const{
			return pt.x < 0 || pt.x > length_ || pt.y < 0 || pt.y > width_;
		}
		
		double length_;
		double width_;
		std::vector<Point> entry_points_;
		std::vector<Point> intersect_points_;
	};
}

#endif /* defined(__MobileSensingSim__area_map__) */
