//
//  phone.h
//  MobileSensingSim
//
//  Created by Yuan on 4/22/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__phone__
#define __MobileSensingSim__phone__

#include "monitor_map.h"
#include "../error_handler.h"
#include <boost/random/mersenne_twister.hpp>
#include "boost/random/discrete_distribution.hpp"

namespace mobile_sensing_sim {
	struct TurnProbability {
		double left, right, straight, precision;
		TurnProbability() {
			// Change turn probability here
			left = 0.25;
			right = 0.25;
			straight = 0.5;
			
			if (left + right + straight != 1.0) {
				ErrorHandler::CodingError("Turn probability is not correctly set!");
			}
		}
	};
	
	struct Costs {
		Costs() : sensing_cost(1.0), transfer_cost(1.0), upload_cost(1.0) {}
		double sensing_cost;
		double transfer_cost;
		double upload_cost;
	};
	
	class Phone {
	public:
		enum Directions {
			UP = 0, // Do not change.
			RIGHT,
			DOWN,
			LEFT,
		};
		
		Phone(const Point& initial_location, Directions moving_direction, double speed, const MonitorMap& map, const TurnProbability& tp);
		
		void Move();
		void MoveTo(const Point& location) {
			location_ = location; // no error checking
			if (monitor_map_ptr_->area_map_.IsOutOfBound(location_)) {
				is_active_ = false;
			}
		}
		Point GetLocation() const {
			return location_;
		}
		
		void SetTurnProbability (const TurnProbability& tp);
		
		Directions moving_direction_;
		double speed_;
		bool is_active_;
		int kID_;
		
		Costs costs_;
		double upload_limit_; // how many data units it can upload to celluar tower.
		
		const MonitorMap *monitor_map_ptr_;
	private:
		std::string GetDirectionName(Directions dir);
		void MoveForward(double distance);
		void IntersectAction(double distance_to_intersection, double total_distance);
		void TurnLeft();
		void TurnRight();
		
		boost::random::discrete_distribution<> dist;
		Point location_;
	};
}

#endif /* defined(__MobileSensingSim__phone__) */
