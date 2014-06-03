//
//  phone.cpp
//  MobileSensingSim
//
//  Created by Yuan on 4/22/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include "phone.h"
#include "random_generator.h"
#include "../simlog.h"

namespace mobile_sensing_sim {
	Phone::Phone(const Point& initial_location, Directions moving_direction, double speed, const MonitorMap& map, const TurnProbability& tp) : location_(initial_location), moving_direction_(moving_direction), speed_(speed), monitor_map_ptr_(&map), is_active_(false), kID_(-1), upload_limit_(0.5) {
		if (monitor_map_ptr_->area_map_.IsOutOfBound(initial_location)) {
			ErrorHandler::CodingError("Phone is generated out of the region!");
		}
		
		SetTurnProbability(tp);
	}
	
	void Phone::SetTurnProbability (const TurnProbability& tp) {
		std::vector<double> probability;
		probability.push_back(tp.left);
		probability.push_back(tp.right);
		probability.push_back(tp.straight);
		
		boost::random::discrete_distribution<int, double>::param_type pt(probability);
		dist.param(pt);
	}
	
	void Phone::Move() {
		if (!is_active_) {
			return;
		}
		
		log <<"---------------------------------------\n";
		log << "Phone " << kID_ << " starts moving.\n";
		
		double total_distance = speed_;
		log << "current location: (" << location_.x << "," << location_.y << ")\n";
		log << "total distance: " << speed_ << "\n";
		
		// The phone may need to turn.
		// Check if the phone passes a intersection point.
		
		// Assume it moves forward and get virtual destination.
		Point cur_location = location_;
		MoveForward(total_distance);
		Point dest_location = location_;
		location_ = cur_location;
		
		// Check through all intersection points.
		bool pass_intersection = false;
		for (int i = 0; i < monitor_map_ptr_->area_map_.intersect_points_.size(); ++i) {
			const Point &pt = monitor_map_ptr_->area_map_.intersect_points_[i];
			if (location_.x == pt.x) {
				if ((location_.y < pt.y && dest_location.y >= pt.y) || (location_.y > pt.y && dest_location.y <= pt.y)) {
					log << "The movement will pass intersection point via Y axis.\n";
					IntersectAction(std::abs(pt.y - location_.y), total_distance);
					pass_intersection = true;
					break;
				}
			} else if (location_.y == pt.y) {
				if ((location_.x < pt.x && dest_location.x >= pt.x) || (location_.x > pt.x && dest_location.x <= pt.x)) {
					log << "The movement will pass intersection point via X axis.\n";
					IntersectAction(std::abs(pt.x - location_.x), total_distance);
					pass_intersection = true;
					break;
				}
			}
		}
		
		// If the phone has not passed a intersection, simply move forward.
		if (!pass_intersection) {
			MoveForward(total_distance);
		}
		
		// Check if it is out of region
		if (monitor_map_ptr_->area_map_.IsOutOfBound(location_)) {
			is_active_ = false;
		}
		log <<"---------------------------------------\n\n";
	}
	
	void Phone::MoveForward(double distance) {
		// Compute destination point following current moving direction.
		// The destination may be out of region.
		log << "Move forward (towards " << GetDirectionName(moving_direction_) << ".)" << distance << ".\n";
		if (moving_direction_ == LEFT) {
			location_.x -= distance;
		} else if (moving_direction_ == RIGHT) {
			location_.x += distance;
		} else if (moving_direction_ == UP) {
			location_.y += distance;
		} else {
			location_.y -= distance;
		}
	}
	
	void Phone::IntersectAction(double distance_to_intersection, double total_distance) {
		
		// Compute turning direction using given probability.
		int turn_decision = dist(RandomGenerator::get_gen());
		if (turn_decision == 2) {
			// Move straight.
			log << "Turn decision: go straight.\n";
			MoveForward(total_distance);
		}else {
			log << "Turn decision: turn left or right. move to intersection first.\n";
			MoveForward(distance_to_intersection);
			if (turn_decision == 0) {
				log << "Turn left.\n";
				TurnLeft();
			} else {
				log << "Turn right.\n";
				TurnRight();
			}
			MoveForward(total_distance - distance_to_intersection);
		}
	}
	
	void Phone::TurnLeft() {
		moving_direction_ = (Directions)((moving_direction_ + 4 - 1) % 4);
	}
	
	void Phone::TurnRight() {
		moving_direction_ = (Directions)((moving_direction_ + 1) % 4);
	}
	
	std::string Phone::GetDirectionName(Phone::Directions dir) {
		static std::string dirnames[] = {"UP", "RIGHT", "DOWN", "LEFT"};
		return dirnames[(int)dir];
	}
}
