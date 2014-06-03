//
//  scenario_generator.cpp
//  MobileSensingSim
//
//  Created by Yuan on 5/1/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//
#include <boost/random/uniform_int.hpp>
#include "phone.h"
#include "random_generator.h"
#include "scenario_generator.h"
#include "../error_handler.h"
#include "../simlog.h"

namespace mobile_sensing_sim{
	const Scenario ScenarioGenerator::GenerateDefaultScenario() {
		std::vector<Phone> phones;
		std::vector<std::vector<int> > start_phones;
		GeneratePhones(phones, start_phones);
		return GenerateScenario(phones, start_phones);
	}
	
	void ScenarioGenerator::GeneratePhones(std::vector<Phone> &phones, std::vector<std::vector<int> >& start_phones) {
		
		phones.clear();
		start_phones.clear();
		
		// Check scenario parameters
		RandomGenerator::set_seed(sp_.seed);
		if (sp_.speed_range.min < 0) {
			ErrorHandler::CodingError("Speed cannot be less than zero!");
		}
		
		if (sp_.start_time_range.min < 0 || sp_.start_time_range.max > sp_.running_time) {
			ErrorHandler::CodingError("Start time range is not properly set!");
		}
		
		const int kEntrypointCount = sp_.map.area_map_.entry_points_.size();
		boost::random::uniform_int_distribution<> entrypointDist(0, kEntrypointCount - 1);
		boost::random::uniform_int_distribution<> speedDist(sp_.speed_range.min, sp_.speed_range.max);
		boost::random::uniform_int_distribution<> startTimeDist(sp_.start_time_range.min, sp_.start_time_range.max);
		
		boost::random::uniform_int_distribution<> sensing_cost_dist(sp_.sensing_cost_range.min, sp_.sensing_cost_range.max);
		boost::random::uniform_int_distribution<> transfer_cost_dist(sp_.transfer_cost_range.min, sp_.transfer_cost_range.max);
		boost::random::uniform_int_distribution<> upload_cost_dist(sp_.upload_cost_range.min, sp_.upload_cost_range.max);
		boost::random::uniform_int_distribution<> upload_limit_dist(sp_.upload_limit_range.min, sp_.upload_limit_range.max);
		
		// Generate a set of phones.
		
		// Arrange phones according to their starting time.
		assert(sp_.running_time > sp_.start_time_range.max);
		for (int i = 0; i < sp_.running_time; ++i) {
			start_phones.push_back(std::vector<int>());
		}
		
		log << "*********************************************\n";
		log << "Generating phones\n";
		log << "*********************************************\n";
		log << "Generating " << sp_.phone_count << " phones.\n" ;
		for (int i = 0; i < sp_.phone_count; ++i) {
			log << "***Phone " << i << ":***\n" ;
			// Randomly choose speed.
			double speed = speedDist(RandomGenerator::get_gen()) * sp_.speed_range.step;
			log << "speed: " << speed << "\n" ;
			
			// Randomly choose entry point.
			int entry_point_id = entrypointDist(RandomGenerator::get_gen());
			Point entry_point = sp_.map.area_map_.entry_points_[entry_point_id];
			log << "entry point: (" << entry_point.x <<","<< entry_point.y << ")\n" ;
			
			// Determin the initial moving direction based on entry point
			Phone::Directions initial_dir = GetDirection(entry_point);
			
			// Create phone and add to phone list.
			// All phone use same turn probability.
			Phone ph(entry_point, initial_dir, speed, sp_.map, sp_.tp);
			ph.costs_.sensing_cost = sensing_cost_dist(RandomGenerator::get_gen()) * sp_.sensing_cost_range.step;
			ph.costs_.transfer_cost = transfer_cost_dist(RandomGenerator::get_gen()) * sp_.transfer_cost_range.step;
			ph.costs_.upload_cost = upload_cost_dist(RandomGenerator::get_gen()) * sp_.upload_cost_range.step;
			ph.upload_limit_ = upload_limit_dist(RandomGenerator::get_gen()) * sp_.upload_limit_range.step;
			ph.kID_ = i;
			phones.push_back(ph);
			
			// Randomly choose start time.
			int start_time = startTimeDist(RandomGenerator::get_gen());
			log << "start time: " << start_time << "\n" ;
			
			// Save phone id to corresponding start time vector.
			start_phones[start_time].push_back(ph.kID_);
			
			// Disable phone until its start time is reached.
			ph.is_active_ = false;
		}
	}
	
	const Scenario ScenarioGenerator::GenerateScenario(const std::vector<Phone> &original_phones, const std::vector<std::vector<int> >& start_phones, int start_time) {
		log.Reset();

		if (original_phones.empty() || start_phones.empty()) {
			ErrorHandler::CodingError("Phone vector Or Start phone vector is empty!");
		}
		
		Scenario scen;
		scen.phones = original_phones;
		scen.start_phones = start_phones;
		scen.scen_param = sp_;
		
		// Make copy of phones for generating adj matrices.
		std::vector<Phone> phones(original_phones);
		
		// Start create and write scenario.
		log << "\n\n";
		log << "*********************************************\n";
		log << "Starting scenario\n";
		log << "*********************************************\n";
		
		// Save number of phones and targets.
		scen.phone_count  = phones.size();
		scen.target_count = sp_.map.monitor_points_.size();
		scen.running_time = sp_.running_time;
		
		scen.adj_mats.Resize(sp_.running_time, sp_.phone_count, sp_.phone_count + sp_.map.monitor_points_.size());
		scen.adj_mats.Fill(0);
		scen.data_mats.Resize(sp_.running_time, sp_.phone_count, sp_.phone_count + sp_.map.monitor_points_.size());
		scen.data_mats.Fill(0.0);
		
		// Initialize scen.phone_locations
		for (int t = 0; t < sp_.running_time; ++t) {
			std::vector<Point> loc_at_t;
			for (int i = 0; i < sp_.phone_count; ++i) {
				loc_at_t.push_back(Point());
			}
			scen.phone_locations.push_back(loc_at_t);
		}
		
		for (int t = start_time; t < sp_.running_time; ++t) {
			log << "*** Time " << t << "***\n";
			
			// Enable phones if they start at this time.
			for (int i = 0; i < start_phones[t].size(); ++i) {
				int ph_id = start_phones[t][i];
				log << "phone " << ph_id << " is enabled.\n";
				phones[ph_id].is_active_ = true;
			}
			
			// Record phone locations.
			for (int i = 0; i < phones.size(); ++i) {
				scen.phone_locations[t][i] = phones[i].GetLocation();
			}
			
			// Record meetups.
			GenerateAdjacencyMatrix(phones, scen.adj_mats, scen.data_mats, t);
			
			// Move phones if they are active.
			for (int i = 0; i < phones.size(); ++i) {
				if (phones[i].is_active_) {
					log << "phone "<< i << " is ready to move.\n";
					phones[i].Move();
				}
			}
		}
		
		return scen;
	}
	
	void ScenarioGenerator::GenerateAdjacencyMatrix(const std::vector<Phone>& phones, ThreeDimVector<int>& adj_mats, ThreeDimVector<double>& data_mats, const int time) const{
		int row_size = phones.size() + sp_.map.monitor_points_.size();
		
		for (int i = 0; i < phones.size(); ++i) {
			if (!phones[i].is_active_) {
				// Phone i is not acive yet. All 0s for adj mat.
				for (int j = 0; j < row_size; ++j) {
					adj_mats(time, i, j) = 0;
				}
				continue;
			}
			
			// Check phon-phone meetup.
			long long comm_range_square = sp_.comm_range * sp_.comm_range;
			for (int j = 0; j < phones.size(); ++j) {
				if (phones[j].is_active_ && i != j && Point::DistanceSquare(phones[i].GetLocation(), phones[j].GetLocation()) <= comm_range_square) {
					adj_mats(time, i, j) = 1;
					data_mats(time, i, j) = sp_.data_per_second;
				} else {
					adj_mats(time, i, j) = 0;
					data_mats(time, i, j) = 0.0;
				}
			}
			
			// Check phone-target meetup.
			long long sensing_range_square = sp_.sensing_range * sp_.sensing_range;
			const int kPhoneSize = phones.size();
			for (int j = 0; j < sp_.map.monitor_points_.size(); ++j) {
				if (Point::DistanceSquare(phones[i].GetLocation(), sp_.map.monitor_points_[j]) <= sensing_range_square) {
					adj_mats(time, i, kPhoneSize + j) = 1;
					data_mats(time, i, kPhoneSize + j) = 1; // Assume whenever phone pass target, it gets all the data.
				} else {
					adj_mats(time, i, kPhoneSize + j) = 0;
					data_mats(time, i, kPhoneSize + j) = 0;
				}
			}
		}
	}
	
	void ScenarioGenerator::WriteScenarioFile(const Scenario& scen, const std::string& outfile) const {
		if (scen.adj_mats.Empty()) {
			ErrorHandler::CodingError("Please generate scenario before writing to file!");
			return;
		}
		using std::endl;
		std::ofstream of(outfile.c_str());
		of << scen.phone_count << endl;
		of << scen.target_count << endl;
		const ThreeDimVector<int> &ams = scen.adj_mats;
		for (int t = 0; t < ams.DimOneSize(); ++t) {
			of << t << endl;
			for (int i = 0; i < ams.DimTwoSize(); ++i) {
				for (int j = 0; j < ams.DimThreeSize(); ++j) {
					of << ams(t, i, j) << ' ';
				}
				of << endl;
			}
		}
	}
	
	Phone::Directions ScenarioGenerator::GetDirection(const Point& entry_point) const{
		if (entry_point.x == 0) {
			return Phone::RIGHT;
		} else if (entry_point.x == sp_.map.area_map_.length_) {
			return Phone::LEFT;
		} else if (entry_point.y == 0) {
			return Phone::UP;
		} else {
			return Phone::DOWN;
		}
	}
}
