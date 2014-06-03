//
//  scenario_generator.h
//  MobileSensingSim
//
//  Created by Yuan on 5/1/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__scenario_generator__
#define __MobileSensingSim__scenario_generator__

#include <fstream>
#include "phone.h"
#include "monitor_map.h"
#include "../error_handler.h"
#include "multidim_vector.h"

namespace mobile_sensing_sim {
	struct Range {
		int min;
		int max;
		double step;
		
		Range() : min(0.0), max(0.0), step(0.0){}
		
		Range(int minval, int maxval, double stepval = 1.0)
		: min(minval), max(maxval), step(stepval){
			if (min > max) {
				ErrorHandler::CodingError("Min cannot be greater than Max for Range!");
			}
		}
	};
	
	struct ScenarioParameters {
		int phone_count;
		int running_time;
		int comm_range;
		int sensing_range;
		Range speed_range;
		Range start_time_range;
		int seed;
		TurnProbability tp;
		MonitorMap map;
		double data_per_second;
		Range sensing_cost_range;
		Range transfer_cost_range;
		Range upload_cost_range;
		Range upload_limit_range;
	};
	
	struct Scenario {
		ScenarioParameters scen_param;
		int phone_count;
		int target_count;
		int running_time;
		std::vector<Phone> phones;
		std::vector<std::vector<int> > start_phones;
		std::vector<std::vector<Point> > phone_locations;
		ThreeDimVector<int> adj_mats;
		// AdjacencyMatrix over t:
		//   Rows: phone count, Cols: phone count + target count
		//   Data on (i, j)  i and j are in comm / sensing range.
		//   (target if j >= phone count)
		ThreeDimVector<double> data_mats;
		// DataCapacityMatrix over t:
		//   Rows: phone count, Cols: phone count
		//   Data on (i, j) Percentage of data unit can be transferred
		//   between i and j. (target if j >= phone count)
	};
	
	class ScenarioGenerator {
	public:
		ScenarioGenerator (const ScenarioParameters& sp) : sp_(sp) {}
		void WriteScenarioFile(const Scenario& scen, const std::string& outfile) const;
		const Scenario GenerateScenario(const std::vector<Phone> &phones, const std::vector<std::vector<int> >& start_phones, int start_time = 0);
		void GeneratePhones(std::vector<Phone>& original_phones, std::vector<std::vector<int> >& start_phones);
		const Scenario GenerateDefaultScenario();
	private:
		Phone::Directions GetDirection(const Point& entry_point) const;
		void GenerateAdjacencyMatrix(const std::vector<Phone>& phones, ThreeDimVector<int>& adj_mats, ThreeDimVector<double>& data_mats, int time) const;
		ScenarioParameters sp_;
	};
}

#endif /* defined(__MobileSensingSim__scenario_generator__) */
