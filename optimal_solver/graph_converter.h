//
//  graph_converter.h
//  MobileSensingSim
//
//  Created by Yuan on 5/7/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__graph_converter__
#define __MobileSensingSim__graph_converter__

#include <vector>
#include <string>
#include "../error_handler.h"
#include "../scenario_generator/scenario_generator.h"

namespace mobile_sensing_sim {
	struct Edge {
		enum EdgeType {
			SRC_TO_TARGET = 0,
			PHONE_TO_SINK,
			PHONE_TO_PHONE,
			TARGET_TO_PHONE,
			PHONE_TO_SELF
		};
		EdgeType type;
		int head;
		int tail;
		double cost;
		double capacity_lower_bound;
		double capacity_upper_bound;
		std::string name;
		int time;
		int phone1_id;
		int phone2_id;
		int target_id;
	};
	
	struct Graph {
		Graph() : vertex_count(0), edge_count(0){}
		int vertex_count;
		int edge_count;
		std::vector<double> vertex_supply; // Size = Vertex count
		std::vector<int> edge_tails; // Size = edge count
		std::vector<int> edge_heads; // Size = edge count
		std::vector<double> edge_costs; // Size = edge count
		std::vector<double> edge_capacity_lower_bounds; // Size = edge count
		std::vector<double> edge_capacity_uppper_bounds; // Size = edge count
		static double kInfinity; // Infinity value
		int source_id;
		int sink_id;
		void Clear() {
			vertex_count = 0;
			edge_count = 0;
			vertex_supply.clear();
			edge_tails.clear();
			edge_heads.clear();
			edge_costs.clear();
			edge_capacity_lower_bounds.clear();
			edge_capacity_uppper_bounds.clear();
			edges.clear();
		}
		std::vector<Edge> edges;
	};
	
	class GraphConverter {
	public:
		void ConvertToGraph(const Scenario& scen);
		std::string GetVertexName(int vertex_id) const;
		const Edge GetEdge(int edge_id) const {
			if (edges_.empty()) {
				ErrorHandler::RunningError("Failed to get edge. Edge vector is empty!");
			}
			return edges_[edge_id];
		}
		const Graph& GetGraph() const{
			return g_;
		}
		void AddEdge(const Edge &e);
		const std::vector<Edge>& GetEdges() {
			return edges_;
		}
		
		void PrintInformation();
		void Clear();
	private:
		int GetVertexID(int phone_count, int time, int index);
		Graph g_;
		std::vector<Edge> edges_;
		std::string ConstructPhoneName(int time, int index);
	};
}

#endif /* defined(__MobileSensingSim__graph_converter__) */
