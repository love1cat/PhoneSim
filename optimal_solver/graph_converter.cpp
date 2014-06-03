//
//  graph_converter.cpp
//  MobileSensingSim
//
//  Created by Yuan on 5/7/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include "graph_converter.h"
#include <boost/lexical_cast.hpp>

namespace mobile_sensing_sim {
	double Graph::kInfinity = 1.0E+20;
	void GraphConverter::ConvertToGraph(const Scenario& scen) {
		Clear();
		Graph &g = g_;
		g.Clear();
		// Suppose there are n phones, m targets running for T times.
		// Vertices count: n * T + m + 2 (2 is for source and sink)
		// n * T nodes are arranged in the sequence of time.
		g.vertex_count = scen.phone_count * scen.running_time + scen.target_count + 2;
		
		// Save frequently used IDs.
		const int kSinkID = g.vertex_count - 1;
		const int kSourceID = g.vertex_count - 2;
		g_.source_id = kSourceID;
		g_.sink_id = kSinkID;
		std::vector<int> target_ids;
		for (int i = 0; i < scen.target_count; ++i) {
			target_ids.push_back(scen.phone_count * scen.running_time + i);
		}
		
		///////////////////////////////////////////////
		// Vertex supply
		///////////////////////////////////////////////
		
		// Only source and sink have supplies = target count.
		g.vertex_supply = std::vector<double>(g.vertex_count, 0.0);
		g.vertex_supply[kSourceID] = scen.target_count;
		g.vertex_supply[kSinkID] = -scen.target_count;
		
		///////////////////////////////////////////////
		// Edges
		///////////////////////////////////////////////
		
		// Types of edges
		// 1. Source to all targets.
		//	  capacity = [0,1], cost = 0
		// 2. All phones at time T to Sink.
		//    cost = upload cost.
		// 3. Edges corresponding to adjacency matrix.
		//    a. phone to phone.
		//	     capacity = [0, data percentage per second], cost = transfer cost
		//       sum of two phones.
		//		 Note: cost vary on different phones
		//    b. target to phone.
		//	     capacity = [0, 1], cost = sensing cost.
		//       Note: cost vary on different phones
		// 4. Edges connecting phones at consecutive times
		//    capacity = [0, +inf], cost = 0
		//    Note: the capcaty can be set to capacity of
		//          phone's storage space
		
		g.edge_count = 0;
		// Type 1
		for (int i = 0; i < scen.target_count; ++i) {
			Edge e;
			e.tail = kSourceID;
			e.head = target_ids[i];
			e.cost = 0.0;
			e.capacity_lower_bound = 0.0;
			e.capacity_upper_bound = 1.0;
			e.name = "Source to Target " + boost::lexical_cast<std::string>(i);
			e.type = Edge::SRC_TO_TARGET;
			e.time = -1; // no time associated
			e.phone1_id = e.phone2_id = -1;
			e.target_id = -1;
			AddEdge(e);
		}
		
		// Type 2
		for (int i = 0; i < scen.phone_count; ++i) {
			Edge e;
			e.tail = GetVertexID(scen.phone_count, scen.running_time - 1, i); // running time is 0-indexed
			e.head = kSinkID;
			e.cost = scen.phones[i].costs_.upload_cost;
			e.capacity_lower_bound = 0.0;
			e.capacity_upper_bound = scen.phones[i].upload_limit_;
			e.name = ConstructPhoneName(scen.running_time - 1, i) + " to Sink";
			e.type = Edge::PHONE_TO_SINK;
			e.time = scen.running_time - 1;
			e.phone1_id = e.phone2_id = i;
			e.target_id = -1;
			AddEdge(e);
		}
		
		// Type 3
		const ThreeDimVector<int> &am = scen.adj_mats;
		assert(scen.running_time == am.DimOneSize());
		for (int t = 0; t < scen.running_time; ++t) {
			for (int i = 0; i < am.DimTwoSize(); ++i) {
				// Only add outgoing edges from i
				// Incoming edges will be added in other
				// adjacency matrices.
				Edge e;
				e.tail = GetVertexID(scen.phone_count, t, i);
				e.capacity_lower_bound = 0.0;
				e.time = t; // time associated
				for (int j = 0; j < am.DimThreeSize(); ++j) {
					if (i!=j && am(t, i, j) != 0) {
						if (j < scen.phone_count) {
							// Type 3a
							e.head = GetVertexID(scen.phone_count, t, j);
							e.cost = scen.phones[i].costs_.transfer_cost + scen.phones[j].costs_.transfer_cost;
							const ThreeDimVector<double> &data_mats = scen.data_mats;
							e.capacity_upper_bound = data_mats(t, i, j); // Example: 0.5 means: One data unit takes 1/0.5 = 2 sec to transmit
							e.name = ConstructPhoneName(t, i) + " to " + ConstructPhoneName(t, j);
							e.type = Edge::PHONE_TO_PHONE;
							e.phone1_id = i;
							e.phone2_id = j;
							e.target_id= -1;
							AddEdge(e);
						} else {
							// Type 3b
							// Only add edge if this is a new target to
							// phone.
							if (t != 0 && am(t-1, i, j) == 1.0) {
								continue;
							}
							int tid = j - scen.phone_count;
							e.tail = target_ids[tid];
							e.head = GetVertexID(scen.phone_count, t, i);
							e.cost = scen.phones[i].costs_.sensing_cost;
							//e.cost = t;// The more time passed, the larger cost.
							// e.cost = 0.0;
							const ThreeDimVector<double> &data_mat = scen.data_mats;
							e.capacity_upper_bound = data_mat(t, i, j);
							e.name = "Target " + boost::lexical_cast<std::string>(tid) + " to " + ConstructPhoneName(t, i);
							e.type = Edge::TARGET_TO_PHONE;
							e.phone1_id = e.phone2_id = i;
							e.target_id = j;
							AddEdge(e);
						}
					}
				} // for j
			} // for i
		} // for t
		
		// Type 4
		for (int i = 0; i < scen.phone_count; ++i) {
			for (int t = 0; t < scen.running_time - 1; ++t) {
				Edge e;
				e.tail = GetVertexID(scen.phone_count, t, i);
				e.head = GetVertexID(scen.phone_count, t + 1, i);
				e.cost = 0.0;
				e.capacity_lower_bound = 0.0;
				e.capacity_upper_bound = Graph::kInfinity;
				e.name = ConstructPhoneName(t, i) + " to " + ConstructPhoneName(t + 1, i);
				e.type = Edge::PHONE_TO_SELF;
				e.time = t;
				e.phone1_id = e.phone2_id = i;
				e.target_id = -1;
				AddEdge(e);
			}
		}
		g.edges = edges_;
	}
	
	void GraphConverter::AddEdge(const Edge &e) {
		g_.edge_heads.push_back(e.head);
		g_.edge_tails.push_back(e.tail);
		g_.edge_costs.push_back(e.cost);
		g_.edge_capacity_lower_bounds.push_back(e.capacity_lower_bound);
		g_.edge_capacity_uppper_bounds.push_back(e.capacity_upper_bound);
		edges_.push_back(e);
		++g_.edge_count;
	}
	
	int GraphConverter::GetVertexID(int phone_count, int time, int index) {
		// Both time and index should be 0-indexed
		return time * phone_count + index;
	}
	
	std::string GraphConverter::ConstructPhoneName(int time, int index) {
		return "Phone " + boost::lexical_cast<std::string>(index) + " at time " + boost::lexical_cast<std::string>(time);
	}
	
	void GraphConverter::PrintInformation() {
		std::cout << "******************************" << std::endl;
		std::cout << "****** Graph Information *****" << std::endl;
		std::cout << "******************************" << std::endl;
		std::cout << "Vertex count: " << g_.vertex_count << std::endl;
		std::cout << "Edge count: " << g_.edge_count << std::endl;
		const int kSinkID = g_.vertex_count - 1;
		const int kSourceID = g_.vertex_count - 2;
		std::cout << "Source: " << kSourceID << " , Supply: " << g_.vertex_supply[kSourceID] << std::endl;
		std::cout << "Sink: " << kSinkID << " , Supply: " << g_.vertex_supply[kSinkID] << std::endl;
		
//		for (int i = 0; i < g_.vertex_count; ++i) {
//			std::cout << "Vertex " << i << ", Supply: " << g_.vertex_supply[i] << std::endl;
//		}
								
		for (int i = 0; i < g_.edge_count; ++i) {
			std::cout << "Edge " << i << ": " << edges_[i].name << " (" << g_.edge_tails[i] << " to " << g_.edge_heads[i] << "), Capacity: [" << g_.edge_capacity_lower_bounds[i] << "," << g_.edge_capacity_uppper_bounds[i] <<"]" << std::endl;
		}
	}
	
	void GraphConverter::Clear() {
		edges_.clear();
		g_.Clear();
	}
}
