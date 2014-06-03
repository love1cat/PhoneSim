//
//  multidim_vector.h
//  MobileSensingSim
//
//  Created by Yuan on 5/21/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef MobileSensingSim_multidim_vector_h
#define MobileSensingSim_multidim_vector_h

#include <vector>
#include <algorithm>

namespace mobile_sensing_sim {	
	template <typename T>
	class ThreeDimVector {
	public:
		ThreeDimVector() : dimone_size_(0), dimtwo_size_(0), dimthree_size_(0){}
		ThreeDimVector(int dimone_size, int dimtwo_size, int dimthree_size) : data(dimone_size * dimtwo_size * dimthree_size), dimone_size_(dimone_size), dimtwo_size_(dimtwo_size), dimthree_size_(dimthree_size){
		}
		
		void Resize(int dimone_size, int dimtwo_size, int dimthree_size) {
			data.resize(dimone_size * dimtwo_size * dimthree_size);
			dimone_size_ = dimone_size;
			dimtwo_size_ = dimtwo_size;
			dimthree_size_ = dimthree_size;
		}
		
		void Fill(const T& value) {
			std::fill(data.begin(), data.end(), value);
		}
		
		int DimOneSize() const {
			return dimone_size_;
		}
		
		int DimTwoSize() const {
			return dimtwo_size_;
		}
		
		int DimThreeSize() const {
			return dimthree_size_;
		}
		
		bool Empty() const {
			return data.empty();
		}
		
		T& operator()(int dimone_id, int dimtwo_id, int dimthree_id) {
			assert(dimone_id < dimone_size_ && dimtwo_id < dimtwo_size_ && dimthree_id < dimthree_size_);
			return data.at(dimone_id * dimtwo_size_ * dimthree_size_ + dimtwo_id * dimthree_size_ + dimthree_id);
		}
		
		const T& operator()(int dimone_id, int dimtwo_id, int dimthree_id) const {
			assert(dimone_id < dimone_size_ && dimtwo_id < dimtwo_size_ && dimthree_id < dimthree_size_);
			return data.at(dimone_id * dimtwo_size_ * dimthree_size_ + dimtwo_id * dimthree_size_ + dimthree_id);
		}
	private:
		int dimone_size_;
		int dimtwo_size_;
		int dimthree_size_;
		std::vector<T> data;
	};
}

#endif
