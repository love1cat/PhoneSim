//
//  simlog.h
//  MobileSensingSim
//
//  Created by Yuan on 5/1/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__simlog__
#define __MobileSensingSim__simlog__

#include <string>
#include <iostream>
#include <fstream>

namespace mobile_sensing_sim {
  class SimLog {
  public:
    template <typename T>
    SimLog& operator << (const T& data) {
      if (!IsLog) {
        return *this;
      }
      of_ << data;
      of_.flush();
      return *this;
    }
    
    ~SimLog(){
      of_.close();
    }
    
    void Close() {
      of_.close();
    }
    
    SimLog(const std::string& filename) {
      if(!IsLog) {
        return;
      }
      file_name_ = filename;
      of_.open(filename.c_str(), std::fstream::out | std::fstream::trunc);
      of_.setf(std::ios::fixed, std::ios::floatfield);
      of_.precision(1);
    }
    
    void Reset() {
      of_.close();
      of_.open(file_name_.c_str(), std::fstream::trunc);
    }
    
    static bool IsLog;
  private:
    std::ofstream of_;
    std::string file_name_;
  };
  
  // Create global log
  extern SimLog log;
  extern SimLog hlog; // heuristic algorithm log.
  extern SimLog hdlog; // heuristic algorithm log.
  extern SimLog ahlog; // aggressive heuristic algorithm log.
  extern SimLog olog; // optimal algorithm log.
  extern SimLog nlog; // naive algorithm log.
  extern SimLog oblog; // optimal balance algorithm log.
}

#endif /* defined(__MobileSensingSim__simlog__) */
