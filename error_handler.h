//
//  error_handler.h
//  MobileSensingSim
//
//  Created by Yuan on 4/22/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef MobileSensingSim_error_handler_h
#define MobileSensingSim_error_handler_h

#include <cstdlib>
#include <string>
#include <iostream>

namespace mobile_sensing_sim {
	class ErrorHandler {
	public:
		static void CodingError(const std::string& error_message) {
			std::cout << "**** Coding error happened ****" << std::endl <<error_message <<std::endl;
			exit(EXIT_FAILURE);
		}
		static void RunningError(const std::string& error_message) {
			std::cout << "**** Running error happened ****" << std::endl <<error_message <<std::endl;
			exit(EXIT_FAILURE);
		}
	private:
		ErrorHandler();
	};

}

#endif
