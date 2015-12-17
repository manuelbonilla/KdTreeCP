#ifndef FMM_PLANNIG_HPP
#define FMM_PLANNIG_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <stack> 
#include <random>

namespace FMM {

#define FROZEN 0
#define UNKNOWN 1
#define NARROW_BAND 2

	class Point
	{
		
		std::vector< float > position_;
		float F_;
		float T_;
		std::std::vector< *Point > neighbours_;
		int status_;

	};


} // FMM

#endif // FMM_PLANNIG_HPP

