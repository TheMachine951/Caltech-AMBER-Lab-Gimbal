#include "IMUAbstract.h"

	IMUAbstract::IMUAbstract(const IMUAbstract::MODE &mode,
		                       const uint32_t &receiveTimeout) :
	receiveTimeout_(receiveTimeout),
	tnm1Received_(0),
	dataCounter_(0),
	mode_(mode),
	status_(IMUAbstract::STATUS::INIT)
	{}

	IMUAbstract::~IMUAbstract()
	{}

	void IMUAbstract::resetTimeout(void)
	{
		tnm1Received_ = micros();
	}
