#ifndef METROBOTICS_POSIX_TIMER_H
#define METROBOTICS_POSIX_TIMER_H

#include "Timer.h"
#include <sys/time.h>

	/**
	 * \class   PosixTimer
	 *
	 * \brief   POSIX implementation of the Timer interface.
	 *
	 * \details This implementation of the high resolution timer is able to count one-second
	 *          intervals of time precise to the microsecond.
	 *
	 * \author  Mark Manashirov <mark.manashirov@gmail.com>
	 */
	class PosixTimer : public Timer
	{
		public:
			/**
			 * \brief   Default constructor.
			 *
			 * \details Initializes the reference point to the time of object creation.
			 */
			PosixTimer();

			// [Implement/override the interface that we inherited from Timer.]
			void start();
			double elapsed() const;
	};


/**
 * Convert POSIX timeval to a double that represents the corresponding time in seconds.
 */
static inline double tvToDouble(const struct timeval &tv)
{
	return tv.tv_sec + (tv.tv_usec / 1000000.0);
}

PosixTimer::PosixTimer()
{
	// Initialize the reference point to the current time.
	start();
}

void PosixTimer::start()
{
	// Read the current time.
	struct timeval tv;
	gettimeofday(&tv, 0);
	// Reset the reference point.
	_ref = tvToDouble(tv);
}

double PosixTimer::elapsed() const
{
	// Read the current time.
	struct timeval tv;
	gettimeofday(&tv, 0);
	// Compute the difference from the reference point.
	return tvToDouble(tv) - _ref;
}




#endif
