
      /**
	*	ID:	LinuxTimer.hpp
	*   EDITED:	22-07-2014
	*    AUTOR:	Andres Gongora
	*
	*	+------ Description: -----------------------------------------------------------+
	*	|										|
	*	|	Class to control elapsed time in linux. Time expressed in seconds. 	|
	*	|	Can also be used to obtain incremental time between calls. 		|
	*	|	Timer is restarted inside the constructor and manually calling reset().	|
	*	|										|
	*	+-------------------------------------------------------------------------------+
	*	
	*/


       /*
	* Copyright (C) 2014 Andres Gongora
	* <https://yalneb.blogspot.com>
	*
	* This program is free software: you can redistribute it and/or modify
	* it under the terms of the GNU General Public License as published by
	* the Free Software Foundation, either version 3 of the License, or
	* (at your option) any later version.
	*
	* This program is distributed in the hope that it will be useful,
	* but WITHOUT ANY WARRANTY; without even the implied warranty of
	* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	* GNU General Public License for more details.
	*
	* You should have received a copy of the GNU General Public License
	* along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/


	//TODO: 

#ifndef __LINUXTIMER_HPP_INCLUDED__
#define __LINUXTIMER_HPP_INCLUDED__


//##### DECLARATION ################################################################################

class LinuxTimer
{
public:
				LinuxTimer();		// Default constructor
				~LinuxTimer();		// Default destructor

				// API
	inline void		reset();			// Reset timeer;
	inline double		getElapsedSeconds();		// Return time since class was intantiated or reseted
	inline double		getIncrementSeconds();		// Return time since last call to this function in seconds


private:
	double 			getElpasedSecondsSince(struct timespec);

	struct timespec 	timeNow;			// Currently measured time
	struct timespec		timeStarted;			// Time at the moment the timer was started
	struct timespec		timeLast;			// Time at last measure. Used for incremental time
};


//##### DEFINITION #################################################################################

/***************************************************************************************************
 *	Constructor & Destructor
 **************************************************************************************************/
LinuxTimer::LinuxTimer()
{
	reset();						// Save current time
}


LinuxTimer::~LinuxTimer()
{
	// Do nothing
}	


/***************************************************************************************************
 *	COMPUTE ELAPSED TIME SINCE TIME MARK
 **************************************************************************************************/
double LinuxTimer::getElpasedSecondsSince(struct timespec olderTime)
{
	long int timeDeltaS;					// Difference in sencods
	long int timeDeltaNS;					// Difference in nano sencods
	double timeReturn;					// Time to return


	// UPDATE TIME
	clock_gettime(CLOCK_REALTIME, &timeNow);
	timeDeltaNS = timeNow.tv_nsec - olderTime.tv_nsec;
	timeDeltaS = timeNow.tv_sec - olderTime.tv_sec;

	//Note:	we need not to check if timeDeltaS gets negative because of tv_sec overflowing.
	//	It is intended to count seconds since the epoch. So overflow is no problem for now.
	//	But timeDeltaNS has to be checked, as tv_nsec overflows every second.	

	// NANOSECONDS-HAS OVERFLOWN
	if(timeDeltaNS < 0)
	{
		timeDeltaNS += 1000000000;			// Compensate timer overflow
		timeDeltaS -= 1;				// Substract 1 to seconds-timer. 
	}

	timeReturn = (double)timeDeltaNS / 1000000000;		// NS' = NS -> S
	timeReturn += timeDeltaS;				// S + NS'

	return timeReturn;
}


/***************************************************************************************************
 *	API
 **************************************************************************************************/
inline void LinuxTimer::reset()
{
	clock_gettime(CLOCK_REALTIME, &timeStarted);
	timeLast = timeStarted;					// because no incremental time read has been performed yet.
}


inline double LinuxTimer::getElapsedSeconds()
{
	return getElpasedSecondsSince(timeStarted);		// Return time since class was intantiated or reseted
}


inline double LinuxTimer::getIncrementSeconds()			// Return time since last call to this function in seconds
{
	double incrementalTime = getElpasedSecondsSince(timeLast);
	clock_gettime(CLOCK_REALTIME, &timeLast);		// Save current time for next iteration
	return incrementalTime;
}


#endif	//LINUXTIMER
