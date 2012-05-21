#ifndef THREAD_FSLEEP_HH
#define THREAD_FSLEEP_HH

#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp" 

inline void 
thread_fsleep(double ss) 
{
    using boost::posix_time::time_duration;

    long count = ss * time_duration::ticks_per_second();
    long secs = count / time_duration::ticks_per_second();
    long frac = count % time_duration::ticks_per_second();
    time_duration td(0,0,secs,frac);
    boost::this_thread::sleep(td);
}

#endif
