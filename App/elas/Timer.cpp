#include "Timer.h"


namespace DDTR {
double _Tic()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec  + 1e-6 * (tv.tv_usec);
}

double _Toc( double dSec )
{
  return _Tic() - dSec;
}

double _TicMS()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  double dTSec = tv.tv_sec  + 1e-6 * (tv.tv_usec);
  return dTSec*1e3;
}

double _TocMS( double dMS )
{
  return _TicMS() - dMS;
}
}

