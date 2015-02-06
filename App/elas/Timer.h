#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>

namespace DDTR {

double _Tic();

double _Toc( double dSec );

double _TicMS();

double _TocMS( double dMS );

}

#endif // TIMER_H
