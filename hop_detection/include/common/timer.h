#ifndef MY_TIMER_H
#define MY_TIMER_H

#include <stdio.h>
#include <boost/timer.hpp>

#define TIMER_START(FUNC) boost::timer t_##FUNC;
#define TIMER_END(FUNC) std::cout << "[" << #FUNC << "]" << "cost time: " << t_##FUNC.elapsed() << std::endl;
#endif