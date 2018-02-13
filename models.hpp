#ifndef MODELS_HPP
#define MODELS_HPP

#include <iostream>
#include <frames_io.hpp>
#include <models.hpp>
#include <chainiksolverpos_lma.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <boost/timer.hpp>

using namespace std;
using namespace KDL;

Chain Puma560();
Chain SnakeRobot();


#endif // MODELS_HPP
