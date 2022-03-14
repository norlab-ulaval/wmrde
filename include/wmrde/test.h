//test.h
//unit tests for functions, ordered from low to high level

#ifndef _WMRDE_TEST_H_
#define _WMRDE_TEST_H_

#include <iostream>
#include <iomanip>
#include <time.h>
#include <vector>
#include <chrono>
#include <ctime>
//#include <memory> //for unique_ptr, C++11 only

#include <Eigen/Dense>

#include <wmrde/demo/terrains.h>
#include <wmrde/demo/models.h>
#include <wmrde/dynamics.h>

//TODO, define timeval, gettimeofday() for windows
//http://stackoverflow.com/questions/10905892/equivalent-of-gettimeday-for-windows
inline double tosec(timeval tim)
{
  return tim.tv_sec + (tim.tv_usec/1000000.0);
}

void test_common();

//algebra
void test_linalg3();
void test_transform();
void test_spatial();
void test_matrix();

//collision
void test_surface();
void test_updateWheelContactGeom();
void test_updateTrackContactGeom();

//state
void test_stateToHT();
void test_qvelToQdot();

//helper sub functions
void sub_initTrackContactGeom(const WmrModel& mdl, TrackContactGeom* contacts);

//kinematics
void test_wheelJacobians();
void test_trackJacobians();
void test_forwardVelKin();
void test_initTerrainContact();

//dynamics
void test_subtreeInertias();
void test_jointSpaceInertia();
void test_jointSpaceBiasForce();
void test_forwardDyn();

void test_simulate();

#endif //_WMRDE_TEST_H_
