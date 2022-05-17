//
// Created by dominic on 2022-04-12.
//

#ifndef OGREAPP_PREDICTOR_H
#define OGREAPP_PREDICTOR_H

#include <wmrde/test.h>
#include <wmrde/algebra/matrix.h>
#include <array>

// PYTHON BIDING
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
//****************************


class Predictor {

private:
    //options
    bool do_dyn =false; //do dynamic simulation, else kinematic
    bool ideal_actuators = true;
    bool do_anim = false; //do animation
    bool do_time = true;

    const static Real dt;
    int nsteps;



    //get from WmrModel
    int nf;
    int nw;
    int nt;
    int ns; //number of states
    //for dynamic sim
    int nv = NUMQVEL(nf); //size of joint space vel
    int na = mdl.get_na();
    int ny;

    WheelContactGeom wcontacts[WmrModel::MAXNW];
    TrackContactGeom tcontacts[WmrModel::MAXNW];
    ContactGeom* contacts =0; //base class

    WmrModel mdl;
    SurfaceVector surfs;
    ControllerIO u;

public:
    //for allocation
    const static int MAXNS = NUMSTATE(WmrModel::MAXNF);
    const static int MAXNV = NUMQVEL(WmrModel::MAXNF);
    const static int MAXNY = MAXNS+MAXNV+WmrModel::MAXNA; //for dynamic sim

    //make WmrModel object
    Real state[MAXNS];
    Real qvel[MAXNV]; //for dynamic sim
    Real y[MAXNY];
    Real ydot[MAXNY];

    HomogeneousTransform HT_parent[WmrModel::MAXNF];
    //constructor
    Predictor();

//    void predict(Real y[MAXNY], Real u_cmd[4], //inputs
//                 Real ydot[MAXNY], const Real dt=0.05);
    std::array<float, MAXNY> predict(std::array<float, Predictor::MAXNY> y,
                                     std::array<float, 4> u_cmd,
                                     std::array<float, Predictor::MAXNY> ydot,
                                     const Real dt=0.05);

    MatrixXr load_csv (const std::string & path);

};

#endif //OGREAPP_PREDICTOR_H
