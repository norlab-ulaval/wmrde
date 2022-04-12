#include <wmrde/predictor.h>

Predictor::Predictor()
{
    //uncomment one of the following:
    //for animation, must also uncomment the corresponding scene function below
//	zoe(mdl,state,qvel);
//    rocky(mdl,state,qvel);
//	talon(mdl,state,qvel);
//    warthog(mdl, state, qvel);
    trackedWarthog(mdl, state, qvel);

    //initialize wheel-ground contact model
    mdl.wheelGroundContactModel(0, mdl.wgc_p, 0, 0, 0, //inputs
                                0, 0); //outputs

    if (ideal_actuators)
        mdl.actuatorModel=0;

    //get from WmrModel
    nf = mdl.get_nf();
    nw = mdl.get_nw();
    nt = mdl.get_nt();
    ns = NUMSTATE(nf); //number of states
    //for dynamic sim
    nv = NUMQVEL(nf); //size of joint space vel
    na = mdl.get_na();

    //terrain
    flat(surfs);
    //ramp(surfs); //must also uncomment flat

    //init contact geometry
    if (nw > 0) {
        contacts = static_cast<ContactGeom*>(wcontacts);
    } else if (nt > 0) {
        sub_initTrackContactGeom(mdl, tcontacts);
        contacts = static_cast<ContactGeom*>(tcontacts);
    }

    // initialize input vector

    initTerrainContact(mdl, surfs, contacts, state); //DEBUGGING

    //allocate

    //init y
    if (do_dyn) { //dynamic sim
        copyVec(ns,state,y);
        copyVec(nv,qvel,y+ns);
        setVec(na,0.0,y+ns+nv); //interr
        ny = ns + nv + na;
    } else {
        copyVec(ns,state,y);
        ny = ns;
    }

    //backup
    Real y0[MAXNY];
    copyVec(ny,y,y0);

}

//void Predictor::predict(Real *y, Real *u_cmd, Real *ydot, const Real dt) {
//
//    Real time = 0;
//    std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;
//    std::cout << "dt="<< dt << std::endl;
//
//    // initialize input vector
//    u.cmd[0] = u_cmd[0];
//    u.cmd[1] = u_cmd[1];
//    u.cmd[2] = u_cmd[2];
//    u.cmd[3] = u_cmd[3];
////    u.cmd[4] = 0.0;
////    u.cmd[5] = 5.0;
////    u.cmd[6] = 5.0;
////    u.cmd[7] = 5.0;
//
//    if (do_dyn) {
//        odeDyn(time, y, mdl, surfs, contacts, dt, u, ydot, HT_parent);
//    } else {
//        odeKin(time, y, mdl, surfs, contacts, u, ydot, HT_parent);
//    }
//    addmVec(ny,ydot,dt,y);
//    std::cout << "dt="<< dt << std::endl;
//    time += dt;
//    std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;
//}

std::array<float, Predictor::MAXNY> Predictor::predict(std::array<float, Predictor::MAXNY> y_array,
                                                       std::array<float, 4> u_cmd_array,
                                                       std::array<float, Predictor::MAXNY> ydot_array,
                                                       const Real dt) {

    std::copy(std::begin(y_array), std::end(y_array), std::begin(y));
    std::copy(std::begin(ydot_array), std::end(ydot_array), std::begin(ydot));
    std::copy(std::begin(u_cmd_array), std::end(u_cmd_array), std::begin(u.cmd));

    Real time = 0;
//    std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;

    // initialize input vector
//    u.cmd[0] = u_cmd[0];
//    u.cmd[1] = u_cmd[1];
//    u.cmd[2] = u_cmd[2];
//    u.cmd[3] = u_cmd[3];
//    u.cmd[4] = 0.0;
//    u.cmd[5] = 5.0;
//    u.cmd[6] = 5.0;
//    u.cmd[7] = 5.0;

    if (do_dyn) {
        odeDyn(time, y, mdl, surfs, contacts, dt, u, ydot, HT_parent);
    } else {
        odeKin(time, y, mdl, surfs, contacts, u, ydot, HT_parent);
    }
    addmVec(ny,ydot,dt,y);
//    std::cout << "dt="<< dt << std::endl;
    time += dt;
//    std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;

    std::copy(std::begin(y), std::end(y), std::begin(y_array));
    return y_array;
}

