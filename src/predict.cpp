#include <wmrde/test.h>

int main(int argc, char *argv[]) //use this for console output
{
    //options
    bool do_dyn = false; //do dynamic simulation, else kinematic
    bool ideal_actuators = true;
    bool do_anim = false; //do animation
    bool do_time = true;

    const Real dt = .05; // 20 Hz
    const Real predictionLength = 10.0; // 10 seconds prediction windows
    const int nsteps = (int) floor(predictionLength/dt);
    Real time = 0;

    //for allocation
    const int MAXNS = NUMSTATE(WmrModel::MAXNF);
    const int MAXNV = NUMQVEL(WmrModel::MAXNF);
    const int MAXNY = MAXNS+MAXNV+WmrModel::MAXNA; //for dynamic sim

    //make WmrModel object
    WmrModel mdl;
    Real state[MAXNS];
    Real qvel[MAXNV]; //for dynamic sim

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
    const int nf = mdl.get_nf();
    const int nw = mdl.get_nw();
    const int nt = mdl.get_nt();
    const int ns = NUMSTATE(nf); //number of states
    //for dynamic sim
    const int nv = NUMQVEL(nf); //size of joint space vel
    const int na = mdl.get_na();
    int ny;

    //terrain
    SurfaceVector surfs;

    flat(surfs);
    //ramp(surfs); //must also uncomment flat

    //init contact geometry
    WheelContactGeom wcontacts[WmrModel::MAXNW];
    TrackContactGeom tcontacts[WmrModel::MAXNW];
    ContactGeom* contacts =0; //base class

    if (nw > 0) {
        contacts = static_cast<ContactGeom*>(wcontacts);
    } else if (nt > 0) {
        sub_initTrackContactGeom(mdl, tcontacts);
        contacts = static_cast<ContactGeom*>(tcontacts);
    }

    // initialize input vector

    ControllerIO u;
    u.cmd[0] = 10.0;
    u.cmd[1] = 10.0;
    u.cmd[2] = 10.0;
    u.cmd[3] = 10.0;
//    u.cmd[4] = 0.0;
//    u.cmd[5] = 5.0;
//    u.cmd[6] = 5.0;
//    u.cmd[7] = 5.0;

    initTerrainContact(mdl, surfs, contacts, state); //DEBUGGING

    //allocate
    Real y[MAXNY];
    Real ydot[MAXNY];

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

    //allocate
    HomogeneousTransform HT_parent[WmrModel::MAXNF];

    std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;

    if (do_time) {
        //time it
        int n= (int) 1;
//		timeval t0, t1;

        time = 0;
        std::cout << dt << std::endl;

        auto t0 = std::chrono::system_clock::now();
        for (int iter=0; iter<n; iter++) {
            copyVec(ny,y0,y); //reset to backup

            for (int i=0; i<nsteps; i++) {
                if (do_dyn) {
                    odeDyn(time, y, mdl, surfs, contacts, dt, u, ydot, HT_parent);
                } else {
                    odeKin(time, y, mdl, surfs, contacts, u, ydot, HT_parent);
                }
                addmVec(ny,ydot,dt,y);
                time += dt;
                std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;
            }
        }
        auto t1 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedTime = t1-t0;

        std::cout << "simulate\n";
        std::cout << "iterations: " << (Real) n << std::endl;
        std::cout << "clock (sec): " << elapsedTime.count() << std::endl;
    }

    return 0;
}

