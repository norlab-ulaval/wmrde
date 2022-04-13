#include <wmrde/demo/trackedwarthogmodel.h>

void trackedWarthog(WmrModel& mdl, Real state[], Real qvel[]) {

	const Real intom = 2.54/100;
	const Real lbtokg = .453592;

    //Tracked warthog rover model

    //dimensions
    const Real k1 = 0;		            //forward offset between (R)over reference to (D)ifferential
    const Real k2 = 0.5826;	            //horizontal offset between R and D
    const Real k3 = 0.24979 + 0.27218;	//height of R from D
    const Real k4_front = 0.457367 - 0.24046; //forward distance between D and tracks
    const Real k4_rear = 0.457367 + 0.24046; //forward distance between D and tracks
    const Real k5 = 0;		            //horizontal distance between D and wheels
    const Real k6_front = 0.012977 + 0.20887; //height from D to wheels
    const Real k6_rear = 0.012977 + 0.18336; //height from D to wheels


    const int nt=4; //number of tracks
    const Real rad_rear = 0.293; //sprocket radius
    const Real rad_front = 0.344; //sprocket radius
//    const Real rad_front = 0.175; //sprocket radius
    const Real Wt = 0.2794; //width of track
    const Real Lt = 0.24046 + 0.43101; // track length

    //masses (kg)
    const Real Mb = 100;	//mass of body
    const Real Md = 50;     //mass of differential
    const Real Mt = 30;		//mass of track
    const Real TotalMass = Mb + 2*Md + nt*Mt;

    const Real Lb = 1.35;		//length of body
    const Real Wb = 0.70;		//width of body
    const Real Hb = 0.30;		//height of body

    const Real Ld = 1.34;       //length of diff
    const Real Wd = 0.28;       //width of diff
    const Real Hd = 0.63;       //height of diff

//    const Real Iy = Mt*(rad_front*rad_front)/2; //rotational inertia of track about y axis

    Vec3 cmb = {0,0, 0}; //body center of mass (in body coords)
    Vec3 cms = {0,0,0}; //sprocket center of mass

	mdl.addBodyFrame("Body");

	HomogeneousTransform HT;
	VecEuler euler = {0,0,0};
	VecOrient orient;
#if WMRSIM_USE_QUATERNION
	eulerToQuat(euler,orient);
#else
	copyEuler(euler,orient);
#endif
	Vec3 pos;

    //LEFT SIDE
    //differential frame
    setVec3(k1,k2,-k3,pos);
    poseToHT(orient,pos,HT);
    mdl.addJointFrame("DL","Body","RY",false,HT);
	
	//left front track
    setVec3(k4_front,k5,-k6_front,pos);
	poseToHT(orient,pos,HT);
	mdl.addSprocketFrame("FL","DL",true,HT,rad_rear,rad_front,Lt);

    //left rear track
    setVec3(-k4_rear,k5,-k6_rear,pos);
	poseToHT(orient,pos,HT);
	mdl.addSprocketFrame("RL","DL",true,HT,rad_front,rad_rear,Lt);

    //RIGHT SIDE
    //differential frame
    setVec3(k1,-k2,-k3,pos);
    poseToHT(orient,pos,HT);
    mdl.addJointFrame("DR","Body","RY",false,HT);

    //right front track
    setVec3(k4_front,-k5,-k6_front,pos);
    poseToHT(orient,pos,HT);
    mdl.addSprocketFrame("FR","DR",true,HT,rad_rear,rad_front,Lt);

//    right rear track
    setVec3(-k4_rear,-k5,-k6_rear,pos);
    poseToHT(orient,pos,HT);
    mdl.addSprocketFrame("RR","DR",true,HT,rad_front,rad_rear,Lt);

    //set mass properties
	int nf = mdl.get_nf();
	const int* sprocketframeinds = mdl.get_sprocketframeinds();

	Mat3 I; //Inertia
    inertiaBox(Mb,Lb,Wb,Hb,I);
    mdl.setFrameMass(0,Mb,cmb,I);

    inertiaBox(Md, Ld, Wd, Hd, I);
    mdl.setFrameMass(1, Md, cmb, I);
    mdl.setFrameMass(4, Md, cmb, I);

    inertiaBox(Mt,Lt,Wt,2*rad_front,I); // Assuming the tracks are a box
//	I[COL1+1] = Iy;
	for (int i=0; i<nt; i++)
		mdl.setFrameMass(sprocketframeinds[i],Mt,cms,I);


	//FOR KINEMATIC SIM
	mdl.min_npic = 1;

    //contact height error
    setVec(nt,-.01,mdl.dz_target);
    setVec(nt,.1,mdl.tc_z);
    mdl.tc_j[0] = .1;

	//FOR DYNAMIC SIM
	mdl.wheelGroundContactModel = uniformWgc;

	int npflat = 3;
	Real Kp = TotalMass*mdl.grav/(nt*npflat*-mdl.dz_target[0]);
	setWgcParams(Kp,mdl.wgc_p);

	mdl.actuatorModel = trackedWarthogAct;
	mdl.act_p[0] = 2e3; //Kp
	mdl.act_p[1] = 0.0; //Ki
	mdl.act_p[2] = REALMAX; //max

	//ODE contact model parameters
	Real erp,cfm;
	KpKdToErpCfm(Kp, Kp/20, .04, erp, cfm);
	Real fds = 1.0/(Kp*1e-1);

	setVec(nt,erp,mdl.erp_z);
	setVec(nt,cfm,mdl.cfm_z);
	setVec(nt,fds,mdl.fds_x);
	setVec(nt,fds,mdl.fds_y);

	//FOR BOTH
	mdl.controller = trackedWarthogController;
    mdl.holonomicJointConstraints = trackedWarthogConstraints;
    mdl.set_njc(1);

	//initialize the state vector
	setEuler(DEGTORAD(0),DEGTORAD(0),DEGTORAD(0),euler);
    setVec3(0.0, 0.0, (1.0), pos);

#if WMRSIM_USE_QUATERNION
	eulerToQuat(euler,orient);
#else
	copyEuler(euler,orient);
#endif
	int ns = NUMSTATE(nf); //number of elements in state
	setVec(ns,0.0,state);
	copyOrient(orient,state+SI_ORIENT);
	copyVec3(pos,state+SI_POS);


	if (qvel != 0) { //not null
		//initialize qvel
		//setVec(mdl.get_na(), 0.0, qvel); //to zeros

		//to cmd
		Real u[WmrModel::MAXNA];
		mdl.controller(mdl, 0.0, state, u, qvel);
	}

}


void trackedWarthogController(const WmrModel& mdl, const Real time, const Real state[], //inputs
                     Real u[], Real qvel_cmd[]) { //outputs

    const Real intom = 2.54/100;

    //dimensions
    const Real k1 = 0;		            //forward offset between (R)over reference to (D)ifferential
    const Real k2 = 0.5826;	            //horizontal offset between R and D
    const Real k3 = 0.24979 + 0.27218;	//height of R from D
    const Real k4_front = 0.457367 - 0.24046; //forward distance between D and tracks
    const Real k4_rear = 0.457367 + 0.24046; //forward distance between D and tracks
    const Real k5 = 0;		            //horizontal distance between D and tracks
    const Real k6_front = 0.012977 + 0.20887; //height from D to tracks
    const Real k6_rear = 0.012977 + 0.18336; //height from D to tracks


    const int nt=4; //number of tracks
    const Real rad_rear = 0.293; //sprocket radius
    const Real rad_front = 0.344; //sprocket radius
    const Real Wt = 0.2794; //width of track
    const Real Lt = 0.24046 + 0.43101; // track length


    const Real B = 2*(k2+k5);
    const Real rad_sprocket = 0.175;

    Real speed, omega; //commanded speed, yaw rate

    speed = 0.5;
    omega = 0;

    Real vl, vr; //vel left, vel right (m/s)

    vl = speed - (B/2)*omega;
    vr = speed + (B/2)*omega;

    u[0] = vl/rad_sprocket;
    u[1] = vl/rad_sprocket;
    u[2] = vr/rad_sprocket;
    u[3] = vr/rad_sprocket;


    if (qvel_cmd != 0) { //not null

        //get from WmrModel
        int nv = NUMQVEL(mdl.get_nf());
        const int nt = mdl.get_nt();
        const int* sprocketframeinds = mdl.get_sprocketframeinds();

        setVec(nv, 0.0, qvel_cmd);
        qvel_cmd[VI_ANG + 2] = omega;
        qvel_cmd[VI_LIN + 0] = speed;

        //sprocket velocities
        for (int i=0; i<nt; i++)
            qvel_cmd[TOQVELI(sprocketframeinds[i])] = u[i];

    }
}

void trackedWarthogConstraints( const WmrModel& mdl, const Real jd[], const Real jr[], //inputs
                         Real c[], Real Jc[], Real f[], Real df_djd[], Real df_djr[]) { //outputs

    //rocker joint indices
    const int D1_ji = 0;
    const int D2_ji = 3;

    c[0] = jd[D1_ji] + jd[D2_ji]; //angles should be equal magnitude, opposite sign


    const int nj = mdl.get_nf()-1; //number of joints


    //init to zeros
    setVec(nj,0.0,Jc);
//    std::cout << nj << std::endl;

    Jc[D1_ji] = 1.0;
    Jc[D2_ji] = 1.0;


    if ( f != 0 ) { //if not null
        //for dynamic sim
        Real Kp = 2250;
        Real Kd = Kp/20;
        Real cd = jr[D1_ji] + jr[D2_ji];

        f[0] = -(Kp*c[0] + Kd*cd);
        if ( df_djd != 0 ) {
            copyVec(nj,Jc,df_djd);
            mulcVec(nj,-Kp,df_djd);

            copyVec(nj,Jc,df_djr);
            mulcVec(nj,-Kd,df_djr);
        }
    }
}