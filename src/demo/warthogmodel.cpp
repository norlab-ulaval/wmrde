#include <wmrde/demo/warthogmodel.h>
//& to pass object by reference


//FUNCTIONS FOR THE WHEELED WARTHOG ROVER
void warthog(WmrModel& mdl, Real state[], Real qvel[]) {
	//Wheeled warthog rover model

	//dimensions
	const Real k1 = 0;		            //forward offset between (R)over reference to (D)ifferential
	const Real k2 = 0.5826;	            //horizontal offset between R and D
	const Real k3 = 0.24979 + 0.27218;	//height of R from D
	const Real k4 = 0.457367;		    //forward distance between D and wheels
	const Real k5 = 0;		            //horizontal distance between D and wheels
	const Real k6 = 0.012977;		    //height from D to wheels
	

	const int nw=4; //number of wheels

	//masses (kg)
	const Real Mb = 100;	//mass of body
	const Real Md = 50;     //mass of differential
    const Real Mw = 15;		//mass of wheel
	const Real TotalMass = Mb + 2*Md + nw*Mw;

	const Real Lb = 1.35;		//length of body
	const Real Wb = 0.70;		//width of body
	const Real Hb = 0.30;		//height of body

    const Real Ld = 1.34;       //length of diff
    const Real Wd = 0.28;       //width of diff
    const Real Hd = 0.63;       //height of diff

    const Real Wr = 0.3;        //wheel radius
	const Real Ww = .25;		//wheel width

	Vec3 cmb = {0,0, 0}; //body center of mass (in body coords)
	Vec3 cmw = {0,0,0}; //wheel center of mass

	//BUILD KINEMATIC TREE
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

	//wheel frame, front left
	setVec3(k4,k5,-k6,pos);
	poseToHT(orient,pos,HT);
	mdl.addWheelFrame("FL","DL",true,HT,Wr);

    //wheel frame, back left
	setVec3(-k4,k5,-k6,pos);
	poseToHT(orient,pos,HT);
	mdl.addWheelFrame("RL","DL",true,HT,Wr);

	//wheel frame, back

	//RIGHT SIDE
    //differential frame
    setVec3(k1,-k2,-k3,pos);
    poseToHT(orient,pos,HT);
    mdl.addJointFrame("DR","Body","RY",false,HT);

    //wheel frame, front left
    setVec3(k4,-k5,-k6,pos);
    poseToHT(orient,pos,HT);
    mdl.addWheelFrame("FR","DR",true,HT,Wr);

    //wheel frame, back left
    setVec3(-k4,-k5,-k6,pos);
    poseToHT(orient,pos,HT);
    mdl.addWheelFrame("RR","DR",true,HT,Wr);

	//set mass properties
	int nf = mdl.get_nf();
	const int* wheelframeinds = mdl.get_wheelframeinds();

	Mat3 I; //moment of inertia
	inertiaBox(Mb,Lb,Wb,Hb,I);
	mdl.setFrameMass(0,Mb,cmb,I);

    inertiaBox(Md, Ld, Wd, Hd, I);
    mdl.setFrameMass(1, Md, cmb, I);
    mdl.setFrameMass(4, Md, cmb, I);

	inertiaCylinder(Mw,Wr,Ww,1,I);
	for (int i=0; i<nw; i++)
		mdl.setFrameMass(wheelframeinds[i],Mw,cmw,I);

	//FOR KINEMATIC MODEL

	mdl.min_npic = nw;

	setVec(nw,-.02,mdl.dz_target);
	setVec(nw,.1,mdl.tc_z);
	mdl.tc_j[0] = .1;

	//FOR DYNAMIC MODEL
	
	mdl.wheelGroundContactModel = uniformWgc;

	Real Kp = TotalMass*mdl.grav/(nw*-mdl.dz_target[0]);
	setWgcParams(Kp,mdl.wgc_p);

	mdl.actuatorModel = warthogAct;
	mdl.act_p[0] = 5; //Kp
	mdl.act_p[1] = 0; //Ki
	mdl.act_p[2] = REALMAX; //max

	//ODE contact model parameters
	Real erp,cfm;
	KpKdToErpCfm(Kp, Kp/20, .04, erp, cfm);
	Real fds = 1.0/(Kp*1e-1);

	setVec(nw,erp,mdl.erp_z);
	setVec(nw,cfm,mdl.cfm_z);
	setVec(nw,fds,mdl.fds_x);
	setVec(nw,fds,mdl.fds_y);
	mdl.erp_j[0] = .2;
	mdl.cfm_j[0] = 1e-6;

	//FOR BOTH
	//set function pointers
	mdl.controller = warthogController;
	mdl.holonomicJointConstraints = warthogConstraints;
	mdl.set_njc(1);

	//initialize the state vector
	setEuler(DEGTORAD(0),DEGTORAD(0),DEGTORAD(0),euler);
	setVec3(0.0, 0.0, (Wr+k6+k3), pos);

#if WMRSIM_USE_QUATERNION
	eulerToQuat(euler,orient);
#else
	copyEuler(euler,orient);
#endif
	int ns = NUMSTATE(nf); //number of elements in state
	setVec(ns,0.0,state);
	copyOrient(orient,state+SI_ORIENT);
	copyVec3(pos,state+SI_POS);

	//initialize the joint space velocity
	if (qvel != 0) { //not null
		
		//setVec(mdl.get_na(), 0.0, qvel); //to zeros

		//to cmd
		Real u[WmrModel::MAXNA];
		mdl.controller(mdl, 0.0, state, u, qvel);
	}

}

void warthogController(const WmrModel& mdl, const Real time, const Real state[], //inputs
					Real u[], Real qvel_cmd[]) { //outputs

	//options

	//Dimensions
	//make sure these match
    Real k1 = 0;		            //forward offset between (R)over reference to (D)ifferential
    Real k2 = 0.5826;	            //horizontal offset between R and D
    Real k3 = 0.24979 + 0.27218;	//height of R from D
    Real k4 = 0.457367;		    //forward distance between D and wheels
    Real k5 = 0;		            //horizontal distance between D and wheels
    Real k6 = 0.012977;		    //height from D to wheels

	Real L = 1.35; //length, front to rear
	Real w = k2; //half track width
	Real Wr = 0.3; //wheel radius

//	//indices of steer frames
//	int S1_fi = 2;
//	int S2_fi = 8;

	//for indexing u
	int S1_ai = 0;
	int A1_ai = 1;
	int A3_ai = 2;
	int A5_ai = 3;

	int S2_ai = 4;
	int A2_ai = 5;
	int A4_ai = 6;
	int A6_ai = 7;



	Real speed,turnrad; //commanded speed, turn radius

	speed = 0.5;
	turnrad = 1000;

	
	//if (time < 2.0) {
	//	speed = .5;
	//	turnrad = 1000;
	//} else if (time >= 2.0 && time < 6.0) {
	//	speed = .5;
	//	turnrad = -3.0;
	//} else {
	//	speed = .5;
	//	turnrad = 3.0;
	//}

	Real omega, gamma_l, gamma_r, vbl, vbr, vfl, vfr;

	if (fabs(turnrad) >= 1000) {
		omega = 0.0;

		gamma_l = 0.0;
		gamma_r = 0.0;

		vbl = speed;
		vbr = speed;
		vfl = speed;
		vfr = speed;

	} else {
		omega = speed/turnrad; //yaw rate

		//steer angles
		gamma_l = atan(L/(turnrad-w));
		gamma_r = atan(L/(turnrad+w));

		//steering limits
		Real lim = DEGTORAD(60);
		if (fabs(gamma_l) > lim)
			gamma_l = REALSIGN(gamma_l)*lim;
		if (fabs(gamma_r) > lim)
			gamma_r = REALSIGN(gamma_r)*lim;

		//wheel velocities
		//TODO, CHECK THIS!
		vbl = speed - omega*w; //back left
		vbr = speed + omega*w; //back right
		vfl = (speed - omega*w)*cos(gamma_l) + omega*L*sin(gamma_l); //front left
		vfr = (speed + omega*w)*cos(gamma_l) + omega*L*sin(gamma_l); //front right
	}

	//set u
	Real tc = .2;
//	u[S1_ai] = 1/tc*(gamma_l - state[TOSTATEI(S1_fi)]);
//	u[S2_ai] = 1/tc*(gamma_r - state[TOSTATEI(S2_fi)]);

	u[A1_ai] = vfl/Wr;
	u[A3_ai] = vbl/Wr;
	u[A5_ai] = vbl/Wr;

	u[A2_ai] = vfr/Wr;
	u[A4_ai] = vbr/Wr;
	u[A6_ai] = vbr/Wr;

	if (qvel_cmd != 0) { //not null

		//get from WmrModel
		const int nv = NUMQVEL(mdl.get_nf());
		const int na = mdl.get_na();
		const int* actframeinds = mdl.get_actframeinds();

		setVec(nv, 0.0, qvel_cmd);
		qvel_cmd[VI_ANG + 2] = omega;
		qvel_cmd[VI_LIN + 0] = speed;

		//wheel velocities
		for (int i=0; i<na; i++)
			qvel_cmd[TOQVELI(actframeinds[i])] = u[i];
	}
}


void warthogConstraints( const WmrModel& mdl, const Real jd[], const Real jr[], //inputs
	Real c[], Real Jc[], Real f[], Real df_djd[], Real df_djr[]) { //outputs

	//rocker joint indices
	const int D1_ji = 1;
	const int D2_ji = 4;

	c[0] = jd[D1_ji] + jd[D2_ji]; //angles should be equal magnitude, opposite sign
	
	
	const int nj = mdl.get_nf()-1; //number of joints
	

	//init to zeros
	setVec(nj,0.0,Jc);

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
