#include <wmrde/wheelgroundcontact.h>

void setWgcParams( const Real Kp, Real params[] ) {

	Real Kd = Kp/20.0;
	Real Crr = 0; //-.05; //Coefficient, rolling resistance
	
#if WGC_MODEL_TYPE == ODE_WGC
	//odeWgc
	Real s = 1e-3; //N to kN

	Real mu = 1.0;
	//Real mu = REALNAN; //*not* REALMAX
	
	params[0]=Kp*s;
	params[1]=Kd*s; //Kd
	params[2]=mu; //mux
	params[3]=mu; //muy

	Real C = -.1*Kp;
	params[4] = C*s;
	params[5] = C*s;
	params[6] = Crr*s;

#elif WGC_MODEL_TYPE == PACEJKA_WGC
	//pacejkaWgc

	Real mu = 1.0;

	params[0] = Kp;
	params[1] = Kd;
	params[2] = mu;
	params[3] = 1.0/15.0; //Bs
	params[4] = 1.5; //Cs
	params[5] = 1.0; //Ds
	params[6] = .3; //Es
	params[7] = 100.0; //Ks
	params[8] = 8.0/75.0; //Ba
	params[9] = 1.5; //Ca
	params[10] = 1.0; //Da
	params[11] = .6; //Ea
	params[12] = 100.0; //Ka
	params[13] = Crr;

#elif WGC_MODEL_TYPE == ISHIGAMI_WGC
	params[0] = Kd;

//	params[1] = 0.18/2;
//	params[2] = 0.11;
	//rocky
	params[1] = .0650;
	params[2] = .08;

	params[3] = 0.8;
	params[4] = DEGTORAD(37.2);
	params[5] = DEGTORAD(26.4);
	params[6] = 1.37e3;
	params[7] = 8.14e5;
	params[8] = 1.0;
	params[9] = 0.4;
	params[10] = 0.15;
	params[11] = (1.6e3)*9.81;
	params[12] = 1.0;
	params[13] = .025;

#elif WGC_MODEL_TYPE == ISHIGAMI_LUT_WGC
	//ishigamiLUTWgc

	params[0] = Kd;
#endif

}

void odeWgc( const Real params[], const Vec3 vc, const Real Rw, const Real dz, //inputs
			Vec3 fw, Real J[]) { //outputs

	Real scale = 1e3; //kN to N

	
	Real Kp = params[0]*scale;
	Real Kd = params[1]*scale;
	//should be > 0
	Real mux = params[2];
	Real muy = params[3];
	//should be <= 0
	Real Cx = params[4]*scale;
	Real Cy = params[5]*scale;
	Real Crr = params[6]*scale;

	if (vc == 0) //if null
		return;

	Real vx = vc[0];
	Real vy = vc[1];
	Real vz = vc[2];

	Real fz = -Kp*dz - Kd*vz;
	if (fz < 0) fz=0; //normal force must be positive
	if (dz > 0) fz=0; //normal force must be zero if not in contact

	Real fx = Cx*vx + Crr*Rw;
	Real fy = Cy*vy;


	bool isover_x = fabs(fx) > mux*fz;
	bool isover_y = fabs(fy) > muy*fz;

	//DEBUGGING
	//isover_x = false;
	//isover_y = false;

	Real sign_fx,sign_fy;

	if (isover_x) {
		sign_fx = REALSIGN(fx);
		fx = sign_fx*mux*fz;
	}
	if (isover_y) {
		sign_fy = REALSIGN(fy);
		fy = sign_fy*muy*fz;
	}

	fw[0]=fx;
	fw[1]=fy;
	fw[2]=fz;

	if (J != 0) {
		//Jacobian of [fx fy fz] wrt [vx vy vz Rw dz]
		setMat(3,5,0.0,J);

		//d fz wrt dz,vz
		Real dfz_ddz, dfz_dvz;
		if (fz > 0) {
			dfz_ddz = -Kp;
			dfz_dvz = -Kd;
		} else {
			dfz_ddz = 0;
			dfz_dvz = 0;
		}

		J[S2I(2,4,3)] = dfz_ddz;
		J[S2I(2,2,3)] = dfz_dvz;

		//d fx,fy wrt vx,vy,Rw
		Real dfx_dvx,dfx_dRw,dfy_dvy;

		dfx_dvx = Cx;
		dfx_dRw = Crr;
		dfy_dvy = Cy;

		if (isover_x) {
			dfx_dvx = 0;
			dfx_dRw = 0;
		}
		if (isover_y) dfy_dvy = 0;

		J[S2I(0,0,3)] = dfx_dvx;
		J[S2I(0,3,3)] = dfx_dRw;
		J[S2I(1,1,3)] = dfy_dvy;

		//d fx,fy wrt dz,vz
		Real dfx_dfz,dfy_dfz;
		dfx_dfz = 0;
		dfy_dfz = 0;

		if (isover_x) dfx_dfz = sign_fx*mux;
		if (isover_y) dfy_dfz = sign_fy*muy;

		J[S2I(0,2,3)] = dfx_dfz*dfz_dvz;
		J[S2I(0,4,3)] = dfx_dfz*dfz_ddz;

		J[S2I(1,2,3)] = dfy_dfz*dfz_dvz;
		J[S2I(1,4,3)] = dfy_dfz*dfz_ddz;

	}
	
}

void pacejkaWgc( const Real params[], const Vec3 vc, const Real Rw, const Real dz, //inputs
		Vec3 fw, Real J[]) { //outputs
	Real Kp = params[0];
	Real Kd = params[1];

	Real mu = params[2];

	Real Bs = params[3];
	Real Cs = params[4];
	Real Ds = params[5];
	Real Es = params[6];
	Real Ks = params[7];

	Real Ba = params[8];
	Real Ca = params[9];
	Real Da = params[10];
	Real Ea = params[11];
	Real Ka = params[12];

	Real Crr = params[13];

	if (vc == 0) //if null
		return;

	Real vx = vc[0];
	Real vy = vc[1];
	Real vz = vc[2];

	//compute normal force
	Real fz = -Kp*dz - Kd*vz;
	if (fz < 0) fz=0; //normal force must be positive
	if (dz > 0) fz=0; //normal force must be zero if not in contact

	//convert to slip ratio and angle
	Real s,alpha,ds_dvx,ds_dRw,dalpha_dvx,dalpha_dvy,dalpha_dRw;

	calcSlip(vx, vy, Rw, 1, //inputs
		&s, &alpha, &ds_dvx, &ds_dRw, &dalpha_dvx, &dalpha_dvy, &dalpha_dRw); //outputs

	//Nicolas Comstock singularity as s==0 or alpha==0

	Real eps = 1e-10;
	if (s >= 0 && s < eps)
		s = eps;
	else if (s < 0 && s > -eps)
		s = -eps;
	if (alpha >= 0 && alpha < eps)
		alpha = eps;
	else if (alpha < 0 && alpha > -eps)
		alpha = -eps;

	//Pacejka,
	Real Fxs = Ds*sin(Cs*atan(Bs*(1-Es)*Ks*s + Es*atan(Bs*Ks*s)));
	Real temp1 = (2.0/M_PI)*alpha;
	Real Fya = Da*sin(Ca*atan(Ba*(1-Ea)*Ka*temp1 + Ea*atan(Ba*Ka*temp1)));

	//Nicolas Comstock
	Real tan_alpha = tan(alpha);
	Real temp2 = fabs(Fxs*Fya) / sqrt( (s*s)*(Fya*Fya) + (Fxs*Fxs)*(tan_alpha*tan_alpha) );

	//handle reverse
	Real Vx = vx + Rw;
	Real sgn = 1;
	if (Vx < 0)
		sgn = -1;

	Real dfx_dfz = sgn * temp2 * s * mu;
	dfx_dfz += Crr*Rw; //add rolling resistance

	Real dfy_dfz = -sgn * temp2 * tan_alpha * mu;
	Real fx = dfx_dfz * fz;
	Real fy = dfy_dfz * fz;

	fw[0] = fx;
	fw[1] = fy;
	fw[2] = fz;

	if (J != 0) { //not null
		//Jacobian of [fx fy fz] wrt [vx vy vz Rw dz]
		setMat(3,5,0.0,J);

		Real dfz_ddz = -Kp;
		Real dfz_dvz = -Kd;
		if (fz <= 0) {
			dfz_ddz = 0;
			dfz_dvz = 0;
		}

		J[S2I(2,4,3)] = dfz_ddz;
		J[S2I(2,2,3)] = dfz_dvz;

		//derivatives of fx,fy wrt abs(s) abs(alpha)
		Real dfx_ds, dfy_ds, dfx_dalpha, dfy_dalpha;
		pacejkaDerivatives(Ba,Bs,Ca,Cs,Da,Ds,Ea,Es,Ka,Ks,alpha,fz,mu,s,sgn, //inputs
			&dfx_ds, &dfy_ds, &dfx_dalpha, &dfy_dalpha); //outputs

		//d fx
		J[S2I(0,0,3)] = dfx_ds*ds_dvx + dfx_dalpha*dalpha_dvx;
		J[S2I(0,1,3)] = dfx_dalpha*dalpha_dvy;
		J[S2I(0,2,3)] = dfx_dfz*dfz_dvz;
		J[S2I(0,3,3)] = dfx_ds*ds_dRw + dfx_dalpha*dalpha_dRw + Crr*fz;
		J[S2I(0,4,3)] = dfx_dfz*dfz_ddz;

		//d fy
		J[S2I(1,0,3)] = dfy_ds*ds_dvx + dfy_dalpha*dalpha_dvx;
		J[S2I(1,1,3)] = dfy_dalpha*dalpha_dvy;
		J[S2I(1,2,3)] = dfy_dfz*dfz_dvz;
		J[S2I(1,3,3)] = dfy_ds*ds_dRw + dfy_dalpha*dalpha_dRw;
		J[S2I(1,4,3)] = dfy_dfz*dfz_ddz;

	}

	//DEBUGGING
	//std::cout << "fw=\n"; printVec3(fw,-1,-1);
	//std::cout << "J=\n"; printMatReal(3,5,J,-1,-1);
}



inline double trapz (const int n, const double x[], const double dy_dx[])
{
  double y = 0;
  for (int i=1; i<n; i++)
  {
    y += .5*(dy_dx[i] + dy_dx[i+1]) * (x[i] - x[i-1]);
  }
  return y;
}

void ishigamiWgcSubfn( const Real params[], const Vec3 vc, const Real Rw, const Real dz, //inputs
    Vec3 fw) { //outputs

  //parse params
  double Kd = params[0];
  double r = params[1];
  double b = params[2];
  double c = params[3];
  double phi = params[4];
  double Xc = params[5];
  double kc = params[6];
  double kphi = params[7];
  double n = params[8];
  double a0 = params[9];
  double a1 = params[10];
  double rhod = params[11];
  double lambda = params[12];
  double K = params[13];

  if (dz > 0)
  {
    //TODO
  }

  double vx = vc[0];
  double vy = vc[1];
  double vz = vc[2];

  const int np = 61;
  //convert to slip ratio and angle
  double s,beta;
  calcSlip(vx,vy,Rw,2, &s,&beta,0,0,0,0,0);

  //entry and exit angles
  double h = -dz;
  if (h < 0) h = 0;
  double theta_f = acos(1.0 - h/r);
  double theta_r = -acos(1.0- (lambda/r)*h);

  //discretize angles
  double theta[np];
  linspace(theta_r, theta_f, np, theta);

  //precompute
  double ctheta[np];
  double stheta[np];
  for (int i=0; i<np; i++)
  {
    ctheta[i] = cos(theta[i]);
    stheta[i] = sin(theta[i]);
  }

  //normal stress distribution
  double sigma[np];
  double theta_m = (a0 + a1*s) * theta_f;
  double temp1 = pow(r,n) * (kc/b + kphi);
  double temp2 = (theta_f - theta_m)/(theta_m - theta_r);

  for (int i=0; i<np; i++)
  {
    if (theta[i] >= theta_m) //front
      sigma[i] = temp1*pow(ctheta[i] - cos(theta_f), n);
    else
      sigma[i] = temp1*pow(cos(theta_f - (theta[i] - theta_r)*temp2) - cos(theta_f),n);
  }

  //soil deformations
  double jx[np];
  double jy[np];

  //shear stress distributions
  double taux[np];
  double tauy[np];

  for (int i=0; i<np; i++)
  {
    jx[i] = r*( theta_f - theta[i] - (1-s)*(sin(theta_f) - stheta[i]) );
    jy[i] = -r*(1-fabs(s))*(theta_f-theta[i])*tan(beta); //symmetric

    double jnorm = sqrt(jx[i]*jx[i] + jy[i]*jy[i]);
    double tau = (c+sigma[i]*tan(phi)) * (1-exp(-jnorm/K));

    if (jnorm > 0.0)
    {
      taux[i] = tau*jx[i]/jnorm;
      tauy[i] = tau*jy[i]/jnorm;
    }
    else
    {
      taux[i] = 0;
      tauy[i] = 0;
    }
  }

  //TODO, bulldozing reaction force in y direction

  //integrate to compute forces
  double dfx_[np];
  double dfz_[np];

  for (int i=0; i<np; i++)
  {
    dfx_[i] = taux[i]*ctheta[i] - sigma[i]*stheta[i];
    dfz_[i] = taux[i]*stheta[i] + sigma[i]*ctheta[i];
  }
  double fx = r*b*trapz(np, theta, dfx_);
  double fy = r*b*trapz(np, theta, tauy);
  double fz = r*b*trapz(np, theta, dfz_);

  //add damping to normal force
  fz = fz - Kd*vz;

  if (fz < 0) fz = 0; //normal force must be positive

  //reverse sign for reverse
  //TODO, check this
  double Vx = vx + Rw;
  if (Rw < 0) fx = -fx;
  if (Vx < 0) fy = -fy;

  setVec3(fx,fy,fz, fw);
}

void ishigamiWgc( const Real params[], const Vec3 vc, const Real Rw, const Real dz, //inputs
    Vec3 fw, Real J[]) { //outputs

  if (vc == 0) //if null
    return;

  ishigamiWgcSubfn(params,vc,Rw,dz, fw);

  //compute Jacobian using finite differences
  //TODO, move this to function, use for pacejkaWgc too
  setMat(3,5,0.0,J);

  double fw_[3];
  double eps = 1e-6;

  for (int ci=0; ci<5; ci++)
  {
    double vc_[3]; copyVec3(vc,vc_);
    double Rw_ = Rw;
    double dz_ = dz;

    if (ci == 0) vc_[0] += eps;
    else if (ci == 1) vc_[1] += eps;
    else if (ci == 2) vc_[2] += eps;
    else if (ci == 3) Rw_ += eps;
    else if (ci == 4) dz_ += eps;

    ishigamiWgcSubfn(params,vc_,Rw_,dz_, fw_);

    for (int ri=0; ri<3; ri++)
      J[S2I(ri,ci,3)] = (fw_[ri] - fw[ri])/eps;
  }
}

void ishigamiLUTWgc( const Real params[], const Vec3 vc, const Real Rw, const Real dz, //inputs
		Vec3 fw, Real J[]) { //outputs

	const int buffer = 101*101*51+1;

	//lookup table data
	static bool init = false;
	static Real lowerlim[3];
	static Real upperlim[3];
	static int n[3];
	static Real d[3];
	static Real FX[buffer];
	static Real FY[buffer];
	static Real FZ[buffer];
	Real* F[3] = {FX,FY,FZ}; //pointers

	if (!init) {
		//load lookup table data from .txt files
		//these files can be generated using MATLAB, ishigamiLUTWgc.m

	  std::string FileNames[3];
	  LUTFilenames( FileNames );

		for (int fileno=0; fileno<3; fileno++) {

			std::ifstream file ( (ResourceDir() + FileNames[fileno]).c_str() );

			std::string value;
			if ( file.is_open() ) {
				for (int i=0; i<3; i++) {
					std::getline (file, value, ','); lowerlim[i] = (Real) atof(value.c_str());
					std::getline (file, value, ','); upperlim[i] = (Real) atof(value.c_str());
					std::getline (file, value, ','); n[i] = atoi(value.c_str());

					assert(upperlim[i] > lowerlim[i]);
					d[i] = (upperlim[i]-lowerlim[i])/(n[i]-1);
				}

				int i=0;
				while ( file.good() ) {
					assert( i < buffer ); //check if buffer exceeded
					std::getline ( file, value, ',' ); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/
					F[fileno][i] = (Real) atof(value.c_str());
					i++;
				}
				file.close();
	
				assert(i == (n[0]*n[1]*n[2])+1); //check if correct number of elements in Z

				init = true;

			} else {
				//TODO
			}
		}
	}

	if (vc == 0) //if null
		return;

	Real vx = vc[0];
	Real vy = vc[1];
	Real vz = vc[2];

	Real Kd = params[0];

	int niter = 1;
	if (J != 0) {
		niter = 5;
		setMat(3,5,0.0,J);
	}
	int cols[5] = {-1, 0, 1, 3, 4}; //columns of Jacobian for each iteration

	for (int iter=0; iter<niter; iter++) {

		Real vx_ = vx;
		Real vy_ = vy;
		Real Rw_ = Rw;
		Real dz_ = dz;

		//for numerical diff.
		Real eps = 1e-4;
		if (iter == 1)
			vx_ += eps;
		else if (iter == 2)
			vy_ += eps;
		else if (iter == 3)
			Rw_ += eps;
		else if (iter == 4)
			dz_ += eps;

		//convert to slip ratio and angle
		Real s,beta;
		calcSlip(vx_, vy_, Rw_, 2, //inputs
			&s, &beta, 0, 0, 0, 0, 0); //outputs

		if (s > upperlim[0])
			s = upperlim[0];
		if (s < lowerlim[0])
			s = lowerlim[0];
		if (beta > upperlim[1])
			beta = upperlim[1];
		if (beta < lowerlim[1])
			beta = lowerlim[1];
		if (dz_ > upperlim[2])
			dz_ = upperlim[2];
		if (dz_ < lowerlim[2])
			dz_ = lowerlim[2];

		//interpolate
		Vec3 fw_;
		Real x[3] = {s,beta,dz_};
		int I[3];
		Real xn[3],f[8];
		trilinearInterp_xn(lowerlim, d, x, I, xn);

		for (int i=0; i<3; i++) {
			trilinearInterp_f(F[i], I, n, f);
			fw_[i] = trilinearInterp(f,xn);
		}

		Real fx = fw_[0];
		Real fy = fw_[1];
		Real fz = fw_[2];

		//add damping to normal force
		fz -= Kd*vz;
		//normal force must be positive
		if (fz < 0)
			fz = 0;

		//reverse sign for reverse, ?
		Real Vx = vx_ + Rw_;
		if (Rw_ < 0)
			fx = -fx;
		if (Vx < 0)
			fy = -fy;

		if (iter == 0) {
			fw[0] = fx;
			fw[1] = fy;
			fw[2] = fz;
		} else {
			int col = cols[iter];
			J[S2I(0,col,3)] = (fx - fw[0])/eps;
			J[S2I(1,col,3)] = (fy - fw[1])/eps;
			J[S2I(2,col,3)] = (fz - fw[2])/eps;

			if (fz > 0)
				J[S2I(2,2,3)] = -Kd;
		}
	}

	//DEBUGGING
	//std::cout << "fw=\n"; printVec3(fw,-1,-1);
	//std::cout << "J=\n"; printMatReal(3,5,J,-1,-1);

}

void pacejkaDerivatives(const Real Ba, const Real Bs, const Real Ca, const Real Cs, const Real Da, const Real Ds, const Real Ea, const Real Es, const Real Ka, const Real Ks, const Real alpha, const Real fz, const Real mu, const Real s, const Real sgn, //inputs
			Real* dfx_ds, Real* dfy_ds, Real* dfx_dalpha, Real* dfy_dalpha) { //outputs

	//code generated using MATLAB symbolic toolbox
	//sym_pacejka_wgc.m

	Real t2 = 1.0/M_PI;
	Real t3 = Ba*Ka*alpha*t2*2.0;
	Real t4 = atan(t3);
	Real t5 = Ea*t4;
	Real t6 = Ea-1.0;
	Real t21 = Ba*Ka*alpha*t2*t6*2.0;
	Real t7 = t5-t21;
	Real t8 = atan(t7);
	Real t9 = Ca*t8;
	Real t10 = sin(t9);
	Real t11 = Bs*Ks*s;
	Real t12 = atan(t11);
	Real t13 = Es*t12;
	Real t14 = Es-1.0;
	Real t20 = Bs*Ks*s*t14;
	Real t15 = t13-t20;
	Real t16 = atan(t15);
	Real t17 = Cs*t16;
	Real t18 = sin(t17);
	Real t19 = tan(alpha);
	Real t22 = Da*Ds*t10*t18;
	Real t23 = fabs(t22);
	Real t24 = Da*Da;
	Real t25 = s*s;
	Real t26 = t10*t10;
	Real t27 = t24*t25*t26;
	Real t28 = Ds*Ds;
	Real t29 = t18*t18;
	Real t30 = t19*t19;
	Real t31 = t28*t29*t30;
	Real t32 = t27+t31;
	Real t33 = cos(t17);
	Real t34 = Bs*Ks*t14;
	Real t35 = Bs*Bs;
	Real t36 = Ks*Ks;
	Real t37 = t25*t35*t36;
	Real t38 = t37+1.0;
	Real t39 = 1.0/t38;
	Real t47 = Bs*Es*Ks*t39;
	Real t40 = t34-t47;
	Real t41 = t15*t15;
	Real t42 = t41+1.0;
	Real t43 = 1.0/t42;
	Real t44 = 1.0/sqrt(t32);
	Real t45 = 1.0/pow(t32,3.0/2.0);
	Real t46 = s*t24*t26*2.0;
	Real t48 = t46-Cs*t18*t28*t30*t33*t40*t43*2.0;
	Real t49 = (t22/fabs(t22));
	Real t50 = cos(t9);
	Real t51 = Ba*Ka*t2*t6*2.0;
	Real t52 = Ba*Ba;
	Real t53 = Ka*Ka;
	Real t54 = 1.0/(M_PI*M_PI);
	Real t55 = alpha*alpha;
	Real t56 = t52*t53*t54*t55*4.0;
	Real t57 = t56+1.0;
	Real t58 = 1.0/t57;
	Real t65 = Ba*Ea*Ka*t2*t58*2.0;
	Real t59 = t51-t65;
	Real t60 = t7*t7;
	Real t61 = t60+1.0;
	Real t62 = 1.0/t61;
	Real t63 = t30+1.0;
	Real t64 = t19*t28*t29*t63*2.0;
	Real t66 = t64-Ca*t10*t24*t25*t50*t59*t62*2.0;

	*dfx_ds = fz*mu*sgn*t23*t44-fz*mu*s*sgn*t23*t45*t48*(1.0/2.0)-Cs*Da*Ds*fz*mu*s*sgn*t10*t33*t40*t43*t44*t49;
	*dfy_ds = fz*mu*sgn*t19*t23*t45*t48*(1.0/2.0)+Cs*Da*Ds*fz*mu*sgn*t10*t19*t33*t40*t43*t44*t49;
	*dfx_dalpha = fz*mu*s*sgn*t23*t45*t66*(-1.0/2.0)-Ca*Da*Ds*fz*mu*s*sgn*t18*t44*t49*t50*t59*t62;
	*dfy_dalpha = -fz*mu*sgn*t23*t44*t63+fz*mu*sgn*t19*t23*t45*t66*(1.0/2.0)+Ca*Da*Ds*fz*mu*sgn*t18*t19*t44*t49*t50*t59*t62;
}

void calcSlip( const Real vx, const Real vy, const Real Rw, const int method, //inputs
		Real* s, Real* alpha, Real* ds_dvx, Real* ds_dRw, Real* dalpha_dvx, Real* dalpha_dvy, Real* dalpha_dRw) { //outputs

	//method:		{1,2} determines equation for slip ratio


	Real Vx = vx + Rw;

	//threshold away from 0
	Real tol = 1e-3;
	if (Vx < tol && Vx > 0)
		Vx = tol;
	else if (Vx > -tol && Vx < 0)
		Vx = -tol;

	//slip ratio
	if (method == 1)
		*s = -vx/Vx;
	else if (method == 2)
		*s = -vx/Rw;

	//slip angle
	Real tan_alpha = vy/Vx;
	*alpha = atan(tan_alpha);

	if (ds_dvx != 0) { //not null
		Real Vx2 = Vx*Vx;

		if (method == 1) {
			*ds_dvx = -1/Vx + vx/Vx2;
			*ds_dRw = vx/Vx2;
		} else if (method == 2) {
			*ds_dvx = -1/Rw;
			*ds_dRw = vx/(Rw*Rw);
		}

		Real temp1 = 1/(1 + tan_alpha*tan_alpha); //d/dx atan(x) = 1/(1 + x^2)

		*dalpha_dvx = temp1*(-vy/Vx2);
		*dalpha_dvy = temp1/Vx;
		*dalpha_dRw = *dalpha_dvx;
	}

}
