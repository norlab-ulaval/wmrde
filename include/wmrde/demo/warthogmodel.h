#ifndef _WMRDE_WARTHOGMODEL_H_
#define _WMRDE_WARTHOGMODEL_H_

#include <wmrde/state.h>
#include <wmrde/algebra/matrix.h>
#include <wmrde/wheelgroundcontact.h>
#include <wmrde/actuator.h>

//& to pass object by reference
void warthog(WmrModel& mdl, Real state[], Real qvel[]);

void warthogController(const WmrModel& mdl, const Real time, const Real state[], //inputs
		Real u[], Real qvel_cmd[]); //outputs

inline void warthogAct( const Real params[], const Real ucmd[], const Real u[], const Real interr[], //inputs
		Real f[], Real err[], Real* dfdu) {

	const int na = 2; //2 independently actuated wheels
	setMat(na,na,0.0,dfdu);
	for (int i=0; i<na; i++)
		PIact( params, ucmd[i], u[i], interr[i], //inputs
				f[i], err[i], dfdu[S2I(i,i,na)]); //outputs
}

void warthogConstraints( const WmrModel& mdl, const Real jd[], const Real jr[], //inputs
		Real c[], Real Jc[], Real f[], Real df_djd[], Real df_djr[]); //outputs

#endif
