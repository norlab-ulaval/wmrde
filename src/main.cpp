#include <wmrde/test.h>
#include <wmrde/predictor.h>
//#include <wmrde/ode/test_ode.h>

//int main()
int main(int argc, char *argv[]) //use this for console output
{
	//in test.h
	//test_common();

	//test_linalg3();
	//test_transform();
	//test_spatial();
	//test_matrix();

	//test_surface();
	//test_updateWheelContactGeom();
	//test_updateTrackContactGeom();

	//test_stateToHT();
	//test_qvelToQdot();

//	test_wheelJacobians();
//	test_trackJacobians();
//	test_forwardVelKin();
//	test_initTerrainContact();

	//test_subtreeInertias();
	//test_jointSpaceInertia();
	//test_jointSpaceBiasForce();
	//test_forwardDyn();

//	test_simulate();


	//in test_ode.h

	//test_convertToWmrModelODE();
//	test_simulate_ODE();
//	test_benchmark();
    const Real dt_init = 0.05;
    const Real predictionLength_init = 10.0;
    const Real time_init = 0.0;
    const Real dt = 0.05;
    Predictor predictor = Predictor();

    Real u_cmd[4];
    u_cmd[0] = 10.0;
    u_cmd[1] = 10.0;
    u_cmd[2] = 10.0;
    u_cmd[3] = 10.0;

    std::array<float, predictor.MAXNY> y_array;
    std::copy(std::begin(predictor.y), std::end(predictor.y), std::begin(y_array));
    std::array<float, predictor.MAXNY> ydot_array;
    std::copy(std::begin(predictor.ydot), std::end(predictor.ydot), std::begin(ydot_array));
    std::array<float, 4> u_array;
    std::copy(std::begin(u_cmd), std::end(u_cmd), std::begin(u_array));
    std::array<float, predictor.MAXNY> pred_y_array;

    pred_y_array = predictor.predict(y_array, u_array, ydot_array);
    for (size_t i = 0; i < 12; i++) {
        std::cout << pred_y_array[i] << ' ';
    }

	
	std::cin.get();
}



