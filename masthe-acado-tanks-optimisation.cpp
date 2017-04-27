//============================================================================
// Name        : masthe-acado-tanks-optimisation.cpp
// Author      : Lukasz Dudek
// Version     :
// Copyright   : LGPL
// Description : Hello World in C, Ansi-style
//============================================================================

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include "tanks_constants.hpp"


int main( ){

    USING_NAMESPACE_ACADO

	Logger::instance().setLogLevel( LVL_DEBUG );


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState h1, h2, h3;
    Control               u    ;
    Parameter             T    ;

    DifferentialEquation  f( 0.0, T );


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    f << dot(h1) == (u - C1*sqrt(h1))/a*w;
    f << dot(h2) == (C1*sqrt(h1) - C2*sqrt(h2))/(c*w + h2*b*w/h_max);
    f << dot(h3) == (C2*sqrt(h2) - C3*sqrt(h3))/(w*sqrt(pow(R, 2) - pow((R - h3), 2)));


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0, T, 50 );

    ocp.minimizeMayerTerm( T );
    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, h1 ==  h10 );
    ocp.subjectTo( AT_START, h2 ==  h20 );
    ocp.subjectTo( AT_START, h3 ==  h30 );

    ocp.subjectTo( AT_END  , h1 == h1_final);
    ocp.subjectTo( AT_END  , h2 == h2_final);
    ocp.subjectTo( AT_END  , h3 == h3_final);

    ocp.subjectTo( 0 <= h1 <=  h_max  );
    ocp.subjectTo( 0 <= h2 <=  h_max  );
    ocp.subjectTo( 0.001 <= h3 <=  h_max  ); //to avoid hitting singularity
    ocp.subjectTo( 0 <= u <=  u_max  );
//    ocp.subjectTo(  5.0 <= T <= 15.0  );


    // VISUALIZE THE RESULTS IN A GNUPLOT WINDOW:
    // ------------------------------------------
    GnuplotWindow window;
        window.addSubplot( h1, "1st tank level");
        window.addSubplot( h2, "2nd tank level");
        window.addSubplot( h3, "3rd tank level");
        window.addSubplot( u, "THE CONTROL INPUT u" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( MAX_NUM_ITERATIONS, 200 );
// 	algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
// 	algorithm.set( HESSIAN_PROJECTION_FACTOR, 1.0 );

    algorithm << window;


//     algorithm.initializeDifferentialStates("tor_states.txt");
//     algorithm.initializeParameters("tor_pars.txt");
//     algorithm.initializeControls("tor_controls.txt");

    algorithm.solve();

//     algorithm.getDifferentialStates("tor_states.txt");
//     algorithm.getParameters("tor_pars.txt");
//     algorithm.getControls("tor_controls.txt");

    return 0;
}

