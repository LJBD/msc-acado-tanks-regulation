//============================================================================
// Name        : masthe-acado-tanks-optimisation.cpp
// Author      : Lukasz Dudek
// Version     :
// Copyright   : LGPL
// Description : Hello World in C, Ansi-style
//============================================================================

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO

	Logger::instance().setLogLevel( LVL_DEBUG );


    // INTRODUCE THE VARIABLES:
    // -------------------------

    DifferentialState     s,v,m;
    Control               u    ;
    Parameter             T    ;

    DifferentialEquation  f( 0.0, T );


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(s) == v;
    f << dot(v) == (u-0.2*v*v)/m;
    f << dot(m) == -0.01*u*u;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0, T, 20 );

    ocp.minimizeMayerTerm( T );
    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, s ==  0.0 );
    ocp.subjectTo( AT_START, v ==  0.0 );
    ocp.subjectTo( AT_START, m ==  1.0 );

    ocp.subjectTo( AT_END  , s == 10.0 );
    ocp.subjectTo( AT_END  , v ==  0.0 );

    ocp.subjectTo( -0.1 <= v <=  1.7  );
    ocp.subjectTo( -1.1 <= u <=  1.1  );
    ocp.subjectTo(  5.0 <= T <= 15.0  );


    // VISUALIZE THE RESULTS IN A GNUPLOT WINDOW:
    // ------------------------------------------
    GnuplotWindow window;
        window.addSubplot( s, "THE DISTANCE s"      );
        window.addSubplot( v, "THE VELOCITY v"      );
        window.addSubplot( m, "THE MASS m"          );
        window.addSubplot( u, "THE CONTROL INPUT u" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( MAX_NUM_ITERATIONS, 20 );
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

