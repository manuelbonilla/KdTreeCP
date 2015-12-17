#ifndef __RRTS_SYSTEM_SINGLE_INTEGRATOR_H_
#define __RRTS_SYSTEM_SINGLE_INTEGRATOR_H_

#include <list>
#include "PQP.h"
#include "model.h"
#include "MatVec.h"

namespace SingleIntegrator {
    class region {
        int numDimensions;
    public:    
        double *center;
        double *size;
        region ();
        ~region ();
        int setNumDimensions (int numDimensionsIn);
        
    };
    class State {
        int numDimensions;
        int setNumDimensions (int numDimensions);
  public:
	    double *x;
        State ();
        ~State ();
        State (const State& stateIn);
        State& operator= (const State& stateIn);
        double& operator[] (const int i) {return x[i];}
        friend class System;
        friend class Trajectory;
    };
    class Trajectory {
        State *endState; 
        double totalVariation;  
    public:    
        Trajectory ();
        ~Trajectory ();
        Trajectory (const Trajectory& trajectoryIn);
        Trajectory& operator= (const Trajectory& trajectoryIn);
        State& getEndState () {return *endState;}
        State& getEndState () const {return *endState;}
        double evaluateCost ();
        friend class System;
    };
    class System {
        int numDimensions;
        int robotModel;
        bool IsInCollision (double *stateIn);
    public:
		State rootState;
		State goalState;
        region regionOperating;
        region regionGoal;
        std::list<region*> obstacles;
        std::list<Rlink*>  objects_inw;
        std::list<Rlink*>  links;
        System ();
        ~System ();
        int setNumDimensions (int numDimensionsIn);
        int RobotNum (int robotNumIn);
        int getRobotNum (){return robotModel;}
        int getNumDimensions () {return numDimensions;}
        State& getRootState () {return rootState;}
		State& getGoalState () {return goalState;}
		int getStateKey (State &stateIn, double *stateKey);
        bool isReachingTarget (State &stateIn);
        int sampleState (State &randomStateOut); 
		int samplegoalState   (State &randomStateOut); 
		int extendTo (State &stateFromIn, State &stateTowardsIn, 
                      Trajectory &trajectoryOut, bool &exactConnectionOut); 
        double evaluateExtensionCost (State &stateFromIn, State &stateTowardsIn, bool &exactConnectionOut);
        double evaluateCostToGo (State& stateIn);
        int getTrajectory (State& stateFromIn, State& stateToIn, std::list<double*>& trajectoryOut);
    };
}
#endif
