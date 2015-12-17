#include "system_single_integrator.h"
#include "PQP.h"
#include <cstdlib>
#include <iostream>
#include "model.h"
#include <stdio.h>
#include "MatVec.h"

using namespace std;
using namespace SingleIntegrator;

#define DISCRETIZATION_STEP 0.01
#define PI 3.141592653589793

region::region () {

  numDimensions = 0;

  center = NULL;
  size = NULL;
}

region::~region () {

  if (center)
    delete [] center;
  if (size)
    delete [] size;

}

int region::setNumDimensions (int numDimensionsIn) {

  numDimensions = numDimensionsIn;

  if (center)
    delete [] center;
  center = new double[numDimensions];

  if (size)
    delete [] size;
  size = new double[numDimensions];

  return 1;

}

State::State () {

  numDimensions = 0;

  x = NULL;
}

State::~State () {

  if (x)
    delete [] x;
}

State::State (const State &stateIn) {

  numDimensions = stateIn.numDimensions;

  if (numDimensions > 0) {
    x = new double[numDimensions];

    for (int i = 0; i < numDimensions; i++)
      x[i] = stateIn.x[i];
  }
  else {
    x = NULL;
  }
}

State& State::operator=(const State &stateIn){

  if (this == &stateIn)
    return *this;

  if (numDimensions != stateIn.numDimensions) {
    if (x)
      delete [] x;
    numDimensions = stateIn.numDimensions;
    if (numDimensions > 0)
      x = new double[numDimensions];
  }

  for (int i = 0; i < numDimensions; i++)
    x[i] = stateIn.x[i];

  return *this;
}

int State::setNumDimensions (int numDimensionsIn) {

  if (x)
    delete [] x;

  if (numDimensions < 0)
    return 0;

  numDimensions = numDimensionsIn;

  if (numDimensions > 0)
    x = new double[numDimensions];

  return 1;
}

Trajectory::Trajectory () {

  endState = NULL;
}

Trajectory::~Trajectory () {

  if (endState)
    delete endState;
}

Trajectory::Trajectory (const Trajectory &trajectoryIn) {

  endState = new State (trajectoryIn.getEndState());

}

Trajectory& Trajectory::operator=(const Trajectory &trajectoryIn) {

  if (this == &trajectoryIn)
    return *this;

  if (endState)
    delete endState;


  endState = new State (trajectoryIn.getEndState());

  totalVariation = trajectoryIn.totalVariation;

  return *this;
}

double Trajectory::evaluateCost () {

  return totalVariation;
}

System::System () {

  numDimensions = 0;
}

System::~System () {

}

int System::setNumDimensions (int numDimensionsIn) {

  if (numDimensions < 0)
    return 0;

  numDimensions = numDimensionsIn;

  rootState.setNumDimensions (numDimensions);
  goalState.setNumDimensions (numDimensions);

  return 1;
}

int System::RobotNum (int robotNumIn){

  robotModel = robotNumIn;

  return 1;

}

int System::getStateKey (State& stateIn, double* stateKey) {

  for (int i = 0; i < numDimensions; i++)
    stateKey[i] =  stateIn.x[i] / regionOperating.size[i];

  return 1;
}

bool System::isReachingTarget (State &stateIn) {

  for (int i = 0; i < numDimensions; i++) {

    if (fabs(stateIn.x[i] - goalState.x[i]) > regionGoal.size[i]/2.0 )
      return false;
  }

  return true;

}

bool System::IsInCollision (double *stateIn) {

  bool des = false;

  int robotnumber;

  robotnumber = getRobotNum();

  switch (robotnumber)
  {
    case 1:{
      Rlink Mtemp, M12, M23, M34, M45, M56, M67, Mbase;

      list<Rlink*>::iterator itero = objects_inw.begin();
      list<Rlink*>::iterator iterl = links.begin();
      Rlink *objcts = *itero;

      MDH(objcts, 0.0, 0.0, 0.0, 0.0);
      objcts->T[0] = 0.0;
      objcts->T[1] = 1.0;
      objcts->T[2] = -0.1;

      Rlink *lks0 = *iterl;

      MDH(lks0, 	-PI/2, 	0.0, 	0.0,   		0.0);
      MDH(&Mbase, 	-PI/2, 	0.0, 	0.0,   		0.0);
      MDH(&Mtemp,  	PI/2, 	0.0,  stateIn[0], 	0.310);
      MDH(&M12,   	-PI/2, 	0.0,  stateIn[1],   0.0);
      MDH(&M23,    	PI/2, 	0.0,  stateIn[2], 	0.400);
      MDH(&M34,   	-PI/2, 	0.0,  stateIn[3],   0.0);
      MDH(&M45,		PI/2, 	0.0,  stateIn[4], 	0.390);
      MDH(&M56,   	-PI/2, 	0.0,  stateIn[5],   0.0);
      MDH(&M67,		0.0, 	0.0,  stateIn[6],  	0.078);

      iterl++;
      Rlink *lks1 = *iterl;
      MxM4(lks1,Mbase.HT,Mtemp.HT);
      iterl++;
      Rlink *lks2 = *iterl;
      MxM4(lks2,lks1->HT,M12.HT);
      iterl++;
      Rlink *lks3 = *iterl;
      MxM4(lks3,lks2->HT,M23.HT);
      iterl++;
      Rlink *lks4 = *iterl;
      MxM4(lks4,lks3->HT,M34.HT);
      iterl++;
      Rlink *lks5 = *iterl;
      MxM4(lks5,lks4->HT,M45.HT);
      iterl++;
      Rlink *lks6 = *iterl;
      MxM4(lks6,lks5->HT,M56.HT);
      iterl++;
      Rlink *lks7 = *iterl;
      MxM4(lks7,lks6->HT,M67.HT);
	}
    break;
    case 2:{
      Rlink Mtemp, M12, M23, M34, M45, M56, M67, M78, M78T, M89, M89T, M910, M910T, Mbase;

      list<Rlink*>::iterator itero = objects_inw.begin();
      list<Rlink*>::iterator iterl = links.begin();
      Rlink *objcts1 = *itero;

	  MDH(objcts1, -PI/2,0.0,0.0,0.0);
      objcts1->T[0] = 0.0;
      objcts1->T[1] = 0.0;
      objcts1->T[2] = 0.6;

//	  itero++;
//	  Rlink *objcts2 = *itero;
//
//	  MDH(objcts2, -PI/2,0.0,0.0,0.0);
//      objcts2->T[0] = 0.0;
//      objcts2->T[1] = 0.98;
//      objcts2->T[2] = 0.76;

      Rlink *lks0 = *iterl;

      MDH(lks0,   -PI/2, 	0.0, 	0.0,   	0.0);
      MDH(&Mbase, -PI/2, 	0.0, 	0.0,   	0.0);
      MDH(&Mtemp,  PI/2, 0.0,  stateIn[0], 0.310);
      MDH(&M12,   -PI/2, 0.0,  stateIn[1],   0.0);
      MDH(&M23,    PI/2, 0.0,  stateIn[2], 0.400);
      MDH(&M34,   -PI/2, 0.0,  stateIn[3],   0.0);
      MDH(&M45,    PI/2, 0.0,  stateIn[4], 0.390);
      MDH(&M56,   -PI/2, 0.0,  stateIn[5],   0.0);
      MDH(&M67,     0.0, 0.0,  stateIn[6], 0.078);


      MDH(&M78,   -PI/2, 0.03325,  0.0, 0.0835);
      MDH(&M89,     0.0, 0.145,  stateIn[7], 0.0);
      MDH(&M910,   PI/2, 0.150,  stateIn[8], 0.0);

      MDH(&M78T,   -PI/2, 0.03325,  PI, 0.0835);
      MDH(&M89T,     0.0, 0.145,  stateIn[9], 0.0);
      MDH(&M910T,   PI/2, 0.150,  stateIn[10], 0.0);

      iterl++;
      Rlink *lks1 = *iterl;
      MxM4(lks1,Mbase.HT,Mtemp.HT);
      iterl++;
      Rlink *lks2 = *iterl;
      MxM4(lks2,lks1->HT,M12.HT);
      iterl++;
      Rlink *lks3 = *iterl;
      MxM4(lks3,lks2->HT,M23.HT);
      iterl++;
      Rlink *lks4 = *iterl;
      MxM4(lks4,lks3->HT,M34.HT);
      iterl++;
      Rlink *lks5 = *iterl;
      MxM4(lks5,lks4->HT,M45.HT);
      iterl++;
      Rlink *lks6 = *iterl;
      MxM4(lks6,lks5->HT,M56.HT);
      iterl++;
      Rlink *lks7 = *iterl;
      MxM4(lks7,lks6->HT,M67.HT);

	  iterl++;
	  Rlink *lks8 = *iterl;
	  MxM4(lks8,lks7->HT,M78.HT);

	  iterl++;
	  Rlink *lks11 = *iterl;
	  MxM4(lks11,lks7->HT,M78T.HT);


	  M89.HT[0][3] = M89.HT[0][3] +.018*sin(stateIn[7]);
	  M89.HT[1][3] = M89.HT[1][3] -.018*cos(stateIn[7]);
	  M89.T[0] = M89.T[0] +.018*sin(stateIn[7]);
	  M89.T[1] = M89.T[1] -.018*cos(stateIn[7]);
	  iterl++;
	  Rlink *lks9 = *iterl;
	  MxM4(lks9,lks8->HT,M89.HT);
	  iterl++;
	  Rlink *lks10 = *iterl;
	  MxM4(lks10,lks9->HT,M910.HT);

	  M89T.HT[0][3] = M89T.HT[0][3] +.018*sin(stateIn[9]);
	  M89T.HT[1][3] = M89T.HT[1][3] -.018*cos(stateIn[9]);
	  M89T.T[0] = M89T.T[0] +.018*sin(stateIn[9]);
	  M89T.T[1] = M89T.T[1] -.018*cos(stateIn[9]);
	  iterl++;
	  Rlink *lks12 = *iterl;
	  MxM4(lks12,lks11->HT,M89T.HT);
	  iterl++;
	  Rlink *lks13 = *iterl;
	  MxM4(lks13,lks12->HT,M910T.HT);
	}
    break;
  }
PQP_CollideResult cres;
for (list<Rlink*>::iterator iter=links.begin(); iter!=links.end(); ++iter){
	Rlink *linkCurr = *iter;
	for(list<Rlink*>::iterator iter2 = objects_inw.begin(); iter2!=objects_inw.end(); ++iter2){
	  Rlink *obstacleCurr = *iter2;
      PQP_Collide(&cres,linkCurr->R,linkCurr->T,&linkCurr->link, obstacleCurr->R,obstacleCurr->T,&obstacleCurr->link, PQP_FIRST_CONTACT);
      if (cres.pairs>0){
//	printf("collision\n");
		des = true;
		break;
      }
	}
    if(des)
      break;
  }


 return des;
}

int System::sampleState (State &randomStateOut) {

  randomStateOut.setNumDimensions (numDimensions);

  for (int i = 0; i < numDimensions; i++) {

    randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i]
      - regionOperating.size[i]/2.0 + regionOperating.center[i];
  }

  if (IsInCollision (randomStateOut.x))
    return 0;

  return 1;
}


int System::samplegoalState (State &randomStateOut) {

  randomStateOut.setNumDimensions (numDimensions);

  for (int i = 0; i < numDimensions; i++) {

    randomStateOut.x[i] = goalState[i];
  }

  if (IsInCollision (randomStateOut.x))
    return 0;

  return 1;
}



int System::extendTo (State &stateFromIn, State &stateTowardsIn, Trajectory &trajectoryOut, bool &exactConnectionOut) {
  //printf("Desde,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", stateFromIn.x[0],stateFromIn.x[1],stateFromIn.x[2],stateFromIn.x[3],stateFromIn.x[4],stateFromIn.x[5],stateFromIn.x[6]);
  //printf("Hasta,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", stateTowardsIn.x[0],stateTowardsIn.x[1],stateTowardsIn.x[2],stateTowardsIn.x[3],stateTowardsIn.x[4],stateTowardsIn.x[5],stateTowardsIn.x[6]);

  double *dists = new double[numDimensions];
  for (int i = 0; i < numDimensions; i++)
    dists[i] = stateTowardsIn.x[i] - stateFromIn.x[i];

  double distTotal = 0.0;
  for (int i = 0; i < numDimensions; i++)
    distTotal += dists[i]*dists[i];
  distTotal = sqrt (distTotal);

  double incrementTotal = distTotal/DISCRETIZATION_STEP;

  // normalize the distance according to the disretization step
  for (int i = 0; i < numDimensions; i++)
    dists[i] /= incrementTotal;

  int numSegments = (int)floor(incrementTotal);

  double *stateCurr = new double[numDimensions];
  for (int i = 0; i < numDimensions; i++)
    stateCurr[i] = stateFromIn.x[i];

  for (int i = 0; i < numSegments; i++) {

    if (IsInCollision (stateCurr))
      return 0;

    for (int i = 0; i < numDimensions; i++)
      stateCurr[i] += dists[i];
  }

  if (IsInCollision (stateTowardsIn.x))
    return 0;

  trajectoryOut.endState = new State (stateTowardsIn);
  trajectoryOut.totalVariation = distTotal;

  delete [] dists;
  delete [] stateCurr;

  exactConnectionOut = true;

  return 1;
}

double System::evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool &exactConnectionOut) {


  exactConnectionOut = true;

  double distTotal = 0.0;
  for (int i = 0; i < numDimensions; i++) {
    //double distCurr = stateTowardsIn.x[i] - stateFromIn.x[i];
	double distCurr = fabs(stateTowardsIn.x[i] - stateFromIn.x[i]);
	distTotal += distCurr;
    //distTotal += distCurr*distCurr;
  }

  return distTotal;
  //return sqrt(distTotal);

}

int System::getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut) {

  double *stateArr = new double[numDimensions];
  for (int i = 0; i < numDimensions; i++)
    stateArr[i] = stateToIn[i];
  trajectoryOut.push_front (stateArr);

  return 1;

}

double System::evaluateCostToGo (State& stateIn) {

  double radius = 0.0;
//  for (int i = 0; i < numDimensions; i++)
//    radius += regionGoal.size[i] * regionGoal.size[i];
//  radius = sqrt(radius);
//
  double dist = 0.0;
  for (int i = 0; i < numDimensions; i++)
    dist += fabs(stateIn[i] - goalState.x[i]);
	//dist += (stateIn[i] - goalState.x[i])*(stateIn[i] - goalState.x[i];
  //dist = sqrt(dist);

  return dist - radius;
}
