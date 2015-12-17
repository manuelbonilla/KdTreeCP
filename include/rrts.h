#ifndef __RRTS_H_
#define __RRTS_H_
#include "kdtree.h"
#include <list>
#include <set>
#include <vector>
namespace RRTstar {
    template<class State, class Trajectory, class System>
    class Planner;
    template<class State, class Trajectory, class System>
    class Vertex {
        Vertex *parent;
        State *state;
        std::set<Vertex*> children;
        double costFromParent;
        double costFromRoot;
        Trajectory *trajFromParent;
    public:
        Vertex ();
        ~Vertex ();    
        Vertex (const Vertex &vertexIn);
        State& getState () {return *state;}
        State& getState () const {return *state;}
        Vertex& getParent () {return *parent;}
        double getCost () {return costFromRoot;}
        friend class Planner<State,Trajectory,System>;
    };
    template<class State, class Trajectory, class System>
    class Planner {
        typedef struct kdtree KdTree;
        typedef struct kdres KdRes;
        typedef Vertex<State,Trajectory,System> vertex_t;
        int numDimensions;
        double gamma;
        double lowerBoundCost;
        vertex_t *lowerBoundVertex;
        KdTree *kdtree;
        vertex_t *root;
		vertex_t *goal;
		int insertIntoKdtree (vertex_t &vertexIn);
        int getNearestVertex (State& stateIn, vertex_t*& vertexPointerOut);    
        int getNearVertices (State& stateIn, std::vector<vertex_t*>& vectorNearVerticesOut);
        int checkUpdateBestVertex (vertex_t& vertexIn);
        vertex_t* insertTrajectory (vertex_t& vertexStartIn, Trajectory& trajectoryIn);    
        int insertTrajectory (vertex_t& vertexStartIn, Trajectory& trajectoryIn, vertex_t& vertexEndIn);
        int findBestParent (State& stateIn, std::vector<vertex_t*>& vectorNearVerticesIn,
                            vertex_t*& vertexBestOut, Trajectory& trajectoryOut, bool& exactConnection);
        int updateBranchCost (vertex_t& vertexIn, int depth);    
        int rewireVertices (vertex_t& vertexNew, std::vector<vertex_t*>& vectorNearVertices); 
    public:
        std::list<vertex_t*> listVertices;
        int numVertices;
        System *system;
        Planner ();
        ~Planner ();
        int setGamma (double gammaIn);
        int setSystem (System& system);
        vertex_t& getRootVertex ();
        int initialize ();
        int iteration ();
		int look_for_goal ();
        double getBestVertexCost () {return lowerBoundCost;}
        vertex_t& getBestVertex () {return *lowerBoundVertex;}
        int getBestTrajectory (std::list<double*>& trajectory);
    };

}

#endif
