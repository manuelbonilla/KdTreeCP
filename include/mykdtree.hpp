#ifndef MYKDTREE_HPP_INCLUDED
#define MYKDTREE_HPP_INCLUDED

#include <iostream>
#include <vector>
#include <algorithm>
#include <stack> 
#include <random>

template < class T_environment, class T_data, class T_collision_detector> class kdtree 
 {
 private:    
 	int dimension;
	// int n_nodes;

public:
	 
	 struct kdnode ///< defines the structure of a node in the kd-tree
 	{
		std::vector< T_data > x_;		///< this is the x vector in $R^d$
		kdnode *c_[2];			///< this is the vector c
		kdnode *P_;				///< Parent P node
		int j;					///< this is the j value means the split dimension
		bool dfs_visited;			///< This value is used in the DFS algorithm

		float T_;				///< Weighted number of samples generated from H
		float M_;				///< estimated measure of the free space in H
		float F_;				///< Weighted number of collision free samples generated from H
		T_environment H_;		///< Hyperplane of the node 
		kdnode (T_environment H_i) :  P_( NULL ), j (1), dfs_visited(false), T_(1.), M_( 0. ), F_(1.), H_(H_i) {  c_[0] = NULL ; c_[1] =  NULL ; x_.clear();  }
	};

	kdnode *root_; ///< kd-tree rrot node
 	T_collision_detector collision_detector; ///< define the colliion detector to be used
 	std::stack< kdnode* > neighbor_stack;

 	// kdtree();

 	void set_environment(T_environment H_i) { root_ = new kdnode( H_i ); root_->M_ = (root_->F_/root_->T_)*Measure( root_ ); };

 	/*!
	*	\fn kdtree::kdtree(T_environment H_i ) 
	*	\brief	Instance a kd-tree
	*	\param  H_i Parameter defining the environment where the kd-treee is defined
	*/

 	void set_dimension(int dim){ dimension = dim;}; 

 	/*!
 	*	\fn kdtree::set_dimension (int dim)
 	*	\brief set the dimension of the space of the kd-tree
 	*	\param dim the dimension of the space of the kd-tee
 	*/

	std::pair< std::vector< T_data >, bool> GenerateSample(T_environment H, kdnode * & v);


	/*!
	*	\fn kdtree::GenerateSample(T_environment H, kdnode * & v)
	*	\brief This functon Generate a new free collision node-sample. This is base don the paper 
	*	<a href="http://ares.lids.mit.edu/papers/Bialkowski.Otte.ea.IROS13.pdf"> </a>
	*	\param H The environment where the new sample will be generated
	*	\param v The node where the new point will be  associated 
	*	\return the new $R^d$ sample and a boolean value if the sample is free of collisions
	*/

	std::vector< T_data >  SampleUniform(T_environment H);

	/*!
	*	\fn SampleUniform(T_environment H)
	*	\brief This function generate a new random sample in a H environment
	*	\param H The environment where the new sample will be generated
	*	\return a new $R^d$ random sample
	*/

	T_data SampleUniform(const T_data min, const T_data max);


	/*!
	*	\fn SampleUniform(const T_data min, const T_data max)
	*	\brief This function generate a new random sample in a range environment
	*	\param min minimum value of the range where the new random sample will be generated
	*	\param max maximum value of the range where the new random sample will be generated
	*	\return a new random sample as scalar
	*/

	std::pair<kdnode * , kdnode * > split(kdnode *v, std::vector< T_data > x);
	/*!
	*	\fn split(kdnode *v, std::vector< T_data > x)
	*	\brief	This function generate two hyperplanes splitting the environment and passing through the point x
	*	\param v the node containing the space to be split
	*	\param x the point where the space will be split
	*	\return  two hyperplanes
	*/

	float Measure( kdnode *v );

	/*!
	*	\fn Measure( kdnode *v )
	*	\brief This function computes a estimated measure of the space defined by the node v
	*	\param v The node containing the space to be estimated
	*	\return the meassure of the space
	*/

	std::stack<kdnode* > GetStack();
	/*!
	*	\fn GetStack()
	*	\brief This function return the stack containing all nodes visited when getting a new random point
	*	\return a Stack
	*/

	kdnode* GetClosestNeighbour();
	/*!
	*	\fn GetClosestNeighbour( kdnode* kdnode_in );
	*	\brief This function returns the closest point in the tree starting from kdnode_in
	*	\param  The node to look fro neighbor_stack 
	*	\return The closet node in the tree to kdnode_in
	*/

	T_data Dist2nodes(kdnode* one, kdnode* two);
};

/*! \class kdtree 
*  \brief This code is an implementation of a modified kd-tree, it is mainly based in the 
 *	BOOK:  Advanced Methods in Computer Graphics - With examples in OpenGL,  Springer, 2012.
 *	This includes the changes presented in the paper Free-configuration Biased Sampling for Motion Planning.
*/


template < class T_environment, class T_data, class T_collision_detector>
std::pair< std::vector< T_data >, bool> kdtree < T_environment, T_data, T_collision_detector >
::GenerateSample(T_environment H, kdnode * & v)
{

	if (v == root_){
		neighbor_stack= std::stack< kdnode* >();
		neighbor_stack.push(root_);
		v->dfs_visited = true;
	}
	else{
		neighbor_stack.push(v);
		v->dfs_visited = true;
	}

	float u;
	std::vector< T_data >  x;
	bool r = false;
	float w = 1.;
	std::pair< std::vector< T_data > , bool > x_r;

	if (v->c_[0] == NULL && v->c_[1] == NULL)	
	{

		x = SampleUniform(H);
		v->T_ += 1.;
		r = collision_detector.iscollision_free (x);
		x_r = std::make_pair(x,r);
		std::pair<kdnode* , kdnode* > split_hyperplanes;

		if (r)
		{
			v->x_ = x;
			v->F_ += 1.;
			split_hyperplanes = split(v, x);
			v->c_[0] = split_hyperplanes.first;
			v->c_[1] = split_hyperplanes.second;

			for (int i = 0; i < dimension; ++i)
			{
				v->c_[i]->P_ = v;
				v->c_[i]->j = (v->j+1) % dimension;
				w = Measure(v->c_[i])/Measure(v);

				v->c_[i]->T_ = w*v->T_;
				v->c_[i]->F_ = w*v->F_;
				v->c_[i]->M_ = (v->c_[i]->F_/v->c_[i]->T_)*Measure(v->c_[i]);
			}

		}
		else
			neighbor_stack.pop();

	}
	else
	{

		u = SampleUniform(0., v->M_);
		if (u <= v->c_[0]->M_)
			x_r  = GenerateSample(v->c_[0]->H_, v->c_[0]);
		else
			x_r =  GenerateSample(v->c_[1]->H_, v->c_[1]);

		v->M_ = v->c_[0]->M_ + v->c_[1]->M_;

	}

	return x_r;
}

template < class T_environment, class T_data, class T_collision_detector>
std::vector< T_data >  kdtree < T_environment, T_data, T_collision_detector >
::SampleUniform(T_environment H)
{
	std::vector< T_data >  x;

	for (int i = 0; i < dimension; ++i)	{ x.push_back( SampleUniform(H[i].get_max(), H[i].get_min()) );}

		return x;
}

template < class T_environment, class T_data, class T_collision_detector>
T_data kdtree < T_environment, T_data, T_collision_detector >
::SampleUniform(const T_data min, const T_data max)
{

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(min, max);

	return dis(gen);
}



template < class T_environment, class T_data, class T_collision_detector>
std::pair<typename kdtree<T_environment, T_data, T_collision_detector>::kdnode*, typename kdtree<T_environment, T_data, T_collision_detector>::kdnode*> 
kdtree<T_environment, T_data, T_collision_detector>
::split(kdnode *v, std::vector< T_data > x)
{
	kdnode *left = new kdnode( v->H_ );
	kdnode *right = new kdnode( v->H_);

	left->H_[v->j].set_min( v->H_[v->j].get_min() ); 
	left->H_[v->j].set_max( x[v->j] ); 

	right->H_[v->j].set_min( x[v->j] ); 
	right->H_[v->j].set_max( v->H_[v->j].get_max() ); 
	return std::make_pair(left, right);
}


template < class T_environment, class T_data, class T_collision_detector>
float kdtree < T_environment, T_data, T_collision_detector >
::Measure( kdnode* v )
{
	float measure_hyperplane = 1.0;

	for (unsigned int i = 0; i < v->H_.size(); ++i){ measure_hyperplane *= ( fabs( (float)v->H_[i].get_min()  - (float)v->H_[i].get_max() ) ) ;}

		return measure_hyperplane;
}

template < class T_environment, class T_data, class T_collision_detector>
typename kdtree<T_environment, T_data, T_collision_detector>::kdnode* kdtree< T_environment, T_data, T_collision_detector >
::GetClosestNeighbour()
{
	
	// std::cout << "Number of Neighbors = "<< neighbor_stack.size() << std::endl;


	kdnode* xNN;
	if ( neighbor_stack.size() <= 1)
	{
		xNN = root_;
		return xNN;
	}
	std::stack< kdnode* > local_stack = neighbor_stack;
	std::stack< kdnode* > local_stack2clean = neighbor_stack;
	kdnode* node_temp;
	T_data distxNN = std::numeric_limits<T_data>::infinity();	
	neighbor_stack.pop();
	xNN = neighbor_stack.top();

	while( !neighbor_stack.empty() ) {
	    
	    if ( Dist2nodes(neighbor_stack.top(), xNN) < distxNN)
	    	xNN = neighbor_stack.top();

	}


	std::cout << "Root node visited = " << (root_->dfs_visited == true ? "true" : "false	") << std::endl;
	xNN = root_;


	// struct dfs_node{
	// 	kdnode* node;
	// 	bool visited;
	// 	dfs_node(kdnode* nodein, bool visitedin): node(nodein), visited(visitedin) {};
	// };



	// kdnode* xNN_temp;
	// // std::stack< kdnode* > neighbor_stack_local =  std::stack< kdnode* >();
	
	// for (unsigned int i = 0; i < neighbor_stack.size(); ++i)
	// {
	// 	xNN_temp->dfs_visited = true;
	//  	neighbor_stack_local.push( xNN_temp);	
	// }

	// std::stack< kdnode* > neighbor_stack_local = neighbor_stack;
	// kdnode* xq = neighbor_stack_local.top();
	// neighbor_stack_local.pop(); // clear the node we are looking for
	// xNN = neighbor_stack_local.top();
	// T_data distxNN = std::numeric_limits<T_data>::infinity();	

	// while( !neighbor_stack_local.empty() ) {

	// 	if (Dist2nodes( neighbor_stack_local.top(), xq ) < distxNN )
	// 	{
	// 		xNN =  neighbor_stack_local.top();
	// 		distxNN = Dist2nodes( xNN, xq );
	// 		neighbor_stack_local.pop();
	// 		if ( Dist2nodes( xNN->c_[0], xNN ) < distxNN);
	// 		 	neighbor_stack_local.push( xNN->c_[0] );
	// 		if ( Dist2nodes( xNN->c_[1], xNN ) < distxNN);
	// 			 neighbor_stack_local.push( xNN->c_[1] );

	// 	}
	// 	else
	// 		neighbor_stack_local.pop();

	// }

	return xNN;

}

template < class T_environment, class T_data, class T_collision_detector>
std::stack< typename kdtree<T_environment, T_data, T_collision_detector>::kdnode* > 
kdtree < T_environment, T_data, T_collision_detector >
::GetStack(){

	std::stack< kdnode* > neighbor_stack_local = neighbor_stack;
	neighbor_stack_local.pop(); 
	return neighbor_stack_local;

};


template < class T_environment, class T_data, class T_collision_detector>
T_data kdtree < T_environment, T_data, T_collision_detector >
::Dist2nodes(kdnode* one, kdnode* two)
{

	T_data dist = 0.0;

	if ( one->x_.empty() || two->x_.empty() ){
		dist = std::numeric_limits<T_data>::infinity();	
		return dist;
	}

	for (unsigned int i = 0; i < one->x_.size(); ++i)
	{
		dist += (one->x_[i]-two->x_[i])*(one->x_[i]-two->x_[i]);
	}

	return std::sqrt(dist);
}


#endif