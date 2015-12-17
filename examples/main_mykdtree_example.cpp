/*! 
 *	\mainpage Project Description
 *  \brief     This code is an implementation of a modified kd-tree, it is mainly based in the 
 *	BOOK:  Advanced Methods in Computer Graphics - With examples in OpenGL,  Springer, 2012.
 *	This includes the changes presented in the paper Free-configuration Biased Sampling for Motion Planning.
 *  \details   TODO
 *  \author    Manuel Bonilla
 *  \version   0.0.1
 *  \date      Dec-2013
 *  \warning   This file is under development 
 *  \copyright WTFPL (Do What the Fuck You Want to Public License).
*/

#include <iostream>
#include <mykdtree.hpp>
#include <example.hpp>

typedef float m_datype;
typedef std::vector<m_datype> x_t;
typedef point2d< m_datype > point2d_t;
typedef std::vector< space_dimension2d < m_datype > > environment_t;
typedef obstacle2d< m_datype> obstacle2d_t;
typedef collision_detector2d< obstacle2d_t, point2d_t > collision_detector2d_t;
typedef space_dimension2d< m_datype > space_dimension2d_t;
typedef kdtree 	< environment_t , m_datype ,collision_detector2d_t >::kdnode kdnode_t;
kdnode_t* new_root;
kdnode_t* new_root_original;
environment_t environment;
std::vector< obstacle2d_t > obstacle_array;
collision_detector2d_t collision_detector;
std::vector< x_t > listofcollisionpoints;
std::vector< x_t > listofpoints_x;
kdtree 	< 	environment_t , m_datype ,collision_detector2d_t > kdtree_t;
int n_pointsincollision = 0;
int n_nodes = 0;
float border = .01*0.; 
std::stack< kdnode_t* > neighbor_list;
std::stack< kdnode_t* > neighbor_list_temp;
kdnode_t* neighbor_temp;
x_t last_free_sample;
kdnode_t* xNN;
bool isLastInCollision = true;



#define VISUALITATION true

	void newtrial()
	{
		std::pair< x_t, bool > res;

			res = kdtree_t.GenerateSample(environment, kdtree_t.root_ ); //Each GenerateSample include a new random sample in the kdtree
			if ( res.second == false) {
				listofcollisionpoints.push_back( res.first );
				++ n_pointsincollision;
				isLastInCollision = true;

			}
			else
			{
				++n_nodes;
				listofpoints_x.push_back(res.first);
				last_free_sample = res.first;
				isLastInCollision = false;
			}
		
		return;
	}


#if VISUALITATION
	#include <GL/glut.h>

 	struct point_t{
 		float x;
 		float y;
 		point_t(float x_i, float y_i) : x(x_i), y(y_i){};
 	};

 	struct vertex_t{
 		point_t min;
 		point_t max;
 		vertex_t(point_t min_i, point_t max_i): min(min_i), max(max_i){};
 	};

 	struct leaf_node_t
	{
		obstacle2d_t environment;
		float probability;

		leaf_node_t( obstacle2d_t environment_i, m_datype probability_i ) : environment( environment_i ), probability ( probability_i ){}; 
	};
	std::vector< leaf_node_t > listofleafnodes;
	std::vector<point_t> listofpoints;
	std::vector<vertex_t> listofvertex;
	 
	int windowWidth=1, windowHeight=1;
	
	m_datype total_area = 1.;
	bool showlines = false; bool showpoints = false; bool showobstacles = false; bool showcollsision = false; bool showleafnodes = false;

	void extrae_node_info( kdnode_t * node)
	{

		listofpoints.push_back(point_t(node->x_[0], node->x_[1]));

		if ( node->j == 1) 
		{
			listofvertex.push_back(	vertex_t( 	point_t( node->c_[0]->H_[0].get_min(), node->c_[0]->H_[1].get_max() ),
											point_t( node->c_[0]->H_[0].get_max(), node->c_[0]->H_[1].get_max() ) ) );
		}
		else
		{
			listofvertex.push_back(	vertex_t( 	point_t( node->c_[1]->H_[0].get_min(), node->c_[1]->H_[1].get_min() ),
											point_t( node->c_[1]->H_[0].get_min(), node->c_[1]->H_[1].get_max() ) ) );
		}
		
	}



	void read_nodes(kdnode_t * node, float comulative_probablity )
	{
	
		// std::cout << " M = " << node->M_ << std::endl;
		if ( node->c_[0] == NULL && node->c_[1] == NULL) {
			// listofleafnodes.push_back( leaf_node_t( obstacle2d_t( node->H_[0].get_min(), node->H_[0].get_max(), node->H_[1].get_min(), node->H_[1].get_max() ), ( (float)node->M_ / (float)node->P_->M_ )*comulative_probablity ) );
			// listofleafnodes.push_back( leaf_node_t( obstacle2d_t( node->H_[0].get_min(), node->H_[0].get_max(), node->H_[1].get_min(), node->H_[1].get_max() ), (node->F_/node->T_)*node->M_ ));
			listofleafnodes.push_back( leaf_node_t( obstacle2d_t( node->H_[0].get_min(), node->H_[0].get_max(), node->H_[1].get_min(), node->H_[1].get_max() ),  kdtree_t.Measure(node)/ total_area ));
			return;
		}
		extrae_node_info( node );
		// if ( node->P_ != NULL ){
		// 	comulative_probablity =  node->M_ / node->P_->M_ *comulative_probablity  ;
		// }
		read_nodes( node->c_[0], comulative_probablity   );
		read_nodes( node->c_[1], comulative_probablity   );


		return; 
	}

	void keyboard(unsigned char key, int x, int y)
	{
		if( key == 'q' || key == 'Q'){
			// std::cout << "root.T = " << new_root->T_ << " total samples, free = " << new_root->F_  << " M =" << new_root->M_ << std::endl;
			exit( 0 );
		}
		if( key == 'l' || key == 'L') showlines = !showlines;
		if( key == 'c' || key == 'C') showcollsision = !showcollsision;
		if( key == 'p' || key == 'P') showpoints = !showpoints;
		if( key == 'o' || key == 'O') showobstacles = !showobstacles;
		if( key == 'f' || key == 'F') showleafnodes = !showleafnodes;
		if( key == 'n' || key == 'N'){ 
			newtrial(); 
			std::cout << "Percentage of collision samples = " << (float)n_pointsincollision *100./(float)n_nodes 
			 										<< "    n_nodes = " << n_nodes << "   n_pointsincollision = " << n_pointsincollision << std::endl;

			//neighbor_list = kdtree_t.GetStack();
			// std::cout << "Number of nodes in the stack = " << neighbor_list.size() << std::endl;
		}

		glutPostRedisplay();

	}


	void drawRectangle(obstacle2d_t Rectangle)
	{
		glBegin(GL_POLYGON);
			glVertex2f( Rectangle.x_min(),  Rectangle.y_min() );
			glVertex2f( Rectangle.x_min(),  Rectangle.y_max() );
			glVertex2f( Rectangle.x_max(),  Rectangle.y_max() );
			glVertex2f( Rectangle.x_max(),  Rectangle.y_min() );
		glEnd();
	}

	float constraint( point_t  pointq ){
		point_t pointq_local = pointq;
 		double l=1.0;
 		pointq_local.x = 3.14159*(2*pointq.x - 1); 
		pointq_local.y = 3.14159*(2*pointq.y - 1);
		double res1 = l*std::cos(pointq_local.x)-l*std::cos(pointq_local.y);
		double res2 = l*std::sin(pointq_local.x)-l*std::sin(pointq_local.y);

		return std::sqrt(res1*res1 + res2*res2);
	}

	void display(void) 
	{
		glClear(GL_COLOR_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0.0, windowWidth, 0.0, windowHeight);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();


		glColor3f(0.0, 0.0, 0.0);

		glBegin(GL_LINE_LOOP);
			glVertex2f( windowWidth*border, windowHeight*border  );
			glVertex2f( windowWidth*border, windowHeight*(1.-border)  );
			glVertex2f( (windowWidth*(1.-border)), windowHeight*(1.-border)  );
			glVertex2f( (windowWidth*(1.-border)), windowHeight*border  );
		glEnd();


		listofpoints.clear();
		listofvertex.clear();
		listofleafnodes.clear();
		read_nodes(new_root, 1.);

		if (showleafnodes)
		{

			for (unsigned int i = 0; i < listofleafnodes.size(); ++i) 
			{
				glColor3f(0., 0., listofleafnodes[i].probability );
				drawRectangle( listofleafnodes[i].environment );
			}
		}


		if ( showlines ) {
			glColor3f(0.0, 1.0, 0.0);
			glBegin(GL_LINES);

				for (unsigned int i = 0; i < listofvertex.size(); ++i) 
				{
					glVertex2f(listofvertex[i].min.x, listofvertex[i].min.y);
					glVertex2f(listofvertex[i].max.x, listofvertex[i].max.y);
				}

			glEnd();

		}

		//print points in the tree
		if (showpoints && listofpoints.size() > 0) {
			glBegin(GL_POINTS);
			// glColor3f(1.0, 0.0, 1.0);
			float violation_color_red = 1.0;
				for (unsigned int i = 0; i < listofpoints.size(); ++i)
				{
					// std::cout << "testin " << i << std::endl;
					violation_color_red = constraint( listofpoints[i] ) / 0.5;

					glColor3f(violation_color_red, 0.0, 0.0);
					glVertex2f(listofpoints[i].x, listofpoints[i].y);

				}
			glEnd();

			// glBegin(GL_POINTS);
			//  	glColor3f(1.0, 1.0, 1.0);
			//  	glVertex2f( last_free_sample[0], last_free_sample[1]);
			// glEnd();


			// glBegin(GL_LINES);
			// glColor3f(1.0, 0.0, 1.0);
			// 		if ( isLastInCollision == false ){
			// 			xNN =  kdtree_t.GetClosestNeighbour();
			// 			glVertex2f(xNN->x_[0], xNN->x_[1]);
			// 			glVertex2f( last_free_sample[0], last_free_sample[1]);
			// 		}

			// glEnd();



			// glBegin(GL_POINTS);
			// glColor3f(1.0, 0.0, 0.0);

			// 	neighbor_list_temp = neighbor_list;
			// 	while ( !neighbor_list_temp.empty() )
			// 	{
			// 	 	neighbor_temp = neighbor_list_temp.top();
			// 	 	if( neighbor_temp->c_[0] != NULL && neighbor_temp->c_[1] != NULL )
			// 			glVertex2f(neighbor_temp->x_[0], neighbor_temp->x_[1]);
			// 	 	neighbor_list_temp.pop();
			// 	}

			// glEnd();

		}
		
		//print lines building the kdtree

		
		//print objects
		if (showobstacles)
		{

			glColor3f(1.0, 0.0, 0.0);
			for (unsigned int i = 0; i < obstacle_array.size(); ++i) 
			{
				drawRectangle( obstacle_array[i] );
			}
		}


		//print points in collision 
		if (showcollsision)
		{
		
			glBegin(GL_POINTS);
			glColor3f(0.5372, 0.5372, 0.5372);
			// glColor3f(1., 0., 1.);

				x_t x_temp;

				for (unsigned int i = 0; i < listofcollisionpoints.size(); ++i) {
					x_temp = listofcollisionpoints[i];
					for (unsigned int j = 0; j < x_temp.size(); ++j) {

						glVertex2f( x_temp[0], x_temp[1] );

					}
				}

			glEnd();
		}

		//Show leaf nodes


		glFlush(); 
	} 






 #endif

int main(int argc, char *argv[])
{

	// define datatypes
	
	if (argc>1)
		n_nodes = atoi(argv[1]);


	//definition of the whole environment
	environment.push_back( space_dimension2d_t( (0. + border), (1. - border) ));
	environment.push_back( space_dimension2d_t( (0. + border), (1. - border) ));

	//definition of obstacles
	obstacle_array.push_back( obstacle2d_t( 0., 1., .5, 1.) );
	// obstacle_array.push_back( obstacle2d_t( .3, 1., .3, 1. ) );
	// obstacle_array.push_back( obstacle2d_t( 15,23,27,30 ) );
	// obstacle_array.push_back( obstacle2d_t( 21,30,9,11 ) );

	//definition of the collision detector
	collision_detector.add_obstacles(obstacle_array);

	//instance the kd-tree
	
	kdtree_t.set_environment( environment );
	kdtree_t.set_dimension(2);

	kdtree_t.collision_detector = collision_detector;

	newtrial();
	

	// std::cout << "Points in Collision = " << n_pointsincollision << std::endl;

	x_t x_temp;
	// for (unsigned int i = 0; i < listofpoints_x.size(); ++i) {
	// 	x_temp = listofpoints_x[i];
	// 	std::cout << "Point " << i+1 << " = " << x_temp[0] << " , " << x_temp[1] << std::endl;
	// }



#if VISUALITATION
	
	new_root = kdtree_t.root_;
	new_root_original = new kdnode_t(environment);
	for (unsigned int i = 0; i < environment.size(); ++i)
	 	total_area *= ( fabs( (float)environment[i].get_min()  - (float)environment[i].get_max() ) );

	glutInit(&argc, argv);            
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);  
	glutInitWindowSize(windowWidth,windowHeight);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("My kdTree");
	glClearColor(1.0, 1.0, 1.0, 0.0); 
	glPointSize(2.0); 
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	// glutFullScreen();
	glutDisplayFunc(display);
    glutReshapeWindow(windowWidth*1366,windowHeight*768);
 	// glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutMainLoop(); 	

#else
	for ( int i = 0; i < n_nodes; ++i)
	{
		newtrial();
	}

	// for (unsigned int i = 0; i < listofpoints_x.size(); ++i) {
	// 	x_temp = listofpoints_x[i];
	// 	std::cout << "Point " << i+1 << " = " << x_temp[0] << " , " << x_temp[1] << std::endl;
	// }
	std::cout 	<< "Percentage of collision samples = " << (float)n_pointsincollision *100./(float)n_nodes 
				<< "    n_nodes = " << n_nodes << "   n_pointsincollision = " << n_pointsincollision << std::endl;
#endif



	return 0;
}


