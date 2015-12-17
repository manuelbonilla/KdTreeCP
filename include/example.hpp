#ifndef EXAMPLE_HPP_INCLUDED
#define EXAMPLE_HPP_INCLUDED


template < class T > class space_dimension2d
{
	T max;
	T min;
public:
	space_dimension2d();
	space_dimension2d(T min_i, T max_i) : max(max_i), min(min_i){}
	void set_max(T max_i) { max = max_i; }
	void set_min(T min_i) { min = min_i; }
	T get_max(){return max;}
	T get_min(){return min;}

};

template < class T> class point2d
{
public:
	std::vector< T > data;
	point2d(std::vector< T > x_i) 
	{ 
		data = x_i;
	}
	void print_info()
	{
		std::cout << "The point is: ";
		for (int i = 0; i < data.size(); ++i)
		{
			std::cout << data[i] << "  ";
		}
		std::cout << std::endl;
	}
};

template <class T> class obstacle2d
{
	T d_x_min;
	T d_x_max;
	T d_y_min;
	T d_y_max;

public:
	obstacle2d(T x_min_i, T x_max_i, T y_min_i, T  y_max_i) : d_x_min(x_min_i), d_x_max(x_max_i), d_y_min(y_min_i), d_y_max(y_max_i){};
	T x_min(){return d_x_min;};
	T x_max(){return d_x_max;};
	T y_min(){return d_y_min;};
	T y_max(){return d_y_max;};

	void print_info(){  std::cout 	<< "x_min = " << d_x_min 	<< ", x_max = " << d_x_max << ", y_min = " << d_y_min << ", y_max = " << d_y_max; 	}

};

template <class T_obstacle, class T_point2ds> class collision_detector2d
{
private:
	int n_obstacles;
	std::vector< T_obstacle > obstacles;
public:
	collision_detector2d(){n_obstacles = 0; obstacles.clear();};
	void add_obstacles(std::vector< T_obstacle > obstacle){

		obstacles = obstacle;
		n_obstacles = obstacle.size();
	}

	int get_n_obstacles(){return n_obstacles;};

	void print_info(){

		std::cout << "Number of obstacles = " << n_obstacles << std::endl;
		for (int i = 0; i < n_obstacles; ++i)
		{
			std::cout << "Obstacle " << i << " dimensions: ";
			obstacles[i].print_info();
			std::cout << std::endl;
		}

	};

	bool iscollision_free (T_point2ds point2d){	

		float epsilon = 0.5;

		if(constraint(point2d) <= epsilon )
			return true;

		return false;
	};

	double constraint(T_point2ds point2d){

		T_point2ds point2d_local = point2d;
		point2d_local.data[0] = 3.14159*(2*point2d.data[0] - 1); 
		point2d_local.data[1] = 3.14159*(2*point2d.data[1] - 1);
		double l=1.0;
		double res1 = l*std::cos(point2d_local.data[0])-l*std::cos(point2d_local.data[1]);
		double res2 = l*std::sin(point2d_local.data[0])-l*std::sin(point2d_local.data[1]);

		return std::sqrt(res1*res1 + res2*res2);
	};

};
#endif
