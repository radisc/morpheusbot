/**
* Name: Navigation.cpp
* Package: Morpheusbot
* Author: Nicola Riste'
* Date: 2015-11-20
*
* History:
* Version Programmer 	Date 		Changes
* 0.0.1   Nicola Riste' 2015-11-20 	File Created
*
*
*/

// ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// Standard C++ library
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <stdio.h>

// OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//Graph
#include <limits> // for numeric_limits
#include <set>
#include <utility> // for pair

#define DEBUG_TIME

using namespace std;

//#define N			20

cv::Point start;
cv::Point goal;
cv::Mat map_;
cv::Mat copy_;

const double SQRTWO	=	sqrtf(2.0);

int N;

////////////
int MAX_NAV_POINTS = 20;

#ifdef DEBUG_TIME
typedef struct {
	timespec start;
	timespec end;
} Timer;

void startTimer(Timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &((*t).start));
}

void stopTimer(Timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &((*t).end));
}

// return timer count in ms
double getTimerValue(Timer* t)
{
	timespec temp;
	if (((*t).end.tv_nsec - (*t).start.tv_nsec) < 0)
		temp.tv_nsec = (*t).end.tv_nsec - (*t).start.tv_nsec + 1000000000;
	else
		temp.tv_nsec = (*t).end.tv_nsec - (*t).start.tv_nsec;
	return temp.tv_nsec / 1000000.;
}
#endif

enum{
	NEW 	= 1,
	OPENED 	= 2,
	CLOSED 	= 4,
	RAISE	= 8,
	LOWER	= 16
};

struct Solution{
	double z_opt;
	int* yStar;
	vector<int> navPoints;	//List of navpoints
	int n_navpoints;		//Non zero count

};

struct Instance{
	int start;
	cv::Mat map_;
	int l;
	int goal;
};


const double max_weight = std::numeric_limits<double>::infinity();

struct Node {
	int index;
	int state;// = NEW;
	double cost;// = 0;
	//Node(int index, )
};

struct Neighboor {			//This represents an arc, it can be added other attributes
	int target;
	double weight;
	Neighboor(int index, double weight)
	        : target(index), weight(weight) { }
};

typedef vector<vector <Neighboor> > adjacency_list_t;

struct Robot{
	int position_x;// = start.x;
	int position_y;// = start.y;
	double theta;// = 0.0;
	vector<Node> list;
};

void DijkstraComputePaths(int source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<double> &min_distance,
                          std::vector<int> &previous){

	int n = adjacency_list.size();		//Number of nodes
    min_distance.clear();
    min_distance.resize(n, max_weight);	//Initialize all nodes to infinity (and beyond)
    min_distance[source] = 0;			//The start point is at 0 distance from itself
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<double, int> > vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));

    while (!vertex_queue.empty()){

        double dist = vertex_queue.begin()->first;
        int u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
        const std::vector<Neighboor> &neighbors = adjacency_list[u];
        for (std::vector<Neighboor>::const_iterator 	neighbor_iter = neighbors.begin();
        												neighbor_iter != neighbors.end();
        												neighbor_iter++){

        	int v = neighbor_iter->target;
            double weight = neighbor_iter->weight;
            double distance_through_u = dist + weight;

            //copy_.at<cv::Vec3b>(cv::Point(( v)%N, (v )/N))[0] = 255;

            if (distance_through_u < min_distance[v]) {

				vertex_queue.erase(std::make_pair(min_distance[v], v));

				copy_.at<cv::Vec3b>(cv::Point(( v)%N, (v )/N))[0] = 0;
				copy_.at<cv::Vec3b>(cv::Point(( v)%N, (v )/N))[1] = 0;

				min_distance[v] = distance_through_u;
				previous[v] = u;
				vertex_queue.insert(std::make_pair(min_distance[v], v));

	            copy_.at<cv::Vec3b>(cv::Point(( v)%N, (v )/N))[1] = 255;


			}



         }
        //cv::imshow("Animation", copy_);
        //cv::waitKey(1);
    }
}

std::list<int> DijkstraGetShortestPathTo(int vertex, const std::vector<int> &previous){

    std::list<int> path;
    for ( ; vertex != -1; vertex = previous[vertex]){
        path.push_front(vertex);
        //Color map with path
        map_.at<cv::Vec3b>(cv::Point((vertex )%N, (vertex )/N))[0] = 0;
        map_.at<cv::Vec3b>(cv::Point((vertex )%N, (vertex )/N))[1] = 0;
        map_.at<cv::Vec3b>(cv::Point((vertex )%N, (vertex )/N))[2] = 255;
    }



    return path;
}
/*
 * For plain djikstra
 */
double pixelCost2(cv::Mat& img, int x, int y){
	if(int(map_.at<cv::Vec3b>(cv::Point((x + 1), y	))[0]) < 128){
		return 1.0;
	} else
		return std::numeric_limits<double>::infinity();
}

//
double wallCost(cv::Mat, int x, int y){

}

/*
 * A* like cost function
 */
double pixelCost(cv::Mat& img, int x, int y){


	const double max_distance = sqrt((goal.y - start.y)*(goal.y - start.y)+(goal.x - start.x)*(goal.x - start.x));

	//Check if pixel is valicable
	if(int(map_.at<cv::Vec3b>(cv::Point((x ), y	))[0]) < 160){

		double cost = 1 - sqrt( (goal.x - x)*(goal.x - x) + (goal.y - y)*(goal.y - y) )/max_distance;

		double wallCost = int(map_.at<cv::Vec3b>(cv::Point((x ), y	))[0])/255.0;

		//cout << "x: " << x << " y: " << y << " wallCost: " << wallCost << endl;

		//cout << "x: " << x << " y: " << y << " cost: " << cost << endl;
		return cost + wallCost;
	} else
		return std::numeric_limits<double>::infinity();
}


int main(int argc, char **argv){

    ros::init(argc, argv, "navigator");
    ros::NodeHandle node;
    printf("Hello, navigator!\n");

    Instance inst;
    Solution sol;



//    cv::Mat map = cv::Mat::zeros(cv::Size(640, 480), CV_8U);
//    cv::imwrite("zeros.png", zeros);

    map_ = cv::imread("map.png", CV_LOAD_IMAGE_COLOR);

    if(!map_.data){
    	cout << "errore caricando la mappa!" << endl;
    	return 0;
    }

	std::stringstream ss;
	ss << "Map " << map_.cols << "x" << map_.rows;
	std::string windowName = ss.str();
	cv::namedWindow(windowName, CV_WINDOW_FREERATIO);

	N = map_.cols;

	start = cv::Point(0,0);
//    int start = 0;	//Upper-Left pixel
//    int goal = 399; //goal = (l-1, l-1) = (l-1)*l+(l-1) (Bottom-Rigth pixel)

	goal = cv::Point(map_.cols - 1, map_.rows - 1);
    //Generate instance
	inst.map_ = map_;
	inst.l = N;
	inst.start = 0;
	inst.goal = goal.x + goal.y*N;

	vector<int> nav_points;

	nav_points.push_back(10 + N*4);
	nav_points.push_back(4 + N*11);
	nav_points.push_back(8 + N*15);
	nav_points.push_back(19 + N*9);

	//Generate node list
	//vector<Node> map_node_list[map_.rows*map_.cols];

	Robot robot;

	//Creating adjacency list
	#ifdef DEBUG_TIME
		Timer mapCreation, planning, all;
		startTimer(&mapCreation);
		startTimer(&all);
	#endif

	adjacency_list_t adjacency_list(N*N);

	double correction_factor = 20000;

	//Connect inner sqaures
	for(int i = 0 ; i < map_.rows - 1 ; i++){

		for(int j = 0 ; j < map_.cols - 1 ; j++){

			if(pixelCost(map_, j, i  ) < 255.0){	//Do not link invalicable pixel with others
				//cout << "i: " << i << " j: " << j << ": " << pixelCost(map_, j, i  ) << " ";

				//Upward pixel
				if(i != 0 && pixelCost(map_, j, i-1) < 255){
					//cout << "N" << " ";
					adjacency_list[j + i*map_.cols].push_back( Neighboor( ((j) + map_.cols*(i - 1) ), 1 +	pixelCost(map_, j, i-1  ) ) );
				}

				//Upward-right pixel
				if(i != 0 && j != map_.cols - 1 && pixelCost(map_, j+1, i-1) < 255){
					//cout << "NE" << " ";
					adjacency_list[j + i*map_.cols].push_back( Neighboor( ((j+ 1) + map_.cols*(i-1)), SQRTWO + pixelCost(map_, j+1, i-1) ) );
				}

				//Right pixel
				if(j != map_.cols - 1 && pixelCost(map_, j+1, i) < 255){
					//cout << "E" << " ";
					adjacency_list[j + i*map_.cols].push_back( Neighboor( ((j+ 1) + map_.cols*i), 1 +	pixelCost(map_, j+1, i  ) ) );
				}

				//Downward-Right pixel
				if(i != map_.rows - 1 && j != map_.cols - 1 && pixelCost(map_, j+1, i+1) < 255){
					//pixel downward-right
					//cout << "SE" << " ";
					adjacency_list[j + i*map_.cols].push_back( Neighboor( ((j+ 1) + map_.cols*(i+1)), SQRTWO + pixelCost(map_, j+1, i+1) ) );
				}

				//Downward pixel
				if(i != map_.rows - 1 && pixelCost(map_, j, i+1) < 255){
					//cout << "S" << " ";
					adjacency_list[j + i*map_.cols].push_back( Neighboor( ( j 	 + map_.cols*(i+1)), 1 + pixelCost(map_, j  , i+1) ) );
				}

				//Downward-left pixel
				if(i != map_.rows - 1 && j != 0&& pixelCost(map_, j-1, i+1) < 255){
					//cout << "SO" << " ";
					adjacency_list[(j) + (i)*map_.cols].push_back( Neighboor( ( (j-1) 	 + map_.cols*(i+1)), SQRTWO + pixelCost(map_, j-1  , i+1) ) );
				}

				//Left pixel
				if(j != 0 && pixelCost(map_, j-1, i) < 255){
					//cout << "O" << " ";
					adjacency_list[(j) + (i)*map_.cols].push_back( Neighboor( ( (j-1) 	 + map_.cols*(i)), 1 + pixelCost(map_, j-1  , i) ) );
				}

				//Upward-Left pixel
				if(i != 0 && j != 0 && pixelCost(map_, j-1, i-1) < 255 ){
					//cout << "NO" << " ";
					adjacency_list[(j) + (i)*map_.cols].push_back( Neighboor( ( (j-1) 	 + map_.cols*(i-1)), SQRTWO + pixelCost(map_, j-1  , i-1) ) );
				}
			}
		}
		//cout << endl;
		//The last square of every row is a special case
		//adjacency_list[(map_.rows - 1) + i*map_.cols].push_back( Neighboor( ( (map_.rows - 1) 	 + map_.cols*(i+1)), map_.at<cv::Scalar>(cv::Point((map_.rows - 1), i))[0] ) ); //Pixel downward
	}

	//Blur map near walls
	cv::GaussianBlur(map_, map_, cv::Size(101, 101), 12.0, 12.0);

	//Make a copy to draw on
	copy_ = map_.clone();


	#ifdef DEBUG_TIME
	  stopTimer(&mapCreation);
	  startTimer(&planning);
	#endif


	vector<double> min_distance;
	vector<int> previous;
	DijkstraComputePaths(0, adjacency_list, min_distance, previous);
    std::cout << "Distance from 0 to (" << goal.x << "," << goal.y << "): " << min_distance[goal.x + goal.y*N] << std::endl;

    std::list<int> path = DijkstraGetShortestPathTo(goal.x + goal.y*N, previous);
	std::cout << "Path : ";
	std::copy(path.begin(), path.end(), std::ostream_iterator<int>(std::cout, " "));
	std::cout << std::endl;

	#ifdef DEBUG_TIME
	  stopTimer(&planning);
	  stopTimer(&all);

		double total_det = getTimerValue(&all);
		printf("processing time: %.2f ms\n", getTimerValue(&all));

		printf("\ntotal processing time:\n");
		double total = getTimerValue(&mapCreation) + getTimerValue(&planning);
		printf("map creation:\t%.2f ms \t(%.2f%%)\n", getTimerValue(&mapCreation), getTimerValue(&mapCreation) / total *100.);
		printf("planning:\t%.2f ms \t(%.2f%%)\n", getTimerValue(&planning), getTimerValue(&planning) / total *100.);

	#endif

    cv::imshow(windowName, map_);
    cv::imwrite("path.png", map_);

    cv::waitKey(0);
    cv::destroyWindow(windowName);
    cv::destroyAllWindows();


    printf("Bye\n");
}

//double cost(Instance& inst, int i, int j){
//
//	double cost = 0.0;
//
//	int x1 = i%inst.l;
//	int y1 = i/inst.l;
//	int x2 = j%inst.l;
//	int y2 = j/inst.l;
//	int steps = std::max(abs(x1-x2), abs(y1-y2));
//	double step = 1.0/steps;
//
//
//	for(double t = 1.0/steps ; t < 1.0; t = t + step){
//		double Xt = x1 + t*(x2 - x1);
//		double Yt = y1 + t*(y2 - y1);
//		cost += inst.map_.at<uchar>(cv::Point(Xt, Yt));
//		cout << "Xt: " << Xt << " Yt: " << Yt <<" val: " << (int) inst.map_.at<uchar>(cv::Point(Xt, Yt)) << endl;
//	}
//
//	return cost;
//}
//
//void calcZ(Solution solution){
//
//	double cost = 0;
//
//	//Iteratively try every step with more nav points
//	for(int k = 0 ; k < MAX_NAV_POINTS; k++){
//
//		for(int i = 0 ; i < k ; k++){
//			cost += 1*1;
//		}
//	}
//}
//
//double routeCost(Instance& inst, Solution sol, int* vett){
//
//	double routeCost = 0.0;
//
//	//Cost of links
//	routeCost += cost(inst, 0, sol.navPoints[vett[0]]);	//Cost of linking start with the first navpoint
//
//	for(int i = 0 ; i < sol.n_navpoints; i++){
//		//cout << "sol n_navpoints" << sol.n_navpoints << endl;
//		cout << "Navpoint #" << i <<": " << sol.navPoints[vett[i]] << endl;
//		routeCost += cost(inst, sol.navPoints[vett[i]], sol.navPoints[vett[i+1]]);
//	}
//	cout << "nav_points[vett[sol.n_navpoints - 1]]" << sol.navPoints[vett[sol.n_navpoints - 1]] << endl;
//	routeCost += cost(inst, sol.navPoints[vett[sol.n_navpoints - 1]], inst.goal);	//Cost of linking last navpoint with goal
//	cout << "routeCost: " << routeCost << endl;
//	return routeCost;
//}
//
//void stampaVett(Instance& inst, Solution& sol,int* vett, int n){
//
//
//	for (long j = 0; j < n; j++) {
//
//		//if ( vett[j] == 0.0)
//			//continue; // skip if zero
//
//		std::cout << sol.navPoints[vett[j]] << " ";
//	}
//	std::cout << std::endl;
//	//Compute cost for particular permutation
//	sol.n_navpoints = n;
//
//	double tsp_cost = routeCost(inst, sol, vett);
//	if(tsp_cost < sol.z_opt){
//		sol.z_opt = tsp_cost;
//	}
//}
//
//
////void stampaVett(Instance& inst, Solution& sol, int* vett, int n){
////	for (long j = 0; j < n; j++) {
////
////		//if ( vett[j] == 0.0)
////			//continue; // skip if zero
////
////		std::cout << nav_points[vett[j]] << " ";
////	}
////	std::cout << std::endl;
////	//Compute cost for particular permutation
////	sol.navPoints = n;
////	routeCost(inst, sol, vett);
////}
//
///*genera tutte le permutazioni sui valori 0,1,...,n-1,
// inizialmente k = -1 e val[i] = 0, per 0≤i<n.
//prec: vett != NULL && val != NULL;
//*postc: stampa a video tutte le permutazioni su 0,1,...,n-1, memorizzate in vett.*/
//void genTPerm(Instance& inst, Solution& sol, int *vett,int k ,int *val, unsigned int n){
//
//
//	int i;
//	if (k == n - 1)
//		stampaVett(inst, sol, vett, n);
//	else
//		for(i=0;i<n;i++)
//			if (val[i] == 0){
//				vett[k+1] = i;
//				val[i] = 1;
//				genTPerm(inst, sol, vett, k+1, val, n);
//				val[i] = 0;
//			}
//}
//
//double solveTSP(Instance& inst, Solution& sol){
//
//
//	double routeCost =0;
//
//    int k = -1;
//	int vett[sol.n_navpoints];
//	int val[sol.n_navpoints];
//
//	unsigned int n = abs(sol.n_navpoints);
//	//Fill index vector
//	for(int i = 0; i < sol.n_navpoints ; i++){
//		vett[i] = i;
//		val[i] = 0;
//	}
//
//	sol.z_opt =DBL_MAX;
//
//    genTPerm(inst, sol, vett, k, val, n);
//
//    //Here select minima
//
//    return routeCost;
//}
//
//void generateFromVector(Instance& inst, Solution& sol, vector<int> nav_points){
//
//	//Generate solution
//	sol.yStar = (int*)calloc(N*N, sizeof(int));
//
//	int k =0;
//	for(int i = 0; i < N*N; i++){
//		if(i == nav_points[k]){
//			sol.yStar[i]=1; //TODO: also compute navpoints cost here
//			k++;
//		} else {
//			sol.yStar[i]=0;
//		}
//	}
//
//	sol.navPoints = nav_points;
//	sol.n_navpoints = nav_points.size();
//	//Compute TSP for selected NavPoints
//	solveTSP(inst, sol);
//
//	//Now add cost for navpoints
//
//
//}


