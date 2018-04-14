#ifndef MYASTAR_PLANNER_H_
#define MYASTAR_PLANNER_H_

#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <set>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace myastar_planner{

	using namespace std;

	struct coupleOfCells{
		unsigned int index;
		unsigned int parent;
		double gCost;
		double hCost;
		double fCost;
	};
	
	struct FComparator{
  		bool operator()(const coupleOfCells &first, const coupleOfCells &second) const{
  			if(first.fCost != second.fCost){
  				return first.fCost < second.fCost;
  			}else{
  				return first.hCost < second.hCost;
  			}
		}
	};

	class MyastarPlanner : public nav_core::BaseGlobalPlanner{
		public:
		      MyastarPlanner();
		      MyastarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
		      void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
		      bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);
		private:
			float porcentajePoda, pesoEstatico;
			base_local_planner::WorldModel* world_model_;
			costmap_2d::Costmap2DROS *costmap_ros_;
			double step_size_, min_dist_from_robot_;
			costmap_2d::Costmap2D *costmap_;
			ros::Publisher plan_pub_;
			multiset<coupleOfCells, FComparator> openList;
			multiset<coupleOfCells, FComparator> closedList;
			double footprintCost(double x_i, double y_i, double theta_i);
			double calculateHCost(unsigned int start, unsigned int goal);
			static bool compareFCost(coupleOfCells const &c1, coupleOfCells const &c2);
			vector <unsigned int> findFreeNeighborCell (unsigned int CellID);
			void addNeighborCellsToOpenList(multiset<coupleOfCells, FComparator> &OPL, vector<unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell);
			double getMoveCost(unsigned int here, unsigned int there);
			void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
			ros::Publisher marker_Open_publisher;
			ros::Publisher marker_Closed_publisher;
			ros::Publisher marker_Goals_publisher;
			visualization_msgs::Marker markers_OpenList;
			visualization_msgs::Marker markers_ClosedList;
			visualization_msgs::Marker markers_Goals;
			void inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b);
			void inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b);
			void visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y);
			void visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z);
			void visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int cell);
			void visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista);
			void limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker);
			bool initialized_;
			void actualizarCostes(multiset<coupleOfCells, FComparator>& openList, multiset<coupleOfCells, FComparator>& closedList, const vector<unsigned int> &identificadores, const unsigned int &id_padre, const double &coste);
	};
};

#endif
