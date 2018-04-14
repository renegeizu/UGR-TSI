#ifndef MYASTAR_PLANNER_H_
#define MYASTAR_PLANNER_H_

#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <set>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace myastar_planner{
	using namespace std;
	
	struct coupleOfCells {
		unsigned int index;
		unsigned int parent;
		double gCost;
		double hCost;
		double fCost;
		unsigned costmapValue;
	};

	bool operator<(const coupleOfCells& c1, const coupleOfCells& c2);

	bool operator>(const coupleOfCells& c1, const coupleOfCells& c2) {
		return !(c1 < c2);
	}

	class MyPriorityQueue { 
		public:
			void push(const coupleOfCells& c);
			coupleOfCells pop();
			void clear();
			bool empty();
			size_t size() const {
				return abiertosIndex.size();
			}
			bool contains(unsigned int index) const {
				return abiertosIndex.find(index) != abiertosIndex.end();
			}
		private:
			set<unsigned int> abiertosIndex;
			priority_queue<coupleOfCells, vector<coupleOfCells>, greater<coupleOfCells> > abiertos;
	};

	class MyastarPlanner : public nav_core::BaseGlobalPlanner {
		public:
			MyastarPlanner();
			MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);
		private:
			costmap_2d::Costmap2DROS* costmap_ros_;
			double step_size_, min_dist_from_robot_;
			costmap_2d::Costmap2D* costmap_;
			ros::Publisher plan_pub_;
			MyPriorityQueue abiertos;
			unsigned costmapLimit;
			unsigned costmapLimitStart;
			coupleOfCells lastGoal;
			map<unsigned, coupleOfCells> closedList;
			bool isValidCell(unsigned cx, unsigned cy);
			bool isValidMoveCell(unsigned cx, unsigned cy) {
				return isValidCell(cx,cy) && costmap_->getCost(cx,cy) <= costmapLimit;
			}
			vector<unsigned> JPSfindNeighbours(bool hasParent, unsigned parent, unsigned currentNode);
			pair<bool,unsigned> JPSJump(bool hasNode, unsigned currentNode, unsigned parent, unsigned endNode);
			void JPSobtainSuccessors(bool hasParent, unsigned parent, unsigned currentNode, float gCostParent, unsigned endNode);
			bool checkConditions(double currentX, double currentY, double nx, double ny, const list<geometry_msgs::PoseStamped>::iterator& it);
			bool isContains(list<coupleOfCells> & list1, int cellID){
				for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
					if (it->index == cellID)
						return true;
				}
				return false;
			}
			unsigned char footprintCost(double x_i, double y_i, double theta_i);
			double calculateHCost(unsigned int start, unsigned int goal);
			static bool compareFCost(coupleOfCells const &c1, coupleOfCells const &c2);
			vector <unsigned int> findFreeNeighborCell (unsigned int CellID);
			void addNeighborCellsToOpenList(vector<unsigned int>& neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell);
			double getMoveCost(unsigned int here, unsigned int there);
			void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
				nav_msgs::Path miPath;
				miPath.header.frame_id = costmap_ros_->getGlobalFrameID();
				miPath.header.stamp = ros::Time::now();
				miPath.poses = path;
				plan_pub_.publish(miPath);
			}
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
	};
};

#endif
