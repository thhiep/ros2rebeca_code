#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <cmath>
#include <time.h>
#include <mutex>
#include <cassert>
#define assertm(expr,msg) assert(((void)msg,expr))

using namespace std::chrono_literals;
using namespace std::placeholders;

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_plan.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "visualization_msgs/msg/marker_array.hpp"

//#include "ros2demo/msg/report_robot_info.hpp"
//#include "ros2demo/msg/path_request.hpp"

#define VERY_LARGE 9999999
#define OCCUPIED_GRAY_LEVEL 127
#define DEG2RAD 0.01745
#define RAD2DEG 57.29578
#define SQRT2 1.4142
#define COS45 0.7071
#define SIN45 0.7071

//must be defined as decimal
#define SEC2MILISEC 1000.0
#define SEC2NANOSEC 1000000000.0
#define MILISEC2NANOSEC 1000000.0

struct Point2;
struct Point2i;

struct Point2i { 
	int i,j;
	
	/* 
	Point2 toReal(double add=0.5){
		Point2 p;
		p.x = (double)i+add;
		p.y = (double)j+add;
		return p;
	}
	*/
	bool equals(Point2i b){
		return i==b.i && j==b.j;
	}
	
	//to allow using in a queue
	bool operator < (const Point2i &b) const {
		return (i+j)<(b.i+b.j);
	}
	
	std::string toString(){
		return std::to_string(i)+","+std::to_string(j); 
	}
		
	int toIndex(int cols){
		return i*cols + j;
	}
	
	int toIndexFlip(int cols){
		return i*cols * cols-1-j;
	}
	
	void swap(){
		int tmp=i;i=j;j=tmp;
	}
	
  	std::vector<Point2i> getNeighbors(){
		//neighboring cells in 8 directions: e, ne, n, nw, w, sw, s, se
		int8_t nxt[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
		std::vector<Point2i> p(8);
		for(int ni=0;ni<8;ni++){
			p[i].i = this->i + nxt[ni][0];
			p[i].j = this->j + nxt[ni][1];
		}
		return p;	
	}
	
	Point2i getNeighborAtDir(int dir){
		int8_t nxt[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
		Point2i p;
		p.i = this->i + nxt[dir][0];
		p.j = this->j + nxt[dir][1];
		return p;
	}
	
	bool isInside(int width, int height){
		return i>=0 && i<width && j>=0 && j<height;
	}
	
};

struct Point2 { 
	double x,y;
	
	bool equals(Point2 b,double epsilon=0.0){
		return this->edistance(b.x,b.y) <= epsilon;
	}
	
	void swap(){
		double tmp=x;x=y;y=tmp;
	}
	
	Point2i toInt(){
		Point2i p;
		p.i = floor(x);
		p.j = floor(y);
		return p;
	}
	
	Point2i toIntCeil(){
		Point2i p;
		p.i = ceil(x);
		p.j = ceil(y);
		return p;
	}
	
	Point2* scale(double k){
		x*=k;
		y*=k;
		return this;
	}
	
	//reverse scale
	Point2* rscale(double k){
		x/=k;
		y/=k;
		return this;
	}
	
	Point2* expand(double a){
		x+=a;y+=a;return this;
	}
	
	//(x,y) ------(a,b) ------ (x',y')
	Point2* mirror(double a, double b){
		x = 2*a - x;
		y = 2*b - y;
		return this;
	}
	
	//(x,y) -------(a,y) ------(x',y)
	Point2* xmirror(double a){
		x=2*a-x;return this;
	}
	
	Point2* ymirror(double b){
		y=2*b-y;return this;
	}

	//shift the origin to O'(a,b), what is the new coord in the new system?
	//shifting is done on the origin, not the point, the point is fixed
	//x' = x - a
	//y' = y - b
	Point2* shift(double a, double b){
		x-=a;
		y-=b;
		return this;
	}
	
	//rotate the axis by a theta angle around the origin, what is the new coord in the new system?
	//rotate is done on the axis, not the point, the point is fixed
	//x' = x*cos(theta) + y*sin(theta)
  	//y' = -x*sin(theta) + y*cos(theta)
  	Point2* rotate(double theta){
		double cosa = cos(theta);
		double sina = sin(theta);
		double px = x*cosa + y*sina;
		double py = -x*sina + y*cosa;
		
		this->x=px;this->y=py;return this;
	}
	
	Point2* rotate_deg(double theta_degrees){
		return this->rotate(theta_degrees * DEG2RAD);
	}
	
	Point2* trans(double a, double b, double theta){
		return this->shift(a,b)->rotate(theta);
	}
	
	//rotate 90 degrees to the left (counter-clockwise)
	//cos(90)=0,sin(90)=1
	//x' = x*cosa + y*sina = y
  	//y' = -x*sina + y*cosa	= -x
	Point2* rotate90l(){
		double px = y;
		double py = -x;
		this->x=px;this->y=py;return this;
	}
	
	//rotate 90 degrees to the right (clockwise)
	//cos(-90) = 0, sin(-90) = -1
	//x' = x*cosa + y*sina = -y
  	//y' = -x*sina + y*cosa	= x
	Point2* rotate90r(){
		double px = -y;
		double py = x;
		this->x=px;this->y=py;return this;
	}	
	
	//cos(180)=-1,sin(180)=0
	//x' = x*cosa + y*sina = -x
  	//y' = -x*sina + y*cosa = -y
	Point2* rotate180(){
		double px = -x;
		double py = -y;
		this->x=px;this->y=py;return this;
	}
	
	
	Point2* rotate45l(){
		double px = x*COS45 + y*SIN45;
		double py = -x*SIN45 + y*COS45;
		this->x=px;this->y=py;return this;
	}
	
	Point2* rotate45r(){
		//cost(-45) = cos(45), sin(-45) = -sin(45)
		double px = x*COS45 - y*SIN45;
		double py = x*SIN45 + y*COS45;
		this->x=px;this->y=py;return this;		
	}
	
	double angle(){
		double a;
		if (x==0) a = y>0?M_PI/2:-M_PI/2; else a = atan2(y,x);
		if (a<0) a+=2*M_PI;
		return a;
	}
	
	int dir(){
		double a = angle()*RAD2DEG;
		int dir = (int)(a/45) % 8;
		if (dir<0) dir+=8;
		return dir;
	}
	
	double length(){
		return sqrt(x*x + y*y);
	}
	
	geometry_msgs::msg::Pose toPose(double yaw=0){
		geometry_msgs::msg::Pose p;
		p.position.x = this->x;
		p.position.y = this->y;
		p.position.z = 0;
		p.orientation.x = 0;
		p.orientation.y = 0;
		p.orientation.z = yaw;
		p.orientation.w = 1;
		return p;		
	}
		
	geometry_msgs::msg::PoseStamped toPoseStamped(double yaw=0){
		geometry_msgs::msg::PoseStamped p;
		p.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
		p.pose = this->toPose(yaw);
		return p;		
	}
	
	//euclide distance
	double edistance(double x2,double y2){
		double dx = this->x-x2;
		double dy = this->y-y2;
		return sqrt(dx*dx + dy*dy);
	}	
	
	//manhattan distance
	double mdistance(double x2, double y2){
		double dx = this->x-x2;
		double dy = this->y-y2;
		return abs(dx) + abs(dy);
	}

	//octile distance
	double odistance(double x2, double y2){
		double dx = this->x-x2;
		double dy = this->y-y2;
		dx = abs(dx);
		dy = abs(dy);
		double f = SQRT2 - 1;
		return dx<dy? f*dx+dy:f*dy+dx;
	}

	double cdistance(double x2, double y2){
		double dx = this->x-x2;
		double dy = this->y-y2; 	
		dx = abs(dx);
		dy = abs(dy);
		return dx>dy?dx:dy;
	}
  
};

struct Rect2 {
	double xmin,xmax,ymin,ymax;
	
	Rect2(Point2 p1, Point2 p2){
		xmin = std::min(p1.x,p2.x);
		xmax = std::max(p1.x,p2.x);
		ymin = std::min(p1.y,p2.y);
		ymax = std::max(p1.y,p2.y);
	}
	
	bool contains(double x,double y) const {
		return x >= xmin && x<=xmax && y>=ymin && y<=ymax;
	}
	
	bool intersects(Rect2 b) const {
		if (contains(b.xmax,b.ymax) || 
			contains(b.xmax,b.ymin) || 
			contains(b.xmin,b.ymin) || 
			contains(b.xmin,b.ymax)
		) { return true; }
		
		if (b.contains(xmax,ymax) || 
			b.contains(xmax,ymin) || 
			b.contains(xmin,ymin) || 
			b.contains(xmin,ymax)
		) { return true; }
		
		return false;
	}
};

struct RobotInfo {
	std::string name;
	double x,y,angle,velocity,angular_velocity;
	double length,width,safe_margin,stop_zone;
	
	RobotInfo(){}
	
	int dir(){
		double a = angle*RAD2DEG;
		int dir = (int)(a/45) % 8;
		if (dir<0) dir+=8;
		return dir;
	}
	
	//get boundary rectangle in map coordinates, possibly with some extra margin
	Rect2 getBoundary(double margin=0.01){
		double xmax = length/2 + margin;
		double ymax = width/2 + margin;
		Point2 p1{xmax,ymax},p2{-xmax,-ymax};
		p1.rotate(-angle);p1.shift(-x,-y);
		p2.rotate(-angle);p2.shift(-x,-y);
		return Rect2{p1,p2};
	}
};

/************************************************************************

  ROS2 view port coordinates (it's not easy to understand correctly!)
  - ROS2 world origin is Ow
  - ROS2 map origin is Om
  - ROS2 grid origin is Og==Om
  - Map width = image width = image cols
  - Map height = image height = image rows
  - Axis colors: XYZ = Red-Green-Blue
 
     pixel[0,0]              pixel[0,w-1]
     grid[0,w-1]    |        grid[0,0]
  xM <------o-------o-------o Om(origin_x,origin_y)
            |       |       |
            |       |Ow     |
  xW <------o-------o-------o--------
            |       |       |
            |       |       |
            |       |       |
            o-------o-------o 
grid[h-1,w-1]       |       |grid[h-1,0]
 pixel[h-1,0]       |       |pixel[h-1,w-1]
                    |       |
                    V yW    V yM								

*************************************************************************/

struct MapInfo {
	int width;
	int height;	
	double resolution;
	double origin_x;
	double origin_y;
	
  MapInfo(){
  }
  
  MapInfo(nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_ptr){
  	this->width = map_msg_ptr->info.width;
  	this->height = map_msg_ptr->info.height;
  	this->resolution = map_msg_ptr->info.resolution;
  	this->origin_x = map_msg_ptr->info.origin.position.x;
  	this->origin_y = map_msg_ptr->info.origin.position.y;
  }
  
  int rows(){
  	return height;
  }
  
  int cols(){
  	return width;
  }
    
  int size(){
  	return width*height;
  }
  
  double xr(){
  	return origin_x;
  }

  double xl(){
  	return origin_x + width * resolution;
  }
    
  double yup(){
  	return origin_y;
  }
  
  double ylow(){
  	return origin_y + height * resolution;
  }

  //world origin O(0,0) is at center of map image
  //map origin I(a,b) = (origin_x,origin_y) is top-right corner of map image 
  Point2 world2map(double wx, double wy){
  	Point2 w = {wx,wy};
  	w.shift(xr(),yup());
  	w.rscale(resolution);
  	return w;
  }
  
  Point2 map2world(double mx, double my){
  	Point2 m = {mx,my};
  	m.scale(resolution);
		m.shift(-xr(),-yup());
		return m;
  }
  
  //grid origin is the same as map origin
  Point2i map2grid(double mx, double my){
  	Point2 m = {mx,my};
  	Point2i g = m.toInt();
  	return g;
  }
  
  Point2 grid2map(int gi, int gj){
  	Point2 g = {gi+0.5,gj+0.5};
  	return g;
  }  
  
  Point2i world2grid(double wx,double wy){
  	Point2 m;
  	m = world2map(wx,wy);
  	return map2grid(m.x,m.y);
  } 
  
  Point2 grid2world(int gi, int gj){
  	Point2 m;
  	m = grid2map(gi,gj);
  	return map2world(m.x,m.y);
  }
  
  Point2i grid2pixel(int gi,int gj){
  	Point2i p;
  	p.i = gj;
  	p.j = width-1-gi;
  	return p;
  }
  
  Point2i pixel2grid(int pi,int pj){
  	Point2i g;
  	g.i = width-1-pj;
  	g.j = pi;
  	return g;
  }
  
  int roundup(double realnbr,double epsilon=0.09){
	int i = (int)realnbr;
	double fract = abs(realnbr - i);
	int add = (fract>epsilon)?1:0;
	i += i>0?add:-add;
	return i;  	
  }  
  
  Point2i map2pixel(double mx, double my){
  	Point2i p;
  	p.i = floor(my);
  	p.j = width-1-floor(mx);
  	return p;	
  }
  
  Point2 pixel2map(int pi,int pj){
  	Point2 m;
  	m.x = width-1-pj+0.5;
  	m.y = pi+0.5;
  	return m;
  }
  
  Point2i world2pixel(double wx,double wy){
  	Point2 m = world2map(wx,wy);
  	return map2pixel(m.x,m.y);
  }
  
  Point2 pixel2world(int pi,int pj){
  	Point2 m = pixel2map(pi,pj);
  	return map2world(m.x,m.y);
  }
  
  int pixel2index(int pi,int pj){
  	return pi*width + width-1-pj;
  }
  
  int grid2index(int gi,int gj){
  	return gj*width + width-1-gi;
  }
  
  bool isInside(int i,int j){
  	return i>=0 && i<height && j>=0 && j<width;
  }
    
  bool isInside(int index){
  	return index>=0 && index<size();
  }
  
};

struct MoveTarget {
	Point2 position;
	double yaw = 0.0;
};

struct Astar_node{
	int i,j;
	double g,h,f;
	int parent;
	bool checked;
	int8_t occupied;
		
	Point2i getPoint(){
		return Point2i{i,j};
	}
};

struct ObstacleDetection{
	double range;
	double angle;
	
	bool operator == (const ObstacleDetection &b) const {
		return abs(this->range - b.range) <= 0.01;
	}
	
	bool operator > (const ObstacleDetection &b) const {
		return this->range > b.range;
	}

	bool operator < (const ObstacleDetection &b) const {
		return this->range < b.range;
	}
};

