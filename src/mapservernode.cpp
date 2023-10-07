
#include "ros2demo/common.hpp"

class MapServerNode : public rclcpp::Node
{
public:
  MapServerNode() : Node("mapservernode")  
  {
    std::vector<std::string> params{
    	"map_pgm","map_yaml","map_resol","map_occupied_thresh","map_free_thresh",
    	"robot_length","robot_width","robot_radius",
    	"safe_margin","stop_zone","collision_margin",
    	"laserscan_rate","fov","angle_inc","max_range","min_range",
    	"combine_ziczac"
    };
    for(int i=0;i<params.size();i++){
    	declare_parameter(params[i]);
    }
  	
  	fov_ = DEG2RAD*this->get_parameter("fov").as_double();
  	angle_inc_ = DEG2RAD*this->get_parameter("angle_inc").as_double();
  	max_range_ = this->get_parameter("max_range").as_double();
  	min_range_ = this->get_parameter("min_range").as_double();
  	collision_margin_ = this->get_parameter("collision_margin").as_double();
  	
  	// Load the PGM map file
    //std::string pkg_dir = "/home/thhiep/ros2_ws/src/ros2demo";
    //std::string pkg_dir = ament_index_cpp::get_package_share_directory("ros2demo");
    
    std::string map_file_path = (this->get_parameter("map_pgm")).as_string();
    cv::Mat map_image = cv::imread(map_file_path, cv::IMREAD_GRAYSCALE);
    
    map_msg_ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    
    // Convert the image to an occupancy grid message
    map_msg_ptr->header.frame_id = "map";
    map_msg_ptr->info.resolution = (this->get_parameter("map_resol")).as_double(); // Change this to match your map's resolution
    map_msg_ptr->info.width = map_image.cols;
    map_msg_ptr->info.height = map_image.rows;
    map_msg_ptr->info.origin.position.x = -(map_image.cols * map_msg_ptr->info.resolution)/2;
    map_msg_ptr->info.origin.position.y = -(map_image.rows * map_msg_ptr->info.resolution)/2;
    map_msg_ptr->info.origin.orientation.w = 1;
    map_msg_ptr->data.resize(map_image.total());
    int count_occupied=0;
    for (int i = 0; i < map_image.rows; i++) {
      for (int j = 0; j < map_image.cols; j++) {
        int occupancy = (map_image.at<uchar>(i, map_image.cols-1-j) > OCCUPIED_GRAY_LEVEL) ? 0 : 100;
        map_msg_ptr->data[i*map_image.cols+j] = occupancy;
        count_occupied += occupancy>0?1:0;
      }
    }
    mapinfo = {map_msg_ptr};
    
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    map_timer_ = this->create_wall_timer(500ms, std::bind(&MapServerNode::publish_grid,this));
    
    reg_sub_ = create_subscription<std_msgs::msg::String>("/register",
    	rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
    	std::bind(&MapServerNode::on_register,this,_1));
    /*
    for(int i=1;i<=num_robots;i++){
    	const std::string rname = "r"+std::to_string(i);
    	this->registerRobot(rname);			
    }
    */
    
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/expanded_obstacles", 1);	
  }

private:

	//sub to robot register
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reg_sub_;
    
    double fov_,angle_inc_,max_range_,min_range_,collision_margin_;
	
	//array to keep track of robot locations
	std::map<std::string,RobotInfo> rloc;
    
	//to publish occupancy grid
	nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_ptr;
	MapInfo mapinfo;
  	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
	rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
	
	//for each robot
    std::map<std::string,rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs;
    std::map<std::string,rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr> path_services;
    std::map<std::string,rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> laser_pubs; 
    std::map<std::string,rclcpp::TimerBase::SharedPtr> laser_timers;
    std::map<std::string,nav_msgs::msg::Path> paths;
    std::map<std::string,rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs;
    std::map<std::string,std::mutex*> mutex_;
    
  rclcpp::Time now(){
  	return rclcpp::Clock(RCL_ROS_TIME).now();
  }
  	
  void on_register(const std_msgs::msg::String::SharedPtr msg){
    std::string rname = msg->data;
  	rname.erase(std::remove(rname.begin(),rname.end(),'/'),rname.end());
 	registerRobot(rname);
  }
  	
  bool registerRobot(const std::string rname){
	if (rloc.find(rname) != rloc.end()){
		return false;
	}
	rloc[rname] = RobotInfo();
  	rloc[rname].name = rname;
  	rloc[rname].x = 0;
  	rloc[rname].y = 0;
  	rloc[rname].angle = 0;
  	rloc[rname].velocity = 0;
  	
  	//each robot should report its own measurements, currently same for all for simplicity
  	rloc[rname].length = get_parameter("robot_length").as_double();
  	rloc[rname].width = get_parameter("robot_width").as_double();
  	rloc[rname].safe_margin = get_parameter("safe_margin").as_double();
  	rloc[rname].stop_zone = get_parameter("stop_zone").as_double();
  	
  	auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  	auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  	
  	std::function<void(const nav_msgs::msg::Odometry::SharedPtr)> odom_fx = std::bind(&MapServerNode::updateRobotLocation, this, rname, _1);
	this->odom_subs[rname] = create_subscription<nav_msgs::msg::Odometry>(
		"/"+rname+"/odom",
		default_qos,
		odom_fx
	);
	
	// Set up the path planning service, must be before the laser service
	std::function<void(
			const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
			std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)> path_fx =
		 
		std::bind(&MapServerNode::generatePathServiceCallback, this, rname, _1, _2);	
 
 	this->path_services[rname] = this->create_service<nav_msgs::srv::GetPlan>(
            "/"+rname+"/generate_path",
            path_fx
            //std::bind(&MapServerNode::generatePathServiceCallback, this, rname, _1, _2)),
    );
	
	this->laser_pubs[rname] = create_publisher<sensor_msgs::msg::LaserScan>(
		"/"+rname+"/laserscan",
		sensor_qos);
		
	std::function<void()>laser_fx = std::bind(&MapServerNode::publish_laser, this, rname);
	this->laser_timers[rname] = this->create_wall_timer(
		std::chrono::milliseconds(this->get_parameter("laserscan_rate").as_int()), 
		laser_fx
	);
	
	this->path_pubs[rname] = create_publisher<nav_msgs::msg::Path>(
		"/"+rname+"/path",
		default_qos);
		
	this->mutex_[rname] = new std::mutex();
	
	RCLCPP_INFO(this->get_logger(),"robot %s is registered",rname.c_str());

	return true;  	
  }
  
  int map_cols(){
	 return this->map_msg_ptr->info.width;
  }
  
  int map_rows(){
    return this->map_msg_ptr->info.height;
  }
  
  double map_resol(){
  	return this->map_msg_ptr->info.resolution;
  }
  
  bool isOccupiedOriginally(int i,int j){
  	int gindex = mapinfo.pixel2index(i,j);
  	if (gindex<0 || gindex>mapinfo.size()) return true;
  	return map_msg_ptr->data[gindex]>0;
  }
   
  
  Point2 getNewCoords(double x, double y, double a, double b, double theta){
     double cosa = cos(theta);
     double sina = sin(theta);
	 Point2 p;
  	 p.x = (x-a)*cosa - (y-b)*sina;
  	 p.y = (x-a)*sina + (y-b)*cosa;
  	 return p; 
  } 
  
  //euclide distance
  double edistance(double dx, double dy){
  	return sqrt(dx*dx + dy*dy);
  }
  
  //manhattan distance
  double mdistance(double dx, double dy){
  	return abs(dx) + abs(dy);
  }
  
  //octile distance
  double odistance(double dx, double dy){
  	dx = abs(dx);
  	dy = abs(dy);
  	double f = SQRT2 - 1;
  	return dx<dy? f*dx+dy:f*dy+dx;
  }
  
  double cdistance(double dx, double dy){
  	dx = abs(dx);
  	dy = abs(dy);
  	return dx>dy?dx:dy;
  }
  
  void publish_grid(){ 
    map_publisher_->publish(*map_msg_ptr);
    for(const auto &it: this->paths){
    	this->path_pubs[it.first]->publish(it.second);
    }
  }
  
  
  void publish_laser(const std::string rname){
  	
  	laser_pubs[rname]->publish(this->scanObstacles(rname,fov_,angle_inc_,max_range_,min_range_));
  	
  }
   
  sensor_msgs::msg::LaserScan scanObstacles(
  			const std::string rname, 
  			double fov, double angle_inc,
  			double max_range=20.0, double min_range=0.02){
  	
  	std::vector<std::int8_t> grid(map_msg_ptr->data);
  	grid.resize(mapinfo.size());
    //update dynamic locations of other robots to the grid
    for(const auto& kv : rloc){
    	if (kv.first != rname){
		    Point2i pi = mapinfo.world2pixel(kv.second.x,kv.second.y);
		    grid[mapinfo.pixel2index(pi.i,pi.j)]=100;
       }
    }
  			
  	double half_fov = fov/2; 	
  	int beams = ceil(fov / angle_inc);
	
	sensor_msgs::msg::LaserScan scan;
	scan.header.stamp = this->now();
    scan.header.frame_id = rname+"/base_link";
    
    double angle_in_frame = 0;//in base_link, the robot's direction is the same as X axis
  	scan.angle_min = angle_in_frame - half_fov;
    scan.angle_max = angle_in_frame + half_fov;
    scan.angle_increment = angle_inc;
    
    //double laser_frequency = 50;
  	scan.time_increment = 0.02/beams;//(1 / laser_frequency) / (beams);
    scan.range_min = min_range;
    scan.range_max = max_range;
	scan.ranges.resize(beams);
    scan.intensities.resize(beams);
    
  	RobotInfo r = getRobotInfo(rname);  	
  	Point2 ro = mapinfo.world2map(r.x,r.y);
  	double factor = 0.6;
  	double step_len0 = factor * map_msg_ptr->info.resolution;
  	double d0 = factor * edistance(mapinfo.roundup(ro.x)-ro.x,mapinfo.roundup(ro.y)-ro.y) 
  		 * map_msg_ptr->info.resolution;
    double angle = r.angle - half_fov;
    for(int i=0;i<beams;i++){
    	angle += angle_inc;
    	//double* ac = this->acache_->get(angle);
    	double cosa = cos(angle);
    	double sina = sin(angle);
    	double range = VERY_LARGE;  	
    	double step_len = step_len0;
    	double d = d0;
    	if (cosa!=0) { 
    		//step_len/=abs(cosa); 
    		d/=abs(cosa); 
    	}
    	for(;d<max_range;d+=step_len){
			double dx = d*cosa;
			double dy = d*sina;
			Point2i p = mapinfo.world2pixel(r.x+dx,r.y+dy);
			if (mapinfo.isInside(p.i,p.j)){
				int index = mapinfo.pixel2index(p.i,p.j);
				if (grid[index]>0){
					range = d;
					break;
				}
			} else {
				range = VERY_LARGE;
				//range = d;
				break;
			}			
		}
		scan.ranges[i]=range;
		scan.intensities[i]=0.5;
    }
  	    
  	return scan;
  }
  
  RobotInfo& getRobotInfo(const std::string rname){
  	//registerRobot(rname);
  	std::lock_guard<std::mutex> guard(*mutex_[rname]);
  	return this->rloc[rname];
  }
  
  void updateRobotLocation(const std::string rname, const nav_msgs::msg::Odometry::SharedPtr msg){
  	double rx = msg->pose.pose.position.x;
  	double ry = msg->pose.pose.position.y;
  	tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
  	double v = msg->twist.twist.linear.x;
  	double av = msg->twist.twist.angular.z;
  	registerRobot(rname);
  	std::lock_guard<std::mutex> guard(*mutex_[rname]);
  	rloc[rname].x = rx;
  	rloc[rname].y = ry;
  	rloc[rname].angle = yaw;
  	rloc[rname].velocity = v;
  	rloc[rname].angular_velocity = av;
  	
  	//get map coordinates of robot's boundary vertices
  	Rect2 r1 = rloc[rname].getBoundary(collision_margin_);

  	for(const auto& kv: rloc){
  		if (kv.first!=rname){
  			Rect2 r2 = rloc[kv.first].getBoundary(collision_margin_); 			
  			//double d = edistance(rx-kv.second.x,ry-kv.second.y);
  			//avoid first time (all robots are initially placed at (0,0))
  			if (!(rx==0.0 && ry==0.0 && kv.second.x==0.0 && kv.second.y==0.0)){
  				bool collided = r1.intersects(r2);			
  				std::string msg = rname + " collides with "+kv.second.name;
  				assertm(!collided,msg);
  			}
  		}
  	}
  	
  }
  
  void getDynamicGrid(std::vector<std::int8_t>& grid, std::string exclude_robot, int expands=1){
  	grid.resize(mapinfo.size());
  	grid = map_msg_ptr->data;
  	
  	expandObstacles(grid);
  	for(int i=1;i<=expands;i++){ expandObstacles(grid); }
  	
  	RobotInfo mi = getRobotInfo(exclude_robot);
  	int min_gap = ceil((mi.width + 2*mi.safe_margin) / mapinfo.resolution);
		    
    //update dynamic locations of other robots to the grid
    for(const auto& kv : rloc){
    	if (kv.first != exclude_robot){
			RobotInfo r = getRobotInfo(kv.first);
    	    Point2i pi = mapinfo.world2pixel(r.x,r.y);
		    grid[mapinfo.pixel2index(pi.i,pi.j)]=100;
		    
		    //block narrow gap between robot and borders if there is
		    int gapleft = pi.j;
		    int gapright = mapinfo.width-pi.j;
		    int gaptop = pi.i;
		    int gapbottom = mapinfo.height-pi.i;
		    
		    if (gapleft<min_gap) for(int j=0;j<pi.j;j++) grid[mapinfo.pixel2index(pi.i,j)]=100;
		    else if (gapright<min_gap) for(int j=pi.j+1;j<mapinfo.width;j++) grid[mapinfo.pixel2index(pi.i,j)]=100;
		    
		    if (gaptop<min_gap) for(int i=0;i<pi.i;i++) grid[mapinfo.pixel2index(i,pi.j)]=100;
		    else if (gapbottom<min_gap) for(int i=pi.i+1;i<mapinfo.height;i++) grid[mapinfo.pixel2index(i,pi.j)]=100;
		     		    
       }
    }
       
	for(const auto& kv : rloc){
	    if (kv.first == exclude_robot) continue;
        RobotInfo r = getRobotInfo(kv.first);
	    Point2 p = {r.x,r.y};
	    Point2i pi = mapinfo.world2pixel(r.x,r.y);
		//if robot is moving, mark next #STOP_ZONE cells in its moving direction as occupied (in the future)
		if (r.velocity>0){
			int stop_cells = ceil(0.2*r.velocity / mapinfo.resolution);
			if (stop_cells < 2) stop_cells = 2;
			//int stop_cells = 2*ceil(stop_zone / mapinfo.resolution);
			int rdir = r.dir();
			for (int j=0;j<stop_cells;j++){
				Point2i n = pi.getNeighborAtDir(rdir);
				if (mapinfo.isInside(n.i,n.j)) {
					grid[mapinfo.pixel2index(n.i,n.j)]=100;						
				}
				pi = n;
			}
		}
	
	}   

	//make sure that the expanded cells do not cover the robot that requests the path
    Point2i pi = mapinfo.world2pixel(mi.x,mi.y);
    grid[mapinfo.pixel2index(pi.i,pi.j)]=0;
    std::vector<Point2i> n = pi.getNeighbors();
    for(int i=0;i<8;i++){
    	int gindex = mapinfo.pixel2index(n[i].i,n[i].j);
    	if (gindex>=0 && gindex<mapinfo.size()){
    		if (!map_msg_ptr->data[gindex]) grid[gindex]=0;
    	} 		
    }
    //delete n;
 
 	std::vector<Point2> markers;
	for(int i=0;i<mapinfo.rows();i++)
		for(int j=0;j<mapinfo.cols();j++){
			int gindex = mapinfo.pixel2index(i,j);
			if (grid[gindex]!=map_msg_ptr->data[gindex]){
				markers.push_back(mapinfo.pixel2world(i,j));
			}
		}
		
	showMarkers(markers,10000);
  }
  
  void expandObstacles(std::vector<std::int8_t>& grid){
  	int8_t nxt[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
  	std::map<int,bool> isnew;
	for(int i=0;i<mapinfo.rows();i++){
		for(int j=0;j<mapinfo.cols();j++){
			int index = mapinfo.pixel2index(i,j);
			//expansion is based on original cells, not expanded cells
			if (isnew.find(index)==isnew.end() && grid[index]>0) {
				for(int k=0;k<8;k++){
					int nx = i+nxt[k][0];
					int ny = j+nxt[k][1];
					int nindex = mapinfo.pixel2index(nx,ny);
					if (nindex>=0 && nindex<mapinfo.size() && !grid[nindex]){ 
						grid[nindex]=100;
						isnew[nindex]=true;
					}
				}
			}
		}
	}
  }
  
  Point2i findNearestUnoccupiedCell(int i, int j, int maxDistance) {
	std::queue<Point2i> Q;
	std::set<Point2i> S;
	
	Q.push({i,j});
	S.insert({i,j});
	int d = 0;
	while (!Q.empty()) {
		++d;
		if (d > maxDistance) break;
		int qSize = Q.size();
		for (int k = 0; k < qSize; ++k) {
		    Point2i p = Q.front(); Q.pop();
		    if (!isOccupiedOriginally(p.i,p.j)) {
		        return p;
		    }
		    for (int x = p.i - 1; x <= p.i + 1; ++x) {
		        for (int y = p.j - 1; y <= p.j + 1; ++y) {
		            if (!isOccupiedOriginally(x,y)) {
		                Point2i q = {x,y};
		                if (S.find(q) == S.end()) {
		                    Q.push(q);
		                    S.insert(q);
		                }
		            }
		        }
		    }
		}
	}
	return {-1,-1};
  }
  
  	// Service callback function
  	void generatePathServiceCallback(
  		const std::string rname,
		const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
		std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
	{
		RobotInfo robot = this->getRobotInfo(rname);

		//prepare the dynamic occupancy grid with static obstacles + mobile obstacles + future obstacles
		int expands = floor( 
			(std::min(
				robot.width,
				robot.length
			)/2 
			+ robot.safe_margin
			) / mapinfo.resolution
		);
		//do not expand obstacles for middle targets (so that the robots do not get stuck)
		std::vector<std::int8_t> grid;
		getDynamicGrid(grid,rname,request->tolerance>0?expands:0);
			
		//A* algorithm to compute path
		std::vector<Astar_node> nodes;
		std::vector<int> open;
		
		nodes.resize(grid.size());
		open.resize(128);
		for(int i=0;i<mapinfo.rows();i++){
			for(int j=0;j<mapinfo.cols();j++){
				int gindex = mapinfo.pixel2index(i,j);
				int nindex = i*mapinfo.cols()+j;
				Astar_node node = Astar_node{i,j,VERY_LARGE,VERY_LARGE,VERY_LARGE,-1,false,grid[gindex]};
				nodes[nindex]=node; //make sure inserting at the right index, do not use push_back()
			}
		}
		//grid is stored in [nodes] array already
		grid.clear(); grid.shrink_to_fit();
		
		Point2 start = {request->start.pose.position.x, request->start.pose.position.y};
		Point2 goal = {request->goal.pose.position.x, request->goal.pose.position.y};
		
		Point2i pstart = mapinfo.world2pixel(request->start.pose.position.x, request->start.pose.position.y);
		Point2i pgoal = mapinfo.world2pixel(request->goal.pose.position.x, request->goal.pose.position.y);	
		
		RCLCPP_INFO(this->get_logger(),
			"[%s] asks for path from [%d,%d] to [%d,%d]",rname.c_str(),
			pstart.i,pstart.j,pgoal.i,pgoal.j);
			
		if (isOccupiedOriginally(pstart.i,pstart.j)){
			Point2i free = findNearestUnoccupiedCell(pstart.i,pstart.j,mapinfo.width/2);
			if (mapinfo.isInside(free.i,free.j)){
				RCLCPP_INFO(this->get_logger(),"start is an occupied cell, changed to [%d,%d]",free.i,free.j);
				pstart = free;
				start = mapinfo.pixel2world(free.i,free.j);
			}						
		}		
			
		if (isOccupiedOriginally(pgoal.i,pgoal.j)){
			Point2i free = findNearestUnoccupiedCell(pgoal.i,pgoal.j,mapinfo.width/2);
			if (mapinfo.isInside(free.i,free.j)){
				RCLCPP_INFO(this->get_logger(),"goal is an occupied cell, changed to [%d,%d]",free.i,free.j);
				pgoal = free;
				goal = mapinfo.pixel2world(free.i,free.j);
			}						
		}	

		int index = pstart.toIndex(mapinfo.cols());
		
		nodes[index].checked=true;
		nodes[index].g = 0; //distance to start point
		nodes[index].h = odistance(pstart.i-pgoal.i,pstart.j-pgoal.j); //cost heuristic to reach goal
		nodes[index].f = nodes[index].g + nodes[index].h; //composite cost
		open.push_back(index);
				
		int8_t offsets[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};		
		bool found = false;
		while (!open.empty()){
			double lowest_f=VERY_LARGE;
			int remove_idx=-1;
			index = -1;
			//get the node with smallest f value (f=g+h)
			for(auto it = open.begin();it != open.end(); it++){
			    Astar_node node = nodes[*it];
				if (node.f < lowest_f){
					lowest_f = node.f;
					remove_idx = std::distance(open.begin(),it);
					index = *it;	
				}
			}
			
			if (index<0) {
				found = false;
				break;
			}
			
			if (nodes[index].i==pgoal.i && nodes[index].j==pgoal.j){
				found = true;
				break;
			}
			
			nodes[index].checked=true;//visited
					
			//remove from open list
			open.erase(open.begin()+remove_idx);

			for(int k=0;k<7;k++){
				int nx = nodes[index].i + offsets[k][0];
				int ny = nodes[index].j + offsets[k][1];
				if (mapinfo.isInside(nx,ny)){
					int nindex = nx*mapinfo.cols() + ny;
					if (!nodes[nindex].checked && nodes[nindex].occupied == 0){					
						//distance to current node: if diagonal --> sqrt(2) else 1
						double g = nodes[index].g + ((nx==nodes[index].i||ny==nodes[index].j)?1:SQRT2);
						if (g < nodes[nindex].g){
							nodes[nindex].checked = true;
							nodes[nindex].g = g;
							nodes[nindex].h = odistance(nx-pgoal.i,ny-pgoal.j);
							nodes[nindex].f = nodes[nindex].g + nodes[nindex].h;	
							nodes[nindex].parent = index;		
							//if not in open list
							if (std::find(open.begin(),open.end(),nindex)==open.end()){
								open.push_back(nindex);
							}
						}
					}
				}
			}
		}
		nav_msgs::msg::Path path_msg;
		path_msg.header.stamp = this->now();
		path_msg.header.frame_id = "map";
		
		if (found){
			RCLCPP_INFO(this->get_logger(),"Path is generated");
			bool combine_ziczac = get_parameter("combine_ziczac").as_int()!=0;
			std::vector<int> path;
			//construct the path based on traceback graph 
			index = pgoal.toIndex(mapinfo.cols());
			path.push_back(index);
			while(true){
				int parent_index = nodes[index].parent;
				if (parent_index>=0){
					path.push_back(parent_index);
				} else {
					break;
				}
				index = parent_index;
			}
			std::reverse(path.begin(),path.end());
			int prevx=pstart.i;
			int prevy=pstart.j;
			int prevdir = 9999;
			int maxi = path.size()-1;
			int removex=-1,removey=-1;
			
			for(int i=1;i<path.size();i++){
				index = path[i];
				Astar_node node = nodes[index];
				int dx = node.i-prevx;
				int dy = node.j-prevy;
				int dir = 0;
				if (dx==0) dir = dy>0?2:6;
				else if (dy==0) dir = dx>0?0:4;
				else if (dx*dy>0) dir = dx>0?1:5;
				else dir = dx<0?3:7;
				
				//combine ziczac points
				if (combine_ziczac){
					if (i>2 && i<path.size()-2){
						Astar_node node1 = nodes[path[i-1]];
						if (!(removex==node1.i && removey==node1.j)){
						Astar_node node2 = nodes[path[i+1]];
						if ( mdistance(node.i-node1.i,node.j-node1.j)==1 
							&& 
							 mdistance(node.i-node2.i,node.j-node2.j)==1)
							{ 
							//TODO: do not skip point if it is to avoid a corner of an obstacle
							prevx = node.i;prevy=node.j;
							removex = node.i,removey=node.j;
							//keep orientation of previous node (node1)
							//prevdir=dir;
							continue; 
						}
						}
					}
				}
				
				Point2 p = mapinfo.pixel2world(node.i,node.j);
				double yaw = dir*45*DEG2RAD;
				
				geometry_msgs::msg::PoseStamped pose;
				pose.header.stamp = this->now();
				pose.header.frame_id = "map";
				pose.pose = p.toPose(yaw);

				//combine points if same direction as the last one
				if (dir == prevdir && path_msg.poses.size()>0){
					path_msg.poses.back() = pose;
				} else {				
					path_msg.poses.push_back(pose);
				}
				prevx = node.i;prevy=node.j;prevdir=dir;							
			}					
		} else {
			RCLCPP_INFO(this->get_logger(),"Path not found");
			//path_msg.poses.push_back(request->start);	
			//path_msg.poses.push_back(request->goal);	
		}
		
		this->paths[rname] = path_msg;
		
		response->plan = path_msg;
		
		//for(auto it=nodes.begin();it!=nodes.end();it++){
			//delete *it;
		//};
		nodes.clear(); nodes.shrink_to_fit();
		
		//publish the path for rviz to visualize
		//this->path_pubs[rname]->publish(path_msg);		
	}

  void showMarker(Point2 p,unsigned int milisecs=10){
  	std::vector<Point2> v;
  	v.push_back(p);
  	showMarkers(v,milisecs);
  }
  
  void showMarkers(std::vector<Point2> points, unsigned int milisecs = 10){
	visualization_msgs::msg::MarkerArray marker_array;
	
		
	std::string ns = "map_markers";
	
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.ns = ns;
	marker.type = visualization_msgs::msg::Marker::SPHERE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	marker.color.a = 0.7;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.lifetime = rclcpp::Duration(std::chrono::nanoseconds(milisecs*1000000));
	marker.frame_locked = true;
	
	marker_array.markers.clear();
	for(int i=0;i<points.size();i++){
		marker.header.stamp = this->now();
		marker.id = i;				
		marker.pose.position.x = points[i].x;
		marker.pose.position.y = points[i].y;
		marker_array.markers.push_back(marker);
	}
	marker_array_pub_->publish(marker_array);	
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
