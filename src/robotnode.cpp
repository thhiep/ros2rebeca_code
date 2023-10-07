
#include "ros2demo/common.hpp"

#define MOVE_UPDATE_RATE 5

class RobotNode : public rclcpp::Node
{
public:
  RobotNode() : Node("robotnode")
  {
    srand((unsigned)time(NULL));
    
	std::vector<std::string> params{
    	"map_pgm","map_yaml","map_resol","map_occupied_thresh","map_free_thresh",
    	"robot_id","robot_length","robot_width","robot_radius","safe_margin","stop_zone",
    	"velocity","angular_velocity","replan","scan_first","max_waiting_time",
    	"x","y","tx","ty","goal_tolerance",
    	/*"laserscan_rate","fov","angle_inc","max_range","min_range"*/
    };
    for(int i=0;i<params.size();i++){
    	declare_parameter(params[i]);
    }    


    std::string map_file_path = (this->get_parameter("map_pgm")).as_string();
    cv::Mat map_image = cv::imread(map_file_path, cv::IMREAD_GRAYSCALE);    

    map_msg_ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        
	map_msg_ptr->header.frame_id = "map";
    map_msg_ptr->info.resolution = (this->get_parameter("map_resol")).as_double(); // Change this to match your map's resolution
    map_msg_ptr->info.width = map_image.cols;
    map_msg_ptr->info.height = map_image.rows;
    map_msg_ptr->info.origin.position.x = -(map_image.cols * map_msg_ptr->info.resolution)/2;
    map_msg_ptr->info.origin.position.y = -(map_image.rows * map_msg_ptr->info.resolution)/2;
    map_msg_ptr->info.origin.orientation.w = 1;
    map_msg_ptr->data.resize(map_image.total());
    for (int i = 0; i < map_image.rows; i++) {
      for (int j = 0; j < map_image.cols; j++) {
        int occupancy = (map_image.at<uchar>(i, map_image.cols-1-j) > OCCUPIED_GRAY_LEVEL) ? 0 : 100;
        map_msg_ptr->data[i * map_image.cols + j] = occupancy;
      }
    }  
    
    this->mapinfo = {map_msg_ptr};
     
    //meters per second
    default_linear_velocity = this->get_parameter("velocity").as_double();
    //radians per second
    default_angular_velocity = get_parameter("angular_velocity").as_double()*DEG2RAD;//180*DEG2RAD;
    
    angle_tolerance = 2*DEG2RAD;//2*MOVE_UPDATE_RATE * default_angular_velocity / 1000;
    linear_tolerance = 0.1 * mapinfo.resolution;//MOVE_UPDATE_RATE * default_linear_velocity / 1000;
    goal_tolerance = 0.5 * mapinfo.resolution;
    
    robot_length = get_parameter("robot_length").as_double();
    robot_width = get_parameter("robot_width").as_double();
    robot_radius = 0.5*sqrt(robot_width*robot_width + robot_length*robot_length);
    safe_margin = get_parameter("safe_margin").as_double();
    stop_zone = get_parameter("stop_zone").as_double();
    if (has_parameter("max_waiting_time"))
    	max_waiting_time_ = get_parameter("max_waiting_time").as_int();
    else
    	max_waiting_time_ = 5000;//miliseconds
    
    scan_first = get_parameter("scan_first").as_int()!=0;
    
    previous_time_ = this->now();
  	last_update_obstacles_ = this->now();
    x_=y_=theta_=prevX=prevY=prevTheta=0.0;
    
	//linear_velocity_ = rand_float(-1.7,2.1);
	//angular_velocity_ = rand_float(0.7,1.8);
	
  	auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  	
  	map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    	"/map", 
    	1, 
    	std::bind(&RobotNode::onOccupancyGrid, this, std::placeholders::_1));
    
  	reg_pub_ = create_publisher<std_msgs::msg::String>("/register",10);
  	timer_every_sec_ = create_wall_timer(std::chrono::seconds(10),
		std::bind(&RobotNode::every_second,this));
		
	marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      this->getNamespace()+"/obstacles", 10);	
  	
    laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    	this->getNamespace() + "/laserscan", 
    	default_qos, 
    	std::bind(&RobotNode::onLaserScan, this, std::placeholders::_1));
    
   // Publish odometry information at 10 Hz
   	//tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(this->getNamespace() + "/odom", 10);
    
    this->targets_.push_back(
    		mapinfo.pixel2world(
    			(this->get_parameter("tx")).as_int(),
    			(this->get_parameter("ty")).as_int()
    		));
    Point2 p = mapinfo.pixel2world(
    	(this->get_parameter("x")).as_int(),
    	(this->get_parameter("y")).as_int()
    );
    
    this->set_pose(p.x,p.y,M_PI+p.angle());

    // Subscribe to the /cmd_vel topic to receive movement commands
    cmd_vel_sub_ =
        create_subscription<geometry_msgs::msg::Twist>(
            this->getNamespace() + "/cmd_vel", 10,
            std::bind(&RobotNode::velocity_callback, this, std::placeholders::_1));
            
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    	this->getNamespace()+"/cmd_vel",
    	default_qos
    );

	//wait for rviz to start up
	rclcpp::sleep_for(5s);
	
    // Initialize timer to publish odometry at 10 Hz
    move_timer_ = create_wall_timer(std::chrono::milliseconds(MOVE_UPDATE_RATE),
                               std::bind(&RobotNode::updateMovingStatus, this));
                               
  }

private:
  bool registered_=false;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_ptr;
  MapInfo mapinfo;
  
  void onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  	//this->map_msg_ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
  }
  
  std::vector<Point2> obstacles_;
  rclcpp::Time last_update_obstacles_;
  void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
  	registered_=true;
	
	obstacles_.clear();
	
	for(int i=0;i<msg->ranges.size();i++){
		if (msg->ranges[i]>=msg->range_min && msg->ranges[i] <=msg->range_max){
			double angle = msg->angle_min + i*msg->angle_increment;
			Point2 p;
			p.x = msg->ranges[i] * cos(angle);
			p.y = msg->ranges[i] * sin(angle);						
			p.rotate(-theta_);
			p.shift(-x_,-y_);
			obstacles_.push_back(p);
		}
	}
	//should be the same as laserscan dots
	//showMarkers(obstacles_,"map");
	last_update_obstacles_ = this->now();
  }
  
  void showMarker(Point2 p,std::string frame_id="", unsigned int milisecs=10){
  	std::vector<Point2> v;
  	v.push_back(p);
  	showMarkers(v,frame_id,milisecs);
  }
  
  void showMarkers(std::vector<Point2> points, std::string frame_id="",unsigned int milisecs = 10){
	visualization_msgs::msg::MarkerArray marker_array;
	
	if (frame_id=="")
		frame_id = this->getRobotName() + "/base_link";
		
	std::string ns = this->getRobotName() + "_markers";
	
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = frame_id;
	marker.ns = ns;
	marker.type = visualization_msgs::msg::Marker::SPHERE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
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
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reg_pub_;	
  
  rclcpp::TimerBase::SharedPtr timer_every_sec_;
  rclcpp::TimerBase::SharedPtr move_timer_;
  rclcpp::Time previous_time_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;	
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  
  std::string robot_name = "";
  double x_,y_,theta_;
  double linear_velocity_,angular_velocity_;
  double prevX,prevY,prevTheta;
  double default_linear_velocity;
  double default_angular_velocity;
  double angle_tolerance;
  double linear_tolerance;
  double goal_tolerance;
  double robot_length;
  double robot_width;
  double robot_radius;
  double safe_margin;
  double stop_zone;
  bool scan_first;
  
  std::vector<Point2> targets_;
  std::vector<Point2> moves_;
  int moveidx_=0;
  bool isWaiting_ = false;
  double waitingDir_;
  rclcpp::Time clockWaiting_;
  uint32_t max_waiting_time_ = 5000;//miliseconds
  int failures_=0;
  
  //std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  
  rclcpp::Time now(){
  	return rclcpp::Clock(RCL_ROS_TIME).now();
  }
  
  float rand_float(float min, float max){
  	float r = min + (max-min)*(float)rand() / RAND_MAX;
  	//int sign = rand() % 2? -1:1;
  	return r;
  }
    
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  	// Update linear and angular velocities based on the received message
    linear_velocity_ = msg->linear.x;
    angular_velocity_ = msg->angular.z;
    //RCLCPP_INFO(this->get_logger(),"%s/cmd_vel %0.2f %0.2f",(this->getNamespace()).c_str(),linear_velocity_,angular_velocity_);
  }
  
  void set_speeds(double v,double av){
  	 geometry_msgs::msg::Twist msg;
  	 msg.linear.x = v;
  	 msg.angular.z = av;
  	 this->cmd_vel_pub_->publish(msg);
  }
  
  std::string getNamespace(){
  	std::string str(this->get_namespace());
  	return str;
  }
  
  std::string getRobotName(){
  	if (robot_name.empty()){
  		std::string str(this->get_namespace());
  		str.erase(std::remove(str.begin(),str.end(),'/'),str.end());
  		robot_name = str;
  	}
  	return robot_name;
  }
  
  std::string getFrameId(const std::string frame,bool strip_first_slash=true){
  	std::string ns(this->get_namespace());
  	if (ns.empty()) return frame;
  	if (strip_first_slash) if (ns.compare(0,1,"/")==0) ns = ns.substr(1,ns.size()-1);
  	return ns+"/"+frame;
//  	return ns.empty() ? frame : ns+"/"+frame;
  } 
  
  //world -> map -> odom -> base_link -> base_laser (if have)
  void publish_odom(){
    
    double delta_x = x_ - prevX;
    double delta_y = y_ - prevY;
    double delta_theta = theta_ - prevTheta;
    
    tf2::Quaternion quaternion;
    
    //TF order: map -> odom -> base_link -> other frames in the robot
    tf2_ros::TransformBroadcaster tf_broadcaster(this);	
    	
	geometry_msgs::msg::TransformStamped tf_msg;
	
	tf_msg.header.frame_id = "map";
	tf_msg.child_frame_id = this->getFrameId("odom");

	tf_msg.transform.translation.x = x_;
	tf_msg.transform.translation.y = y_;
	tf_msg.transform.translation.z = 0.0;
	quaternion.setRPY(0, 0, theta_);
	tf_msg.transform.rotation = tf2::toMsg(quaternion);

	tf_msg.header.stamp = this->now();
	tf_broadcaster.sendTransform(tf_msg);   
	
	//base_link is same as odom, for simplicity
	tf_msg.header.frame_id = this->getFrameId("odom");
	tf_msg.child_frame_id = this->getFrameId("base_link");
	tf_msg.transform.translation.x = 0;
	tf_msg.transform.translation.y = 0;
	tf_msg.transform.translation.z = 0;
	quaternion.setRPY(0, 0, 0);
	tf_msg.transform.rotation = tf2::toMsg(quaternion);  
	tf_msg.header.stamp = this->now();
	tf_broadcaster.sendTransform(tf_msg);   
  
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = this->getFrameId("odom");

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0;

    quaternion.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();

    odom_msg.twist.twist.linear.x = linear_velocity_;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;

    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = angular_velocity_;

    odom_pub_->publish(odom_msg);       
  
  }
  
  double normalizeAngleRad(double a){
  	 a = a - 2*M_PI*floor(a/(2*M_PI));
  	 return a;
  }
  
  double normalizeAngleDegree(double a){
  	 a = a-360*floor(a/360);
  	 return a;
  }
    
  void set_pose(double x, double y, double angle){
  	//if (!isOccupied(p.i,p.j))
  	{
    	prevX=x_;prevY=y_;prevTheta=theta_;
  		x_ = x;
  		y_ = y;
  		theta_ = normalizeAngleRad(angle);
    }
   	publish_odom();
  }
  
  void registerRobot(){
	std_msgs::msg::String msg; 
	msg.data = this->getNamespace();
	reg_pub_->publish(msg);	  
  }
  
  void stop(bool flush=false){
  	 linear_velocity_ = 0;
  	 angular_velocity_ = 0;
  	 if (flush){
  	 	this->moves_.clear();
  	 	this->moveidx_ = 0;
  	 } 	 
  }
  
  void every_second(){
  	registerRobot();
  }
  
  Point2 world2robot(double x,double y,double angle){
  	Point2 p = {x,y};
  	p.rotate(angle);
  	p.shift(x_,y_);
  	return p;
  }
  
  Point2 robot2world(double xr,double yr,double angle){
  	Point2 p = {xr,yr};
  	p.rotate(-angle);
  	p.shift(-x_,-y_);
  	return p;
  }
  
  double getNextDir(){
  	double dir = theta_;
  	if (this->moves_.size()>0 && this->moveidx_ < (this->moves_.size()-1)){
  		Point2 current = {x_,y_};
  		Point2 next = moves_[moveidx_+1];
  		Point2 v = {next.x-current.x,next.y-current.y};
  		dir = v.angle();
  	}
  	return dir;
  }
  
Point2 getBoundaryIntersection(double vx,double vy,double vtheta, double width, double height) {
    double slope = tan(vtheta);
    double y_intercept = vy - slope * vx;

    double left_x = 0;
    double right_x = width;
    double top_y = 0;
    double bottom_y = height;

    double left_y = slope * left_x + y_intercept;
    double right_y = slope * right_x + y_intercept;
    double top_x = (top_y - y_intercept) / slope;
    double bottom_x = (bottom_y - y_intercept) / slope;

    Point2 left_intersection = {left_x, left_y};
    Point2 right_intersection = {right_x, right_y};
    Point2 top_intersection = {top_x, top_y};
    Point2 bottom_intersection = {bottom_x, bottom_y};

    double min_distance = INFINITY;
    Point2 closest_intersection;

    double left_distance = sqrt(pow(left_x - vx, 2) + pow(left_y - vy, 2));
    double right_distance = sqrt(pow(right_x - vx, 2) + pow(right_y - vy, 2));
    double top_distance = sqrt(pow(top_x - vx, 2) + pow(top_y - vy, 2));
    double bottom_distance = sqrt(pow(bottom_x - vx, 2) + pow(bottom_y - vy, 2));

    if (left_distance < min_distance) {
        min_distance = left_distance;
        closest_intersection = left_intersection;
    }
    if (right_distance < min_distance) {
        min_distance = right_distance;
        closest_intersection = right_intersection;
    }
    if (top_distance < min_distance) {
        min_distance = top_distance;
        closest_intersection = top_intersection;
    }
    if (bottom_distance < min_distance) {
        min_distance = bottom_distance;
        closest_intersection = bottom_intersection;
    }

    return closest_intersection;
}  
  
  
  double getObstacleAtDir(double angle,bool stop_at_border=false){
  	static double margin = robot_width/2 + safe_margin;
  	static double safe = robot_length/2 + stop_zone;
  	static double time_to_stop = SEC2MILISEC * safe / default_linear_velocity;
  	
  	if (obstacles_.empty()) return VERY_LARGE;
  	
  	double nearest = VERY_LARGE;
  	for(int i=0;i<obstacles_.size();i++){
	  Point2 o = obstacles_[i];	//in map frame
	  o.trans(x_,y_,angle);	//in current odom frame
	  if (o.x>0 && abs(o.y)<=margin){
		 double ol = o.x;//o.edistance(0,0);
	  	 if (ol < nearest) nearest = ol;
	  }
	}
	
	if (nearest==VERY_LARGE && stop_at_border){
		//Point2 p1=mapinfo.pixel2world(-1,-1);		
		//Point2 p2=mapinfo.pixel2world(mapinfo.rows(),mapinfo.cols());
		Point2 r = mapinfo.world2map(x_,y_);
		Point2 s = getBoundaryIntersection(r.x,r.y,angle,mapinfo.width,mapinfo.height);
		nearest = r.edistance(s.x,s.y) * mapinfo.resolution;
		showMarker(mapinfo.map2world(s.x,s.y),"map",10000);
	}
	
	return nearest;
  }
  
  //get direction with closest obstacle to avoid
  double getDirBlocked(){
	double mind = VERY_LARGE;
	double mina = 0;
  	for(int i=0;i<obstacles_.size();i++){
	  Point2 o = obstacles_[i];	//in map frame
	  o.shift(x_,y_);
	  double ol = o.edistance(0,0);
	  if (ol < mind) {
	  	mind = ol;
	  	mina = o.angle();
	  }
	}
	return mina;	
  }
  
  ObstacleDetection getDirWithoutObstacle(
  		double angle_min,double angle_max,double angle_step,
  		double min_range,bool stop_at_border=false
  ){
	angle_min = normalizeAngleRad(angle_min);
	angle_max = normalizeAngleRad(angle_max);  
  	double a = angle_min;
  	//if (angle_min>angle_max) angle_step = -angle_step;	
  	int steps = ceil(2*M_PI/angle_step);
  	double maxd=0;
  	double maxa=angle_min; 	
  	std::vector<ObstacleDetection> free;//directions that are free of obstacles within min_range
  	for(int i=0;i<steps;i++,a+=angle_step){
  		double d = getObstacleAtDir(a,stop_at_border);
  		//if (d==VERY_LARGE) return a;
  		if (d>min_range){
  			free.push_back({d,a});
	  		if (d>maxd) {
	  			maxd = d;
	  			maxa = a;
	  		}
	  	}
  	}
  	
  	ObstacleDetection od = {maxd,maxa};
  	//return od; 	
  	//select a random free direction to avoid sticking to just the one with farthest obstacle
  	if (free.size()>0){ 
	  	std::sort(free.begin(),free.end(),std::greater<ObstacleDetection>());
	  	int i = rand() % (1 + (int)((double)free.size()/2));
	  	od = free[i];
  	}
  	return od;
  }  
  
  bool canMoveAtDir(double angle){
  	static double margin = robot_width/2 + safe_margin;
  	
  	double steplen = default_linear_velocity*(MOVE_UPDATE_RATE/1000); 
  	double safe = steplen + robot_length/2 + stop_zone; 	
  	double time_to_stop = SEC2MILISEC * safe / default_linear_velocity;
  	
  	//if scan before move, do not use outdated scanning result
  	if (scan_first){
  		rclcpp::Time now = this->now();
  		double passed = (now - last_update_obstacles_).nanoseconds()/MILISEC2NANOSEC;
  		double maxage = 1000*this->mapinfo.resolution / default_linear_velocity;
  		if (passed > maxage) return false;
  	}
  	
  	if (obstacles_.empty()) return true;	
  	
  	double nearest = VERY_LARGE;
  	std::vector<Point2> markers;
	for(int i=0;i<obstacles_.size();i++){
	  Point2 o = obstacles_[i];	//in map frame
	  o.trans(x_,y_,angle);	//in current odom frame
	  if (o.x>0 && abs(o.y)<=margin){
	  	 double ol = o.x;//o.edistance(0,0);
	  	 if (ol < nearest) nearest = ol;
	  	 markers.push_back(o); 
	  }
	}
	this->showMarkers(markers);	
	bool can = nearest > safe;
	return can; 
  }
  
  void startWaiting(){
  	if (!isWaiting_){
  		isWaiting_ = true;
  		clockWaiting_ = now();
  		waitingDir_ = theta_;
  	}
  	linear_velocity_ = 0.0;
  }
  
  void updateMovingStatus() {
  	//find path to first goal in queue
  	if (registered_ && !this->targets_.empty() && this->moves_.empty()){
		Point2 m = this->targets_.front();
  		setTarget(m.x,m.y);
 		
  		if (targets_.size()>1){
  			Point2i i1 = mapinfo.world2pixel(x_,y_);
  			Point2i i2 = mapinfo.world2pixel(m.x,m.y);
  			RCLCPP_INFO(this->get_logger(),"is finding path from [%d,%d] to [%d,%d]",i1.i,i1.j,i2.i,i2.j);
  		}
  	}  
    auto current_time = this->now();
    double elapsed_time = (current_time - previous_time_).nanoseconds() / SEC2NANOSEC;
    previous_time_ = current_time;
    
    //reached target
    double delta_x=0.0;
    double delta_y=0.0;
    double delta_theta=0.0;
    bool waiting_in_progress = isWaiting_;
    
	if (canMoveAtDir(theta_)){
		isWaiting_ = false;
	} else {
		if (waiting_in_progress){
			double waiting_duration = (current_time - clockWaiting_).nanoseconds() / MILISEC2NANOSEC; 	
			//RCLCPP_INFO(this->get_logger()," has been waiting for %0.0f ms",waiting_duration);
			
			if (waiting_duration > max_waiting_time_){
				//divert away from the congestion point, try another path
				double cells = ceil((robot_length/2 + stop_zone) / mapinfo.resolution);
				int rid = get_parameter("robot_id").as_int();
				double goback = rand_float(1.0,3.0) * cells * mapinfo.resolution;
				double angle_start = getDirBlocked() + M_PI/2;
				double angle_end = angle_start + M_PI;
				ObstacleDetection od = getDirWithoutObstacle(angle_start,angle_end,M_PI/8,goback,true);
				
				if (od.range > goback){
					Point2 moveto = {goback,0};
					//moveto.rotate(-(od.angle-theta_));
					moveto.rotate(-od.angle);
					moveto.shift(-x_,-y_);
					isWaiting_ = false;
					this->targets_.insert(this->targets_.begin(),moveto);
					this->moves_.clear();
					this->moveidx_=0;
					this->showMarker(moveto,"map",10000);
				}
				//keep looking around to find free directions
				angular_velocity_ = default_angular_velocity;									
 			} else {
 				//keep waiting in the same
	 			angular_velocity_ = 0.0;									
 			}
		} else if (linear_velocity_ > 0) {
			isWaiting_ = true;
			if (!waiting_in_progress){
				clockWaiting_ = this->now();
				waitingDir_ = theta_;
			}
			//linear_velocity_ *= 0.5;//deaccelerate
			linear_velocity_ = 0;
		}		
	}
	    
    double v = linear_velocity_;
    double av = angular_velocity_;
    
    // Calculate new pose based on velocities and time elapsed
    delta_theta = av * elapsed_time;
	if (v!=0){
		delta_x = v * std::cos(theta_) * elapsed_time;
		delta_y = v * std::sin(theta_) * elapsed_time;
	}
	
	set_pose(x_+delta_x,y_+delta_y,theta_+delta_theta);
    
    if (this->moves_.size()>0 && this->moveidx_<this->moves_.size()){
    	if (this->moveidx_<0) this->moveidx_=0;//start moving
    	Point2 m = this->moves_[this->moveidx_];
    	Point2 vec = {m.x - x_, m.y - y_};
    	double vangle = vec.angle();
    	double delta_theta2 = vangle - theta_;
    	double gap_theta = abs(delta_theta2);
    	int rotate_dir = (delta_theta2 > 0 ? 1:(delta_theta2<0?-1:0));
    	double gap_xy = vec.length();
    	//RCLCPP_INFO(this->get_logger(),"%s [%0.2f=%0.2f-%0.2f,%0.2f",this->getRobotName().c_str(),gap_theta,theta_,vec.angle(),gap_xy);
    	
    	double reached_theta = gap_theta <= angle_tolerance;
    	//set to the right angle if close enough, not really true in reality but better for simulation
    	if (reached_theta){theta_ = vangle;}
    	
    	double reached_xy = gap_xy <= linear_tolerance;
    	bool isfinal = moveidx_ == (moves_.size()-1);
    	if (isfinal){
    		reached_xy = gap_xy <= goal_tolerance;
    	}
    	//if not to the right orientation, continue rotating only
    	if (!reached_xy){
    		if (reached_theta){
    			angular_velocity_ = 0;
    			bool canmove = canMoveAtDir(theta_);
    			if (canmove){
    				isWaiting_ = false;
    				linear_velocity_ = default_linear_velocity;
    			} else {
    				isWaiting_ = true;
    				if (!waiting_in_progress) {
    					clockWaiting_ = this->now();
    					waitingDir_ = theta_;
    				}
    				linear_velocity_ = 0;		
    			}
    		} else {
    			linear_velocity_ = 0;
    			angular_velocity_ = rotate_dir * default_angular_velocity;
    		}
    	} else {
    		this->moveidx_++;
    		if (isfinal){
    			linear_velocity_ = 0;
    			angular_velocity_ = 0;
    			reached_theta=true;
    			isWaiting_ = false;
    			moves_.clear();
    			moveidx_=0;
    			targets_.erase(targets_.begin());
    			RCLCPP_INFO(this->get_logger(),
    				targets_.size()>0?"reached a middle target":"reached final target");
    		} else {
				Point2 m = this->moves_[this->moveidx_];
				Point2 vec = {m.x - x_, m.y - y_};
				double vangle = vec.angle();
				double delta_theta2 = vangle - theta_;
				double gap_theta = abs(delta_theta2);
    			int rotate_dir = (delta_theta2 > 0 ? 1:(delta_theta2<0?-1:0));
				if (gap_theta > angle_tolerance){
					angular_velocity_ = rotate_dir * default_angular_velocity;
					linear_velocity_ = 0;
				} else {
					bool canmove = canMoveAtDir(vangle);
					angular_velocity_ = 0;
					if (canmove){
						isWaiting_ = false;
						linear_velocity_ = default_linear_velocity;
					} else {
						isWaiting_ = true;
						if (!waiting_in_progress){
							clockWaiting_ = this->now();
							waitingDir_ = theta_;
						}
						linear_velocity_ = 0;
					}
				}
    		}
    	}
    }
  }
  
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr path_service_ = NULL; 
  	
  void setTarget(double tx, double ty){
  //	stop(true);
  
  	if (failures_>10){
  		RCLCPP_INFO(this->get_logger(),"Failed many times to get path. Quit requesting.");
  		if (targets_.size()>1) targets_.erase(targets_.begin(),targets_.end()-1);
  		this->moves_.clear();
  		this->moveidx_=0;
  		failures_=0;
  		return;
  	}

  	if (path_service_ == nullptr){
  		path_service_ = this->create_client<nav_msgs::srv::GetPlan>(
  			this->getNamespace()+"/generate_path");
  	}
  	
  	if (path_service_->wait_for_service(std::chrono::seconds(1))){	  		
  	  	auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
		
		request->start.header.frame_id = "map";   
		request->start.pose.position.x = x_;
		request->start.pose.position.y = y_;
		request->start.pose.orientation.w = 1.0;
		
		request->goal.header.frame_id = "map";
		request->goal.pose.position.x = tx;
		request->goal.pose.position.y = ty;
		request->goal.pose.orientation.w = 1.0;
		
		//we utilize unused fields to send other options of path generation
		int isfinal = targets_.size()>1?0:1;
		request->tolerance = isfinal;
		
		//request->planner_id = "GridBased";
		
		using ServiceResponseFuture =
			rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture;
			
		auto on_response = [this](ServiceResponseFuture future) {
			this->onNewPath(future.get()->plan.poses);			
		};
		auto result = path_service_->async_send_request(request, on_response);
	}	
  }
  
  void onNewPath(std::vector<geometry_msgs::msg::PoseStamped> poses){
  	this->moves_.clear();
  	this->moveidx_=0;
  	this->isWaiting_=false;
  	
  	if (poses.empty()){
  		//RCLCPP_INFO(this->get_logger(),"No path found");
  		failures_++;
  	} else {
  		failures_--;
  		for (auto p : poses) {
  			Point2 m;
		    m.x = p.pose.position.x;
		    m.y = p.pose.position.y;
		    this->moves_.push_back(m);
			
			//int dir = floor(m.yaw/(M_PI/4));
			//Point2i pi = mapinfo.world2pixel(p.pose.position.x,p.pose.position.y);
		    //RCLCPP_INFO(this->get_logger(),"[%d,%d,%d] ",pi.i,pi.j,dir);		    
		}
		
		//same for goal point
		targets_.front() = moves_.back();
		
    }
  }
  
   
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<RobotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
