#include <rover_controller/controller.h>
#include <Eigen/Dense>

namespace rover_controller
{

  Controller::Controller() : 
    has_goal_(false),
    map_frame_("map"),
    rover_frame_("rover"),
    rover_radius_(0.8)
  {
  }

  Controller::~Controller()
  {
  }

  bool Controller::initialize(ros::NodeHandle& nh)
  {
    // setup ros connections

    //advertise a geometry_msgs::Twist to replace the previous keyboard topic
    command_pub_ = nh.advertise<geometry_msgs::Twist>("/key_vel",1);// advertise the topic

    field_pub_ = nh.advertise<geometry_msgs::PoseArray>("/field",1);// advertise a geometry_msgs::PoseArray with name "/field"
    //Hint: see http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseArray.html

    //goal_sub_ nh.subscribe();

    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, 
                             &Controller::goalCallback,
                             this);

    // Subscribe to the obstacles msg published by the package object_server                     
    obstacles_sub_ = nh.subscribe("/obstacles", 1, 
                             &Controller::obstacleCallback, this);
    //not sure about the name of this topic, need to check!!!
    //get the parameters of this node (see config.yaml)

    controller_sub_ = nh.subscribe("/mobile_base_controller/odom",1,&Controller::controllerCallback,this);
    controller_pub_ = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",1);

    std::vector<double> v;
    double d;

    if(!ros::param::get("attractor_gain", v))
      return false;
    K_attractor_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    if(!ros::param::get("repulsive_gain", d))
      return false;
    k_repulsive_ = d;

    if(!ros::param::get("lambda_repulsive", d))
      return false;
    lambda_repulsive_ = d;
  
    if(!ros::param::get("vortex_gain", d))
      return false;
    k_vortex_ = d;

    if(!ros::param::get("lambda_vortex", d))
      return false;
    lambda_vortex_ = d;

    // setup variables
    createVectorField(field_points_w_);

    prev_time_ = ros::Time::now();
    return true;
  }

  void Controller::update(const ros::Time& time, const ros::Duration durtation)
  {
    // get the rover pose in the map frame
    Eigen::Vector3d rover_pos_w = roverPos();

    // Rotation rover with respect to world
    Eigen::Matrix3d R_b_w = Eigen::Matrix3d::Identity();
    R_b_w.topLeftCorner(2,2) = Eigen::Rotation2Dd(rover_pos_w.z()).matrix();

    // compute the thrust vector of the rover
    Eigen::Vector3d thrust_w = computeForce(
      obstacles_, goal_pos_w_, rover_pos_w);

    // transform into body frame for the rover command
    Eigen::Vector3d thrust_b = R_b_w.transpose()*thrust_w;
   
    // send to the rover
    publishCommand(thrust_b);

    // visualize the field at low frequency
    if(time - prev_time_ > ros::Duration(1./5.))
    {
      prev_time_ = time;
      visualizerVectorField(obstacles_, goal_pos_w_, field_points_w_);
    }
  }

  Eigen::Vector3d Controller::computeForce(
    std::vector<Obstacle> obstacles, const Eigen::Vector3d& goal, 
    const Eigen::Vector3d& pos)
  {
    // compute the thrust vector F with respect to the world frame as:
    // F = F_attractor + F_repulsive + F_vortex

    Eigen::Vector3d F = Eigen::Vector3d::Zero();  
    if(has_goal_)
    {
      // pull to goal
      F += goalAttractorForce(goal_pos_w_, pos);
      ROS_WARN_STREAM_THROTTLE(0.5, "This is F:" << F);
    }

    // avoid collision
      F += obstacleRepulsiveForce(obstacles_, pos);
      //ROS_ERROR_STREAM("F is: "<< F);

    // add vortex/curl field
      F += obstacleVortexForce(obstacles_, pos);
    
    return F;
  }

  Eigen::Vector3d Controller::goalAttractorForce(
    const Eigen::Vector3d& goal, const Eigen::Vector3d& pos)
  {
    // Implement the attractor force
    //#>>>>Hint: Eigen>> offers function such as: norm(), normalize(), etc,...
    Eigen::Vector3d e;
    double dist = sqrt(pow(goal.x() - pos.x(), 2) + pow(goal.y() - pos.y(), 2));
    e = goal - pos;
    double z_err = e.z();

    // ROS_INFO_STREAM_THROTTLE(0.5, "This is dist:" << dist);
    // ROS_INFO_STREAM_THROTTLE(0.5, "This is goal:" << goal);
    // ROS_INFO_STREAM_THROTTLE(0.5, "This is pos:" << pos);
    if(dist>1){
      e /= dist;
    } else{
      
    }
    e.z() = z_err;
   // return Eigen::Vector3d::Zero(); //#>>>>TODO: Replace
    // ROS_INFO_STREAM_THROTTLE(0.5, "This is e:" << e);
    // ROS_INFO_STREAM_THROTTLE(0.5, "This is e:" << e);
    ROS_INFO_STREAM_THROTTLE(0.5, "This is e:" << e);
   
    return K_attractor_* e;
  }

  Eigen::Vector3d Controller::obstacleRepulsiveForce(
    std::vector<Obstacle> obstacles, const Eigen::Vector3d& pos)
  {
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    try
    {
      //if(obstacles.size()>0)
      //{ ROS_WARN_STREAM( "This is obstacles1:" << obstacles[0].pos);}
    }
    catch (const std::exception& e) // reference to the base of a polymorphic object
    {
        std::cout << e.what(); // information from length_error printed
    }
   
    // Uncomment this part to implement the repulsive force

    Eigen::Vector3d v;
    double lambda = 20;
    for(size_t i = 0; i < obstacles.size(); ++i)
    { 
      // ROS_WARN_STREAM("TTTTT");
      v << (obstacles[i].pos - pos).head(2),0;// 2d distance of obstacle and rover (note z component is zero!)
      
      double v_norm = v.norm();//norm of v
      double a = obstacles[i].radius + rover_radius_;// radius obstacle + radius rover ?????
      double d = std::max(0.0, v_norm-a);  // the surface distance rover obstacle*/
    
      double f = k_repulsive_*std::exp(-lambda*d);// force magnitude
    
      force -= f*(v/v_norm);//add up the repulsive force vector for this obstacle
      //ROS_WARN_STREAM( "This is f:" << f << " this is v"<<v);
      
    }
    ROS_WARN_STREAM_THROTTLE(0.5, "This is REP_FORCE:" << force);
    return force;
  }

  Eigen::Vector3d Controller::obstacleVortexForce(
    std::vector<Obstacle> obstacles, const Eigen::Vector3d& pos)
  {
    Eigen::Vector3d force = Eigen::Vector3d::Zero();

    //#>>>>OPTIONAL: Uncomment this part to implement the vortex force

    // get the heading vector of the rover
    Eigen::Vector3d n_rover(std::sin(pos[2]), std::cos(pos[2]), 0.0);

    Eigen::Vector3d v;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    for(size_t i = 0; i < obstacles.size(); ++i)
    {
      // get the vector rover to obstacle 
      v << (obstacles[i].pos - pos).head(2), 0;
      Eigen::Vector3d n_obs = v.normalized();

      // the sign of the dot product between obstacle vector and heading
      // tells us if an object is to the right or to the left
      double alpha = n_rover.dot(n_obs);
      if(alpha < 0)
      { 
        // set the upper left 2x2 corner of the rotation matrix R
        // to create - pi/2 = - 90 degree rotation
        R.topLeftCorner(2,2) = Eigen::Rotation2Dd(n_rover.y()).matrix();
      }
      else
      {
        // set the upper left 2x2 corner of the rotation matrix R
        // to create + pi/2 = + 90 degree rotation
        R.topLeftCorner(2,2) = Eigen::Rotation2Dd(n_rover.x()).matrix();
      }

      // compute vortex field
      double v_norm = v.norm();
      double a = obstacles[i].radius + rover_radius_;
      double d = std::max(0.0, d - a);
      double f = k_vortex_*std::exp(-lambda_vortex_*d);
      force -= f * R * v.normalized();
      
      ROS_WARN_STREAM("force =" << force);


     }
    return force;
  }

  void Controller::publishCommand(const Eigen::Vector3d& thrust)
  {
    geometry_msgs::Twist msg;
    msg.linear.x = thrust.x();
    msg.linear.y = thrust.y();
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = thrust.z();
    command_pub_.publish(msg);
  }

  void Controller::createVectorField(std::vector<Eigen::Vector3d>& points)
  {
    // create a 10x10 mesh with evenly spaced points
    size_t n = 100;
    double len = 10.0;
    double dx = len / n;
    double dy = len / n;
    Eigen::Vector3d p;
    points.reserve(n*n);
    for(double x = -5.0; x < 5.0; x+=dx)
    {
      for(double y = -5.0; y < 5.0; y+=dx)
      {
        p << x, y, 0.0;
        points.push_back(p);
      }
    }
  }

  void Controller::visualizerVectorField(
    std::vector<Obstacle> obstacles, 
    const Eigen::Vector3d& goal,
    const std::vector<Eigen::Vector3d>& points)
  {
    geometry_msgs::PoseArray field_msg;
    field_msg.header.frame_id = map_frame_;
    field_msg.header.stamp = ros::Time::now();

    // evaluate the field at the points, obtain the directions
    field_msg.poses.resize(points.size());
    for(size_t i = 0; i < points.size(); ++i)
    {
      const Eigen::Vector3d& p = points[i];
      Eigen::Vector3d v = computeForce(obstacles, goal, p).normalized();
      Eigen::Quaterniond Q = Eigen::Quaterniond::FromTwoVectors(v, p);

      geometry_msgs::Pose& pose = field_msg.poses[i];
      pose.position.x = p.x();
      pose.position.y = p.y();
      pose.position.z = p.z();

      pose.orientation = tf::createQuaternionMsgFromYaw( std::atan2(v[1], v[0] ));
    }
    field_pub_.publish(field_msg);
  }

  //----------------------------------------------------------------------------
  // tfs

  Eigen::Vector3d Controller::roverPos()
  {
    // get the current rover pose in the map frame
    // returns a Eigen::Vector3d containing [x,y,theta]
    
    tf::StampedTransform transform;
    try {
      listener_.lookupTransform(
        "map",/*#>>>>TODO: the target frame name*/
        "rover"/*#>>>>TODO: the source frame name*/, ros::Time(0), transform);

      //#>>>> Hint: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
      //#>>>> Hint: lookupTransform() gives the transformation source with respect to target
    }
    catch (tf::TransformException ex) 
    {
      transform.setIdentity();
    }

    // extract the position 
    Eigen::Vector3d pos;
    pos.x() = transform.getOrigin().x();
    pos.y() = transform.getOrigin().y();

    // extract the theta angle of the rover
    Eigen::Quaterniond Q(
      transform.getRotation().getW(),
      transform.getRotation().getX(),
      transform.getRotation().getY(),
      transform.getRotation().getZ());
    Eigen::Matrix2d M = Q.toRotationMatrix().topLeftCorner(2,2);
    pos.z() = Eigen::Rotation2Dd(M).angle();

    return pos;
  }

  //----------------------------------------------------------------------------
  // callbacks

  void Controller::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    // note: the goal is given in the map frame and should be stored in the 
    // member variable goal_pos_w_ = [pos_x, pos_y, theta] \in R^3

    has_goal_ = true;
    goal_pos_w_.x() = msg->pose.position.x;//#>>>>TODO: get the x position
    goal_pos_w_.y() = msg->pose.position.y;//#>>>>TODO: get the y position
    //Hint: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html

    Eigen::Quaterniond Q_b_w(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z
      // create a Quaternion(w,x,y,z) from the orientation part of the message*/);
    // Hint: https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html

    // Next we use Eigen to convert the Quaternion into a RotationMatrix and 
    // extract the theta angle from its upper 2x2 part
    Eigen::Matrix2d M = Q_b_w.toRotationMatrix().topLeftCorner(2,2);
    goal_pos_w_.z() = Eigen::Rotation2Dd(M).angle();

    // print the new goal
    //#>>>>TODO: Print the goal vector in the console
    // std::cout << "This is the new goal" << goal_pos_w_;
    ROS_INFO_STREAM_THROTTLE(0.5, "**************This is the new goal****************" << goal_pos_w_);
    ROS_WARN("**************This is the new goal****************");
  }

  void Controller::obstacleCallback(const object_msgs::ObjectsConstPtr& msg)
  {
    // note: all obstacle positions (x,y,z) are stored inside the Objects message
    // This callback copies the position into the member variable obstacles_
    // Hint: See the Obstacle struct definition in the header file.

    // Hint: The message obstacle is defined by me inside the package:
    // utilites/object_msgs/msg/Object.msg
    
    obstacles_.resize(msg->objects.size());
    for(size_t i = 0; i < msg->objects.size(); ++i)
    {
      obstacles_[i].pos << msg->objects[i].pose.position.x,msg->objects[i].pose.position.y,msg->objects[i].pose.position.z;//#>>>>TODO: Save the 3d position
      obstacles_[i].radius = msg->objects[i].radius.data;//#>>>>TODO: Save the radius
    }
  }

void Controller::controllerCallback(const nav_msgs::OdometryConstPtr& msg)
{
    ROS_INFO("controllerCallback in");
    //get the postion of robot
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf::getYaw (msg->pose.pose.orientation);
    double d = 0.3;

    //get the postion of reference point

    double Xr = x + d*sin(theta);
    double Yr = y + d*cos(theta);

    //get the distance bwtween reference point and goal
    ROS_WARN("CUT1 ");
    Eigen::Vector2d e;
    //Eigen::VectorXd test;
    
    e[0] = goal_pos_w_.x() - Xr;
    e[1] = goal_pos_w_.y() - Yr;
    //test = e;
    ROS_WARN("CUT2 ");
    //the control velocity 
    Eigen::Matrix<double, 2, 2> K;
    K << 0.5,0,0.5,0;
    ROS_WARN("CUT3 ");
    Eigen::Matrix<double, 2, 2> A;
    A << cos(theta), -d*sin(theta), sin(theta), d*cos(theta); 

    ROS_WARN("CUT4 ");
    //twist command

    geometry_msgs::Twist velocity;
    velocity.linear.x = (A.inverse()*K*e)[0];
    velocity.linear.y = (A.inverse()*K*e)[1];

    velocity.linear.z = 0.0;
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = 0.0;
    ROS_WARN("CUT5 ");
    controller_pub_.publish(velocity);




    

    
    


    

 

    
   

    












    
}


}


