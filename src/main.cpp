#include "headers/visualization.h"
#include "headers/robot.h"

Visualization *vis;
Robot *rob;
int enable;

// void ImuCallback(const  sensor_msgs::Imu::ConstPtr msg){
  
//   float qx = msg->orientation.x;
//   float qy = msg->orientation.y;
//   float qz = msg->orientation.z;
//   float qw = msg->orientation.w;

//   double t2 = 2.0f * (qw * qy - qx * qx);
//   t2 = (t2 > 1.0f)? 1.0f : t2;
//   t2 = (t2 < -1.0f)? -1.0f : t2;
//   double yAngle = asin(t2);

//   double t3 = +2.0 * (qw * qz + qx * qy);
//   double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);    
//   double zAngle = (double) (atan2(t4, t3) + M_PI/2);  //z angulo de euler

//   rob->setAngles(yAngle, zAngle);


// }

void velodyneCallback(const  sensor_msgs::PointCloud2::ConstPtr msg){
  if(!(enable & (1 << _VELODYNE_ENABLED)) || enable & (1 << _ARM_CHANGING_POSE))
    return;
   
  vis->processImages(msg);
  vis->printRect();
  vis->setAvoidingObs(rob->getAvoidingObs());
  rob->processMap(vis->getSidesInfo());


  if(rob->getRodar()){
    rob->rodarFunction(vis->getSidesInfo());
    return;
  } 

}

void statesCallback(const std_msgs::Int32::ConstPtr & _enable){
  enable = _enable->data;
}

void anglesCallback(const std_msgs::Float32MultiArray::ConstPtr angles){
    rob->setAngles(angles->data.at(1), angles->data.at(2));
}


int main(int argc, char **argv){
  
  enable = 1 << _VELODYNE_ENABLED;

  vis = new Visualization();
  rob = new Robot();
  
  vis->createRectangles();

  //inicia o Ros  
  ros::init(argc, argv, "velodyne");

  ros::NodeHandle n;

  //le o publisher do vrep
  ros::Rate loop_rate(1);
  ros::Subscriber subVelodyne = n.subscribe("/sensor/velodyne", 1, velodyneCallback);
  //ros::Subscriber subImu = n.subscribe("/sensor/imu", 1, ImuCallback);
  ros::Subscriber subState = n.subscribe("/pra_vale/estados", 1, statesCallback);
  ros::Subscriber Angles = n.subscribe("/pra_vale/imu", 1, anglesCallback);
  

  ros::Publisher speedPub = n.advertise<pra_vale::RosiMovementArray>("/rosi/command_traction_speed",1);
  ros::Publisher wheelPub = n.advertise<pra_vale::RosiMovementArray>("/rosi/command_arms_speed",1);
  
  rob->setPublishers(speedPub, wheelPub);
  
  ros::spin();
  

  return 0;
}