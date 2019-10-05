#include "headers/visualization.h"
#include "headers/robot.h"

Visualization *vis;
Robot *rob;
int enable = _ENABLE_VELODYME;

void velodyneCallback(const  sensor_msgs::PointCloud2::ConstPtr msg){

  //enable = _ENABLE_VELODYME;
  if(!(enable & (1 << _ENABLE_VELODYME)) || enable & (1 << _ARM_CHANGING_POSE))
    return;


  vis->createRectangles(); 
  vis->processImages(msg);
  vis->printRect();
 
  vis->processImages(msg);
  vis->printRect();
  

  if(rob->getRodar())
    rob->rodarFunction(vis->getSidesInfo());
  else if(rob->getProvavelEscada())
    rob->aligneEscada(vis->getSidesInfo());
  else{
    vis->setAvoidingObs(rob->getAvoidingObs());
    rob->processMap(vis->getSidesInfo());
  }


}

void statesCallback(const std_msgs::Int32::ConstPtr & _enable){
  enable = _enable->data;
}

void anglesCallback(const std_msgs::Float32MultiArray::ConstPtr angles){
    rob->setAngles(angles->data.at(1), angles->data.at(2));
}


int main(int argc, char **argv){
  
  std::cout << "Velodyne running" << std::endl;
  
  enable = 1 << _ENABLE_VELODYME;

  vis = new Visualization();
  rob = new Robot();
  
  //vis->createRectangles();

  //inicia o Ros  
  ros::init(argc, argv, "velodyne");

  ros::NodeHandle n;

  //le o publisher do vrep
  ros::Rate loop_rate(1);
  ros::Subscriber subVelodyne = n.subscribe("/sensor/velodyne", 1, velodyneCallback);
  ros::Subscriber subState = n.subscribe("/pra_vale/estados", 1, statesCallback);
  ros::Subscriber Angles = n.subscribe("/pra_vale/imu", 1, anglesCallback);
  

  ros::Publisher speedPub = n.advertise<std_msgs::Float32MultiArray>("/pra_vale/rosi_speed",1);
  ros::Publisher wheelPub = n.advertise<std_msgs::Float32MultiArray>("/pra_vale/rosi_arm_speed",1);
  
  rob->setPublishers(speedPub, wheelPub);
  
  ros::spin();
  

  return 0;
}