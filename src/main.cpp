#include "headers/visualization.h"
#include "headers/robot.h"

Visualization *vis;
Robot *rob;
std_msgs::Int32 enable;


//Callback do Velodyne
void velodyneCallback(const  sensor_msgs::PointCloud2::ConstPtr msg){
  if(!(enable.data & (1 << ENABLE_VELODYME)) || enable.data & (1 << ARM_CHANGING_POSE))
    return;


  vis->createRectangles(); 
  vis->processImages(msg);
  vis->printRect();
 
  
  if(rob->getRodar()){
    rob->rodarFunction(vis->getSidesInfo());

  }else if((enable.data & (1 << FOUND_STAIR) || rob->getProvavelEscada()) && !rob->getIsInStairs()){
    std::cout << (int) rob->getProvavelEscada() << (int) (enable.data & (1 << FOUND_STAIR)) << std::endl;
  
    rob->aligneEscada(vis->getSidesInfo());

  }else{
    vis->setAvoidingObs(rob->getAvoidingObs());
    rob->processMap(vis->getSidesInfo());
  }

  //mudando os estados
  if(rob->getSentido() == _HORARIO){
    enable.data |= (1 << ROBOT_CLOCKWISE);
    enable.data &= ~(1 << ROBOT_ANTICLOCKWISE);
  }else{
    enable.data |= (1 << ROBOT_ANTICLOCKWISE);
    enable.data &= ~(1 << ROBOT_CLOCKWISE);
  }
  
  if(rob->getStraitPath())
    enable.data |= (1 << STRAIT_PATH);
  else
    enable.data &= ~(1 << STRAIT_PATH);

  
  rob->setStatePub(enable);
  
}

//Callback do estado em que o robô se encontra
void statesCallback(const std_msgs::Int32Ptr & _enable){
  enable.data = _enable->data;
}

//Callback para pegar os ângulos do IMU
void anglesCallback(const std_msgs::Float32MultiArray::ConstPtr angles){
    rob->setAngles(angles->data.at(1), angles->data.at(2));
}


int main(int argc, char **argv){
  
  std::cout << "Velodyne running" << std::endl;
  
  enable.data = 1 << ENABLE_VELODYME;

  vis = new Visualization();
  rob = new Robot();
  

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
  ros::Publisher statePub = n.advertise<std_msgs::Int32>("/pra_vale/set_state",1);
  
  
  rob->setPublishers(enable,speedPub, wheelPub, statePub);
  
  ros::spin();
  

  return 0;
}