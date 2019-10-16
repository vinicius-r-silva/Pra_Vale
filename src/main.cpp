#include "headers/visualization.h"
#include "headers/robot.h"

//objetos
Visualization *vis;
Robot *rob;
std_msgs::Int32 enable;



void velodyneCallback(const  sensor_msgs::PointCloud2::ConstPtr msg){
  //confere se o velodyne pode ser ativado
  if(!(enable.data & (1 << ENABLE_VELODYME)) || enable.data & (1 << ARM_CHANGING_POSE))
    return;


  vis->createRectangles(); 
  vis->processImages(msg);
  vis->printRect();
 
  //confere se o robo deve rodar
  
  vis->setAvoidingObs(rob->getAvoidingObs());
  vis->setNothing(rob->getNothing());
  rob->processMap(vis->getSidesInfo());

}

//Callback do estado em que o robô se encontra
void statesCallback(const std_msgs::Int32Ptr & _enable){
  enable.data = _enable->data;
  rob->setEnable(enable); //atualiza o estado do robo
}

//Callback para pegar os ângulos do IMU
void anglesCallback(const std_msgs::Float32MultiArray::ConstPtr angles){
    rob->setAngles(angles->data.at(1), angles->data.at(2)); //atualiza o angulo do robo
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
  ros::Subscriber subVelodyne = n.subscribe("/sensor/velodyne", 1, velodyneCallback); //velodyne
  ros::Subscriber subState = n.subscribe("/pra_vale/estados", 1, statesCallback); //estados do robo
  ros::Subscriber Angles = n.subscribe("/pra_vale/imu", 1, anglesCallback); //angulos do robo
  

  ros::Publisher speedPub = n.advertise<std_msgs::Float32MultiArray>("/pra_vale/rosi_speed",1); //velocidade do robo
  ros::Publisher wheelPub = n.advertise<std_msgs::Float32MultiArray>("/pra_vale/rosi_arm_speed",1); //velocidade da esteira do robo
  ros::Publisher statePub = n.advertise<std_msgs::Int32>("/pra_vale/def_state",15); //estados do robo
  
  
  rob->setPublishers(speedPub, wheelPub, statePub); //seta os ponteiros
  
  ros::spin(); //mantem o codigo num loop
  

  return 0;
}