#include "headers/robot.h"

using namespace std;

#define _KP_REC 0.6
#define NICE_DIST_TRACK 0.80
#define FRONT_WHEELS 0.15
#define REAR_WHEELS 0.35
#define OKAY 0.044

Robot::Robot(){
    _state = WALKING;
    sentido = _HORARIO;
    _isInStairs = false;
    rodar = false;
    avoidingObs = false;
    inObs = false;
    saveAngle = 10;
}

void Robot::processMap(SidesInfo *sidesInfo){
  if(rodar)
    return;


  int stairsDir = (_state == LADDER_DOWN) ? -1 : 1;
  pra_vale::RosiMovement tractionCommandDir;
  pra_vale::RosiMovement tractionCommandEsq;
  float erro;

  if(_state == IN_LADDER){
    static int i = 0;
    i++;
    if(i > 15)
      _state = LADDER_DOWN;
  }

  if(_isInStairs && !(_state == LADDER_UP || _state == LADDER_DOWN)){
    _state = LADDER_UP;  
  }

  //Implementacao da maquina de estado

  // switch(_state){
  //   case WALKING:
  //     cout << "WALKING\t";
  //     erro = 0.0;

  //   break;

  //   case LADDER_UP:
  //     climbStairs();
  //     erro = zAngle * _KP_OBSTACLE;
  //     cout << "LADDER_UP\t yAngle: " << yAngle;

  //   break;

  //   case IN_LADDER:
  //     erro = zAngle - *_KP_OBSTACLE;
  //     cout << "IN_LADDER\t";

  //   break;

  //   case LADDER_DOWN:
  //'     erro = zAngle *_KP_OBSTACLE;
  //     cout << "LADDER_DOWN\t";

  //   break;

  //   case FRONT_OBS:
  //     erro = (sentido == HORARIO) ? 1/(sidesInfo[_FRONT].medY) : -1/(sidesInfo[_FRONT].medY);
  //     avoidingObs = true;

  // }

  //sobe a escada 
  if(_state == LADDER_UP){

    climbStairs();

    erro = zAngle *_KP_OBSTACLE;

    cout << "E: SubirEscada\t yAngle: " << yAngle;

  }else if(_state == IN_LADDER || _state == LADDER_DOWN){
    
    erro = zAngle *_KP_OBSTACLE;

    cout << "E: NaEscada\t ZAngle: " << zAngle << "Erro:" << erro;

  //desvia do obstaculo na frente    
  }else if(sidesInfo[_FRONT].medY < _MIN_DIST_FRONT){

    if(sentido ==_HORARIO)
      erro = 1/(sidesInfo[_FRONT].medY);
    else
      erro = -1/(sidesInfo[_FRONT].medY);

    avoidingObs = true;
    
    if(abs(zAngle) < 0.1)
      inObs = true;

    cout << "E: DesviaFr";
    cout << " | AF: " << sidesInfo[_FRONT].area;
    cout << " | DFY: " << sidesInfo[_FRONT].medY;

  //esta entre dois obstaculos
  /*}else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC && sidesInfo[_LEFT].area > _MIN_AREA_REC){
    if(sidesInfo[_RIGHT].medX < sidesInfo[_LEFT].medY){
      erro = sidesInfo[_RIGHT].medX - sidesInfo[_LEFT].medX;
    }
  */


  //Recupera o trajeto da direita
  }else if(inObs && sidesInfo[_RIGHT].medY < -0.35 && sentido == _HORARIO){
    
    erro = -1/(sidesInfo[_RIGHT].medX);

    cout << "E: RercuDir";
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;

    avoidingObs = false;
    inObs = false;

  }else if(!inObs && abs(sidesInfo[_RIGHT].medX - NICE_DIST_TRACK) > 0.15){

    erro = (NICE_DIST_TRACK - sidesInfo[_RIGHT].medX) * _KP_REC;

    cout << "E: AproxDir | DDX: " << sidesInfo[_RIGHT].medX;

  //segue a parede da direita
  }else if(!inObs && sidesInfo[_RIGHT].area > _MIN_AREA){

    erro = zAngle * _KP_OBSTACLE;

    avoidingObs = false;

    cout << "E: SegueDir";
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;


  //Recupera o trajeto da esquerda
  }else if(sidesInfo[_LEFT].medY < -0.35 && sentido == _ANTI_HORARIO){
    
    erro = 1/(sidesInfo[_LEFT].medX);

    avoidingObs = false;
    inObs = false;

    cout << "E: RercuEsq";
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;


  //segue a parede da esquerda
  }else if(sidesInfo[_LEFT].area > _MIN_AREA){

    erro = zAngle *_KP_OBSTACLE;

    cout << "E: SegueEsq";
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;


  //segue reto caso nao tenha nada
  }else{
    
    erro = 0.0;

    cout << "E: NormalEt";
  
  }


  if(_isInStairs && !(_state == IN_LADDER)){
    tractionCommandDir.joint_var = (float) stairsDir * (_MAX_SPEED + erro);
    tractionCommandEsq.joint_var = (float) stairsDir * (_MAX_SPEED - erro); 
  }else{
    tractionCommandDir.joint_var = (float) _V0 + _KP*erro;
    tractionCommandEsq.joint_var = (float) _V0 - _KP*erro;
  }

  if(tractionCommandDir.joint_var > _MAX_SPEED)
    tractionCommandDir.joint_var = _MAX_SPEED;
  else if(tractionCommandDir.joint_var < -_MAX_SPEED)
    tractionCommandDir.joint_var = -_MAX_SPEED;


  if(tractionCommandEsq.joint_var > _MAX_SPEED)
    tractionCommandEsq.joint_var = _MAX_SPEED;
  else if(tractionCommandEsq.joint_var < -_MAX_SPEED)
    tractionCommandEsq.joint_var = -_MAX_SPEED;


  cout << " | zAngle: " << zAngle << " | VEsq: " << tractionCommandEsq.joint_var << " | VDir: " << tractionCommandDir.joint_var << endl;


  //altera o vetor das velocidades das 'joints'

  //Direita:
  tractionCommandDir.nodeID = 1;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  tractionCommandDir.nodeID = 2;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  
  //Esquerda:
  tractionCommandEsq.nodeID = 3;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);
  tractionCommandEsq.nodeID = 4;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);

  speedPub.publish(tractionCommandList);

  //limpa o vetor
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();

}

void Robot::climbStairs(){
  pra_vale::RosiMovement wheelCommand;
  float wheelFrontSpeed;
  float wheelRearSpeed;
  bool _climbing;

  if(abs(yAngle) < OKAY){

    cout << "IT'S OKAY\n";
    wheelRearSpeed = 0.0f;
    wheelFrontSpeed = 0.0f;
  
    if(_climbing){
      static int i = 0;
      i++;
      if(i > 15){
        _state = IN_LADDER;
      }
    } 

  }else if(abs(yAngle) > REAR_WHEELS){

    cout << "REAR WHEELS IS ON\n";
    wheelRearSpeed = -1.0f *_MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else if(abs(yAngle) < FRONT_WHEELS){
    
    cout << "FRONT WHEELS IS ON\n";
    wheelRearSpeed = _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else{

    cout << "ESTABILIZING\n";
    wheelRearSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = 0.0f;

  }

  for(int i = 0; i < 4; i++){
    wheelCommand.joint_var = (i == 0 || i == 2)? wheelFrontSpeed : wheelRearSpeed;
    wheelCommand.nodeID = i + 1;
    wheelsCommandList.movement_array.push_back(wheelCommand);    
  }

  wheelPub.publish(wheelsCommandList);

  wheelsCommandList.movement_array.pop_back();
  wheelsCommandList.movement_array.pop_back();
  wheelsCommandList.movement_array.pop_back();
  wheelsCommandList.movement_array.pop_back();      
}

void Robot::rodarFunction(SidesInfo* sidesInfo){
  float dif;

  pra_vale::RosiMovement tractionCommandDir;
  pra_vale::RosiMovement tractionCommandEsq;

  

  if(zAngle < 0)
    zAngle += M_PI*2;
    
  if(saveAngle == 10)
    saveAngle = zAngle;

  if(sidesInfo[_FRONT].medY < _MIN_SAFE_DIST_SPIN){
    tractionCommandDir.joint_var = -3;
    tractionCommandEsq.joint_var = -3;
  
  }else if(sentido == _HORARIO){
    

    tractionCommandDir.joint_var = -3;
    tractionCommandEsq.joint_var = 0;

    if(saveAngle >= 0 && saveAngle < M_PI && zAngle > M_PI)
        dif = saveAngle + M_PI*2 - zAngle;

    else
      dif = saveAngle - zAngle;
  
  }else{
    tractionCommandDir.joint_var = 0;
    tractionCommandEsq.joint_var = -3;

    if(zAngle >= 0 && zAngle < M_PI && saveAngle > M_PI)
        dif =  M_PI*2 - saveAngle + zAngle;
      
    else
      dif = saveAngle - zAngle;
  }

  if(dif < 0)
    dif *=-1;

  if(dif > 3){
    rodar = false; //para de rodar
    sentido = !sentido; //troca o sentido
    saveAngle = 10; //da um reset no angulo
  } 

  cout <<"MedY: " << sidesInfo[_FRONT].medY << " | girando" << " | zAngle: " << zAngle << " | saveAngle: " << saveAngle  << " | dif: " << dif <<" | velE: " << tractionCommandEsq.joint_var << " | velD: " << tractionCommandDir.joint_var << endl;

  //Direita:
  tractionCommandDir.nodeID = 1;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  tractionCommandDir.nodeID = 2;
  tractionCommandList.movement_array.push_back(tractionCommandDir);
  
  //Esquerda:
  tractionCommandEsq.nodeID = 3;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);
  tractionCommandEsq.nodeID = 4;
  tractionCommandList.movement_array.push_back(tractionCommandEsq);


  speedPub.publish(tractionCommandList);
  
  //limpa o vetor
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();
  tractionCommandList.movement_array.pop_back();  

}

void Robot::setAngles(double y, double z){
    yAngle = y;
    zAngle = z;
}

bool Robot::getRodar(){
    return rodar;
}

void Robot::setPublishers(ros::Publisher speedPub, ros::Publisher wheelPub){
    this->speedPub = speedPub;
    this->wheelPub = wheelPub;
}

bool Robot::getAvoidingObs(){
  return avoidingObs;
}