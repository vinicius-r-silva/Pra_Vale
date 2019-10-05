#include "headers/robot.h"

using namespace std;

#define _KP_REC 0.7
#define NICE_DIST_TRACK 0.80
#define FRONT_WHEELS 0.15
#define REAR_WHEELS 0.35
#define OKAY 0.044

Robot::Robot(){
    _state = WALKING;
    sentido = _HORARIO;
    _isInStairs = true;
    rodar = false;
    avoidingObs = false;
    inObs = false;
    saveAngle = 10;
    straitPath = false;
    distToTrack = NICE_DIST_TRACK;
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

  if(_HORARIO && sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT && !(sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT)){

    straitPath = true;
    distToTrack = NICE_DIST_TRACK - 0.3;
    cout << "StraitPath\t";
  
  }

  // if(straitPath && _HORARIO && sidesInfo[_LEFT].medX > 0.5){
  //   straitPath = false;
  //   distToTrack = NICE_DIST_TRACK;
  // }

  if(_isInStairs && !(_state == LADDER_UP || _state == LADDER_DOWN)){
    _state = LADDER_UP;  
  }

  if(!avoidingObs){
    saveAngleZ = zAngle;
  }

  //Implementacao da maquina de estado

  //sobe a escada 
  if(_state == LADDER_UP){

    climbStairs();

    erro = zAngle *_KP_OBSTACLE;

    cout << "E: SubirEscada\t yAngle: " << yAngle;

  }else if(_state == IN_LADDER || _state == LADDER_DOWN){
    
    erro = zAngle *_KP_OBSTACLE;

    cout << "E: NaEscada\t ZAngle: " << zAngle << "Erro:" << erro;

  //desvia do obstaculo na frente    
  }else if(!straitPath && sidesInfo[_FRONT].medY < _MIN_DIST_FRONT){

    if(sentido ==_HORARIO)
      erro = 1/(sidesInfo[_FRONT].medY);
    else
      erro = -1/(sidesInfo[_FRONT].medY);

    avoidingObs = true;
    
    if(abs(saveAngleZ) < 0.1)
      inObs = true;

    cout << "E: DesviaFr | Erro: " << erro;
    cout << " | AF: " << sidesInfo[_FRONT].area;
    cout << " | DFY: " << sidesInfo[_FRONT].medY;

  //Recupera o trajeto da direita
  }else if(sidesInfo[_RIGHT].medY < -0.20 && sentido == _HORARIO){
    
    erro = -1/(sidesInfo[_RIGHT].medX);

    cout << "E: RecuDir | Erro: " << erro;
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;

    avoidingObs = false;
    inObs = false;

  }else if(abs(sidesInfo[_RIGHT].medX - distToTrack) > 0.15){

    erro = (sidesInfo[_RIGHT].medX != 10) ? (distToTrack - sidesInfo[_RIGHT].medX) * _KP_REC : -0.1;

    cout << "E: AproxDir | erro: " << erro << " | DDX: " << sidesInfo[_RIGHT].medX;

  //segue a parede da direita
  }else if(sidesInfo[_RIGHT].area > _MIN_AREA){

    erro = zAngle * _KP_OBSTACLE;

    if(straitPath && _HORARIO && sidesInfo[_LEFT].medX > 0.5){
    straitPath = false;
    distToTrack = NICE_DIST_TRACK;
    }

    cout << "E: SegueDir | Erro: " << erro;
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;


  //Recupera o trajeto da esquerda
  }else if(sidesInfo[_LEFT].medY < -0.20 && sentido == _ANTI_HORARIO){
    
    erro = 1/(sidesInfo[_LEFT].medX);

    avoidingObs = false;
    inObs = false;

    cout << "E: RecuEsq | Erro: " << erro;
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;

  }else if(abs(sidesInfo[_LEFT].medX - distToTrack) > 0.15){

    erro = (sidesInfo[_LEFT].medX != 10) ? (distToTrack - sidesInfo[_LEFT].medX) * _KP_REC : -0.1;

    cout << "E: AproxEsq | Erro: " << erro << " | DDX:" << sidesInfo[_LEFT].medX;

  //segue a parede da esquerda
  }else if(sidesInfo[_LEFT].area > _MIN_AREA){

    erro = zAngle *_KP_OBSTACLE;

    if(straitPath && _HORARIO && sidesInfo[_LEFT].medX > 0.5){
    straitPath = false;
    distToTrack = NICE_DIST_TRACK;
    }

    cout << "E: SegueEsq | Erro: " << erro;
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