#include "headers/robot.h"

using namespace std;

#define _KP_REC 0.7
#define NICE_DIST_TRACK 0.80
#define FRONT_WHEELS 0.2
#define REAR_WHEELS 0.3
#define OKAY 0.06

Robot::Robot(){
    _state = WALKING;
    sentido = _ANTI_HORARIO;
    _isInStairs = false;
    _provavelEscada = false;
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
  float traction = 0;
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float erro;

  if(_state == IN_LADDER){
    static int i = 0;
    i++;
    if(i > 15)
      _state = LADDER_DOWN;
  }

  if(!_isInStairs && _HORARIO && sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT && !(sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT)){

    straitPath = true;
    distToTrack = NICE_DIST_TRACK - 0.3;
    cout << "StraitPath\t";

  
  }

  // if(straitPath && _HORARIO && sidesInfo[_LEFT].medX > 0.5){
  //   straitPath = false;
  //   distToTrack = NICE_DIST_TRACK;
  // }

  cout << " | isInStairs: " << _isInStairs;
  if(_isInStairs && !(_state == IN_LADDER || _state == LADDER_DOWN)){
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
    traction = (float) (stairsDir * (_MAX_SPEED + erro));
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 
    
    cout << " | VelD: " << traction;

    msg.data.push_back(traction);
    msg.data.push_back(traction);

    traction = (float) (stairsDir * (_MAX_SPEED - erro));
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 
    
    msg.data.push_back(traction);
    msg.data.push_back(traction);

    cout << " | VelE: " << traction;

  }else{
    traction = (float) (_V0 + _KP*erro);
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 

    msg.data.push_back(traction);
    msg.data.push_back(traction);

    cout << " | VelD: " << traction;

    traction = (float) (_V0 - _KP*erro);
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 

    cout << " | VelE: " << traction;
    
    msg.data.push_back(traction);
    msg.data.push_back(traction);
  }

  cout << " | zAngle: " << zAngle << endl;
  
  // msg.data.clear();

  // msg.data.push_back(0);
  // msg.data.push_back(0);
  // msg.data.push_back(0);
  // msg.data.push_back(0);

  
  speedPub.publish(msg);
}


void Robot::aligneEscada(SidesInfo *sidesInfo){


  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float tractionDir;
  float tractionEsq;

  
  static bool frontToTrack = false;
  static bool closeToTrack = false;


  if(sentido == _ANTI_HORARIO){
    cout << "ANTI_HORARIO";

    if(!frontToTrack && !closeToTrack){ //se nao estiver de frente para escada, endireita
      _provavelEscada = true; //nao depende mais da identificacao da escada

      cout << " | Endireitando" << " | zAngle: " << zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_LEFT].medX;
      

      if(sidesInfo[_FRONT].medY < 1.1){ //esta de frente e perto da esteira
        frontToTrack = true;
        return;
      
      }else if(sidesInfo[_LEFT].medX < _MAX_DIST_SIDE_ESCADA){ //esta de lado para a esteira
        closeToTrack = true;
        return;
      
      
      }else if((zAngle > M_PI*3/2 - 0.2 || zAngle < -M_PI/2 + 0.2) || sidesInfo[_LEFT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }else if(zAngle > -M_PI/2 && zAngle < M_PI/2){ //alinha antes de avancar para a esteira
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }else{ //alinha antes de avancar para a esteira
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }

    }else if(frontToTrack && !closeToTrack){
      _provavelEscada = false; //depende do reconhecimento da escada
      cout << "seguindo a esteira" << " | DX: " << sidesInfo[_LEFT].medX;

      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && zAngle < -0.2){ //esta de frente mas precisa alinhar
        tractionDir = 0;
        tractionEsq = _V0*1.5;
      
      }else if(sidesInfo[_LEFT].area > _MIN_AREA_REC){ //ja esta perto da esteira
        
        if(_MIN_DIST_SIDE_ESCADA > sidesInfo[_LEFT].medX || sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA){ //chega ele mais perto
          tractionDir = _V0 + _KP*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 - _KP*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);

        
        }else{ //ja esta perto da esteira
          tractionEsq = _V0;
          tractionDir = _V0;
          closeToTrack = true;
        }
        
      }else{ //perdeu a esteira
        tractionEsq = 0;
        tractionDir = 0;
        frontToTrack = false;

      }
    
    }else if(closeToTrack){ //esta perto da esteira
      _provavelEscada = true; //nao depende mais da identificacao da escada
      cout <<" | Perto da esteira" << " | DX: " << sidesInfo[_LEFT].medX;

      if(sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){ //esta de frente com a escada
        cout << " | Alinhando com escada"; 
        if(zAngle > -_MAX_ERRO_ESCADA && zAngle < _MAX_ERRO_ESCADA){ //se estiver alinhado comeca a subir
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false; 
        }

        //corrige a inclinacao do robo
        tractionDir = +_KP*zAngle*2.5;
        tractionEsq = -_KP*zAngle*2.5;
        cout << " | MedY: " << sidesInfo[_FRONT].medY;
      

      }else{
        if(sidesInfo[_LEFT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){ //se nao esiver identidicando mais nada
          cout << "PERDEU TUDO" << endl;
          closeToTrack = false;
          frontToTrack = false;
          return;

        }else if(_MIN_DIST_SIDE_ESCADA - 0.1 > sidesInfo[_LEFT].medX || sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA + 0.1 && sidesInfo[_FRONT].area < _MIN_AREA_REC){
          //se estiver muito distante
          tractionDir = _V0 + _KP*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 - _KP*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          cout << " | se endireitando";

        }else{
          //esta perto mas precisa alinhar
          cout << " | Corrigindo zAngle" << " | zAngle: " << zAngle;
          cout << " | MedY: " << sidesInfo[_FRONT].medY;
          tractionDir = (_V0 + _KP*zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP*zAngle*1.5)/2;
        }
      }
    }

  }else{ //sentido horario
    cout << "HORARIO";

    if(!frontToTrack && !closeToTrack){  //se nao estiver de frente para escada, endireita
      _provavelEscada = true; //nao depende mais da identificacao da escada

      cout << " | Endireitando" << " | zAngle: " << zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_RIGHT].medX;
      
      if(sidesInfo[_FRONT].medY < 1.1){ //esta de frente e perto da esteira
        frontToTrack = true;
        return;

      }else if(sidesInfo[_RIGHT].medX < _MAX_DIST_SIDE_ESCADA){ //esta de lado para a esteira
        closeToTrack = true;
        return;

      }else if((zAngle > M_PI/2 - 0.2 && zAngle < M_PI/2 + 0.2) || sidesInfo[_RIGHT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }else if(zAngle <= M_PI/2){ //alinha antes de avancar para a esteira
          tractionDir = -_V0;
          tractionEsq = _V0;
          
      }else{ //alinha antes de avancar para a esteira
          tractionDir = -_V0;
          tractionEsq = _V0;
  
          
      }

    }else if(frontToTrack && !closeToTrack){
      _provavelEscada = false; //depende do reconhecimento da escada
      cout << " | seguindo a esteira" << " | DX: " << sidesInfo[_RIGHT].medX;

      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && zAngle > 0.2){ //esta de frente mas precisa alinhar
        tractionDir = _V0*1.5;
        tractionEsq = 0;
      
      }else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC){ //ja esta perto da esteira
        
        if(_MIN_DIST_SIDE_ESCADA > sidesInfo[_RIGHT].medX || sidesInfo[_RIGHT].medX > _MAX_DIST_SIDE_ESCADA){ //chega ele mais perto
          tractionDir = _V0 - _KP*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 + _KP*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);

        
        }else{ //ja esta perto da esteira
          tractionEsq = _V0;
          tractionDir = _V0;
          closeToTrack = true;
        }
        
      }else{ //perdeu a esteira
        tractionEsq = 0;
        tractionDir = 0;
        frontToTrack = false;

      }
    
    }else if(closeToTrack){ //esta perto da esteira
      _provavelEscada = true; //nao depende mais da identificacao da escada
      cout <<" | Perto da esteira" << " | DX: " << sidesInfo[_RIGHT].medX;

      if(sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){ //esta de frente com a escada
        cout << " | Alinhando com escada"; 
        if(zAngle > -_MAX_ERRO_ESCADA && zAngle < _MAX_ERRO_ESCADA){  //se estiver alinhado comeca a subir
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false;
          _enable.data &= ~(1<< FOUND_STAIR);
          statePub.publish(_enable);
        }

        //corrige a inclinacao do robo
        tractionDir = +_KP*zAngle*2.5;
        tractionEsq = -_KP*zAngle*2.5;
        cout << " | MedY: " << sidesInfo[_FRONT].medY;
      

      }else{
        if(sidesInfo[_RIGHT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){ //se nao esiver identidicando mais nada
          cout << "PERDEU TUDO" << endl;
          closeToTrack = false;
          frontToTrack = false;
          return;

        }else if(_MIN_DIST_SIDE_ESCADA-0.1 > sidesInfo[_RIGHT].medX || sidesInfo[_RIGHT].medX > _MAX_DIST_SIDE_ESCADA + 0.1 && sidesInfo[_FRONT].area < _MIN_AREA_REC){
          //se estiver muito distante
          tractionDir = _V0 - _KP*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 + _KP*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          cout << " | se endireitando";

        }else{
          //esta perto mas precisa alinhar
          cout << " | Corrigindo zAngle" << " | zAngle: " << zAngle;
          cout << " | MedY: " << sidesInfo[_FRONT].medY;
          tractionDir = (_V0 + _KP*zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP*zAngle*1.5)/2;
        }
      }
    }
  }  


  cout <<" | velE: " << tractionEsq << " | velD: " << tractionDir << endl;

  //altera o vetor das velocidades das 'joints'


  msg.data.push_back(tractionDir);
  msg.data.push_back(tractionDir);

  msg.data.push_back(tractionEsq);
  msg.data.push_back(tractionEsq);

  speedPub.publish(msg);
}


void Robot::climbStairs(){
  //pra_vale::RosiMovement wheel;4
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float wheelFrontSpeed;
  float wheelRearSpeed;
  bool _climbing;

  if(abs(yAngle) < OKAY){

    cout << " IT'S OKAY\n";
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

    cout << " REAR WHEELS IS ON\n";
    wheelRearSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else if(abs(yAngle) < FRONT_WHEELS){
    
    cout << " FRONT WHEELS IS ON\n";
    wheelRearSpeed = _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else{

    cout << " ESTABILIZING\n";
    wheelRearSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -1.0f * _MAX_WHEEL_R_SPEED;

  }

  for(int i = 0; i < 4; i++){
    // wheel.joint_var = (i == 0 || i == 2)? wheelFrontSpeed : wheelRearSpeed;
    // wheel. = i + 1;
    // wheelsList.movement_array.push_back(wheel); 
    msg.data.push_back((i == 0 || i == 2) ? wheelFrontSpeed : wheelRearSpeed);
  }

  wheelPub.publish(msg);

}

void Robot::rodarFunction(SidesInfo* sidesInfo){
  float dif;

  //pra_vale::RosiMovement tractionDir;
  //pra_vale::RosiMovement tractionEsq;

  std_msgs::Float32MultiArray msg;
  msg.data.clear();

  if(zAngle < 0)
    zAngle += M_PI*2;
    
  if(saveAngle == 10)
    saveAngle = zAngle;

  if(sidesInfo[_FRONT].medY < _MIN_SAFE_DIST_SPIN){
    // tractionDir.joint_var = -3;
    // tractionEsq.joint_var = -3;
    msg.data.push_back(-3);
    msg.data.push_back(-3);
    msg.data.push_back(-3);
    msg.data.push_back(-3);
  
  }else if(sentido == _HORARIO){
        // tractionDir.joint_var = -3;
    // tractionEsq.joint_var = 0;
    msg.data.push_back(-3);
    msg.data.push_back(-3);
    msg.data.push_back(0);
    msg.data.push_back(0);

    if(saveAngle >= 0 && saveAngle < M_PI && zAngle > M_PI)
        dif = saveAngle + M_PI*2 - zAngle;

    else
      dif = saveAngle - zAngle;
  
  }else{
    // tractionDir.joint_var = 0;
    // tractionEsq.joint_var = -3;
    msg.data.push_back(0);
    msg.data.push_back(0);
    msg.data.push_back(-3);
    msg.data.push_back(-3);

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

  //cout <<"MedY: " << sidesInfo[_FRONT].medY << " | girando" << " | zAngle: " << zAngle << " | saveAngle: " << saveAngle  << " | dif: " << dif <<" | velE: " << tractionEsq.joint_var << " | velD: " << tractionDir.joint_var << endl;

  // //Direita:
  // tractionDir. = 1;
  // tractionList.movement_array.push_back(tractionDir);
  // tractionDir. = 2;
  // tractionList.movement_array.push_back(tractionDir);
  
  // //Esquerda:
  // tractionEsq. = 3;
  // tractionList.movement_array.push_back(tractionEsq);
  // tractionEsq. = 4;
  // tractionList.movement_array.push_back(tractionEsq);


  speedPub.publish(msg);
  
  //limpa o vetor
  // tractionList.movement_array.pop_back();
  // tractionList.movement_array.pop_back();
  // tractionList.movement_array.pop_back();
  // tractionList.movement_array.pop_back();  

}

void Robot::setAngles(double y, double z){
    yAngle = y;
    zAngle = z;
}

bool Robot::getRodar(){
    return rodar;
}

void Robot::setPublishers(std_msgs::Int32 enable, ros::Publisher speedPub, ros::Publisher wheelPub, ros::Publisher statePub){
    _enable.data = enable.data;
    this->speedPub = speedPub;
    this->wheelPub = wheelPub;
    this->statePub = statePub;
}

bool Robot::getAvoidingObs(){
  return avoidingObs;
}

bool Robot::getProvavelEscada(){
  return _provavelEscada;
}

bool Robot::getIsInStairs(){
  return _isInStairs;
}

bool Robot::getSentido(){
  return sentido;
}

bool Robot::getStraitPath(){
  return straitPath;
}

void Robot::setStatePub(std_msgs::Int32 _enable){
  statePub.publish(_enable);
}