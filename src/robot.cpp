#include "headers/robot.h"

using namespace std;


Robot::Robot(){
    _begin = true;
    _state = WALKING;
    _sentido = _HORARIO;
    _isInStairs = false;
    _provavelEscada = false;
    _rodar = false;
    _avoidingObs = false;
    _nothing = true;
    _straitPath = false;
    _climbing = false;
    _distToTrack = NICE_DIST_TRACK;
}


void Robot::processMap(SidesInfo *sidesInfo){

  bool needSpeed = false;
  float stairsDir = (_state == LADDER_DOWN) ? -0.7 : 1;
  float tractionDir = 0;
  float tractionEsq = 0;
  float erro;


  //define a precisao dos dados na hora de exibilos
  cout.precision(4);
  

  //corrige o angulo inicial do robo
  if(_begin){
    if(_zAngle < 0)
      _zAngle += M_PI*2;
    
    
    if(sidesInfo[_FRONT].area > _MIN_AREA_REC/2)
      _begin = false;

    else if(_zAngle > M_PI && _zAngle < 2*M_PI-0.1){
      erro = -5*_V0/_KP;
      cout << "Alinhando posiIni";
    }
    else if(_zAngle < M_PI && _zAngle > 0.1){
      erro = 5*_V0/_KP;
      cout << "Alinhando posiIni";
    }
    
    else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC && sidesInfo[_LEFT].area < _MIN_AREA_REC){
      cout << "Setando para horario";
      _begin = false;
      _sentido = _HORARIO;
    
    }else if(sidesInfo[_RIGHT].area < _MIN_AREA_REC && sidesInfo[_LEFT].area > _MIN_AREA_REC){
      cout << "Setando para anti-horario";      
      _begin = false;
      _sentido = _ANTI_HORARIO;
    }else{
      tractionDir = _V0;
      tractionEsq = _V0;
    }
    
    setSpeed(tractionDir, tractionDir, tractionEsq, tractionEsq);


    cout << " | zAngle: " << _zAngle << endl;
    

    return;
  }



  if(sidesInfo[_RIGHT].distance > 2 && sidesInfo[_LEFT].distance > 2 && sidesInfo[_FRONT].area < _MIN_AREA_REC){
    cout << "Longe | ";
    _nothing = true;
  }else
    _nothing = false;
  


  //atualiza o estado do robo
  if(_sentido == _ANTI_HORARIO){
    _enable.data = ROBOT_ANTICLOCKWISE;
    statePub.publish(_enable);

    _enable.data = -ROBOT_CLOCKWISE;
    statePub.publish(_enable);

  }else{
    _enable.data = ROBOT_CLOCKWISE;
    statePub.publish(_enable);

    _enable.data = -ROBOT_ANTICLOCKWISE;
    statePub.publish(_enable);    
  }

  //roda o robo depois de descer a escada
  if(_rodar){
    _avoidingObs = false;
    rodarFunction(sidesInfo);
    return;
  }

  //achou a escada
  if((_states.data & (1 << FOUND_STAIR) || _provavelEscada) && !_isInStairs){
    _avoidingObs = false;
    aligneEscada(sidesInfo);
    return;
  }


  //chegou no final da esteira
  
  if(_state == IN_LADDER && _states.data & (1 << END_STAIR)){
    _state = LADDER_DOWN;
    _enable.data = -END_STAIR;
    statePub.publish(_enable);
  }
  
  //caminho estreito
  if(sidesInfo[_FRONT_MIDLE].area < 20 && (!_isInStairs && ((_sentido == _HORARIO && sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT && !(sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT))
    || (_sentido == _ANTI_HORARIO && !(sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT) && sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT)))){

    _straitPath = true;
    _enable.data = STRAIT_PATH;
    statePub.publish(_enable);
    _distToTrack = NICE_DIST_TRACK - 0.25;
    cout << "StraitPath ";

  } 

  //confere se chegou na escada
  if(_isInStairs && !(_state == IN_LADDER || _state == LADDER_DOWN)){
    _state = LADDER_UP;  
  }

  //Implementacao da maquina de estado

  //sobe a escada 
  if(_state == LADDER_UP){

    needSpeed = climbStairs();

    _enable.data = CLIMB_STAIR;
    statePub.publish(_enable);

    erro = _zAngle * _KP_OBSTACLE;

    cout << "E: SubirEscada\t yAngle: " << _yAngle;
  
    return;
  //está na escada ou descendo dela
  }else if(_state == IN_LADDER){  

    _enable.data = IN_STAIR;
    statePub.publish(_enable);

    erro = _zAngle *_KP_OBSTACLE;

    cout << "E: NaEscada\t" << "Erro:" << erro;
  }else if(_state == LADDER_DOWN){

    erro = _zAngle * _KP_OBSTACLE;

    if(fabs(_yAngle) > PLANE){
      _climbing = true;
      _enable.data = CLIMB_STAIR; 
      statePub.publish(_enable);
      downStairs();

    }else if(_climbing){
      _rodar = true;
      _state = WALKING;
      _isInStairs = false;
      _enable.data = -IN_STAIR; 
      statePub.publish(_enable);
    }

    cout << "E: DescendoEscada";

  //desvia do obstaculo na frente    
  }else if(!_straitPath && sidesInfo[_FRONT].medY < _MIN_DIST_FRONT){

    if(_sentido ==_HORARIO)
      erro = 1/(sidesInfo[_FRONT].medY);
    else
      erro = -1/(sidesInfo[_FRONT].medY);

    _avoidingObs = true;

    cout << "E: DesviaFr | Erro: " << erro;
    cout << " | AF: " << sidesInfo[_FRONT].area;
    cout << " | DFY: " << sidesInfo[_FRONT].medY;

  //Recupera o trajeto da direita
  }else if(sidesInfo[_RIGHT].medY < -0.15 && _sentido == _HORARIO){
    
    erro = -1/(sidesInfo[_RIGHT].medX);

    cout << "E: RecuDir | Erro: " << erro;
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;

    _avoidingObs = false;
  //Aproxima da esteira quando ela está a direita
  }else if(sidesInfo[_RIGHT].medX != 10 &&  _sentido == _HORARIO && abs(sidesInfo[_RIGHT].medX - _distToTrack) > 0.10){

    erro = (_distToTrack - sidesInfo[_RIGHT].medX) * _KP_REC;

    cout << "E: AproxDir | erro: " << erro << " | DDX: " << sidesInfo[_RIGHT].medX;

  //segue a parede da direita
  }else if(sidesInfo[_RIGHT].area > _MIN_AREA){

    erro = _zAngle * _KP_OBSTACLE;

    if(_straitPath && _sentido == _HORARIO && sidesInfo[_LEFT].medX > 0.5){
      _straitPath = false;
      _enable.data = -STRAIT_PATH;
      statePub.publish(_enable);
      _distToTrack = NICE_DIST_TRACK;
    }

    cout << "E: SegueDir | Erro: " << erro;
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;



  //Recupera o trajeto da esquerda
  }else if(sidesInfo[_LEFT].medY < -0.15 && _sentido == _ANTI_HORARIO){
    
    erro = 1/(sidesInfo[_LEFT].medX);

    _avoidingObs = false;

    cout << "E: RecuEsq | Erro: " << erro;
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;

  //Aproxima da esteira quando ela está a esquerda
  }else if(sidesInfo[_LEFT].medX != 10 && _sentido == _ANTI_HORARIO && abs(sidesInfo[_LEFT].medX - _distToTrack) > 0.10){

    erro = (_distToTrack - sidesInfo[_LEFT].medX) * -_KP_REC;

    cout << "E: AproxEsq | Erro: " << erro << " | DDX:" << sidesInfo[_LEFT].medX;

  //segue a parede da esquerda
  }else if(sidesInfo[_LEFT].area > _MIN_AREA){

    erro = _zAngle *_KP_OBSTACLE;

    if(_straitPath && _sentido ==_ANTI_HORARIO && sidesInfo[_RIGHT].medX > 0.5){
      _straitPath = false;
      _enable.data = -STRAIT_PATH;
      statePub.publish(_enable);
      _distToTrack = NICE_DIST_TRACK;
    }

    cout << "E: SegueEsq | Erro: " << erro;
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;


  //segue reto caso nao tenha nada
  }else{
    
    erro = 0.0;
    cout << "E: NormalEt";
  
  }
  

  //define as velocidades
  if(_straitPath)
    erro *= 1.2;

  if(_isInStairs && !(_state == IN_LADDER)){
    tractionDir = (float) (stairsDir * 3.5 + erro);
    if(tractionDir > _MAX_SPEED)
      tractionDir = _MAX_SPEED;
    else if(tractionDir < -_MAX_SPEED)
      tractionDir = -_MAX_SPEED; 
    

    tractionEsq = (float) (stairsDir * 3.5 - erro);
    if(tractionEsq > _MAX_SPEED)
      tractionEsq = _MAX_SPEED;
    else if(tractionEsq < -_MAX_SPEED)
      tractionEsq = -_MAX_SPEED; 


  }else{
    tractionDir = (float) (_V0 + _KP*erro);
    if(tractionDir > _MAX_SPEED)
      tractionDir = _MAX_SPEED;
    else if(tractionDir < -_MAX_SPEED)
      tractionDir = -_MAX_SPEED; 


    tractionEsq = (float) (_V0 - _KP*erro);
    if(tractionEsq > _MAX_SPEED)
      tractionEsq = _MAX_SPEED;
    else if(tractionEsq < -_MAX_SPEED)
      tractionEsq = -_MAX_SPEED; 

  }


  if(needSpeed)
    setSpeed(tractionDir + _SPEED_BOOST, tractionDir + _SPEED_BOOST, tractionEsq + _SPEED_BOOST, tractionEsq + _SPEED_BOOST);
  else
    setSpeed(tractionDir, tractionDir, tractionEsq, tractionEsq);


  cout << " | zAngle: " << _zAngle << endl;
  
}


void Robot::aligneEscada(SidesInfo *sidesInfo){

  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float tractionDir;
  float tractionEsq;

  
  static bool frontToTrack = false;
  static bool closeToTrack = false;

  cout << " | MedX: " << sidesInfo[_FRONT].medX;



  if(_sentido == _ANTI_HORARIO){
    cout << " ANTI_HORARIO";

    if(!frontToTrack && !closeToTrack){ //se nao estiver de frente para escada, endireita
      _provavelEscada = true; //nao depende mais da identificacao da escada

      cout << " | Endireitando" << " | zAngle: " << _zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_LEFT].medX;
      
      if(sidesInfo[_LEFT].medX < _MAX_DIST_SIDE_ESCADA + 0.15){ //esta de lado para a esteira
        closeToTrack = true;
        _provavelEscada = true;
        return;
      
      }else if(sidesInfo[_FRONT].medY < _MIN_DIST_ALIGNE_FRONT_STAIR ){ //esta de frente e perto da esteira
        frontToTrack = true;
        return;

      }else if((_zAngle > M_PI*3/2 - 0.2 || _zAngle < -M_PI/2 + 0.2) || sidesInfo[_LEFT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }else if(_zAngle > -M_PI/2 && _zAngle < M_PI/2){ //alinha antes de avancar para a esteira
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }else{ //alinha antes de avancar para a esteira
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }

    }else if(frontToTrack && !closeToTrack){
      _provavelEscada = false; //depende do reconhecimento da escada
      cout << "seguindo a esteira" << " | DX: " << sidesInfo[_LEFT].medX;

    
      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && _zAngle < -0.2){
        tractionDir = 0;
        tractionEsq = _V0*1.5;
      
      }else if(sidesInfo[_LEFT].area > _MIN_AREA_REC){ //ja esta perto da esteira
        
        if(_MIN_DIST_SIDE_ESCADA > sidesInfo[_LEFT].medX || sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA){ //chega ele mais perto
          tractionDir = _V0 + _KP_ALIGNE_ESCADA*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 - _KP_ALIGNE_ESCADA*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);

        
        }else{ //ja esta perto da esteira
          tractionEsq = _V0;
          tractionDir = _V0;
          closeToTrack = true;
          frontToTrack = false;
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
        if(_zAngle > -_MAX_ERRO_ESCADA && _zAngle < _MAX_ERRO_ESCADA){
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = true;
        }

        tractionDir = +_KP_ALIGNE_ESCADA*_zAngle*2.5;
        tractionEsq = -_KP_ALIGNE_ESCADA*_zAngle*2.5;
        cout << " | MedY: " << sidesInfo[_FRONT].medY;
      

      }else{
        if(sidesInfo[_LEFT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){ //se nao esiver identidicando mais nada
          cout << "PERDEU TUDO" << endl;
          closeToTrack = false;
          frontToTrack = false;
          return;

        }else if(_MIN_DIST_SIDE_ESCADA - _MAX_ERRO_SIDE_ESCADA > sidesInfo[_LEFT].medX || sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA + _MAX_ERRO_SIDE_ESCADA && sidesInfo[_FRONT].area < _MIN_AREA_REC){
          //se estiver muito distante
          tractionDir = _V0 + _KP_ALIGNE_ESCADA*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 - _KP_ALIGNE_ESCADA*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          cout << " | se endireitando";

        }else{
          
          cout << " | Corrigindo zAngle" << " | zAngle: " << _zAngle;
          cout << " | MedY: " << sidesInfo[_FRONT].medY;
          tractionDir = (_V0 + _KP_ALIGNE_ESCADA*_zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP_ALIGNE_ESCADA*_zAngle*1.5)/2;
        }
      }
    }

  }else{ //_sentido horario
    cout << "HORARIO";

    if(!frontToTrack && !closeToTrack){
      
      cout << " | Endireitando" << " | zAngle: " << _zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_RIGHT].medX;
      
      if(sidesInfo[_FRONT].medY < _MIN_DIST_ALIGNE_FRONT_STAIR){ //esta de frente e perto da esteira
        frontToTrack = true;
        return;

      }else if(sidesInfo[_RIGHT].medX < _MAX_DIST_SIDE_ESCADA + 0.15){ //esta de lado para a esteira
        closeToTrack = true;
        return;

      }else if((_zAngle > M_PI/2 - 0.2 && _zAngle < M_PI/2 + 0.2) || sidesInfo[_RIGHT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }else if(_zAngle <= M_PI/2){
          tractionDir = -_V0;
          tractionEsq = _V0;
          
      }else{ //alinha antes de avancar para a esteira
          tractionDir = -_V0;
          tractionEsq = _V0;
  
          
      }

    }else if(frontToTrack && !closeToTrack){
      _provavelEscada = false; //depende do reconhecimento da escada
      cout << " | seguindo a esteira" << " | DX: " << sidesInfo[_RIGHT].medX;

    
      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && _zAngle > 0.2){
        tractionDir = _V0*1.5;
        tractionEsq = 0;
      
      }else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC){ //ja esta perto da esteira
        
        if(_MIN_DIST_SIDE_ESCADA > sidesInfo[_RIGHT].medX || sidesInfo[_RIGHT].medX > _MAX_DIST_SIDE_ESCADA){ //chega ele mais perto
          tractionDir = _V0 - _KP_ALIGNE_ESCADA*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 + _KP_ALIGNE_ESCADA*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);

        
        }else{ //ja esta perto da esteira
          tractionEsq = _V0;
          tractionDir = _V0;
          closeToTrack = true;
          frontToTrack = false;
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
        if(_zAngle > -_MAX_ERRO_ESCADA && _zAngle < _MAX_ERRO_ESCADA){
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false;
        }

        tractionDir = +_KP_ALIGNE_ESCADA*_zAngle*2.5;
        tractionEsq = -_KP_ALIGNE_ESCADA*_zAngle*2.5;
        cout << " | MedY: " << sidesInfo[_FRONT].medY;
      

      }else{
        if(sidesInfo[_RIGHT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){ //se nao esiver identidicando mais nada
          cout << "PERDEU TUDO" << endl;
          closeToTrack = false;
          frontToTrack = false;
          return;

        }else if(_MIN_DIST_SIDE_ESCADA - _MAX_ERRO_SIDE_ESCADA > sidesInfo[_RIGHT].medX || sidesInfo[_RIGHT].medX > _MAX_DIST_SIDE_ESCADA + _MAX_ERRO_SIDE_ESCADA && sidesInfo[_FRONT].area < _MIN_AREA_REC){
          //se estiver muito distante
          tractionDir = _V0 - _KP_ALIGNE_ESCADA*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 + _KP_ALIGNE_ESCADA*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          cout << " | se endireitando";

        }else{
          
          cout << " | Corrigindo zAngle" << " | zAngle: " << _zAngle;
          cout << " | MedY: " << sidesInfo[_FRONT].medY;
          tractionDir = (_V0 + _KP_ALIGNE_ESCADA*_zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP_ALIGNE_ESCADA*_zAngle*1.5)/2;
        }
      }
    }
  }  


  //altera o vetor das velocidades das 'joints'


  setSpeed(tractionDir,tractionDir,tractionEsq,tractionEsq);


  statePub.publish(_enable);
}


bool Robot::climbStairs(){
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  bool needSpeed = false;
  float wheelFrontSpeed;
  float wheelRearSpeed;
  static double stairState = PLANE;



  if(fabs(_yAngle) < PLANE){

    cout << " IT'S PLANE" << endl;
    wheelRearSpeed = 0.0;
    wheelFrontSpeed = -_MAX_WHEEL_R_SPEED;
  
    setSpeed(_V0,_V0,_V0,_V0);

    if(_climbing){
      _climbing = false;

      _state = IN_LADDER;
      wheelFrontSpeed = 0.0;
      wheelRearSpeed = 0.0;

      _enable.data = -CLIMB_STAIR;
      statePub.publish(_enable);

      _enable.data = IN_STAIR;
      statePub.publish(_enable);
    } 

  }else if(fabs(_yAngle) < FRONT_WHEELS && (stairState == PLANE || stairState == FRONT_WHEELS)){
    
    stairState = FRONT_WHEELS;

    needSpeed = true;

    cout << " FRONT WHEELS IS ON" << endl;
    wheelRearSpeed = _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = 0;
    _climbing = true;

  
  }else if(fabs(_yAngle) > REAR_WHEELS && (stairState == FRONT_WHEELS || stairState == REAR_WHEELS)){

    stairState = REAR_WHEELS;

    needSpeed = true;

    cout << " REAR WHEELS IS ON" << endl;
    wheelRearSpeed = -_MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED/3;
    _climbing = true;

  }else{
    setSpeed(_V0,_V0,_V0,_V0);

    cout << " ESTABILIZING\n";
    wheelRearSpeed =  _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -_MAX_WHEEL_R_SPEED/5;
  }
  

  for(int i = 0; i < 4; i++)
    msg.data.push_back((i == 0 || i == 2) ? wheelFrontSpeed : wheelRearSpeed);
  

  wheelPub.publish(msg);
  return needSpeed;

}

void Robot::downStairs(){
  std_msgs::Float32MultiArray msg;
  msg.data.clear();

  float wheelRearSpeed = -_MAX_WHEEL_R_SPEED;

  for(int i = 0; i < 4; i++){
    msg.data.push_back((i == 0 || i == 2) ? 0.0f : wheelRearSpeed);
  }

  wheelPub.publish(msg);
}


void Robot::rodarFunction(SidesInfo* sidesInfo){ //roda o robo depois dele sair da escada

  if(_zAngle < 0) //deixa o angulo padronizado
    _zAngle += M_PI*2;


  cout << "Girando" << " | Zangle: " << _zAngle;

  if(sidesInfo[_FRONT].medY < _MIN_SAFE_DIST_SPIN) //afasta o robo
    setSpeed(-_V0 ,-_V0,-_V0, -_V0);
    

  else if(_sentido == _HORARIO) //roda no sentido HORARIO
    setSpeed(-_V0 ,-_V0,_V0, _V0);

  else //roda no sentido ANTI-HORARIO
    setSpeed(_V0 ,_V0,-_V0, -_V0);
    

  


  if(_zAngle > M_PI - 0.2 && _zAngle < M_PI + 0.2){
    _rodar = false; //para de rodar
    
    if(_sentido == _ANTI_HORARIO){
      _sentido = _HORARIO;
      
      _enable.data = ROBOT_CLOCKWISE;
      statePub.publish(_enable);

      _enable.data = -ROBOT_ANTICLOCKWISE;
      statePub.publish(_enable);

    }else{
      _sentido = _ANTI_HORARIO;
      
      _enable.data = ROBOT_ANTICLOCKWISE;
      statePub.publish(_enable);

      _enable.data = ROBOT_CLOCKWISE;
      statePub.publish(_enable);

    }

  }

  _avoidingObs = false;
  statePub.publish(_enable);
  
}


void Robot::setSpeed(float tractionDirFront, float tractionDirBack, float tractionLeftFront, float tractionLeftBack){

  std_msgs::Float32MultiArray msg;
  msg.data.clear();

  msg.data.push_back(tractionDirFront);
  msg.data.push_back(tractionDirBack);
  msg.data.push_back(tractionLeftFront);
  msg.data.push_back(tractionLeftBack);


  cout << " | VelE: " << tractionLeftFront << " | VelD: " << tractionDirFront << endl;


  speedPub.publish(msg);
}


void Robot::setAngles(double y, double z){ //recebe os angulos do IMU
    _yAngle = y;
    _zAngle = z;
}


void Robot::setPublishers(ros::Publisher speedPub, ros::Publisher wheelPub, ros::Publisher statePub){
    this->speedPub = speedPub;
    this->wheelPub = wheelPub;
    this->statePub = statePub;
}

bool Robot::getAvoidingObs(){
  return _avoidingObs;
}

bool Robot::getNothing(){
  return _nothing;
}


void Robot::setEnable(std_msgs::Int32 states){
  _states.data = states.data;
}

