#include "headers/robot.h"

using namespace std;

#define _SIDESRATIO 3.0


Robot::Robot(){
    _begin = true;
    _state = WALKING;
    _sentido = _HORARIO;
    _isInStairs = false;
    _provavelEscada = false;
    _rodar = false;
    _avoidingObs = false;
    _nothing = true;
    _narrowPath = false;
    _climbing = false;
    _distToTrack = _MIN_DIST_TRACK;
    _isInNarPath = false;
    _wheelsStable = true;
}


void Robot::processMap(SidesInfo *sidesInfo){

  bool needSpeed = false;
  float stairsDir = (_state == LADDER_DOWN) ? -0.7 : 1;
  float tractionDir = 0;
  float tractionEsq = 0;
  float erro;


  //define a precisao dos dados na hora de exibilos
  cout.precision(4);
  

  cout << "AvoidSide: " << _avoidSide << " | ";

  //corrige o angulo inicial do robo e define o sentido do mesmo
  if(_begin){
    if(_zAngle < 0)
      _zAngle += M_PI*2;
    
    
    if(sidesInfo[_FRONT].area > _MIN_AREA_REC/2)
      _begin = false;

    else if(_zAngle > M_PI && _zAngle < 2*M_PI-0.1){
      erro = -5*_V0/_KP;
      cout << "Alinhando posiIni | ";
    }
    else if(_zAngle < M_PI && _zAngle > 0.1){
      erro = 5*_V0/_KP;
      cout << "Alinhando posiIni | ";
    }
    
    else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC && sidesInfo[_LEFT].area < _MIN_AREA_REC){
      cout << "Setando para horario | ";
      _begin = false;
      _sentido = _HORARIO;
    
    }else if(sidesInfo[_RIGHT].area < _MIN_AREA_REC && sidesInfo[_LEFT].area > _MIN_AREA_REC){
      cout << "Setando para anti-horario | ";      
      _begin = false;
      _sentido = _ANTI_HORARIO;
    }else{
      cout << "Procurando... | ";
      erro = 0;
      
    }
    
    tractionDir = _V0 + erro;
    tractionEsq = _V0 - erro;


    cout << "zAngle: " << _zAngle << " | ";

    setSpeed(tractionDir, tractionDir, tractionEsq, tractionEsq);

    return;
  }


  //Confere se precisa aumentar a area de analise
  if(sidesInfo[_RIGHT].distance > _FAR && sidesInfo[_LEFT].distance > _FAR && sidesInfo[_FRONT].area < _MIN_AREA_REC){
    cout << "Longe | ";
    _nothing = true;
  }else{
    _nothing = false;
  }


  //Atualiza o estado do robo e determina o lado para qual ele deve desviar
  if(_sentido == _ANTI_HORARIO){
    _enable.data = ROBOT_ANTICLOCKWISE;
    statePub.publish(_enable);

    _enable.data = -ROBOT_CLOCKWISE;
    statePub.publish(_enable);

    _avoidSide = (_zAngle > -M_PI_2 || _zAngle < M_PI_2) ? _RIGHT : _LEFT;

  }else{
    _enable.data = ROBOT_CLOCKWISE;
    statePub.publish(_enable);

    _enable.data = -ROBOT_ANTICLOCKWISE;
    statePub.publish(_enable);    

    _avoidSide = (_zAngle > -M_PI_2 || _zAngle < M_PI_2) ? _LEFT : _RIGHT;

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

  //Determina se as esteiras das rodas devem ser arrumadas
  if(!_wheelsStable){
    stableWheelTrack();
    setSpeed(0, 0, 0, 0);
    return;
  }


  
  //Caminho Estreito
  double sidesRatio = (sidesInfo[_FRONT_LEFT].area == 0 || sidesInfo[_FRONT_RIGHT].area == 0)? 0.0 : sidesInfo[_FRONT_LEFT].area/sidesInfo[_FRONT_RIGHT].area;
  
  if(_avoidSide == _RIGHT && sidesRatio == 0)
    sidesRatio = 1000000;
  
  if(!_isInStairs && fabs(_zAngle)  < 0.3 && sidesInfo[_FRONT].medY < _MIN_DIST_FRONT &&
    ((_avoidSide == _LEFT && (sidesInfo[_FRONT_RIGHT].area == 0 || sidesRatio > _SIDESRATIO)) ||
    (_avoidSide == _RIGHT && (sidesInfo[_FRONT_LEFT].area == 0 || 1/sidesRatio > _SIDESRATIO)))) {

    _narrowPath = true;
    _enable.data = NARROW_PATH;
    statePub.publish(_enable);
    
    if(!_isInNarPath)
      _distToTrack = _MIN_DIST_TRACK - 0.25;

  } 

  //Condições para avaliar o caminho estreito
  if(_narrowPath){
    cout << "NarrowPath | ";

    if(_avoidSide == _LEFT){
      if(sidesInfo[_LEFT].medX > 0.45 && _distToTrack != 0.65){
        _distToTrack += 0.1;
      }

      if(sidesInfo[_LEFT].medX < 0.4 && _distToTrack > 0.6)
        _distToTrack -= 0.1;

      if(sidesInfo[_LEFT].medY < 0.3){
        _isInNarPath = true;
      }

      if(_isInNarPath && (sidesInfo[_LEFT].medY == 10 || sidesInfo[_LEFT].medX > 0.5)){
        _isInNarPath = false;
        _narrowPath = false;
        _enable.data = -NARROW_PATH;
        statePub.publish(_enable);
        _distToTrack = _MIN_DIST_TRACK;
      }
    }else{
      if(sidesInfo[_RIGHT].medX > 0.45 && _distToTrack != 0.65){
        _distToTrack += 0.1;
      }

      if(sidesInfo[_RIGHT].medX < 0.4 && _distToTrack > 0.6)
        _distToTrack -= 0.1;

      if(sidesInfo[_RIGHT].medY < 0.3){
        _isInNarPath = true;
      }

      if(_isInNarPath && (sidesInfo[_RIGHT].medY == 10 || sidesInfo[_RIGHT].medX > 0.5)){
        _isInNarPath = false;
        _narrowPath = false;
        _enable.data = -NARROW_PATH;
        statePub.publish(_enable);
        _distToTrack = _MIN_DIST_TRACK;
      }
    }




  }

  //confere se chegou na escada
  if(_isInStairs && _state != IN_LADDER && _state != LADDER_DOWN){
    _state = LADDER_UP;  
  }

  //Implementacao da maquina de estado

  //sobe a escada 
  if(_state == LADDER_UP){

    _enable.data = CLIMB_STAIR;
    statePub.publish(_enable);

    climbStairs();

    cout << "E: SubirEscada | yAngle: " << _yAngle << " | ";

    return;
  //está na escada ou descendo dela
  }else if(_state == IN_LADDER){  

    //chegou no final da esteira
    if(_states.data & (1 << END_STAIR)){
      _state = LADDER_DOWN;
      _enable.data = -END_STAIR;
      statePub.publish(_enable);
    }

    _enable.data = IN_STAIR;
    statePub.publish(_enable);

    erro = _zAngle *_KP_OBSTACLE; 
    cout << "E: NaEscada | " << "Erro: " << erro << " | ";

  }else if(_state == LADDER_DOWN){

    erro = _zAngle * _KP_OBSTACLE;

    //Verifica quando finaliza de descer a escada
    if(fabs(_yAngle) > PLANE){
      _climbing = true;
      _enable.data = CLIMB_STAIR; 
      statePub.publish(_enable);
      downStairs();

    }else if(_climbing && fabs(_yAngle) < 0.06){
      _rodar = true;
      _state = WALKING;
      _isInStairs = false;
      _enable.data = -IN_STAIR; 
      statePub.publish(_enable);
      _enable.data = -CLIMB_STAIR;
      statePub.publish(_enable);
    }

    cout << "E: DescendoEscada | ";

  //desvia do obstaculo na frente    
  }else if(!_narrowPath && sidesInfo[_FRONT].medY < _MIN_DIST_FRONT){

    erro = (_avoidSide == _LEFT) ? 1/(sidesInfo[_FRONT].medY) : -1/(sidesInfo[_FRONT].medY);

    _avoidingObs = true;

    cout << "E: DesviaFr | Erro: " << erro << " | ";
    cout << "AF: " << sidesInfo[_FRONT].area << " | ";
    cout << "DFY: " << sidesInfo[_FRONT].medY << " | ";

  //Recupera o trajeto da direita
  }else if(!_isInNarPath && sidesInfo[_RIGHT].medY < -0.15 && _avoidSide == _LEFT){
    
    erro = -1/(sidesInfo[_RIGHT].medX);

    cout << "E: RecuDir | Erro: " << erro << " | ";
    cout << "AD: " << sidesInfo[_RIGHT].area << " | ";
    cout << "DDX: " << sidesInfo[_RIGHT].medX << " | ";

    _avoidingObs = false;

  //Aproxima da esteira quando ela está a direita
  }else if(sidesInfo[_RIGHT].medX != 10 &&  _avoidSide == _LEFT && abs(sidesInfo[_RIGHT].medX - _distToTrack) > 0.05){

    erro = (_distToTrack - sidesInfo[_RIGHT].medX) * _KP_REC;
    cout << "E: AproxDir | erro: " << erro << " | ";
    cout << "DDX: " << sidesInfo[_RIGHT].medX << " | ";

  //segue a parede da direita
  }else if(sidesInfo[_RIGHT].area > _MIN_AREA){


    if(_avoidSide == _RIGHT)
      erro = (_zAngle - M_PI) * _KP_OBSTACLE;
    else
      erro = _zAngle * _KP_OBSTACLE;

    
    cout << "E: SegueDir | Erro: " << erro << " | ";
    cout << "AD: " << sidesInfo[_RIGHT].area << " | ";
    cout << "DDX: " << sidesInfo[_RIGHT].medX << " | ";
    cout << "DEX: " << sidesInfo[_LEFT].medX << " | ";

  //Recupera o trajeto da esquerda
  }else if(!_isInNarPath && sidesInfo[_LEFT].medY < -0.15 && _avoidSide == _RIGHT){
    
    erro = 1/(sidesInfo[_LEFT].medX);

    _avoidingObs = false;

    cout << "E: RecuEsq | Erro: " << erro << " | ";
    cout << "AE: " << sidesInfo[_LEFT].area << " | ";
    cout << "DEX: " << sidesInfo[_LEFT].medX << " | ";

  //Aproxima da esteira quando ela está a esquerda
  }else if(sidesInfo[_LEFT].medX != 10 && _avoidSide == _RIGHT && abs(sidesInfo[_LEFT].medX - _distToTrack) > 0.05){

    erro = (_distToTrack - sidesInfo[_LEFT].medX) * -_KP_REC;

    cout << "E: AproxEsq | Erro: " << erro << " | ";
    cout << "DDX: " << sidesInfo[_LEFT].medX << " | ";

  //segue a parede da esquerda
  }else if(sidesInfo[_LEFT].area > _MIN_AREA){

    if(_avoidSide == _RIGHT)
      erro = (_zAngle - M_PI) *_KP_OBSTACLE;
    else
      erro = _zAngle *_KP_OBSTACLE;

    cout << "E: SegueEsq | Erro: " << erro << " | ";
    cout << "AE: " << sidesInfo[_LEFT].area << " | ";
    cout << "DEX: " << sidesInfo[_LEFT].medX << " | ";

  //segue reto caso nao tenha nada
  }else{
    
    erro = 0.0;
    cout << "E: NormalEt" << " | ";
  
  }
  

  //define as velocidades
  if(_narrowPath)
    erro *= 1.3;

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


  cout << "zAngle: " << _zAngle << " | ";

  setSpeed(tractionDir, tractionDir, tractionEsq, tractionEsq);
  
}


void Robot::aligneEscada(SidesInfo *sidesInfo){

  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float tractionDir;
  float tractionEsq;

  
  static bool frontToTrack = false;
  static bool closeToTrack = false;

  cout << "MedX: " << sidesInfo[_FRONT].medX << " | ";



  if(_sentido == _ANTI_HORARIO){

    if(!frontToTrack && !closeToTrack){ //se nao estiver de frente para escada, endireita
      _provavelEscada = true; //nao depende mais da identificacao da escada

      cout << "Endireitando | " << "zAngle: " << _zAngle << " | ";
      cout << "DFY: " << sidesInfo[_FRONT].medY << " | ";
      cout << "DX: " << sidesInfo[_LEFT].medX << " | ";
      
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
      cout << "seguindo a esteira | " << "DX: " << sidesInfo[_LEFT].medX << " | ";

    
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
      cout <<"Perto da esteira | " << "DX: " << sidesInfo[_LEFT].medX << " | ";

      if(sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){ //esta de frente com a escada
        cout << "Alinhando com escada | "; 
        if(_zAngle > -_MAX_ERRO_ESCADA && _zAngle < _MAX_ERRO_ESCADA){
          cout << "escada | ";
          _isInStairs = true;
          _provavelEscada = true;
        }

        tractionDir = +_KP_ALIGNE_ESCADA*_zAngle*2.5;
        tractionEsq = -_KP_ALIGNE_ESCADA*_zAngle*2.5;
        cout << "MedY: " << sidesInfo[_FRONT].medY << " | ";
      

      }else{
        if(sidesInfo[_LEFT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){ //se nao esiver identidicando mais nada
          cout << "PERDEU TUDO | " << endl;
          closeToTrack = false;
          frontToTrack = false;
          return;

        }else if(_MIN_DIST_SIDE_ESCADA - _MAX_ERRO_SIDE_ESCADA > sidesInfo[_LEFT].medX || sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA + _MAX_ERRO_SIDE_ESCADA && sidesInfo[_FRONT].area < _MIN_AREA_REC){
          //se estiver muito distante
          tractionDir = _V0 + _KP_ALIGNE_ESCADA*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 - _KP_ALIGNE_ESCADA*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          cout << "se endireitando | ";

        }else{
          
          cout << "Corrigindo zAngle | " << "zAngle: " << _zAngle << " | ";
          cout << "MedY: " << sidesInfo[_FRONT].medY << " | ";
          tractionDir = (_V0 + _KP_ALIGNE_ESCADA*_zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP_ALIGNE_ESCADA*_zAngle*1.5)/2;
        }
      }
    }

  }else{ //_sentido horario

    if(!frontToTrack && !closeToTrack){
      
      cout << "Endireitando | " << "zAngle: " << _zAngle << " | ";
      cout << "DFY: " << sidesInfo[_FRONT].medY << " | ";
      cout << "DX: " << sidesInfo[_RIGHT].medX << " | ";
      
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
      cout << "seguindo a esteira | " << "DX: " << sidesInfo[_RIGHT].medX << " | ";

    
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
      cout <<"Perto da esteira | " << "DX: " << sidesInfo[_RIGHT].medX << " | ";

      if(sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){ //esta de frente com a escada
        cout << "Alinhando com escada" << " | "; 
        if(_zAngle > -_MAX_ERRO_ESCADA && _zAngle < _MAX_ERRO_ESCADA){
          cout << "escada | ";
          _isInStairs = true;
          _provavelEscada = false;
        }

        tractionDir = +_KP_ALIGNE_ESCADA*_zAngle*2.5;
        tractionEsq = -_KP_ALIGNE_ESCADA*_zAngle*2.5;
        cout << "MedY: " << sidesInfo[_FRONT].medY << " | ";
      

      }else{
        if(sidesInfo[_RIGHT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){ //se nao esiver identidicando mais nada
          cout << "PERDEU TUDO | " << endl;
          closeToTrack = false;
          frontToTrack = false;
          return;

        }else if(_MIN_DIST_SIDE_ESCADA - _MAX_ERRO_SIDE_ESCADA > sidesInfo[_RIGHT].medX || sidesInfo[_RIGHT].medX > _MAX_DIST_SIDE_ESCADA + _MAX_ERRO_SIDE_ESCADA && sidesInfo[_FRONT].area < _MIN_AREA_REC){
          //se estiver muito distante
          tractionDir = _V0 - _KP_ALIGNE_ESCADA*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 + _KP_ALIGNE_ESCADA*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          cout << "se endireitando | ";

        }else{
          
          cout << "Corrigindo zAngle | " << "zAngle: " << _zAngle << " | ";
          cout << "MedY: " << sidesInfo[_FRONT].medY << " | ";
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


void Robot::climbStairs(){
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  bool needSpeed = false;
  float wheelFrontSpeed;
  float wheelRearSpeed;
  static double stairState = PLANE;
  static int i = 0;


  if(fabs(_yAngle) < PLANE){

    cout << "IT'S PLANE | ";

    wheelRearSpeed = 0.0;
    wheelFrontSpeed = -_MAX_WHEEL_R_SPEED/1.3;
  
    setSpeed(_V0,_V0,_V0,_V0);
    if(_climbing) i++;

    if(i > 3){
      _climbing = false;

      _state = IN_LADDER;
      wheelFrontSpeed = 0.0;
      wheelRearSpeed = 0.0;

      _wheelsStable = false;

      _enable.data = -CLIMB_STAIR;
      statePub.publish(_enable);

      _enable.data = IN_STAIR;
      statePub.publish(_enable);
    } 

  }else if(fabs(_yAngle) < FRONT_WHEELS && (stairState == PLANE || stairState == FRONT_WHEELS)){
    cout << "FRONT WHEELS IS ON | ";

    setSpeed(_V0,_V0,_V0,_V0);
    //setSpeed(_MAX_SPEED, _MAX_SPEED, _MAX_SPEED, _MAX_SPEED);

    stairState = FRONT_WHEELS;

    needSpeed = true;

    wheelRearSpeed = _MAX_WHEEL_R_SPEED;
    //wheelFrontSpeed = -_MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED/3;

    _climbing = true;

  
  }else if(fabs(_yAngle) > REAR_WHEELS && (stairState == FRONT_WHEELS || stairState == REAR_WHEELS)){
    cout << "REAR WHEELS IS ON | ";

    setSpeed(_V0,_V0,_V0,_V0);
    //setSpeed(_MAX_SPEED, _MAX_SPEED, _MAX_SPEED, _MAX_SPEED);    

    stairState = REAR_WHEELS;

    needSpeed = true;

    wheelRearSpeed = -_MAX_WHEEL_R_SPEED;
    //wheelFrontSpeed = _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED/3;
    _climbing = true;

  }else{
    cout << "ESTABILIZING |";

    // wheelRearSpeed =  -_MAX_WHEEL_R_SPEED;
    // wheelFrontSpeed = -_MAX_WHEEL_R_SPEED;
    setSpeed(_V0,_V0,_V0,_V0);

    wheelRearSpeed =  _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -_MAX_WHEEL_R_SPEED/6;
  }
  

  for(int i = 0; i < 4; i++)
    msg.data.push_back((i == 0 || i == 2) ? wheelFrontSpeed : wheelRearSpeed);
  

  wheelPub.publish(msg);
  return;

}


void Robot::downStairs(){
  std_msgs::Float32MultiArray msg;
  msg.data.clear();

  float wheelRearSpeed = _MAX_WHEEL_R_SPEED;

  for(int i = 0; i < 4; i++){
    msg.data.push_back((i == 0 || i == 2) ? 0.0f : wheelRearSpeed);
  }

  wheelPub.publish(msg);
}


void Robot::rodarFunction(SidesInfo* sidesInfo){ //roda o robo depois dele sair da escada

  if(_zAngle < 0) //deixa o angulo padronizado
    _zAngle += M_PI*2;


  cout << "Girando | " << "Zangle: " << _zAngle << " | ";

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


  cout << "VelE: " << tractionLeftFront << " | " << "VelD: " << tractionDirFront << endl;

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

bool Robot::getisInStair(){
  return _provavelEscada;
}

void Robot::setEnable(std_msgs::Int32 states){
  _states.data = states.data;
}

void Robot::stableWheelTrack(){
  std_msgs::Float32MultiArray msg;
  msg.data.clear();

  float wheelFrontSpeed;
  float wheelRearSpeed;

  static bool frontWheels = true;
  static float yAngle = _yAngle;
  
  if(frontWheels){
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED/3;
    wheelRearSpeed = 0;

    cout << "dif: " << yAngle - _yAngle << " ";

    if(yAngle - _yAngle > 0.005){
      frontWheels = false;
      yAngle = _yAngle;
    }
  
  }else{
    wheelFrontSpeed = 0;
    wheelRearSpeed = -_MAX_WHEEL_R_SPEED/3;

    cout << "dif: " << _yAngle - yAngle << " ";

    if(_yAngle - yAngle > -0.0021)
      _wheelsStable = true;
  }


  /*  
  static int i = 0;

  if(frontWheels){
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED;
    wheelRearSpeed = 0.0;

    if(i == 0 && fabs(_yAngle) > 0.05){
      i = 5;
    }

    if(i != 0){
      wheelFrontSpeed = -_MAX_WHEEL_R_SPEED;
      i--;
      if(i == 0)
        frontWheels = false;
    }

  }else{
    wheelFrontSpeed = 0.0;
    wheelRearSpeed = -_MAX_WHEEL_R_SPEED;

    if(i == 0 && fabs(_yAngle) > 0.05){
      i = 5;
    }

    if(i != 0){
      wheelFrontSpeed = -_MAX_WHEEL_R_SPEED;
      i--;
      if(i == 0)
        _wheelsStable = true;
    }

  }
  */

  for(int i = 0; i < 4; i++){
    msg.data.push_back((i == 0 || i == 2) ? wheelFrontSpeed : wheelRearSpeed);
  }

  wheelPub.publish(msg);
}