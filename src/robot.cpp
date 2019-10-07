#include "headers/robot.h"

using namespace std;

#define _KP_REC 0.7
#define NICE_DIST_TRACK 0.80
#define FRONT_WHEELS 0.15
#define REAR_WHEELS 0.25
#define OKAY 0.06

Robot::Robot(){
    _state = WALKING;
    _sentido = _HORARIO;
    _isInStairs = false;
    _provavelEscada = false;
    _rodar = false;
    _avoidingObs = false;
    _nothing = false;
    _straitPath = false;

    _distToTrack = NICE_DIST_TRACK;
}

void Robot::processMap(SidesInfo *sidesInfo){

  _nothing = false;

  if(_sentido == _ANTI_HORARIO){
    _enable.data |= (1 << ROBOT_ANTICLOCKWISE);
    _enable.data &= ~(1 << ROBOT_CLOCKWISE);

  }else{
    _enable.data |= (1 << ROBOT_CLOCKWISE);
    _enable.data &= ~(1 << ROBOT_ANTICLOCKWISE);    
  }


  if(_rodar){
    _avoidingObs = false;
    rodarFunction(sidesInfo);
    return;
  }

  if((_enable.data & (1 << FOUND_STAIR) || _provavelEscada) && !_isInStairs){
    _avoidingObs = false;    
    aligneEscada(sidesInfo);
    return;
  }
    

  float stairsDir = (_state == LADDER_DOWN) ? -0.7 : 1;
  float traction = 0;
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float erro;

  if(_state == IN_LADDER && _enable.data & (1 << END_STAIR)){
    _state = LADDER_DOWN;
    _enable.data &= ~(1 << END_STAIR); 
  }

  if(!_isInStairs && ((_sentido == _HORARIO && sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT && !(sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT))
    || (_sentido == _ANTI_HORARIO && !(sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT) && sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT))){

    _straitPath = true;
    _enable.data |= (1 << STRAIT_PATH);
    _distToTrack = NICE_DIST_TRACK - 0.25;
    cout << "StraitPath\t";

  }

  if(_isInStairs && !(_state == IN_LADDER || _state == LADDER_DOWN)){
    _state = LADDER_UP;  
  }

  if(_state == LADDER_DOWN && abs(_yAngle) < OKAY && _enable.data & (1 << FOUND_STAIR)){
    _rodar = true;
    _enable.data &= ~(1 << FOUND_STAIR);
    _state = WALKING;
    _isInStairs = false;
  }

  //Implementacao da maquina de estado

  //sobe a escada 
  if(_state == LADDER_UP){

    climbStairs();

    _enable.data |= (1 << CLIMB_STAIR);

    erro = _zAngle *_KP_OBSTACLE;

    cout << "E: SubirEscada\t yAngle: " << _yAngle;
  
  //está na escada ou descendo dela
  }else if(_state == IN_LADDER){
    
    _enable.data |= (1 << IN_LADDER);

    erro = _zAngle *_KP_OBSTACLE;

    cout << "E: NaEscada\t" << "Erro:" << erro;
  }else if(_state == LADDER_DOWN){

    erro = _zAngle * _KP_OBSTACLE;

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
  }else if(sidesInfo[_RIGHT].medX!= 10 &&  _sentido == _HORARIO && abs(sidesInfo[_RIGHT].medX - _distToTrack) > 0.10){

    erro = (_distToTrack - sidesInfo[_RIGHT].medX) * _KP_REC;

    cout << "E: AproxDir | erro: " << erro << " | DDX: " << sidesInfo[_RIGHT].medX;

  //segue a parede da direita
  }else if(sidesInfo[_RIGHT].area > _MIN_AREA){

    erro = _zAngle * _KP_OBSTACLE;

    if(_straitPath && _HORARIO && sidesInfo[_LEFT].medX > 0.5){
      _straitPath = false;
      _enable.data &= ~(1 << STRAIT_PATH);
      _distToTrack = NICE_DIST_TRACK;
    }

    cout << "E: SegueDir | Erro: " << erro;
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;


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

    if(_straitPath && _ANTI_HORARIO && sidesInfo[_RIGHT].medX > 0.5){
    _straitPath = false;
    _enable.data &= ~(1 << STRAIT_PATH);
    _distToTrack = NICE_DIST_TRACK;
    }

    cout << "E: SegueEsq | Erro: " << erro;
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;


  //segue reto caso nao tenha nada
  }else{
    
    erro = 0.0;

    if(sidesInfo[_FRONT].area < _MIN_AREA_REC && sidesInfo[_LEFT].area < _MIN_AREA_REC && sidesInfo[_RIGHT].area < _MIN_AREA_REC)
      
    cout << "E: NormalEt";
  
  }

  if(_straitPath)
    erro *= 1.2;

  if(_isInStairs && !(_state == IN_LADDER)){
    traction = (float) (stairsDir * 3.5 + erro);
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 
    
    cout << " | VelD: " << traction;

    msg.data.push_back(traction);
    msg.data.push_back(traction);

    traction = (float) (stairsDir * 3.5 - erro);
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

  cout << " | zAngle: " << _zAngle << endl;
  
  speedPub.publish(msg);
  statePub.publish(_enable);
}


void Robot::aligneEscada(SidesInfo *sidesInfo){


  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float tractionDir;
  float tractionEsq;

  
  static bool frontToTrack = false;
  static bool closeToTrack = false;


  if(_sentido == _ANTI_HORARIO){
    cout << "ANTI_HORARIO";

    if(!frontToTrack && !closeToTrack){ //se nao estiver de frente para escada, endireita
      _provavelEscada = true; //nao depende mais da identificacao da escada

      cout << " | Endireitando" << " | zAngle: " << _zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_LEFT].medX;
      

      if(sidesInfo[_FRONT].medY < 1.1){ //esta de frente e perto da esteira
        frontToTrack = true;
        return;
      
      }else if(sidesInfo[_LEFT].medX < _MAX_DIST_SIDE_ESCADA){ //esta de lado para a esteira
        closeToTrack = true;
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
        if(_zAngle > -_MAX_ERRO_ESCADA && _zAngle < _MAX_ERRO_ESCADA){
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false;
        }

        tractionDir = +_KP*_zAngle*2.5;
        tractionEsq = -_KP*_zAngle*2.5;
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
          
          cout << " | Corrigindo zAngle" << " | zAngle: " << _zAngle;
          cout << " | MedY: " << sidesInfo[_FRONT].medY;
          tractionDir = (_V0 + _KP*_zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP*_zAngle*1.5)/2;
        }
      }
    }

  }else{ //_sentido horario
    cout << "HORARIO";

    if(!frontToTrack && !closeToTrack){
      
      cout << " | Endireitando" << " | zAngle: " << _zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_RIGHT].medX;
      
      if(sidesInfo[_FRONT].medY < 1.1){ //esta de frente e perto da esteira
        frontToTrack = true;
        return;

      }else if(sidesInfo[_RIGHT].medX < _MAX_DIST_SIDE_ESCADA){ //esta de lado para a esteira
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
        if(_zAngle > -_MAX_ERRO_ESCADA && _zAngle < _MAX_ERRO_ESCADA){
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false;
        }

        tractionDir = +_KP*_zAngle*2.5;
        tractionEsq = -_KP*_zAngle*2.5;
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
          
          cout << " | Corrigindo zAngle" << " | zAngle: " << _zAngle;
          cout << " | MedY: " << sidesInfo[_FRONT].medY;
          tractionDir = (_V0 + _KP*_zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP*_zAngle*1.5)/2;
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
  statePub.publish(_enable);
}


void Robot::climbStairs(){
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float wheelFrontSpeed;
  float wheelRearSpeed;
  static bool _climbing = false;
  static float hystCoef = 1.0;

  if(abs(_yAngle) < OKAY){

    cout << " IT'S OKAY\n";
    wheelRearSpeed = 0.0f;
    wheelFrontSpeed = -_MAX_WHEEL_R_SPEED;
  
    if(_climbing){
      static int i = 0;
      i++;
      if(i > 3){
        _state = IN_LADDER;
        wheelFrontSpeed = 0.0;
        wheelRearSpeed = 0.0;
        _enable.data &= ~(1 << CLIMB_STAIR);
      }
    } 

  }else if(abs(_yAngle) > REAR_WHEELS * hystCoef){

    cout << " REAR WHEELS IS ON\n";
    wheelRearSpeed = -_MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else if(abs(_yAngle) < FRONT_WHEELS){
    
    cout << " FRONT WHEELS IS ON\n";
    wheelRearSpeed = _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -_MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else{

    cout << " ESTABILIZING\n";
    wheelRearSpeed = (_yAngle > 0)? -_MAX_WHEEL_R_SPEED : -_MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = (_yAngle > 0)? _MAX_WHEEL_R_SPEED : _MAX_WHEEL_R_SPEED;

  }

  for(int i = 0; i < 4; i++){
    msg.data.push_back((i == 0 || i == 2) ? wheelFrontSpeed : wheelRearSpeed);
  }

  wheelPub.publish(msg);

}

void Robot::rodarFunction(SidesInfo* sidesInfo){ //roda o robo depois dele sair da escada
 
  if(_zAngle < 0) //deixa o angulo padronizado
    _zAngle += M_PI*2;


  std_msgs::Float32MultiArray msg;
  msg.data.clear();

  cout << "Girando";


  if(sidesInfo[_FRONT].medY < _MIN_SAFE_DIST_SPIN){ //afasta o robo
    cout << " | Afastando o robo";
    msg.data.push_back(-_V0);
    msg.data.push_back(-_V0);
    msg.data.push_back(-_V0);
    msg.data.push_back(-_V0);
  
  }else if(_sentido == _HORARIO){ //roda no sentido HORARIO
    msg.data.push_back(-_V0);
    msg.data.push_back(-_V0);
    msg.data.push_back(0);
    msg.data.push_back(0);


  }else{ //roda no sentido ANTI-HORARIO
    msg.data.push_back(0);
    msg.data.push_back(0);
    msg.data.push_back(-_V0);
    msg.data.push_back(-_V0);

  }


  if(_zAngle > M_PI - 0.2 && _zAngle < M_PI + 0.2){
    _rodar = false; //para de rodar
    
    if(_sentido == _ANTI_HORARIO){
      _sentido = _HORARIO;
      _enable.data |= (1 << ROBOT_CLOCKWISE);
      _enable.data &= ~(1 << ROBOT_ANTICLOCKWISE);
    }else{
      _sentido = _ANTI_HORARIO;
      _enable.data |= (1 << ROBOT_ANTICLOCKWISE);
      _enable.data &= ~(1 << ROBOT_CLOCKWISE);
    }

  }

  cout << " | VelE: " << msg.data[3] << " | VelD: " << msg.data[0] << endl;

  _avoidingObs = false;
  speedPub.publish(msg);
  statePub.publish(_enable);
  msg.data.clear();
  
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


void Robot::setEnable(std_msgs::Int32 enable){
  _enable.data = enable.data;
}


void Robot::setStatePub(std_msgs::Int32 _enable){ //publica o estado
  statePub.publish(_enable);
}

