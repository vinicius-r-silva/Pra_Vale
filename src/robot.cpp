#include "headers/robot.h"

using namespace std;

#define _KP_REC 0.7
#define NICE_DIST_TRACK 0.80
#define FRONT_WHEELS 0.18
#define REAR_WHEELS 0.3
#define OKAY 0.06

Robot::Robot(){
    _state = WALKING;
    _sentido = _HORARIO;
    _isInStairs = true;
    _provavelEscada = false;
    _rodar = false;
    _avoidingObs = false;
    _saveAngle = 10;
    _straitPath = false;

    _distToTrack = NICE_DIST_TRACK;
}

void Robot::processMap(SidesInfo *sidesInfo){
  if(_rodar)
    return;

  int stairsDir = (_state == LADDER_DOWN) ? -0.7 : 1;
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

  if(!_isInStairs && ((_sentido == _HORARIO && sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT && !(sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT))
    || (_sentido == _ANTI_HORARIO && !(sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT) && sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT))){

    _straitPath = true;
    _distToTrack = NICE_DIST_TRACK - 0.25;
    cout << "StraitPath\t";
  
  }

//  cout << " | isInStairs: " << _isInStairs;
  if(_isInStairs && !(_state == IN_LADDER || _state == LADDER_DOWN)){
    _state = LADDER_UP;  
  }

  //Implementacao da maquina de estado

  //sobe a escada 
  if(_state == LADDER_UP){

    climbStairs();

    erro = _zAngle *_KP_OBSTACLE;

    cout << "E: SubirEscada\t yAngle: " << _yAngle;
  
  //está na escada ou descendo dela
  }else if(_state == IN_LADDER || _state == LADDER_DOWN){
    
    erro = _zAngle *_KP_OBSTACLE;

    cout << "E: NaEscada\t ZAngle: " << _zAngle << "Erro:" << erro;

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
  }else if(sidesInfo[_RIGHT].medY < -0.20 && _sentido == _HORARIO){
    
    erro = -1/(sidesInfo[_RIGHT].medX);

    cout << "E: RecuDir | Erro: " << erro;
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;

    _avoidingObs = false;
  //Aproxima da esteira quando ela está a direita
  }else if(_sentido == _HORARIO && abs(sidesInfo[_RIGHT].medX - _distToTrack) > 0.10){

    erro = (sidesInfo[_RIGHT].medX != 10) ? (_distToTrack - sidesInfo[_RIGHT].medX) * _KP_REC : -0.1;

    cout << "E: AproxDir | erro: " << erro << " | DDX: " << sidesInfo[_RIGHT].medX;

  //segue a parede da direita
  }else if(sidesInfo[_RIGHT].area > _MIN_AREA){

    erro = _zAngle * _KP_OBSTACLE;

    if(_straitPath && _HORARIO && sidesInfo[_LEFT].medX > 0.5){
      _straitPath = false;
      _distToTrack = NICE_DIST_TRACK;
    }

    cout << "E: SegueDir | Erro: " << erro;
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;


  //Recupera o trajeto da esquerda
  }else if(sidesInfo[_LEFT].medY < -0.20 && _sentido == _ANTI_HORARIO){
    
    erro = 1/(sidesInfo[_LEFT].medX);

    _avoidingObs = false;

    cout << "E: RecuEsq | Erro: " << erro;
    cout << " | AE: " << sidesInfo[_LEFT].area;
    cout << " | DEX: " << sidesInfo[_LEFT].medX;
  //Aproxima da esteira quando ela está a esquerda
  }else if(_sentido == _ANTI_HORARIO && abs(sidesInfo[_LEFT].medX - _distToTrack) > 0.10){

    erro = (sidesInfo[_LEFT].medX != 10) ? (_distToTrack - sidesInfo[_LEFT].medX) * -_KP_REC : -0.1;

    cout << "E: AproxEsq | Erro: " << erro << " | DDX:" << sidesInfo[_LEFT].medX;

  //segue a parede da esquerda
  }else if(sidesInfo[_LEFT].area > _MIN_AREA){

    erro = _zAngle *_KP_OBSTACLE;

    if(_straitPath && _ANTI_HORARIO && sidesInfo[_RIGHT].medX > 0.5){
    _straitPath = false;
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

  if(_straitPath)
    erro *= 1.2;

  if(_isInStairs && !(_state == IN_LADDER)){
    traction = (float) (stairsDir * (3.5 + erro));
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 
    
    cout << " | VelD: " << traction;

    msg.data.push_back(traction);
    msg.data.push_back(traction);

    traction = (float) (stairsDir * (3.5 - erro));
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
}


void Robot::aligneEscada(SidesInfo *sidesInfo){


  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float tractionDir;
  float tractionEsq;
  //pra_vale::RosiMovement tractionDir;
  //pra_vale::RosiMovement tractionEsq;

  static bool frontToTrack = false;
  static bool closeToTrack = false;


  if(_sentido == _ANTI_HORARIO){
    cout << "ANTI_HORARIO";

    if(!frontToTrack && !closeToTrack){
      
      cout << " | Endireitando" << " | zAngle: " << _zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_LEFT].medX;
      
      if(sidesInfo[_FRONT].medY < 1.1){
        frontToTrack = true;
        return;
      
      }else if(sidesInfo[_LEFT].medX < _MAX_DIST_SIDE_ESCADA){
        closeToTrack = true;
        return;
      
      
      }else if((_zAngle > M_PI*3/2 - 0.2 || _zAngle < -M_PI/2 + 0.2) || sidesInfo[_LEFT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }/*else if(sidesInfo[_LEFT].area > _MIN_AREA_REC/2 && sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA && sidesInfo[_LEFT].medX < 0.9){

        tractionDir = _V0;
        tractionEsq = _V0;
      

      }*/else if(_zAngle > -M_PI/2 && _zAngle < M_PI/2){
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }else{
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }

    }else if(frontToTrack && !closeToTrack){ //esta de frente mas precisa alinhar
      _provavelEscada = false;
      cout << "seguindo a esteira" << " | DX: " << sidesInfo[_LEFT].medX;

    
      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && _zAngle < -0.2){
        tractionDir = 0;
        tractionEsq = _V0*1.5;
      
      }else if(sidesInfo[_LEFT].area > _MIN_AREA_REC){
        
        if(_MIN_DIST_SIDE_ESCADA > sidesInfo[_LEFT].medX || sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA){ //chega ele mais perto
          tractionDir = _V0 + _KP*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 - _KP*(sidesInfo[_LEFT].medX - _MIN_DIST_SIDE_ESCADA);

        
        }else{
          tractionEsq = _V0;
          tractionDir = _V0;
          closeToTrack = true;
        }
        

      }else{
        tractionEsq = 0;
        tractionDir = 0;
        frontToTrack = false;

      

      }
    
    }else if(closeToTrack){
      _provavelEscada = true;
      cout <<" | Perto da esteira" << " | DX: " << sidesInfo[_LEFT].medX;

      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){
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
        if(sidesInfo[_LEFT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){
            cout << "PERDEU TUDO" << endl;
            closeToTrack = false;
            frontToTrack = false;
            return;

        }else if(_MIN_DIST_SIDE_ESCADA - 0.1 > sidesInfo[_LEFT].medX || sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA + 0.1 && sidesInfo[_FRONT].area < _MIN_AREA_REC){
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
      
      if(sidesInfo[_FRONT].medY < 1.1){
        frontToTrack = true;


      }else if((_zAngle > M_PI/2 - 0.2 && _zAngle < M_PI/2 + 0.2) || sidesInfo[_RIGHT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }/*else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC/2){

        tractionDir = _V0;
        tractionEsq = _V0;

      }*/else if(_zAngle <= M_PI/2){
          tractionDir = -_V0;
          tractionEsq = _V0;
          
      }else{
          tractionDir = -_V0;
          tractionEsq = _V0;
  
          
      }

    }else if(frontToTrack && !closeToTrack){ //esta de frente mas precisa alinhar
      cout << " | seguindo a esteira" << " | DX: " << sidesInfo[_RIGHT].medX;

    
      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && _zAngle > 0.2){
        tractionDir = _V0*1.5;
        tractionEsq = 0;
      
      }else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC){
        
        if(_MIN_DIST_SIDE_ESCADA > sidesInfo[_RIGHT].medX || sidesInfo[_RIGHT].medX > _MAX_DIST_SIDE_ESCADA){ //chega ele mais perto
          tractionDir = _V0 - _KP*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);
          tractionEsq = _V0 + _KP*(sidesInfo[_RIGHT].medX - _MIN_DIST_SIDE_ESCADA);

        
        }else{
          tractionEsq = _V0;
          tractionDir = _V0;
          closeToTrack = true;
        }
        

      }else{
        tractionEsq = 0;
        tractionDir = 0;
        frontToTrack = false;

      }
    
    }else if(closeToTrack){
      _provavelEscada = true;
      cout <<" | Perto da esteira" << " | DX: " << sidesInfo[_RIGHT].medX;

      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){
        cout << " | Alinhando com escada"; 
        if(_zAngle > -_MAX_ERRO_ESCADA && _zAngle < _MAX_ERRO_ESCADA){
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false;
          _enable.data &= ~(1<< FOUND_STAIR);
          statePub.publish(_enable);
        }

        tractionDir = +_KP*_zAngle*2.5;
        tractionEsq = -_KP*_zAngle*2.5;
        cout << " | MedY: " << sidesInfo[_FRONT].medY;
      

      }else{
        if(sidesInfo[_RIGHT].area < _MIN_AREA_REC/2 && sidesInfo[_FRONT].area < _MIN_AREA_REC/2){
            cout << "PERDEU TUDO" << endl;
            closeToTrack = false;
            frontToTrack = false;
            return;

        }else if(_MIN_DIST_SIDE_ESCADA-0.1 > sidesInfo[_RIGHT].medX || sidesInfo[_RIGHT].medX > _MAX_DIST_SIDE_ESCADA + 0.1 && sidesInfo[_FRONT].area < _MIN_AREA_REC){
            //se estiver muito ruim...
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
}


void Robot::climbStairs(){
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  float wheelFrontSpeed;
  float wheelRearSpeed;
  static bool _climbing = false;

  if(abs(_yAngle) < OKAY){

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

  }else if(abs(_yAngle) > REAR_WHEELS){

    cout << " REAR WHEELS IS ON\n";
    wheelRearSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = _MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else if(abs(_yAngle) < FRONT_WHEELS){
    
    cout << " FRONT WHEELS IS ON\n";
    wheelRearSpeed = 0.5f * _MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = -1.0f * _MAX_WHEEL_R_SPEED;
    _climbing = true;

  }else{

    cout << " ESTABILIZING\n";
    wheelRearSpeed = (_yAngle > 0)? _MAX_WHEEL_R_SPEED : -_MAX_WHEEL_R_SPEED;
    wheelFrontSpeed = (_yAngle > 0)? -_MAX_WHEEL_R_SPEED : _MAX_WHEEL_R_SPEED;

  }

  for(int i = 0; i < 4; i++){
    msg.data.push_back((i == 0 || i == 2) ? wheelFrontSpeed : wheelRearSpeed);
  }

  wheelPub.publish(msg);

}

void Robot::rodarFunction(SidesInfo* sidesInfo){
  float dif;

  std_msgs::Float32MultiArray msg;
  msg.data.clear();

  if(_zAngle < 0)
    _zAngle += M_PI*2;
    
  if(_saveAngle == 10)
    _saveAngle = _zAngle;

  if(sidesInfo[_FRONT].medY < _MIN_SAFE_DIST_SPIN){
    msg.data.push_back(-3);
    msg.data.push_back(-3);
    msg.data.push_back(-3);
    msg.data.push_back(-3);
  
  }else if(_sentido == _HORARIO){
    msg.data.push_back(-3);
    msg.data.push_back(-3);
    msg.data.push_back(0);
    msg.data.push_back(0);

    if(_saveAngle >= 0 && _saveAngle < M_PI && _zAngle > M_PI)
        dif = _saveAngle + M_PI*2 - _zAngle;

    else
      dif = _saveAngle - _zAngle;
  
  }else{
    msg.data.push_back(0);
    msg.data.push_back(0);
    msg.data.push_back(-3);
    msg.data.push_back(-3);

    if(_zAngle >= 0 && _zAngle < M_PI && _saveAngle > M_PI)
        dif =  M_PI*2 - _saveAngle + _zAngle;
      
    else
      dif = _saveAngle - _zAngle;
  }

  if(dif < 0)
    dif *=-1;

  if(dif > 3){
    _rodar = false; //para de rodar
    _sentido = !_sentido; //troca o _sentido
    _saveAngle = 10; //da um reset no angulo
  } 

  speedPub.publish(msg);
  
}

void Robot::setAngles(double y, double z){
    _yAngle = y;
    _zAngle = z;
}

bool Robot::getRodar(){
    return _rodar;
}

void Robot::setPublishers(std_msgs::Int32 enable, ros::Publisher speedPub, ros::Publisher wheelPub, ros::Publisher statePub){
    _enable.data = enable.data;
    this->speedPub = speedPub;
    this->wheelPub = wheelPub;
    this->statePub = statePub;
}

bool Robot::getAvoidingObs(){
  return _avoidingObs;
}

bool Robot::getProvavelEscada(){
  return _provavelEscada;
}

bool Robot::getIsInStairs(){
  return _isInStairs;
}