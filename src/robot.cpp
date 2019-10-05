#include "headers/robot.h"

using namespace std;

#define _KP_REC 0.6
#define NICE_DIST_TRACK 0.80
#define FRONT_WHEELS 0.15
#define REAR_WHEELS 0.35
#define OKAY 0.044

Robot::Robot(){
    _state = WALKING;
    sentido = _ANTI_HORARIO;
    _isInStairs = false;
    _provavelEscada = true;
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

  if(_HORARIO && sidesInfo[_FRONT_LEFT].medY < _MIN_DIST_FRONT && !(sidesInfo[_FRONT_RIGHT].medY < _MIN_DIST_FRONT)){

    erro = zAngle *_KP_OBSTACLE;
    straitPath = true;
    distToTrack = NICE_DIST_TRACK - 0.3;
    cout << "StraitPath\t";
  
  }

  if(_isInStairs && !(_state == LADDER_UP || _state == LADDER_DOWN)){
    _state = LADDER_UP;  
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
    
    if(abs(zAngle) < 0.1)
      inObs = true;

    cout << "E: DesviaFr";
    cout << " | AF: " << sidesInfo[_FRONT].area;
    cout << " | DFY: " << sidesInfo[_FRONT].medY;

  //Recupera o trajeto da direita
  }else if(inObs && sidesInfo[_RIGHT].medY < -0.35 && sentido == _HORARIO){
    
    erro = -1/(sidesInfo[_RIGHT].medX);

    cout << "E: RercuDir";
    cout << " | AD: " << sidesInfo[_RIGHT].area;
    cout << " | DDX: " << sidesInfo[_RIGHT].medX;

    avoidingObs = false;
    inObs = false;

  }else if(!inObs && abs(sidesInfo[_RIGHT].medX - distToTrack) > 0.15){

    erro = (distToTrack - sidesInfo[_RIGHT].medX) * _KP_REC;

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
    traction = (float) (stairsDir * (_MAX_SPEED + erro));
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 
    
    msg.data.push_back(traction);
    msg.data.push_back(traction);

    traction = (float) (stairsDir * (_MAX_SPEED - erro));
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 
    
    msg.data.push_back(traction);
    msg.data.push_back(traction);

  }else{
    traction = (float) (_V0 + _KP*erro);
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 

    msg.data.push_back(traction);
    msg.data.push_back(traction);

    traction = (float) (_V0 - _KP*erro);
    if(traction > _MAX_SPEED)
      traction = _MAX_SPEED;
    else if(traction < -_MAX_SPEED)
      traction = -_MAX_SPEED; 
    
    msg.data.push_back(traction);
    msg.data.push_back(traction);
  }

  //cout << " | zAngle: " << zAngle << " | VEsq: " << tractionEsq.joint_var << " | VDir: " << tractionDir.joint_var << endl;
  
  msg.data.clear();

  msg.data.push_back(0);
  msg.data.push_back(0);
  msg.data.push_back(0);
  msg.data.push_back(0);

  
  
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


  if(sentido == _ANTI_HORARIO){
    cout << "ANTI_HORARIO";

    if(!frontToTrack && !closeToTrack){
      
      cout << " | Endireitando" << " | zAngle: " << zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_LEFT].medX;
      
      if(sidesInfo[_FRONT].medY < 1.1){
        frontToTrack = true;
        return;
      
      }else if(sidesInfo[_LEFT].medX < _MAX_DIST_SIDE_ESCADA){
        closeToTrack = true;
        return;
      
      
      }else if((zAngle > M_PI*3/2 - 0.2 || zAngle < -M_PI/2 + 0.2) || sidesInfo[_LEFT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }else if(sidesInfo[_LEFT].area > _MIN_AREA_REC/2 && sidesInfo[_LEFT].medX > _MAX_DIST_SIDE_ESCADA){

        tractionDir = _V0;
        tractionEsq = _V0;
      

      }else if(zAngle > -M_PI/2 && zAngle < M_PI/2){
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }else{
          tractionDir = _V0;
          tractionEsq = -_V0;
          
      }

    }else if(frontToTrack && !closeToTrack){ //esta de frente mas precisa alinhar
      cout << "seguindo a esteira" << " | DX: " << sidesInfo[_LEFT].medX;

    
      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && zAngle < -0.2){
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
      
      cout <<" | Perto da esteira" << " | DX: " << sidesInfo[_LEFT].medX;

      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){
        cout << " | Alinhando com escada"; 
        if(zAngle > -_MAX_ERRO_ESCADA && zAngle < _MAX_ERRO_ESCADA){
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false;
        }

        tractionDir = -_KP*zAngle*2.5;
        tractionEsq = +_KP*zAngle*2.5;
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
          
          cout << " | Corrigindo zAngle" << " | zAngle: " << zAngle;
          cout << " | MedY: " << sidesInfo[_FRONT].medY;
          tractionDir = (_V0 + _KP*zAngle*1.5)/2;
          tractionEsq = (_V0 - _KP*zAngle*1.5)/2;
        }
      }
    }

  }else{ //sentido horario
    cout << "HORARIO";

    if(!frontToTrack && !closeToTrack){
      
      cout << " | Endireitando" << " | zAngle: " << zAngle << " | DFY: " << sidesInfo[_FRONT].medY << " | DX: " << sidesInfo[_RIGHT].medX;
      
      if(sidesInfo[_FRONT].medY < 1.1){
        frontToTrack = true;


      }else if((zAngle > M_PI/2 - 0.2 && zAngle < M_PI/2 + 0.2) || sidesInfo[_RIGHT].medX < _MIN_DIST_SIDE_ESCADA){ //se estiver de frente para esteira
        
        
        tractionDir = _V0;
        tractionEsq = _V0;

      }else if(sidesInfo[_RIGHT].area > _MIN_AREA_REC/2){

        tractionDir = _V0;
        tractionEsq = _V0;

      }else if(zAngle <= M_PI/2){
          tractionDir = -_V0;
          tractionEsq = _V0;
          
      }else{
          tractionDir = -_V0;
          tractionEsq = _V0;
  
          
      }

    }else if(frontToTrack && !closeToTrack){ //esta de frente mas precisa alinhar
      cout << " | seguindo a esteira" << " | DX: " << sidesInfo[_RIGHT].medX;

    
      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && zAngle > 0.2){
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
      
      cout <<" | Perto da esteira" << " | DX: " << sidesInfo[_RIGHT].medX;

      if(sidesInfo[_FRONT].area > _MIN_AREA_REC && sidesInfo[_FRONT].medY < _MIN_DIST_ESCADA){
        cout << " | Alinhando com escada"; 
        if(zAngle > -_MAX_ERRO_ESCADA && zAngle < _MAX_ERRO_ESCADA){
          cout << " | escada";
          _isInStairs = true;
          _provavelEscada = false;
        }

        tractionDir = +_KP*zAngle*2.5;
        tractionEsq = -_KP*zAngle*2.5;
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

    cout << " REAR WHEELS IS ON\n";
    wheelRearSpeed = -1.0f *_MAX_WHEEL_R_SPEED;
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
    wheelFrontSpeed = 0.0f;

  }

  for(int i = 0; i < 4; i++){
    // wheel.joint_var = (i == 0 || i == 2)? wheelFrontSpeed : wheelRearSpeed;
    // wheel. = i + 1;
    // wheelsList.movement_array.push_back(wheel); 
    msg.data.push_back((i == 0 || i == 2) ? -wheelFrontSpeed : wheelRearSpeed);
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

void Robot::setPublishers(ros::Publisher speedPub, ros::Publisher wheelPub){
    this->speedPub = speedPub;
    this->wheelPub = wheelPub;
}

bool Robot::getAvoidingObs(){
  return avoidingObs;
}

bool Robot::getProvavelEscada(){
  return _provavelEscada;
}