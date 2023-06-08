//Leonardo Gracida Munoz A01379812
//Daniel Fuentes Castro A01750425
//Santiago Ortiz Suzarte A01750402

//caminata con ZMP en eje x (version inicial, aun faltan mejoras)
#include "IK.h"
#include "functions.h"
#include <ros.h> //header file para poder usar ROS
#include <std_msgs/Int16.h> //tipo de mensaje que habilita movimiento del robot

ros::NodeHandle  nh; //node handler 
bool move_robot_forward = false; //estado en el que el robot camina hacia adelante (no se utiliza en version actual)
bool move_robot_reverse = false; //estado en el que el robot camina en reversa (aun no se implementa)
bool stop_robot= false; //estado en el que el robot se detiene
int estado = 0; //indica en que pose estamos (iniciamos en pose 0)

unsigned long previousMillis = 0;
const long interval = 500; //cada 500 milisegundos

void cmd_vel_cb( const std_msgs::Int16& move_msg){ //callback que permite que robot camine o se detenga de acuerdo a mensaje que reciba
  if (move_msg.data > 0){ //entero mayor a 0 hace que robot se mueva
    estado = 1; //estado de caminata inicial
    move_robot_forward = true;
    stop_robot = false; //condicion de robot detenido en false
    move_robot_reverse = false;
      
  } else if (move_msg.data < 0){
    move_robot_forward = false;
    stop_robot = false;
    move_robot_reverse = true;
    
  } else if (move_msg.data == 0) { //si se recibe un 0 el robot se tiene que detener
    stop_robot = true; //indica que robot se tiene que detener
    move_robot_forward = false;
    move_robot_reverse = false; 
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

//poses de stop 
//de stopPose2 a stopPose6 son poses que ayudan al robot a realizar transicion a pose de stopPose (robot erguido) 
               //pierna: derecha  izquierda  
                      //x y  z    x y  z
double stopPose[14] =  {0,0,-30,0,0,0,-30,0,5,15,-30,-5,15,-30};
double stopPose2[14] = {0,2.5,-30,0,0,-2.5,-25,0,5,15,-30,-5,15,-30};
double stopPose3[14] = {0,10,-30,0,0,0,-25,0,5,15,-30,-5,15,-30};
double stopPose4[14] = {0,2.5,-30,0,0,-2.5,-25,0,5,15,-30,-5,15,-30};
double stopPose5[14] = {0,0,-25,0,0,5,-30,0,5,15,-30,-5,15,-30};
double stopPose6[14] = {0,0,-25,0,0,5,-30,0,5,15,-30,-5,15,-30};


//////////caminata ver 3
//arreglo que se encarga de almacenar pose actual en la que se encuentra el robot
double poseCopy[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//poses que inician movimiento de caminata
//medio paso (objetivo: mover pie izquierdo a y=10)
                          //derecha  izquierda
                          //x  y   z    x y  z
double forwardPose21[14] = {0,2.5,-25,0,0,-2.5,-30,0,5,15,-30,-5,15,-30}; //inicio medio paso izquierdo
double forwardPose22[14] = {0,5,-30,0,0,-5,-30,0,5,15,-30,-5,15,-30}; //final medio paso izquierdo
//paso derecho completo
double forwardPose23[14] = {0,5,-30,0,0,-5,-25,0,5,15,-30,-5,15,-30};  //inicia paso completo derecho levanta pie derecho
double forwardPose24[14] = {0,0,-30,0,0,5,-25,0,5,15,-30,-5,15,-30};  //levanta pie derecho y lo mueve hacia en frente
double forwardPose25[14] = {0,-5,-30,0,0,15,-30,0,5,15,-30,-5,15,-30};  //planta pie derecho 
//paso izquierdo completo 
double forwardPose26[14] = {0,-5,-25,0,0,15,-30,0,5,15,-30,-5,15,-30};  //inicia paso completo izquierdo, levanta pie izquierdo
double forwardPose27[14] = {0,5,-25,0,0,5,-30,0,5,15,-30,-5,15,-30};  //levanta pie izquierdo y lo mueve hacia en frente
double forwardPose28[14] = {0,15,-30,0,0,-5,-30,0,5,15,-30,-5,15,-30};  //planta pie izquierdo




double Vel[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double actualPose[18];
double dxl_goal_position[20];         // Goal position
double PresentPose[19];
double tmpPose[19]; //lee y guarda el valor actual de cada motor (en bits) despues de realizar movimiento

ros::Subscriber<std_msgs::Int16> sub("cmd_vel", cmd_vel_cb); //suscribirse a topico cmd_vel donde se reciben mensajes que indican si el robot debe avanzar o detenerse


void setup() { //configuracion inicial del robot
  //iniciar nodo
  nh.initNode(); //inciar nodo de rosserial 
  nh.subscribe(sub); //suscribirse a topico cmd_vel
  //while(!Serial);
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME); //crear port handler de dynamixel
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t param_goal_position[2];
  uint16_t dxl1_present_position = 0, dxl2_present_position = 0;              // Present position
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  /////////////////////////////////
  bool activo;
  readPoseAll(PresentPose); //adquiere pose actual del robot
  activo = movRobot(PresentPose,stopPose,dxl_goal_position,0.1, tmpPose);  //erguir robot
  copyArray(PresentPose, tmpPose); //actualizar pose, determinar valor de cada motor
  /////////////////////////////////
}
//int estado = 1; //indica en que pose estamos
int prev_state = 0; //Indica estado anterior
bool activo; // define si se realizo adecuadamente el movimiento del robot
float tf = 0.1; //tiempo de interpolacion


int increment = 3; //incremento, mientras mas grande el movimiento del robot es mas lento, define cuantas poses intermedias se necesitan para llegar a una pose objetivo
//definir pose destino
double goal_y_l = 0; //posicion a la que se desea que llegue pie izquierdo en y 
double goal_z_l = 0;//posicion a la que se desea que llegue pie izquierdo en z 
double goal_y_r = 0; //posicion a la que se desea que llegue pie izquierdo en y 
double goal_z_r = 0; //posicion a la que se desea que llegue pie izquierdo en z 


double step_y_l = 0; //obtenemos el paso, define que tanto avanzar en y para llegar a posicion objetivo
double step_z_l = 0; //obtenems el paso, define que tanto avanzar en z para llegar a posicion objetivo
double step_y_r = 0; //obtenemos el paso
double step_z_r = 0; //obtenemos el paso
int contador = 0;


void loop() {
  // put your main code here, to run repeatedly:
  //copyArray(PresentPose, tmpPose);
  //copyArray(PresentPose, tmpPose); //obtener en que configuracion estan los motores
  unsigned long currentMillis = millis();
  
  
  if (estado == 0 && !stop_robot){
    readPoseAll(PresentPose); //lee la pose actual del robot
    estado = 1; //transicion a estado 1 (robot da medio paso izquierdo)
  }


  //if (stop_robot) {
    //estado = stopRobot2(estado);
  //}

  
  
  if (estado == 1){
    //copyPoseArray(poseCopy, stopPose);
    readPoseAll(PresentPose);
    copyPoseArray(poseCopy, stopPose);
    moveRobot3(currentMillis, 5, poseCopy, 2.5, -30, 2.5, 5, true, 0,1,0);
    estado = 2;
    
  } else if (estado == 2){

     if (prev_state = 3){
      copyPoseArray(poseCopy,forwardPose28);
      
    } else {
      copyPoseArray(poseCopy, forwardPose22); //actualiza pose actual del robot (robot se encuentra detenido)
      
    }
    //copyPoseArray(poseCopy, forwardPose22);
    moveRobot3(currentMillis, 5, poseCopy, 5, -30, 10, 5, false, 0,0,1);
    estado = 3;  
    
  } else if (estado == 3){
    //copyPoseArray(poseCopy, forwardPose25);
    copyPoseArray(poseCopy,forwardPose25); //actualiza pose actual del robot (robot se encuentra detenido)
    //moveRobot2(poseCopy, 5, -30, 10, 5, true, -5);
    moveRobot3(currentMillis, 5, poseCopy, 5, -30, 10, 5, true, 0,0,1);
    estado = 2;
    prev_state = 3;
    
  }
  
  nh.spinOnce();

  
  
    //activo = movRobot(tmpPose,stopPose,dxl_goal_position,0.1, tmpPose);  //movRobot(tmpPose,stopPose,dxl_goal_position,0.1, tmpPose);  tmpPose se envia dos veces para ahorrar tiempo de procesamieto
}


void copyArray(double arr1[], double arr2[]){ //copia el valor de un array, no se usa porque ya se hace desde que se invoca la funcion movRobot
  for (int i = 0; i < 19;i++){
    arr1[i] = arr2[i];  
  }
}

void copyPoseArray(double arr1[], double arr2[]){
  for (int i = 0; i < 14; i++){
    arr1[i] = arr2[i];  
  }  
  
}

double getyGoal(double ypos, double goal){ //obtienen distancia a pose deseada en eje y
  if (goal < 0 ){
    return (goal + -1*ypos);  
  } else if ( goal >= 0){
    return (goal - ypos);
      
  }
}


double getzGoal(double zpos, double goal){ //obtiene distancia a pose deseada en eje z
  if (goal < 0) {
    return (goal + -1*zpos);  
  }  else if (goal >= 0){
    return (goal - zpos);  
  }
}

//primero determinar pose actual en base a switch case, después usar for loop para regresar a pose de stop





int stopRobot2(int estado){
  switch (estado){ //de acuerdo del estado se determina en que pose se encuentra el robot
    case 1:
      copyPoseArray(poseCopy, stopPose);
      break;
    case 2:
      if (prev_state = 3){
      copyPoseArray(poseCopy,forwardPose28);
      moveRobot2(poseCopy, -2.5, -30, 2.5, 5, false, 0);
      copyPoseArray(poseCopy,stopPose);
      prev_state = 0;
    } else {
      copyPoseArray(poseCopy, forwardPose22); //actualiza pose actual del robot (robot se encuentra detenido)
      moveRobot2(poseCopy, -2.5, -30, 2.5, 5, false, 0);
      copyPoseArray(poseCopy,stopPose);     
    }

      break; 
   case 3:
        copyPoseArray(poseCopy, forwardPose25);
        moveRobot2(poseCopy, -2.5, -30, 2.5, 5, true, 0);
        copyPoseArray(poseCopy,stopPose); 
        break;
  }
  return 20;
}


void moveRobot(double pose[], double target[]){
    goal_y_l = getyGoal(pose[1],target[1]);
    goal_z_l = getzGoal(pose[2],target[2]);
    goal_y_r = getyGoal(pose[5],target[5]);
    goal_z_r = getzGoal(pose[6],target[6]);
    step_y_l = goal_y_l/increment; //obtenemos el paso
    step_z_l = goal_z_l/increment; //obtenems el 
    step_y_r = goal_y_r/increment; //obtenemos el paso
    step_z_r = goal_z_r/increment; //obtenems el paso
    for (int i =0;i<increment;i++){
      poseCopy[1] += step_y_l;
      poseCopy[2] += step_z_l;
      poseCopy[5] += step_y_r;
      poseCopy[6] += step_z_r;
      activo = movRobot(tmpPose,poseCopy,dxl_goal_position,0.1, tmpPose);
      
    }
  
    
}


void moveRobot2(double pose[], double Cy, double Cz, double ry, double rz, bool left, double target_y){
    double increment = 0.523599; //30 grados de incremento//0.0174533; //incremento en radianes (1 grado de incremento) es muy lento
    double forwardGoal = 0;
    if (left){
        forwardGoal = getyGoal(pose[5],target_y); //determina distancia para llegar a pose objetivo
    } else {
        forwardGoal = getyGoal(pose[1],target_y);  //distancia para llegar a objetivo
    }

    double forwardStep = forwardGoal/6; //incremento para ir de posicion inicial en y hasta posicion final en intervalo de 180 grados con un 30 grados de paso (6 incrementos) 
    //trayectoria tiene forma de media circunferencia, se inicia en 0 y va hasta pi
    const double Pi = 3.14159; //constante de PI
    double angle = 0; //angulo inicial 
    double y = Cy + ry*cos(Pi - angle);
    double z = Cz + rz*sin(Pi - angle);
    
    while (angle <= Pi){
      if (left) { //trayectoria para pierna izquierda 
        poseCopy[1] = Cy + ry*cos(Pi - angle); //nueva posicion en y de pie izquierdo
        poseCopy[2] = Cz + rz*sin(Pi - angle); //nueva posicion en z de pie izquierdo
        poseCopy[5] += forwardStep; //nueva posicion en y de pie derecho (pie de soporte)
      } else { //trayectoria para pierna derecha
        poseCopy[5] = Cy + ry*cos(Pi - angle); //nueva posicion en y de pie derecho
        poseCopy[6] = Cz + rz*sin(Pi - angle); //nueva posicion en z de pie derecho
        poseCopy[1] += forwardStep; //nueva posicion en y de pie izquiero (pie de soporte)
      }
      activo = movRobot(tmpPose,poseCopy,dxl_goal_position,0.1, tmpPose); //mover piernas a pose deseada
      angle += increment; //incrementar angulo 
        
    }

    //obtenemos la ultima pose (cuando el robot planta el pie) 
    if (left) { //trayectoria para pierna izquierda 
        poseCopy[1] = Cy + ry*cos(Pi - Pi);
        poseCopy[2] = Cz + rz*sin(Pi - Pi);
        
      } else { //trayectoria para pierna derecha
        poseCopy[5] = Cy + ry*cos(Pi - Pi);
        poseCopy[6] = Cz + rz*sin(Pi - Pi);
        
      }

      activo = movRobot(tmpPose,poseCopy,dxl_goal_position,0.1, tmpPose);
  
}


void moveRobot3(unsigned long currentMillis, double T, double pose[], double Cy, double Cz, double ry, double rz, bool left, double target_y, double Op ,double Cl){
  double counter = 0; //determina cuanto tiempo ha transcurrido (ayuda a determinar si ya se cumplio con el tiempo T)
  double Xzmp = 8; //ubicacion del zero moment point en x
  double Yzmp = 5;  //ubicacion Zero moment point en y
  double r = sqrt(9.81/0.32); 
  double x = 0; // posicion de cadera del robot en x
  double e = 2.7183; //numero e (aproximacion de su valor) 
  double temp1 = pow(e,r*T); //esta variable es de utilidad para los calculos
  double k1 = -1*Xzmp/(1+ temp1);  //constante 1
  double k2 = k1*temp1; //constante 2

  const double Pi = 3.14159; //constante PI
  double increment = Pi/10;//(T/(200/1000)); //incremento angular para trayectoria de pierna que se despega del piso (intervalos de 0.2 segundos)
  double angle = 0; //angulo que recorre circunferencia que describe poses para pierna que se levanta
  //double y = Cy +ry*cos(Pi - angle); 
  //double z = Cz + rz*sin(Pi - angle);

  //banderas para determinar desplazamiento de cadera en y
  //double Op = 0;
  //double Cl = 0;

  //double M1 = -Yzmp*(Cl + Op*temp1)/(1 - temp1)*(1 + temp1);
  //double M2 = Yzmp*((Op + Cl*temp1)*temp1)/((1-temp1)*(1 + temp1));
    
   //sqrt((9.81/30)*counter));
  
  
  while (counter <= T){
    unsigned long currentMillis = millis();
    //if (currentMillis - previousMillis >= interval) {
    if (true){
      previousMillis = currentMillis; // guardar instante en el que se realizo ultimo calculo de poses 
      x = k1*pow(e,counter*r) + k2*pow(e,counter*r*-1) + Xzmp;
      //y = M1*pow(e,counter*r) + M2*pow(e,counter*r*-1); //ZMP en eje y
      if (left) { //pierna izquierda permanece fija, derecha se despega del suelo 
        poseCopy[0] = 1*x; // alinear centro de masa del robot con ZMP en eje x de pie de soporte, en sentido contrario para mover cadera de robot en direcion deseada, se controla posicion en x de pierna izquierda
        //poseCopy[1] = -1*y;
        poseCopy[4] = 1*x; 
        poseCopy[5] = Cy + ry*cos(Pi - angle); //mover efector final de pierna derecha en y
        poseCopy[6] = Cz + rz*sin(Pi - angle); //mover efector final de pierna derecha en z
      } else { //pie derecho fijo
        poseCopy[4] = -1*x; //alinear centro de masa del robot con ZMP en eje x de pie de soporte
        //poseCopy[5] = 1*y;
        poseCopy[1] = Cy + ry*cos(Pi - angle); //mover efector final de pierna derecha en y
        poseCopy[2] = Cz + rz*sin(Pi - angle); //mover efector final de pierna derecha en z
        poseCopy[0] = -1*x;
      }
      activo = movRobot(tmpPose,poseCopy,dxl_goal_position,0.4, tmpPose); //articulaciones para alcanzar la pose que se acaba de calcular
      angle += increment; //incrementar angulo que recorre media circunferencia para generar poses de pie que esta en movimiento
      counter += 0.5; //incrementar contador
  }
}
  x = k1*pow(e,counter*r) + k2*pow(e,counter*r*-1) + Xzmp;
  if (left) {
    poseCopy[0] = 0; //en sentido contrario para mover cadera de robot en direcion deseada, se controla posicion en x de pierna izquierda
    poseCopy[5] = Cy + ry*cos(Pi - Pi); //mover efector final de pierna derecha en y
    poseCopy[6] = Cz + rz*sin(Pi - Pi); //mover efector final de pierna derecha en z
    poseCopy[4] = 0;
  } else {
    poseCopy[4] = 0;
    //poseCopy[5] = 0; 
    poseCopy[1] = Cy + ry*cos(Pi - angle); //mover efector final de pierna derecha en y
    poseCopy[2] = Cz + rz*sin(Pi - angle); //mover efector final de pierna derecha en z
    poseCopy[0] = 0;
  }

  activo = movRobot(tmpPose,poseCopy,dxl_goal_position,0.4, tmpPose);
}
