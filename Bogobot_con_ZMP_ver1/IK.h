
#include "poses.h"
double sign (double a) {
  if (a>=0) {return 1.0;}
  return -1.0;
}

double atan2_(double y, double x){    // Como la función atan2 no está bien implementada, implementé la mia.
  if (x>0) {return atan(y/x);}
  if (x<0) {return atan(y/x)+ sign(y)*PI;}
  if(x==0 && y != 0) {return sign(y)*PI/2;} 
  return 0;
}

float interpolation(float t ,float tf, float q0, float qf){
  float a3,a4,a5,a0,q;
  q = (qf - q0)*(10 - 15*(t/tf) + 6*((t/tf)*(t/tf)))*((t/tf)*(t/tf)*(t/tf))+q0;
  //Serial.println(q);
  //Serial.println(q);
  return q;
  
}


void IK_Feet(double Pose_Feet[], double q[]) {   // Dato de entrada:  Pose_Feet[8] = {Pxi, Pyi, Pzi, alfai,  Pxd, Pyd, Pzd, alfad}
  static double Pxg, Pyg, Pzg;                   // Resultados:   q[19] = {q[0] = interpolating time,   q[1] to q[12] = piernas,  q[13] to q[18] = brazos}
  static double c3, s6, c6, c9, s12, c12;
  static double L = 18.5;

  // IK de pata IZQUIERDA
  q[6] = Pose_Feet[3];  s6 = sin(q[6]); c6 = cos(q[6]);
  Pxg = Pose_Feet[0] * c6 - Pose_Feet[1] * s6;
  Pyg = Pose_Feet[0] * s6 + Pose_Feet[1] * c6;
  Pzg = Pose_Feet[2];
  c3 = (Pxg*Pxg + Pyg*Pyg + Pzg*Pzg)/(2.0*L*L) - 1.0;
  if (c3*c3 > 1) {c3 = 1;}
  q[3] = acos(c3);
  q[5] = atan2_(-Pxg, -Pzg);
  q[4] = atan2_( Pzg * cos(q[5]) + Pxg * sin(q[5]), -Pyg) - atan2_(-1.0 - c3, sin(q[3])); 
  q[4] = atan2_(sin(q[4]),cos(q[4]));
  q[2] = q[3] + q[4]; 
  q[2] = atan2_(sin(q[2]), cos(q[2]));
  q[1] = q[5];
  
  // IK de pata DERECHA
  q[12] = Pose_Feet[7];  s12 = sin(q[12]); c12 = cos(q[12]);
  Pxg = Pose_Feet[4] * c12 - Pose_Feet[5] * s12;
  Pyg = Pose_Feet[4] * s12 + Pose_Feet[5] * c12;
  Pzg = Pose_Feet[6];
  c9 = (Pxg*Pxg + Pyg*Pyg + Pzg*Pzg)/(2.0*L*L) - 1.0;
  if (c9*c9 > 1) {c9 = 1;}
  q[9] = -acos(c9);
  q[11] = atan2_(-Pxg, -Pzg);
  q[10] = atan2_( Pyg, -Pzg * cos(q[11]) - Pxg * sin(q[11])) - atan2_(sin(q[9]), 1.0 + c9); 
  q[10] = atan2_(sin(q[10]),cos(q[10]));
  q[8] = q[9] + q[10]; 
  q[8] = atan2_(sin(q[8]), cos(q[8]));
  q[7] = q[11];
  float factor = 180.0/(0.0879 * PI);
  for (int i=1; i <= 18; ++i) { q[i] = q[i]*factor + StandUp[i]; }

}

void IK_arms(double Pose_Arm[], double q[]) {   // Dato de entrada:  Pose_Feet[8] = {Pxi, Pyi, Pzi, alfai,  Pxd, Pyd, Pzd, alfad}
  static double Pxg, Pyg, Pzg;                   // Resultados:   q[19] = {q[0] = interpolating time,   q[1] to q[12] = piernas,  q[13] to q[18] = brazos}
  double c15,c18,s15,s18;
  static double L1 = 20, L2 = 19;
  // IK brazo izq
  Pxg = Pose_Arm[8];
  Pyg = Pose_Arm[9];
  Pzg = Pose_Arm[10];
  c15 = ((Pxg*Pxg + Pyg*Pyg + Pzg*Pzg - L1*L1 - L2*L2)/(2*L1*L2));
  if (c15*c15 > 1) {c15 = 1;}
  q[15] =  -acos(c15);
  q[14] = atan2_(Pxg,sqrt(Pyg*Pyg + Pzg*Pzg)) - atan2_(-L2*sin(q[15]),L1+L2*cos(q[15]));
  q[13] = atan2_(-Pyg,-Pzg);

  // IK brazo der
  Pxg = Pose_Arm[11];
  Pyg = Pose_Arm[12];
  Pzg = Pose_Arm[13];
  c18 = ((Pxg*Pxg + Pyg*Pyg + Pzg*Pzg - L1*L1 - L2*L2)/(2*L1*L2));
  if (c18*c18 > 1) {c18 = 1;}
  q[18] =  acos(c18);
  q[17] = atan2_(Pxg,sqrt(Pyg*Pyg + Pzg*Pzg)) - atan2_(-L2*sin(q[18]),L1+L2*cos(q[18]));
  q[17] = atan2_(sin(q[17]),cos(q[17]));
  q[16] = atan2_(Pyg,-Pzg);

  float factor = 180.0/(0.0879 * PI);
  for (int i=1; i <= 18; ++i) { q[i] = q[i]*factor + StandUp[i]; }
}
