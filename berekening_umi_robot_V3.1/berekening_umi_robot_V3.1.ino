#include<stdio.h>
//spoel rotatie en tilt
//links en rechts elleboog
///////////////////
//maximum rage or motors: NUMBERS NEED TO BE CHECKED AND IMPLEMENTED!
#define SHOULDER_MAX_POS      5260    
#define ELBOW_MAX_POS         4836
#define ZED_MAX_POS           2767   
#define WRIST_PITCH_MAX_POS   2142
#define WRIST_ROLL_MAX_POS    8442
#define YAW_MAX_POS           2142
#define GRIPPER_MAX       
#define DEBUG
#define ELBOW_INITPOS         1103  // te berekenen door 180/0.06844/2 DIT - 2418

int CAP_R =7;
int CAP_L =8;

float L1 = 252.5;
float L2 = 252.5;
float L3 = 253;
int L_arm = 758;
float rad;
float deg;

//functie voor graden naar radialen
float degtorad(float deg){
  float rad= deg*(PI/180); //0.017453292519943295769236907684886;
  return rad;
} 

//functie voor radialen naar graden
float radtodeg(float rad){
  float deg= rad*(180/PI);//57.295779513082320876798154814105;
  return deg;
}
//**************************************************************************
// mega functie die bijna alle hoeken en encoder count uitrekend.
// De zed word hier niet uit gerekend en en word ook nog geen rekening gehouden met huidige posities
//**************************************************************************
void *berekening(float *phi,int *rot,int *x,int *z, int *y,int *encoderS, int *encoderE,int *encoderY,int *encoderW1,int *encoderW2,int *encoderZ){
  float L12;
  float xcor;
  float zcor;
  float xpols;
  float zpols;
  float c2;
  float Theta1; //dit is de hoek van de schouder bereik 0-180graden
  float Theta2; //dit is de hoek van de elleboog bereik 0-331graden
  float Theta10;
  float Theta11;
  float Theta3; //dit is de hoek van de pols bereik 70-290graden
  float z1;
  float x1;
  xcor = L3*cos(degtorad(*phi));//berekend x verplaatsing vanaf cordinaat
  zcor = L3*sin(degtorad(*phi));//berekend z verplaatsing vanaf cordinaat
  xpols = *x - xcor;                         //X-positie wrist motor
  zpols = *z - zcor;                         //Z-positie wrist motor
  c2 = (sq(abs(xpols)) + sq(abs(zpols))); 
  L12 = sqrt(c2);                            //lengte tussen oorsprong en pols cordinaten
  
//***********************************************************************************************************
// hier worden de hoeken van de motoren berekend links van het midden dus met de elleboog naar links
//***********************************************************************************************************
 if (xpols<0){
    int lastTheta1= Theta1; //onthoud vorige theta
    int lastTheta2= Theta2; //onthoud vorige theta
    int lastTheta3= Theta3; //onthoud vorige theta
    Theta10= radtodeg(asin(abs(xpols/L12))); //hoek tussen x-as en loodlijn tussen schouder en pols
    Theta11= radtodeg(acos((L12/2)/L1));// hoek tussen loodlijn en schouder segment
    Theta1= Theta10+Theta11; //Totaal Theta 1
    Theta2= 2*radtodeg(asin((L12/2)/L1));      //hoek van elleboog
    Theta3 = 360-(Theta1 + Theta2) + *phi; //Bij het uit tekenen van de armen tot deze formule gekomen
    /*
      // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta1<0 || Theta1>180){
      Serial.print("THETA1 out of range");
      Serial.println(Theta1);
      *encoderS = round((Theta1/0.03422)/2);
      if (Theta1<0)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
      if (Theta1>180)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
    } 
      // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta2<0 || Theta2>331){
      Serial.print("THETA2 out of range");
      Serial.println(Theta2);
      if (Theta2<0)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
      if (Theta2>331)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
    }
    // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta3<70 || Theta3>290){
      Serial.print("THETA3 out of range");
      Serial.println(Theta3);
      if (Theta3<70)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
      if (Theta3>290)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
    }   
  }*/
//***********************************************************************************************************
// hier worden de hoeken van de motoren berekend voor de rechterkant van de arm, met de elleboog naar rechts.
//***********************************************************************************************************
  else if (xpols>=0){ 
    int lastTheta1= Theta1; //onthoud vorige theta
    int lastTheta2= Theta2; //onthoud vorige theta
    int lastTheta3= Theta3; //onthoud vorige theta
    Theta10= radtodeg(asin(zpols/L12));// hoek tussen x-as en loodlijn
    Theta11= radtodeg(acos((L12/2)/L1)); // hoek tussen schouder seg en loodlijn
    Theta1= abs(Theta10) -abs(Theta11); // hoek tussen x-as en schouder seg
    Theta2 = 360-(180-(2*Theta11)); //Bij het uit tekenen van de armen tot deze formule gekomen
    Theta3 = 180-(2*Theta11 +Theta1)+*phi;//Bij het uit tekenen van de armen tot deze formule gekomen
    // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta1<0 || Theta1>180){
      Serial.print("THETA1 out of range");
      Serial.println(Theta1);
      *encoderS = round((Theta1/0.03422)/2);
      if (Theta1<0)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
      if (Theta1>180)Theta1=lastTheta1;return; //zet oude waarde terug en springt uit de functie
    }
      // volgende if statement zorgt dat het berijk van het gewricht niet word overschreden maar zet m tegen de maximale waarde
    if (Theta2<0 || Theta2>331){
      Serial.print("THETA2 out of range");
      Serial.println(Theta2);
      if (Theta2<0)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
      if (Theta2>331)Theta2=lastTheta2;return; //zet oude waarde terug en springt uit de functie
    }
    if (Theta3<70 || Theta3>290){
      Serial.print("THETA3 out of range");
      Serial.println(Theta3);
      if (Theta3<70)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
      if (Theta3>290)Theta3=lastTheta3;return; //zet oude waarde terug en springt uit de functie
    }   
  }
  
  float yc = *y;// berekening van Zed eerst even data omzetten, c staat voor conversion
  *encoderZ = round((yc/0.2667)/2);//encoder van Zed uitrekenen
  *encoderW1= coil_rot(*rot);//functie die de encoders van pols rotatie uitrekenen.
  *encoderW2= -1*(*encoderW1);
  if(L12<=505){  
    *encoderS = round((Theta1/0.03422)/2);
    *encoderE = round((Theta2/0.06844)/2);
    *encoderY = round((Theta3/0.10267)/2);
    #ifdef DEBUG
    Serial.print("L12:");Serial.println(L12);
    Serial.print("zpols:");Serial.println(zpols);
    Serial.print("xpols:");Serial.println(xpols);
    Serial.print("theta10:");Serial.println(Theta10);
    Serial.print("theta11:");Serial.println(Theta11);
    Serial.print("theta1:");Serial.println(Theta1);
    Serial.print("theta2:");Serial.println(Theta2);
    Serial.print("theta3:");Serial.println(Theta3);
    #endif
 
    Serial.print("Shouder:");
    Serial.println(*encoderS);
    Serial.print("Elleboog");
    Serial.println(*encoderE);
    Serial.print("Pols: ");
    Serial.println(*encoderY);
    Serial.print("ZED: ");
    Serial.println(*encoderZ);
  }
  else Serial.println("coordinaten buiten bereik");return;
}
//*******************************************************************************
//berekening voor de encoder counts van spoel rotatie deze bestaat uit 2motoren.
// rotation(deg)=(w1-w2*0.07415)/2
//eerst moet er geinitialiseerd worden naar 1kant toe de enekant op 181graden draaien andere kant 132 totaal 313 graden vrijheid.
//w1-w2 =4882 to -3560
//bij de som van 4882 is w1=2441 en w2=-2441
//********************************************************************************
int coil_rot(int rot){
  int encoder;
  encoder= ((2*rot)/0.07415)/2;
  return encoder;
}
int retrieve_x(){
  Serial.print("voer X coordinaat in tussen de -600 en 600:");
  while(Serial.available()==0);
  int x= Serial.parseInt();
  Serial.println(x);
  while (x<-600 || x>600){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer X coordinaat in tussen de -600 en 600:");
    while(Serial.available()==0);
    x= Serial.parseInt();
    Serial.println(x);
  }
  return x;  
}

int retrieve_z(){
  Serial.print("voer Z coordinaat in tussen de 0 en 700:");
  while(Serial.available()==0);
  int z= Serial.parseInt();
  Serial.println(z);
  while (z<0 || z>700){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer Z coordinaat in tussen de 0 en 700:");
    while(Serial.available()==0);
    z= Serial.parseInt();
    Serial.println(z);
  }  
  return z;
}

int retrieve_y(){
  Serial.print("voer y coordinaat in tussen de 0 en 915:");
  while(Serial.available()==0);
  int y= Serial.parseInt();
  Serial.println(y);
  while (y<0 || y>915){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer y coordinaat in tussen de 0 en 915:");
    while(Serial.available()==0);
    y= Serial.parseInt();
    Serial.println(y);
  }  
  return y;
}
float retrieve_phi(){
  Serial.print("voer phi in tussen 0 en 180 graden:");
  while(Serial.available()==0);
  float phi= Serial.parseInt();
  Serial.println(phi);
  while (phi<0 || phi>180){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer phi in tussen 0 en 180 graden:");
    while(Serial.available()==0);
    phi= Serial.parseInt();
    Serial.println(phi);
  }  
  return phi;
}
int retrieve_rot(){
  Serial.print("voer rot graden in tussen de 181 en -132:");
  while(Serial.available()==0);
  int rot= Serial.parseInt();
  Serial.println(rot);
  while (rot<-132 || rot>181){
    Serial.println("buiten bereik pannekoek, LEZEN!");
    Serial.print("voer rot graden in tussen de -132 en 181:");
    while(Serial.available()==0);
    rot= Serial.parseInt();
    Serial.println(rot);
  }  
  return rot;
}


void coil_rot(boolean CAP_R,boolean CAP_L,float *rot){
  int var;
  if (*rot>4882)*rot=4882;
  if (*rot<-3560)*rot=-3560;
  if (CAP_R==LOW && CAP_L==HIGH)var=1;
  if (CAP_R==HIGH && CAP_L==LOW)var=2;
  if (CAP_R==LOW && CAP_L==LOW)var=3;
  if (CAP_R==HIGH && CAP_L==HIGH)var=0;
  switch (var){
    case 1:
      (*rot) = (2/0.07415)+*rot;
      Serial.println("draai links");
      break;
    case 2:
      *rot= *rot-(2/0.07415);
      Serial.println("draai rechts");
      break;
    case 3:
      Serial.println("rotatie goed, TMS-spoel aan!");
      break;
    default:
      Serial.println("ALARM: TMS-spoel UIT, te ver weg!");
      break;
  }
}

void encoder_max(int *room_S,int *room_E,int *room_Y,int *encoderS, int *encoderE,int *encoderY){
  if (*encoderS>2630)*encoderS=2630; 
  if(*encoderS<0)*encoderS=0;
  
  if (*encoderE>2418)*encoderS=2418; 
  if(*encoderE<0)*encoderS=0;
  
  if (*encoderY>1070)*encoderS=1070; 
  if(*encoderY<0)*encoderS=0;
  
  float theta1=(*encoderS)*2*0.03422;
  float theta2=(*encoderE)*2*0.06844;
  float theta3=(*encoderY)*2*0.10267;
  
  float XXX= L1*cos(degtorad(theta1))+L2*cos(degtorad(theta2)); //x afstand naar pols positie
  float arm_angle = acos(XXX/L_arm); //hoek van de arm om berijk te berekenen.
  float Z_range= L_arm*sin(arm_angle);//volgende 2 regels rekend het berijk van de arm als alle gewrichten recht staan behalve de shoulder
  float X_range= L_arm*cos(arm_angle);
  
  *room_S= 1315-*encoderS;
  *room_E= 1103-*encoderE;
  *room_Y= 535-*encoderY;
  Serial.print("shoulder:");Serial.println(*room_S);
  Serial.print("encoderS:");Serial.println(*encoderS);
  Serial.print("elbow   :");Serial.println(*room_E);
  Serial.print("encoderE:");Serial.println(*encoderE);
  Serial.print("Yaw     :");Serial.println(*room_Y);
  Serial.print("encoderY:");Serial.println(*encoderY);
}


void setup(){
  pinMode(CAP_R, INPUT);
  pinMode(CAP_L, INPUT);
  Serial.available();
  Serial.begin(250000);
  //Serial.println("Voor de X en Z as mag er alleen een integer worden ingevuld, phi is de hoek van de eerste arm");
}


void loop() {
static int room_S,room_E,room_Y,encoderS, encoderE,encoderY,encoderW1,encoderW2,encoderZ; //S=shoulder, E=elbow, W=wrist(rot=rotation),Z=zed(y-as)
static float rot;
boolean capr, capl;
capr=digitalRead(CAP_R);
capl=digitalRead(CAP_L);
coil_rot(capr,capl,&rot);
Serial.println(rot);
delay(100);







//int x   = retrieve_x();
//int z   = retrieve_z();
//int y   = retrieve_y();
//float phi = retrieve_phi();
//int rot = retrieve_rot();
////int x   = 200;int z   = 200;int y   = 0;float phi = 45;int rot = 180;
//berekening(&phi, &rot, &x, &z,&y, &encoderS, &encoderE,&encoderY,&encoderW1,&encoderW2,&encoderZ);


#ifdef encoder
encoderS = 500;
encoderE = 2000;
encoderY = 300;
encoder_max(&room_S,&room_E,&room_Y,&encoderS,&encoderE,&encoderY);
#endif
/*for (int i =0;i<100;i++){
int x   = 200+i;
int z   = 200+i;
int y   = 0;
float phi = 0+i;
int rot = -50+i;
berekening(&phi, &rot, &x, &z,&y, &encoderS, &encoderE,&encoderY,&encoderW1,&encoderW2,&encoderZ);
}
*/
}



