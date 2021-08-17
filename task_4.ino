// C++ code
//
#include <math.h>
#define pi 3.1415
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder knobLeft(18,19);
Encoder knobRight(3, 2);


void setup()
{
  Serial.begin(9600);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
}

long int positionLeft  = 0;
long int positionRight = 0;
long int newLeft, newRight;
long int valLeft, valRight;
    
float *wheels(float v, float w, float r, float b)             //angular velocity of left and right wheel
{
  static float wheel[2]={0,0};
  wheel[0]=(v - (b/2.0)*w)/r; //left
  wheel[1]=(v + (b/2.0)*w)/r; //right
  return wheel;
  
}
float* pose2(float nl, float nr, float r, float c)          //calculates real linear and angular velocities using the encoder values
{
  static float pose_2[3];
  float x=0,y=0;
  float dt=0.1;
  float vi=((2.0*pi*r)/c)*(nr+nl)/2.0*(1.0/dt);
  float wi=((2.0*pi*r)/c)*(nr-nl)/2.0*(1.0/dt);
  float Ti=atan2(sin(Ti+wi*dt),cos(Ti+wi*dt));
  
  float xi=x+vi*cos(Ti)*dt;
  float yi=y+vi*sin(Ti)*dt;

   pose_2[0]=vi;
   pose_2[1]=wi;
   pose_2[2]=Ti;
   
  return pose_2;
  
}
float *linearW(float v, float b, float r, float w)
{
  static float linear_w[2]={0,0};
  linear_w[0]=(v-(b/2.0)*w)/r;//left
  linear_w[1]=(v+(b/2.0)*w)/r;//right
  return linear_w;
  
  }
void encfunc()
{
  static int i=0;
  i++;
}

    static float prev_PE[2] = {0,0};
    static float prev_IE[2] = {0,0};

float* PID(float desired_w_l,float desired_w_r, float real_w_l,float real_w_r)
{
  
  static float G[2]={0,0};
  float KP_l=8.0, KI_l=0.0, KD_l=0.0;
  float KP_r=8.0, KI_r=0.0, KD_r=0.0;
  float period=100;
  
  
  float PE_l = desired_w_l-real_w_l;
  float PE_r = desired_w_r-real_w_r;
  
  
  float DE_l = PE_l - prev_PE[0];
  float DE_r = PE_r - prev_PE[1];
  
 
  float IE_l = prev_IE[0] + PE_l;
  float IE_r = prev_IE[1] + PE_r;
  

  G[0] = KP_l*PE_l + KI_l*IE_l*(0.1) + KD_l*DE_l/(0.1);
  G[1] = KP_r*PE_r + KI_r*IE_r*(0.1) + KD_r*DE_r/(0.1);

  
  prev_PE[0] = PE_l;
  prev_PE[1] = PE_r;
  prev_IE[0] = IE_l;
  prev_IE[1] = IE_r;

  return G;
}
  

unsigned long mytime=0;
void loop()
{

    float v=0.2;
    float w=0.5;

    float *wheel;
    float nl,nr,r=0.016,b=0.093,c=3500;
    float *pose;
    float *linear;
    float *G;

    if(millis()- mytime >=100)
    {
    mytime=millis();

    valLeft=  knobLeft.read();
    valRight= knobRight.read();  
    newLeft = valLeft-positionLeft;
    newRight = valRight-positionRight;
   
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
  
    positionLeft =  valLeft;
    positionRight =  valRight;
    nl=newLeft;
    nr=newRight;
      
      
        wheel = wheels(v, w,r,b);
        pose =  pose2(nl,nr,r,c);
        linear= linearW(pose[0],b,r,pose[1]);
        G=PID(wheel[0],wheel[1],linear[0],linear[1]);
       // G=PID(wheel[0],wheel[1],0,0);
        
     
    /*  Serial.print(wheel[0]);
      Serial.print(" ");
      Serial.println(wheel[1]);

     Serial.print(pose[0]);
      Serial.print(", ");
      Serial.print(pose[1]);
      Serial.print(", ");
      Serial.println(pose[2]);

      Serial.print(linear[0]);
      Serial.print(", ");
      Serial.println(linear[1]);
      
*/
      Serial.print(G[0]);
      Serial.print(", ");
      Serial.print(G[1]);
      Serial.println("  ");


       if(v== 0.0 && w == 0.0)
    {
        analogWrite(4,0);
        digitalWrite(5,LOW);
        digitalWrite(6,LOW);
        analogWrite(9,0);
    }
    else
    {

       if(G[0]<0)
       {
        analogWrite(4,G[0]);
        digitalWrite(5,HIGH);
       }
       if(G[1]<0)
       {
        digitalWrite(6,HIGH);
        analogWrite(9,G[1]);
        }
       if(G[0]>0)
       {
        analogWrite(4,G[0]);
        digitalWrite(5,LOW);
       }
       if(G[1]>0)
       {
        digitalWrite(6,LOW);
        analogWrite(9,G[1]);
        }
      
      }


  }
}
