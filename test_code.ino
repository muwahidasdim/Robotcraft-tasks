
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>
#define pi 3.1415
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define VOLTS_PER_UNIT    .00488F        // (.0049 for 10 bit A-D)  :1/v *225@5 -.887  ,  148.5@3.3: -.391087
float voltsLIR,voltsRIR,voltsMIR;
float distanceLIR,distanceRIR,distanceMIR;
float des_w,des_v;

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(18,19);
Encoder knobRight(3, 2);
//   avoid using pins with LEDs attached
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 100;  //the value is a number of milliseconds
int right_dir=5;
int right_speed=4;
int left_dir=6;
int left_speed=9;


  // PID speed
    float v=0.0;
    float w=0.0;

  //IRS
  int LeftIR;
  int MiddleIR;
  int RightIR;


  long int positionLeft  = 0;
  long int positionRight = 0;
  long int newLeft, newRight;
  long int valLeft, valRight;

  float wheel[2]={0,0};
  float x=0,y=0;
  float pose_2[5];
  float dt=0.1;
  float vi=0;
  float wi=0;
  float Ti=0;
  float xi=0;
  float yi=0;
  float linear_w[2]={0,0};
  static float prev_PE[2] = {0,0};
  static float prev_IE[2] = {0,0};
  float G[2]={0,0};
  float KP=8.0;
  float KI=0.0;
  float KD=0.0; 
  float PE_l=0;
  float PE_r=0;
  float DE_l=0;
  float DE_r=0;
  float IE_l=0;
  float IE_r=0; 
  float prev_Ti=0;
  unsigned long mytime=0;
  float nl,nr,r=0.016,b=0.093,c=3500;
    
ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist &msg)
 {
  v = msg.linear.x ;
  w = msg.angular.z;
  nh.loginfo("Program info");
  }

   
std_msgs::Float32 distancef, distancel,distancer;
geometry_msgs::Pose2D p;


ros::Subscriber<geometry_msgs::Twist>  cmd_vel("cmd_vel",&messageCb);

ros::Publisher left_distance("left_distance",&distancel);
ros::Publisher right_distance("right_distance",&distancer);
ros::Publisher front_distance("front_distance",&distancef);
ros::Publisher pose("pose",&p);



  void SensL()
  {
      LeftIR=analogRead(A2);
      voltsLIR = (float)LeftIR * VOLTS_PER_UNIT; 
      distanceLIR = 225 * pow(voltsLIR,-.887);     // same in mm
      distancel.data = distanceLIR;
      left_distance.publish(&distancel);

   
  }

  void SensF()
  {
      MiddleIR=analogRead(A3);
      voltsMIR = (float)MiddleIR * VOLTS_PER_UNIT; 
      distanceMIR = 225 * pow(voltsMIR,-.887);     // same in mm
 
      distancef.data = distanceMIR;
      front_distance.publish(&distancef);
 
  }
  void SensR()
  {
      RightIR=analogRead(A4); 
      voltsRIR = (float)RightIR * VOLTS_PER_UNIT; 
      distanceRIR = 225 * pow(voltsRIR,-.887);     // same in mm
      distancer.data = distanceRIR;
      right_distance.publish(&distancer);
  }


void setup() {
  // put your setup code here, to run once:
 
  nh.initNode();
  nh.advertise(left_distance);
  nh.advertise(right_distance);
  nh.advertise(front_distance);
  nh.advertise(pose);
  nh.subscribe(cmd_vel);

    nh.loginfo("Program start");

    startMillis = millis();  //initial start time
  pinMode(right_speed,OUTPUT);
  pinMode(right_dir,OUTPUT);
  pinMode(left_dir,OUTPUT);
  pinMode(left_speed,OUTPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
}


void wheels(float v, float w, float r, float b)             //angular velocity of left and right wheel
{

  wheel[0]=(v - (b/2.0)*w)/r; //left
  wheel[1]=(v + (b/2.0)*w)/r; //right
 
  
}
  

void pose2(float nl, float nr, float r, float c)          //calculates real linear and angular velocities using the encoder values
{



  vi=((2.0*pi*r)/c)*(nr+nl)/2.0*(1.0/dt);
  wi=((2.0*pi*r)/c)*(nr-nl)/2.0*(1.0/dt);
  Ti=atan2(sin(prev_Ti+wi*dt),cos(Ti+wi*dt));
  prev_Ti=Ti;
  
  xi=(x+vi*cos(Ti)*dt);
  yi=y+vi*sin(Ti)*dt;
   p.x=xi/2.31604288;
   p.y=yi;
   p.theta=Ti;
  x=xi;
  y=yi;


   pose_2[0]=vi;
   pose_2[1]=wi;
   pose_2[2]=Ti;
   pose_2[3]=xi;
   pose_2[4]=yi;
   

  
}
void linearW(float v, float b, float r, float w)
{
 
  linear_w[0]=(v-(b/2.0)*w)/r;//left
  linear_w[1]=(v+(b/2.0)*w)/r;//right
  
  
  }




void PID(float desired_w_l,float desired_w_r, float real_w_l,float real_w_r)
{
  

  
  
  
  PE_l = desired_w_l-real_w_l;
  PE_r = desired_w_r-real_w_r;
  
  
  DE_l = PE_l - prev_PE[0];
  DE_r = PE_r - prev_PE[1];
  
 
  IE_l = prev_IE[0] + PE_l;
  IE_r = prev_IE[1] + PE_r;
  

  G[0] = KP*PE_l + KI*IE_l*(0.1) + KD*DE_l/(0.1);
  G[1] = KP*PE_r + KI*IE_r*(0.1) + KD*DE_r/(0.1);

  
  prev_PE[0] = PE_l;
  prev_PE[1] = PE_r;
  prev_IE[0] = IE_l;
  prev_IE[1] = IE_r;


}




void loop() {


    
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)

  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    startMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)

    valLeft=  knobLeft.read();
    valRight= knobRight.read();  
    newLeft = valLeft-positionLeft;
    newRight = valRight-positionRight;
   
  
    positionLeft =  valLeft;
    positionRight =  valRight;
    nl=newLeft;
    nr=newRight;
      
      
    wheels(v, w,r,b);
    pose2(nl,nr,r,c);
    linearW(pose_2[0],b,r,pose_2[1]);
    PID(wheel[0],wheel[1],linear_w[0],linear_w[1]);
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
     /* Serial.print(G[0]);
      Serial.print(", ");
      Serial.print(G[1]);
      Serial.println("  ");
*/

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
        analogWrite(4,abs(G[0]*1.014155));
        digitalWrite(5,HIGH);
       }
       if(G[1]<0)
       {
        digitalWrite(6,HIGH);
        analogWrite(9,abs(G[1]));
        }
       if(G[0]>0)
       {
        analogWrite(4,G[0]*1.014155);
        digitalWrite(5,LOW);
       }
       if(G[1]>0)
       {
        digitalWrite(6,LOW);
        analogWrite(9,G[1]);
        }
      
      }


  
 
    SensL();
    SensR();
    SensF();


  pose.publish(&p);
  
  nh.spinOnce();
  }


}
