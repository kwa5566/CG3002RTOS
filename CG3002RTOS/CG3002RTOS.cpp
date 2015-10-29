/*
 * CG3002RTOS.cpp
 *
 * Created: 13/10/2015 4:24:03 PM
 *  Author: WaiMin
 */ 

#include	<avr/io.h>
#include	<FreeRTOS.h>
#include	<task.h>
#include	<Arduino.h>
#include	"NewPing.h"
#include	"Wire.h"
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
// #include "Wire.h"
// #endif
#include	"L3G.h"
#include	"LSM303.h"


int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1};


//=======================================================================================================================================//
/*
 * imu_mis.c
 *
 * Created: 10/20/2015 6:04:15 PM
 *  Author: caiyunyang
 */ 

//SONAR 
#define MAX_DISTANCE 250 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SONAR_NUM     6 // Number or sensors.
#define PING_INTERVAL 30
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
#define DATA_SIZE 11
#define UP_TH 70
#define DOWN_TH 100
#define SIDE_TH 40 
#define BASE_TH 10 

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

bool isConnected = false;
bool isDataRequested = false;
unsigned int data[DATA_SIZE] = {0};

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
	NewPing(22, 23, 100), //UR 0  Right Side
	NewPing(24, 25, MAX_DISTANCE),	//UR1  Right Front
	NewPing(26, 27, MAX_DISTANCE),	//UR2 Left Front
	NewPing(28, 29, 100),	//UR3 Left Side
	NewPing(30, 31, MAX_DISTANCE),	//UR4 Center(right leg) 
	NewPing(32, 33, MAX_DISTANCE)	// right leg down 
};
unsigned int uS;

//IR 
float ir_value=0.0;
unsigned int ir_distance=0;

//MOTOR 
#define motorL 5
#define motorR 6
#define motorD 9 
int flagL, flagR , flagD ;

//IMU 
// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252f)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131f)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board

#define M_X_MIN -1832
#define M_Y_MIN -2672
#define M_Z_MIN -2760
#define M_X_MAX 2196
#define M_Y_MAX 1313
#define M_Z_MAX 1141

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_STEPS_DIST 0

//#define STATUS_LED 13

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
	{
	1,0,0  }
	,{
	0,1,0  }
	,{
	0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
	{
	0,0,0  }
	,{
	0,0,0  }
	,{
	0,0,0  }
};

//----------------------------------------------------------------------------------------------// 




//===================================================MATRIX====================================================================//
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
	float op[3];
	for(int x=0; x<3; x++)
	{
		for(int y=0; y<3; y++)
		{
			for(int w=0; w<3; w++)
			{
				op[w]=a[x][w]*b[w][y];
			}
			mat[x][y]=0;
			mat[x][y]=op[0]+op[1]+op[2];
			
			///float test=mat[x][y];
		}
	}
}


//===============================VECTOR=============================================//
//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
	float op=0;
	
	for(int c=0; c<3; c++)
	{
		op+=vector1[c]*vector2[c];
	}
	
	return op;
}

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
	vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
	vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
	vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar.
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
	for(int c=0; c<3; c++)
	{
		vectorOut[c]=vectorIn[c]*scale2;
	}
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
	for(int c=0; c<3; c++)
	{
		vectorOut[c]=vectorIn1[c]+vectorIn2[c];
	}
}

//===================================I2C==============================================//


L3G gyro;
LSM303 compass;

void I2C_Init()
{
	Wire.begin();
// 	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
// 	Wire.begin();
// 	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
// 	Fastwire::setup(400, true);
// 	#endif
}

void Gyro_Init()
{
	gyro.init();
	gyro.enableDefault();
	gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
	gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void Read_Gyro()
{
	gyro.read();
	
	AN[0] = gyro.g.x;
	AN[1] = gyro.g.y;
	AN[2] = gyro.g.z;
	gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
	gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
	gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init()
{
	compass.init();
	compass.enableDefault();
	switch (compass.getDeviceType())
	{
		case LSM303::device_D:
		compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
		break;
		case LSM303::device_DLHC:
		compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
		break;
		default: // DLM, DLH
		compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
	}
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
	compass.readAcc();
	
	AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
	AN[4] = compass.a.y >> 4;
	AN[5] = compass.a.z >> 4;
	accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
	accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
	accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void Compass_Init()
{
	// doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
}

void Read_Compass()
{
	compass.readMag();
	
	magnetom_x = SENSOR_SIGN[6] * compass.m.x;
	magnetom_y = SENSOR_SIGN[7] * compass.m.y;
	magnetom_z = SENSOR_SIGN[8] * compass.m.z;
}
//==========================dprintf==============================//
char debugBuffer[1024];
void debugPrint(const char *str)
{
	Serial.println(str);
	Serial.flush();
}
void dprintf(const char *fmt,...)
{
	va_list argptr;
	va_start(argptr, fmt);
	vsprintf(debugBuffer, fmt, argptr);
	va_end(argptr);
	debugPrint(debugBuffer);
}


//========================OUTPUT===========================// 
#define THRESHOLD 7
#define HEIGHT 0.9
#define PIE 3.14159265359
#define PRINT_SWING 0
#define PRINT_DEG 0
#define PRINT_DIST 0
#define PRINT_STEPS 1

int curDeg=0,prvDeg=0,step=0;
float alpha=0,beta=0,distance=0,prvHeading=0;
bool aDone=false,bDone=false,rotating=false;

void calculate()
{
  //float curHeading = ToDeg(MAG_Heading);
  //rotating = abs(curHeading-prvHeading)>9;
  //prvHeading = curHeading;
  //if(rotating){
	  ////Serial.println("rotaing yo!");
	  //return;
  //}
  //
  
  curDeg= ToDeg(pitch);
  dprintf("%d",curDeg);
  
  if(curDeg>0) // if moving forward
  {
	  if(curDeg>prvDeg && curDeg>alpha) // if increasing
	  {
		  alpha = curDeg;
		  //dprintf("al");
		  //#if PRINT_DEG == 1
		  //Serial.print("Alpha: ");
		  //Serial.println(alpha);
		  //#endif
		  } else if (curDeg < prvDeg && alpha> THRESHOLD && alpha<=25 && aDone==false){
		  alpha = alpha*2*PIE/180.0;
		  distance += HEIGHT*tan(alpha);
		  step+=1;
		  aDone=true;
		  //dprintf("%d",step);
		  //#if PRINT_SWING == 1
		  //Serial.print("Front Swing: ");
		  //Serial.println(HEIGHT*tan(alpha));
		  //#endif
		 // dprintf("[]");
		
	  }
	  
	  
  }
 // dprintf("a");
  if(curDeg<0) // if moving backward
  {
	  if(abs(curDeg) > abs(prvDeg) ) // if increasing
	  {
		  beta = abs(curDeg);
		 // dprintf("bet");
		  //#if PRINT_DEG == 1
		  //Serial.print("Beta: ");
		  //Serial.println(beta);
		  //#endif
		  
		  } else if (abs(curDeg) < abs(prvDeg) && beta> 3 && beta<=10 && bDone==false){
		  beta = beta*3.0*PIE/180.0;
		  distance += HEIGHT*tan(alpha);
		  step+=1;
		  bDone=true;
		  //dprintf("+");
		  //#if PRINT_SWING == 1
		  //Serial.print("Back Swing: ");
		  //Serial.println(HEIGHT*tan(beta));
		  //#endif
		  		  
	  }
	  
  }
  if(abs(curDeg)== 0)
  {
	  alpha=0;
	  beta=0;
	  aDone=false;
	  bDone=false;
	  //dprintf("rst");
  }
  
  prvDeg=curDeg;
  //#if PRINT_DIST == 1
  //Serial.print("Distance: ");
  //Serial.println(distance);
  //#endif
  
  //#if PRINT_STEPS == 1
  ////Serial.print("Steps: ");
  ////Serial.println(step);
  //Serial.println(compass.heading());
  //#endif
  
//  dprintf("steps: %d",step);
//Serial.print("steps :");
//Serial.println(step);
	dprintf("%d",step);
  //data[5]=step;
  unsigned int temp;
  if(step>=255)
  {
	  temp = step/255;
	  data[6] = step - temp*255;
	  data[7] = temp;
  }else{
	  data[6] = step;
	  data[7] = 0;
  }
  int headingVal=(int) ToDeg(MAG_Heading);
  dprintf("%d",headingVal);
  if(headingVal>=255)
  {
	  temp = headingVal/255;
	  data[8] = headingVal - temp*255;
	  data[9] = temp;
  }else{
	  data[8] = headingVal;
	  data[9] = 0;
  }
  
  //Serial.println(data[5]);
  //dprintf("%d",data[6]);
 // dprintf("a");
// Serial.println(data[6]);
}

void printdata(void)
{    
      //Serial.print("!");

      if (PRINT_EULER == 1){
		dprintf("p");
		dprintf( "%d ",(int)ToDeg(pitch));
		//	dprintf("%f, ",ToDeg(pitch));
		//	dprintf("%f, ",ToDeg(yaw));
		//	dprintf("%f \n ",compass.heading());		
      //Serial.print("ANG:");
      //Serial.print(ToDeg(roll));
      //Serial.print(",");
      //Serial.print(ToDeg(pitch));
      //Serial.print(",");
      //Serial.print(ToDeg(yaw));
      //Serial.print(",");
      ////MAG_Heading = ToDeg(MAG_Heading);
      ////if(MAG_Heading<0) MAG_Heading= 360+MAG_Heading;
      ////Serial.print(ToDeg(MAG_Heading));
      ////Serial.print(",");
     //// compass.heading((LSM303::vector<int>){0, -1, 0});
      //Serial.print(compass.heading());
      //Serial.println(); 
	 } 
      //
      //#if PRINT_STEPS_DIST==1
      //calculate();
      //#endif
      //
      //#if PRINT_ANALOGS==1
      //Serial.print(",AN:");
      //Serial.print(AN[0]);  //(int)read_adc(0)
      //Serial.print(",");
      //Serial.print(AN[1]);
      //Serial.print(",");
      //Serial.print(AN[2]);  
      //Serial.print(",");
      //Serial.print(AN[3]);
      //Serial.print (",");
      //Serial.print(AN[4]);
      //Serial.print (",");
      //Serial.print(AN[5]);
      //Serial.print(",");
      //Serial.print(c_magnetom_x);
      //Serial.print (",");
      //Serial.print(c_magnetom_y);
      //Serial.print (",");
      //Serial.print(c_magnetom_z);
      //#endif
      /*#if PRINT_DCM == 1
      Serial.print (",DCM:");
      Serial.print(convert_to_dec(DCM_Matrix[0][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][2]));
      #endif*/
     // Serial.println();    
      
}

long convert_to_dec(float x)
{
  return x*10000000;
}

/**********************DCM************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
*/
/**************************************************/

void Matrix_update(void)
{
  Gyro_Vector[0]=Gyro_Scaled_X(gyro_x); //gyro x roll
  Gyro_Vector[1]=Gyro_Scaled_Y(gyro_y); //gyro y pitch
  Gyro_Vector[2]=Gyro_Scaled_Z(gyro_z); //gyro Z yaw
  
  Accel_Vector[0]=accel_x;
  Accel_Vector[1]=accel_y;
  Accel_Vector[2]=accel_z;
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
  
 #if OUTPUTMODE==1         
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

//===============================COMPASS===================================================//

void Compass_Heading()
{
	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;
	
	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);
	
	// adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
	c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
	c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
	c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;
	
	//// Tilt compensated Magnetic filed X:
	//MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
	//// Tilt compensated Magnetic filed Y:
	//MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
	//// Magnetic Heading
	//MAG_Heading = atan2(-MAG_Y,MAG_X);
	////MAG_Heading = compass.heading(
	
	  MAG_Heading = atan2(c_magnetom_x,c_magnetom_z);
	  if(MAG_Heading < 0)
	  MAG_Heading += 2*M_PI;

	  if(MAG_Heading > 2*M_PI)
	  MAG_Heading -= 2*M_PI;
}

//=======================================================================================================================================//



int freeRam () {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


void readSonar(uint8_t sensor){	
	
	//vTaskDelay(PING_INTERVAL);	
	uS = sonar[sensor].ping_median(3); 	
	cm[sensor] = uS/US_ROUNDTRIP_CM; 
	data[sensor]=cm[sensor];
	
		//dprintf("%d\n",cm[sensor]);	
}

void readIRSensor(void){
	ir_value=analogRead(A0)*(5.0/1024.0);
	ir_distance=54.06*pow(ir_value,-1.21);
}

void onVMotor(){
	
	if ( flagR){
		analogWrite(motorR,255);
	}
	else{
		analogWrite(motorR,0);
	}
	
	
	if(flagL){ 
		analogWrite(motorL,225);		
	} 
	else {
		analogWrite(motorL,0);
	}
	
	
	if (flagD){
		analogWrite(motorD,255);		
	}
	else{
		analogWrite(motorD,0);
	}	
}




void	task1(void	*p)
{
	while(1)
	{	
		//dprintf("%d\n",millis());	
		
		//readSonar(); 
		vTaskDelay(20);			
		readSonar(1);	
		vTaskDelay(5);
		readSonar(0);
		vTaskDelay(5);
		readSonar(2);
		vTaskDelay(5);		
		readSonar(3);
		vTaskDelay(5);		
		readSonar(4);	
		vTaskDelay(30);			
		readSonar(5); 
		
		
		readIRSensor(); 		
		
		for (uint8_t i = 0; i < SONAR_NUM; i++)  // Loop through all the sensors.						
			dprintf("%d: %d",i,cm[i]);	
		//dprintf("IR: %d\n", ir_distance);
		
		flagD = 0 ; 
		flagL = 0 ; 
		flagR = 0 ; 
		if((cm[0]> BASE_TH && cm[0]< SIDE_TH) || (cm[1]> BASE_TH && cm[1]<UP_TH)){
			flagR = 1; 
		}		
		if((cm[3]> BASE_TH && cm[3]< SIDE_TH) || (cm[2]> BASE_TH && cm[2]<UP_TH)){
			flagL = 1;
		}
		if((cm[4]> BASE_TH && cm[4]< DOWN_TH) || (cm[5]> BASE_TH && cm[5]<DOWN_TH)){
			flagD = 1;
		}
		onVMotor();		//if(ir_distance>=20 && ir_distance<=50)
			//onVMotor(0);
		//else if (ir_distance>=50 && ir_distance<=100)
			//onVMotor(1);
		//else if (ir_distance>=100 && ir_distance<=150)
			//onVMotor(2);	
		//vTaskDelay(100);	
		
		dprintf("flagL: %d",flagL);
		dprintf("flagR: %d",flagR);
		dprintf("flagD: %d",flagD);	
	}
}

void task2(void	*p)
{
	while(1)
	{	
		
		//if((millis()-timer)>=30)  // Main loop runs at 50Hz
		//{
			unsigned long start = millis();
			counter++;
			timer_old = timer;
			timer=millis();
			if (timer>timer_old)
			G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
			else
			G_Dt = 0;
			
			// *** DCM algorithm
			// Data adquisition
			Read_Gyro();   // This read gyro data
			Read_Accel();     // Read I2C accelerometer
			
			if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
			{
				counter=0;
				Read_Compass();    // Read I2C magnetometer
				Compass_Heading(); // Calculate magnetic heading
			}
			
			// Calculations...
			Matrix_update();
			Normalize();
			Drift_correction();
			Euler_angles();
			// ***
			//dprintf("p");
			calculate();
			unsigned long duration = millis() - start;
			//dprintf("%d\n",ToDeg(pitch));
			//dprintf("%d\n",step);
			//dprintf("%d\n",ToDeg(MAG_Heading));
			//Serial.println(millis());
			vTaskDelay(10);
			//printdata();
			//}
	
	}
}

void task3(void	*p)
{
	while (1)
	{
		dprintf("3");

		if (isConnected==true)
		{
			if(Serial1.available()>0){
				
				int readyToSend = Serial1.read();
				/*dprintf(atoi(readyToSend));*/
				if(readyToSend==2){
					//Serial1.write(0xef);
					/*dprintf("1234");*/
					for(int i=0;i<DATA_SIZE;i++){
						Serial1.write(data[i]);
					}
					Serial1.write(0xff);
					Serial1.flush();
					}else if(readyToSend==3){
					analogWrite(5,255);
					}else if(readyToSend==4){
					analogWrite(5,0);
				}
				
			}

			vTaskDelay(10);
		}
	}
	
}

#define	STACK_DEPTH	128
void	vApplicationIdleHook()
{
	//	Do	nothing.
}

bool handShake(){
	while(Serial1.available()<=0);
	
	int incomingByte = Serial1.read();
	incomingByte += 1;
	if (incomingByte == 2)
	{
		//Serial1.write(1);
		Serial1.print(incomingByte);
		dprintf("%d",incomingByte);
		Serial1.flush();
		return true;
	}
	else
	return false;
}


void setup()
{
	Serial.begin(115200);	
	//Serial1.begin(115200);
	//
	//
	//dprintf("hs\n");
	//// wait for handshake
 	//while(isConnected==false)
 	//isConnected = handShake();
	//
	//////pinMode (STATUS_LED,OUTPUT);  // Status LED
	////
	//I2C_Init();
	////dprintf("be");
	////Serial.println("Pololu MinIMU-9 + Arduino AHRS");
//
	////digitalWrite(STATUS_LED,LOW);
	////vTaskDelay(1500);
	//
	//Accel_Init();
	//Compass_Init();
	//Gyro_Init();
	//
	////vTaskDelay(20);
	//delay(20);
	//
	//for(int i=0;i<32;i++)    // We take some readings...
	//{
		//Read_Gyro();
		//Read_Accel();
		//for(int y=0; y<6; y++)   // Cumulate values
		//AN_OFFSET[y] += AN[y];
	    ////vTaskDelay(20);
		//delay(20);
	//}
	//
	//for(int y=0; y<6; y++)
	//AN_OFFSET[y] = AN_OFFSET[y]/32;
	//
	//AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
	//
	//Serial.println("Offset:");
	//for(int y=0; y<6; y++)
	//Serial.println(AN_OFFSET[y]);
	//
	////vTaskDelay(2000);
	//delay(2000);
	////digitalWrite(STATUS_LED,HIGH);
	//
	//timer=millis();
	//delay(20);
	////vTaskDelay(20);
	//counter=0;
	////prvHeading=compass.heading();
	//dprintf("en");
}


int	main(void)
{
	init();
	setup();
	TaskHandle_t t1,t2,t3;
	//	Create	tasks
	xTaskCreate(task1,	"Task 1",	STACK_DEPTH,	NULL,	3,	&t1);
	//xTaskCreate(task2,	"Task 2",	STACK_DEPTH,	NULL,	2,	&t2);
	//xTaskCreate(task3,  "Task 3",	STACK_DEPTH,	NULL,	1,	&t3);
	vTaskStartScheduler();
	
}