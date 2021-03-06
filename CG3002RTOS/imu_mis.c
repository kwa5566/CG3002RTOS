/*
 * imu_mis.c
 *
 * Created: 10/20/2015 6:04:15 PM
 *  Author: caiyunyang
 */ 




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
	
	// Tilt compensated Magnetic filed X:
	MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
	// Tilt compensated Magnetic filed Y:
	MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-MAG_Y,MAG_X);
	//MAG_Heading = compass.heading(
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

//===================================I2C==============================================//
#include "L3G.h"
#include "LSM303.h"

L3G gyro;
LSM303 compass;

void I2C_Init()
{
	Wire.begin();
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
	//compass.heading((LSM303::vector<int>){0, 0, -1});
	//compass.m_min = (LSM303::vector<int16_t>){-2304, -2194, -2156};
	//compass.m_max = (LSM303::vector<int16_t>){+2771, +2860, +2753};
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

//========================OUTPUT===========================// 
#define ANGLE_THRESHOLD 10
#define HEIGHT 0.7
float curDeg=0,prvDeg=0,alpha=0,beta=0,distance=0,step=0;

void calculate()
{
    curDeg=ToDeg(roll);
    
     if( ((int) curDeg)==0 || ( abs((int) curDeg) <=3 ))
    {
      if(alpha>0)
      {
          if(alpha> ANGLE_THRESHOLD)
        {
          distance+= HEIGHT*(tan(ToRad(alpha)));
        }
        Serial.print("alpha ");
        Serial.println(alpha);
        alpha=0;
      }
      if(beta>0)
      {
        if(beta> ANGLE_THRESHOLD-5)
        {
          distance+= HEIGHT*(tan(ToRad(beta)));
          
        }
        Serial.print("beta ");
        Serial.println(beta);
        beta=0;
      }
      
    }
    
    
    
    else if(curDeg>0) // leg swinging front
    {

      
      if(curDeg>prvDeg)  // still moving havent' got alpha
      {
       // Serial.print("moving front ");
        alpha = curDeg;       
        //Serial.print("alpha ");
        //Serial.println(alpha); 
      }
      
    } else {      // leg swinging back
     
      if(abs(curDeg)>abs(prvDeg))
      {
       // Serial.print("moving back ");
        beta = abs(curDeg);
        //Serial.print("beta ");
        //Serial.println(beta);
      }
      
    }
    
   

    prvDeg = curDeg;
    //Serial.print("distance: ");
    //Serial.println(distance);
    
}

void printdata(void)
{    
      //Serial.print("!");

      #if PRINT_EULER == 1
      Serial.print("ANG:");
      Serial.print(ToDeg(roll));
      Serial.print(",");
      Serial.print(ToDeg(pitch));
      Serial.print(",");
      Serial.print(ToDeg(yaw));
      Serial.print(",");
      //MAG_Heading = ToDeg(MAG_Heading);
      //if(MAG_Heading<0) MAG_Heading= 360+MAG_Heading;
      //Serial.print(ToDeg(MAG_Heading));
      //Serial.print(",");
     // compass.heading((LSM303::vector<int>){0, -1, 0});
      Serial.print(compass.heading());
      Serial.println(); 
      #endif      
      
      #if PRINT_STEPS_DIST==1
      calculate();
      #endif
      
      #if PRINT_ANALOGS==1
      Serial.print(",AN:");
      Serial.print(AN[0]);  //(int)read_adc(0)
      Serial.print(",");
      Serial.print(AN[1]);
      Serial.print(",");
      Serial.print(AN[2]);  
      Serial.print(",");
      Serial.print(AN[3]);
      Serial.print (",");
      Serial.print(AN[4]);
      Serial.print (",");
      Serial.print(AN[5]);
      Serial.print(",");
      Serial.print(c_magnetom_x);
      Serial.print (",");
      Serial.print(c_magnetom_y);
      Serial.print (",");
      Serial.print(c_magnetom_z);
      #endif
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
			
			float test=mat[x][y];
		}
	}
}

