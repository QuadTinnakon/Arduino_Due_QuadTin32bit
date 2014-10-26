//project_Quad 32 bit Arduino Due
//1. stabilized quadrotor 
//by: tinnakon kheowree 
//0860540582
//tinnakon_za@hotmail.com
//tinnakonza@gmail.com
//The type of multicopter  
//#define HEX_X
#define Quad_X
//#define Quad_P
//#define PPM

///////////////Mode///////////////////////////
#define AltHold 1250
#define PositionHold 1400
#define Auto 1600
int Mode = 0;
//Mode 0 = Stabilize
//Mode 1 = Altitude Hold
//Mode 2 = Position Hold ,Loiter
//Mode 3 = Automatic  Takeoff ,Landing
/////////////////////////////////////////////
// Automatic take-off and landing 
#define h_control 2.2  //0.6 0.9 meter
//Parameter system Quadrotor
#define m_quad 1.1 //kg
#define L_quad 0.175 //m quad+=0.25   I = (1/3)mL2 = 0.02291
/////////////////////////////////////////////////////////////////////
//PID-------------Rate
float Kp_Roll = 6.62;//9.92
float Kd_Roll = 2.55;//4.15
float Ka_Roll = 0.0942;//0.0128 I = (1/3)mL2 = 0.02291
//Sliding mode control Roll
float lambda_Roll = 1.22;
float K_Roll = 2.5;
float Ki_Roll = 0.85;//0.65

float Kp_Pitch = 6.62;//9.92
float Kd_Pitch = 2.55;//4.15
float Ka_Pitch = 0.0942;//0.0128
//Sliding mode control Pitch
float lambda_Pitch = 1.22;
float K_Pitch = 2.5;
float Ki_Pitch = 0.85;//0.65

float Kp_Yaw = 5.9;//11.9
float Kd_Yaw = 2.25;//4.95
float Ka_Yaw = 0.0185;//0.0395
//Sliding mode control Yaw
float lambda_Yaw = 1.22;
float K_Yaw = 2.5;
float Ki_Yaw = 2.65;//0.65

//stat feedback--------------Altitude
float Kp_altitude = 185.2;//425.2 365 165.0
float Ki_altitude = 108.15;//24.15 0.018 1.13 82.5
float Kd_altitude = 350.5;//240.5 370.5 315.5 120
float Ka_altitude = 28.5;//31.5 38.5 41.5 35 25 - 45
float lambda_alt = 0.11;//0.25

//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
#define tarremote 0.062  //0.092 slow 0.12 0.02 0.08 remote 
//#define tar 0.011 //0.012 0.015
//////////////////////////////////////////////////

//PID GPS////////////////////////////////////////////
float Kp_gps = 0.045;//0.085 0.15 0.101 2.101 5.101
float Ki_gps = 0.085;//0.15
float Kd_gps = 1.05;//1.9 4.3 0.35 1.35 3.35
float Kp_speed = 0.27;//0.15 0.35 0.095 min 0.15

//GPS //สตาร์ท//////////////////////////////////////
float GPS_LAT_HOME = 13.867000579 ;//13.867000579  13.867021560  13.867017745      14.907173156
float GPS_LON_HOME = 100.483291625; //100.483291625 100.483261108  100.483276367  100.206214904
//ลงจอด
float waypoint1_LAT = 13.875096;//F, 13.875096, 100.484546     ,R     13.867492, 100.501004
float waypoint1_LON = 100.484546;//B, 13.857347, 100.483344   ,L     13.866868, 100.473152
//
float waypoint2_LAT = 13.867051124;
float waypoint2_LON = 100.483238220;//13.866970, 100.483240
//
float waypoint3_LAT = 13.867051124;
float waypoint3_LON = 100.483238220;
//
float waypoint4_LAT = 13.867051124;
float waypoint4_LON = 100.483238220;
////////////////////////////////////////////////////////////////////
//Accelerometer calibration constants; use the Calibrate example from print(accelRaw[XAXIS]);
int A_X_MIN = -4160;    //
int A_X_MAX = 3968;     //
int A_Y_MIN = -4137;    //
int A_Y_MAX = 4063;     //
int A_Z_MIN = -4216;    //
int A_Z_MAX = 4007;     //4007
////////////////////////////////////////////////////////////////////
//magnetometer calibration constants; use the Calibrate example from print(MagXf);
// the Pololu library to find the right values for your board
int M_X_MIN = -490;    //-654  -693   -688
int M_X_MAX = 310;     //185   209    170
int M_Y_MIN = -369;    //-319  -311   -310
int M_Y_MAX = 397;     //513   563    546
int M_Z_MIN = -392;    //-363  -374   -377
int M_Z_MAX = 346;     //386   429    502
////////////////////////////////////////////////////////////////////
//Observer hz
float Altitude_Hold = 0.0;
//float Altitude_hat=0.0;//Observer hx
float vx_hat=0.0;
float vx_hat2=0.0;
float vy_hat=0.0;
float vy_hat2=0.0;
//float vz_hat=0.0;
//float vz_hat2=0.0;
//float h=0.0;
float seth=0.0;//set control
float uthrottle=0.0;
float uAltitude = 1000.0;
float accrX_Earth = 0.0;
float accrY_Earth = 0.0;
float accrZ_Earth = 0.0;
float accrZ_Earthf = 0.0;
//float vz = 0.0;
//kalman
float z1_hat = 0.0;
float z2_hat = 0.0;
float z1_hat2 = 0.0;
float z2_hat2 = 0.0;
float u_z = 0.0;
float baro_vz = 0.0;
float baro_vz_old = 0.0;
float baro_vz_old2 = 0.0;

//GPS
float GPS_LAT1 = 0.0;
float GPS_LON1 = 0.0;
float GPS_LAT1f = 0.0;
float GPS_LON1f = 0.0;
float GPS_LAT1Lf = 0.0;
float GPS_LON1Lf = 0.0;
float GPS_LAT1lead = 0.0;
float GPS_LON1lead = 0.0;
float GPS_speed = 0.0;
float actual_speedX = 0.0;
float actual_speedY = 0.0;
float actual_speedXf = 0.0;
float actual_speedYf = 0.0;
float actual_speedXold = 0.0;
float actual_speedYold = 0.0;
float _last_velocityX = 0.0;
float _last_velocityY = 0.0;
float GPS_LAT1_old = GPS_LAT_HOME;
float GPS_LON1_old = GPS_LON_HOME;
float Control_XEf = 0.0;
float Control_YEf = 0.0;
float Control_XBf = 0.0;
float Control_YBf = 0.0;
float target_LAT = 0.0;
float target_LON = 0.0;
byte currentCommand[23];
byte Read_command = 0;
float GPS_hz = 0.0;
float GPS_vz = 0.0;
float GPS_ground_course2 = 0.0;
float GPS_Distance = 0.0;
float error_LAT = 0.0;
float error_LON = 0.0;  
float GPSI_LAT = 0.0;
float GPSI_LON = 0.0;  
uint8_t GPS_filter_index = 0;
float GPS_SUM_LAT[5];
float GPS_SUM_LON[5];
//float courseRads = 0.0;

////////time roop////////////////////////////////////////////
#define TASK_100HZ 4
#define TASK_50HZ 8
#define TASK_20HZ 20
#define TASK_10HZ 40
#define TASK_5HZ 80
#define TASK_2HZ 200
#define TASK_1HZ 400
//#define RAD_TO_DEG 57.295779513082320876798154814105
//#define DEG_TO_RAD 0.017453292519943295769236907684886

//direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
float DCM00 = 1.0;
float DCM01 = 0.0;
float DCM02 = 0.0;
float DCM10 = 0.0;
float DCM11 = 1.0;
float DCM12 = 0.0;
float DCM20 = 0.0;
float DCM21 = 0.0;
float DCM22 = 1.0;
//float DCM23 = 1.0;
float cos_rollcos_pitch = 1.0;

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;

uint16_t frameCounter = 0;
uint8_t timeLanding = 0;
uint8_t timeOff = 0;
byte armed = 0;
float G_Dt = 0.01; 
long Dt_sensor = 1000;
long Dt_roop = 10000;
bool Status_LED = LOW;
int ESC_calibra = 0;

//Baro
MS561101BA32bit baro = MS561101BA32bit();
#define  MOVAVG_SIZE 100 //20
float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;
float sea_press=1013.25;
float temperaturetr = 32.5;
float presser;
float presserf;
float Altitude_baro = 0.0;
float Altitude_barof=0.0;
float Altitude_Ground = 0.0;
