//Sliding mode control ,PID sliding surface ,,,Control_SlidingPIDsam3x8e.h
//project_Quad 32 bit Arduino Due
//1. stabilized quadrotor 
//by: tinnakon kheowree 
//0860540582
//tinnakon_za@hotmail.com
//tinnakonza@gmail.com
float u2_roll = 0;
float u3_pitch = 0;
float u4_yaw = 0;

float setpoint_rollold = 0.0;
float setpoint_rate_roll = 0.0;
float setpoint_rate_rollold = 0.0;
float setpoint_rate_rollold2 = 0.0;
float roll_I_rate = 0.0;
float sign_roll = 0.0;

float setpoint_pitchold = 0.0;
float setpoint_rate_pitch = 0.0;
float setpoint_rate_pitchold = 0.0;
float setpoint_rate_pitchold2 = 0.0;
float pitch_I_rate = 0.0;
float sign_pitch = 0.0;
          
float yaw_I_rate = 0.0;
float error_rate_yawold = 0.0;
float setpoint_rate_yawold = 0.0;
float setpoint_rate_yawold2 = 0.0;
float sign_yaw = 0.0;

//Automatic take-off and landing
int time_auto = 0;
float  h_counter = 0.03;//0.08
float hz_I = 0.0;
uint8_t takeoff = 0;
uint8_t endAuto = 0;

void Control_SlidRate(){
// ROLL CONTROL PID sliding surface control ///////////
  float setpoint_roll = ((CH_AILf-CH_AIL_Cal)*0.076);// + Control_YBf;//0.076 0.12 max +-45 deg  ////+-18  + Control_YBf
  applyDeadband(setpoint_roll, 1.2);//1.2
  setpoint_rate_roll = (0.065*setpoint_rate_roll/(0.065+G_Dt)) + ((setpoint_roll-setpoint_rollold)/(0.065+G_Dt));//Diff remote
  setpoint_rollold = setpoint_roll;
  setpoint_rate_roll = constrain(setpoint_rate_roll, -80, 80);//+-100 deg/s
  float error_roll = setpoint_roll - ahrs_r;//error
  float error_roll_dot = setpoint_rate_roll - (GyroXf*RAD_TO_DEG);//e_dot
  float error_roll_dotdot = (error_roll_dot - setpoint_rate_rollold2)/0.01;//e_dotdot
  setpoint_rate_rollold2 = setpoint_rate_rollold;
  setpoint_rate_rollold = error_roll_dot;
  float surface_roll = error_roll_dot + (lambda_Roll*error_roll);//surface
  roll_I_rate += surface_roll*Ki_Roll*G_Dt;//I control
  roll_I_rate = constrain(roll_I_rate, -100, 100);//+-150
    if (abs(surface_roll) <= 1.0)
  {
    sign_roll = surface_roll;//saturation function
  }
  else//sign function
  {
    if(surface_roll < 0) sign_roll = -1.0;
    if(surface_roll > 0) sign_roll = 1.0;
    if(surface_roll == 0) sign_roll = 0.0;
  }
  u2_roll = Ka_Roll*error_roll_dotdot + Kd_Roll*error_roll_dot + Kp_Roll*error_roll + roll_I_rate + K_Roll*sign_roll;//PID sliding surface
  //u2_roll = I_x*(-C2_Roll*error_rate_roll - K_Roll*sgn_roll - Kp_Roll*slid_roll - roll_I_rate)*Nm_to_PWM;
  u2_roll = constrain(u2_roll, -300, 300);//+-300
  
////// PITCH CONTROL PID sliding surface control///////////
  float setpoint_pitch = ((CH_ELEf-CH_ELE_Cal)*-0.076);// + Control_XBf;//0.076 max +-45 deg  ////+-18 - Control_XBf
  applyDeadband(setpoint_pitch, 1.2);//1.2
  setpoint_rate_pitch = (0.065*setpoint_rate_pitch/(0.065+G_Dt)) + ((setpoint_pitch-setpoint_pitchold)/(0.065+G_Dt));//Diff remote
  setpoint_pitchold = setpoint_pitch;
  setpoint_rate_pitch = constrain(setpoint_rate_pitch, -80, 80);
  float error_pitch =  setpoint_pitch - ahrs_p;//error
  float error_pitch_dot = setpoint_rate_pitch - (GyroYf*RAD_TO_DEG);//e_dot
  float error_pitch_dotdot = (error_pitch_dot - setpoint_rate_pitchold2)/0.02;//e_dotdot
  setpoint_rate_pitchold2 = setpoint_rate_pitchold;
  setpoint_rate_pitchold = error_pitch_dot;
  float surface_pitch = error_pitch_dot + (lambda_Pitch*error_pitch);//surface
  pitch_I_rate += surface_pitch*Ki_Pitch*G_Dt;
  pitch_I_rate = constrain(pitch_I_rate, -100, 100);//+-150
   if(abs(surface_pitch) <= 1.0)
  {
    sign_pitch = surface_pitch;//saturation function
  }
  else//sign function
  {
   if(surface_pitch < 0) sign_pitch = -1.0;
   if(surface_pitch > 0) sign_pitch = 1.0;
   if(surface_pitch == 0) sign_pitch = 0.0;
  }
  u3_pitch = Ka_Pitch*error_pitch_dotdot + Kd_Pitch*error_pitch_dot + Kp_Pitch*error_pitch + pitch_I_rate + K_Pitch*sign_pitch;//PID sliding surface
  //u3_pitch = I_y*(-C2_Pitch*error_rate_pitch - K_Pitch*sgn_pitch - Kp_Pitch*slid_pitch - pitch_I_rate)*Nm_to_PWM;
  u3_pitch = constrain(u3_pitch, -300, 300);//+-300
  
////// YAW CONTROL PID sliding surface control///////////
  float setpoint_rate_yaw = (CH_RUDf-CH_RUD_Cal)*0.4;//0.35
  applyDeadband(setpoint_rate_yaw, 7.1);//6.5
  if(abs(setpoint_rate_yaw) > 0.1)
  {
    setHeading = ahrs_y;
  }
  float error_yaw = 0.0 - Heading;//error
  error_yaw = constrain(error_yaw, -30, 30);
  float error_yaw_dot = setpoint_rate_yaw - (GyroZf*RAD_TO_DEG);//e_dot
  float error_yaw_dotdot = (error_yaw_dot - setpoint_rate_yawold2)/0.02;//e_dotdot
  setpoint_rate_yawold2 = setpoint_rate_yawold;
  setpoint_rate_yawold = error_yaw_dot;
  error_rate_yawold = error_yaw_dot;
  float surface_yaw = error_yaw_dot + (lambda_Yaw*error_yaw);//surface
  yaw_I_rate += surface_yaw*Ki_Yaw*G_Dt;
  yaw_I_rate = constrain(yaw_I_rate, -100, 100);//+-100
   if(abs(surface_yaw) <= 1.0)
  {
    sign_yaw = surface_yaw;
  }
  else
  {
   if(surface_yaw < 0) sign_yaw = -1.0;
   if(surface_yaw > 0) sign_yaw = 1.0;
   if(surface_yaw == 0) sign_yaw = 0.0;
  }
  u4_yaw = Ka_Yaw*error_yaw_dotdot + Kd_Yaw*error_yaw_dot + Kp_Yaw*error_yaw + yaw_I_rate + K_Yaw*sign_yaw;
  //u4_yaw = I_z*(-C2_Yaw*error_rate_yaw - K_Yaw*sgn_yaw - Kp_Yaw*slid_yaw - yaw_I_rate)*Nm_to_PWM;
  u4_yaw = constrain(u4_yaw, -250, 250);//+-300
 ////Altitude//////////////////////////////////////////////////////////////////////////////////////////////////////  
  if(Mode == 1 || Mode == 2)//Altitude Hold, 
  {
    float err_hz = Altitude_Hold - z1_hat;
    //err_hz = constrain(err_hz, -2, 2);//+-2 m
    //applyDeadband(err_hz, 0.11);//nois 0.2 m
    float surface_alt = z2_hat - (lambda_alt*err_hz);//surface
    hz_I = hz_I + (Ki_altitude*surface_alt*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -200, 200);//+-200
    float uthrottle2 = err_hz*Kp_altitude - hz_I - z2_hat*Kd_altitude - accrZ_Earth*Ka_altitude;//- (accrZ_cutg*0.129);//- (accrZ_cutg*0.0129) + 9.81; //state feedback control Altitude m = 1290 g
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad;//  /cos_rollcos_pitch
  }
  else if(Mode == 3)//Automatic  Takeoff, Landing
  {
    float err_hz = h_counter - z1_hat;
    err_hz = constrain(err_hz, -2, 2);//+-2 m
    //applyDeadband(err_hz, 0.11);//nois 0.2 m
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -150, 150);//+-200
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I - (z2_hat*Kd_altitude) - (accrZ_Earth*Ka_altitude);//- (accrZ_cutg*0.129);//- (accrZ_cutg*0.0129) + 9.81; //state feedback control Altitude m = 1290 g
    //uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
    uthrottle = uthrottle2*m_quad/cos_rollcos_pitch;//  /cos_rollcos_pitch
  }
  else
  {
    hz_I = 0.0;
    uthrottle = 0.0;
    Altitude_Hold = z1_hat;
  }//end Altitude Hold
uAltitude = CH_THR + uthrottle;//m*g = 10.8 N = 
}
void Chack_Command(){
   if(AUX_1 <= (AltHold-10))//Stabilize 
  {
    Mode = 0;
  }
   if(AUX_1 > (AltHold-10) && AUX_1 <= (AltHold+10))//Altitude Hold, 
  {
    Mode = 1;
  }
   if(AUX_1 > (PositionHold-10) && AUX_1 <= (PositionHold+10))//Position Hold
  {
    Mode = 2;
  }  
   if(AUX_1 > (Auto-10))//Automatic  Takeoff
  {
    Mode = 3;
  }
//    if(AUX_1 > (RTH-10) && AUX_1 <= 1860)//RTH
//  {
//    Mode = 4;
//    target_LAT = GPS_LAT_HOME;//GPS_LAT_Hold
//    target_LON = GPS_LON_HOME;//GPS_LON_Hold
//  }
}//end Chack_Command()
