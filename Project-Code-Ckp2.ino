#include <DynamixelShield.h>
#include <stdlib.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

int total_legs=4; //number of legs

uint8_t IDs[]={3,2,30,4}; // Leg ID corresponding to [RF,LF,LR,RR] legs. 
uint8_t Directions[]={1,0,0,1}; // Leg rotating direction corresponding to [RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; 
//Walk {0.5, 0.25, 0.75}
float gait[]={0.5, 0.25, 0.75}; // (\phi_1, \phi_2, \phi_3) = [LF-RF,LR-RF,RR-RF]; 
float gait_deg[]={0,0,180,180};  //gait in deg
float leg_phase[4];
int Leg_zeroing_offset[]={52,69,64,55}; //zeroing calibration offset in deg

// --------------------
// PROJECT VIBRATION SETTINGS
// --------------------
bool use_vibration = true;
float vibrate_duration = 0.6; // seconds
float vibrate_amplitude = 5.0;  // degrees
float vibrate_frequency = 10.0; // Hz

//clock parameters belows need to be calculated in clock_init()///////////////////////////////////////////////
float time_slow_start=1; 
float time_slow_end=3; 
float time_vibrate_end = 1.5;

float degree_slow_start=150; 
float degree_slow_end=210; 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//You can change the desired Buehler clock parameters here
float phi_s=1.1; //in rad, ϕ_s is the angular extent of the slow phase
float phi_0=2.64; //in rad, ϕ_0 is the center of the slow phase 
float d_c=0.45; // d_c is the duty factor of the slow phase
float clock_period=4; //in seconds, time to complete 1 rotation

// Dead-zone hysteresis
const float ENTER_DEADZONE = 285.0;
const float EXIT_DEADZONE  = 240.0;

//change code to return desired speed in slow phase
float omega_slow(){ 
  float t_s = d_c * clock_period;
  float phi_s_deg = phi_s * 180.0 / PI;
  float omega_s = phi_s_deg / t_s;
  return omega_s;   //return desired leg speed in slow phase in deg/s
}

// FIX 1: Updated omega_fast to account for vibration duration!
float omega_fast(){  
  float t_slow = d_c * clock_period;
  float t_fast_total = clock_period - t_slow - vibrate_duration;
  float phi_s_deg = phi_s * 180.0 / PI;
  float total_deg = 360.0;
  float omega_f = (total_deg - phi_s_deg) / t_fast_total;
  return omega_f;  //return desired leg speed in fast phase in deg/s
}

float wrap360(float angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

// FIX 1: Updated clock_init for perfect 3-phase timing alignment
void clock_init(){   
  float phi_s_deg = phi_s * 180.0 / PI;
  float phi_0_deg = phi_0 * 180.0 / PI;
  degree_slow_start=  phi_0_deg - phi_s_deg / 2.0;
  
  float t_slow = d_c * clock_period;
  float omega_f = omega_fast();
  float omega_s = omega_slow();
  
  time_slow_start = degree_slow_start / omega_f;
 
  // Vibrate immediately before slow phase
  if (vibrate_duration < 0.0) vibrate_duration = 0.0;
  
  time_vibrate_end = time_slow_start + vibrate_duration;
  time_slow_end = time_vibrate_end + t_slow;

  // Make degree_slow_end match the actual slow-phase travel
  degree_slow_end = degree_slow_start + (omega_s * t_slow);

  return;
}

void translate_gait_phase() {
  leg_phase[0] = 0.0;      // RF
  leg_phase[1] = gait[0]; // LF − RF
  leg_phase[2] = gait[1]; // LR − RF
  leg_phase[3] = gait[2]; // RR − RF
}

// compute desired motor angle at any time instance
float get_desired_angle(int leg, long elapsed) { 
  float phase_shift = leg_phase[leg] * clock_period;
  float t = fmod((elapsed / 1000.0) + phase_shift, clock_period);

  float angle;

  if (t < time_slow_start) {
    float omega_f = omega_fast();
    angle = omega_f * t;
  }
  else if (use_vibration && t < time_vibrate_end) {
    // short burst right before stance
    float t_vib = t - time_slow_start;
    angle = degree_slow_start +
            vibrate_amplitude * sin(2.0 * PI * vibrate_frequency * t_vib);
  }
  else if (t < time_slow_end) {
    float omega_s = omega_slow();
    angle = degree_slow_start + omega_s * (t - time_vibrate_end);
  }
  else {
    float omega_f = omega_fast();
    angle = degree_slow_end + omega_f * (t - time_slow_end);
  }
        
  return wrap360(angle); //in deg
}

// print desired motor position and associated elapsed time to debug serial
void print_position(long t, int leg, float desired_pos){
  float time_sec = t / 1000.0;
  float t_norm  = fmod(time_sec, clock_period);  // wraps every 4s
  desired_pos= wrap360(desired_pos);
  DEBUG_SERIAL.print(t_norm);   
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.print(leg);
  DEBUG_SERIAL.print("\t");
  DEBUG_SERIAL.println(desired_pos);
        
  return;
}

void translate_gait_deg(){ 
  gait_deg[0]=0;  //RF
  gait_deg[1]=gait[0]*360; //LF
  gait_deg[2]=gait[1]*360; //LB
  gait_deg[3]=gait[2]*360; //RB
   return;
}

int dead_zone_speed_tuning=-10; 

long start;

void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  for (int i=0;i<total_legs;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
  clock_init();
  translate_gait_deg(); 
  translate_gait_phase();
}

long last_time=0;
int time_step=20;
bool in_dead_zone[]={0,0,0,0}; //0=not, 1=in

void loop() {
  long elapsed = millis() - start;
  
  if (elapsed-last_time>time_step){
    last_time=elapsed;
    for (int i=0;i<total_legs;i++){
      float desired_pos=get_desired_angle(i,elapsed);
      if (Directions[i]==1) desired_pos=360.0-desired_pos;
       
      print_position(elapsed, i,desired_pos);
      desired_pos = desired_pos + Leg_zeroing_offset[i];
      
      // FIX 2: WRAP IS UNCOMMENTED! This is vital so the offset doesn't break the dead-zone logic
      desired_pos = wrap360(desired_pos);

      if (in_dead_zone[i]==0){
        
        if (desired_pos<300) {
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        } else {
          in_dead_zone[i]=1;

          float present_speed=dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          
          delay(10);
          if(Directions[i]==0)
            dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
          else
            dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
        }
      }
      else {
        int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        bool emerged = false;
        
        // FIX 3: "WAIT FOR CLOCK" LOGIC to prevent XL-320 windshield wiping
        if(Directions[i]==0) {
            emerged = (current_pos > 5 && current_pos < 90);
        } else {
            emerged = (current_pos < 295 && current_pos > 210);
        }
          
        if (emerged) {
            if (desired_pos >= 300) {
                // The servo raced ahead of the clock! Stop it and wait.
                if (Directions[i] == 0) dxl.setGoalVelocity(IDs[i], 0);
                else dxl.setGoalVelocity(IDs[i], 1024);
            } else {
                // Clock has caught up! Hand control cleanly back to Position Mode.
                in_dead_zone[i]=0;
                dxl.torqueOff(IDs[i]);
                dxl.setOperatingMode(IDs[i], OP_POSITION);
                dxl.torqueOn(IDs[i]);
                delay(10);
                dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
            }
        } else {
            // Still in the blind spot, keep spinning to cross it
            if (Directions[i]==0)
                dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
            else
                dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
        }
      }
    }
  }
}