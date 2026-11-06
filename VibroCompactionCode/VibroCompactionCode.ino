/*
 * Pre-Stance Vibro-Compaction for Legged Locomotion
 * Authors: Arad Firouzkouhi, Aryan Barik, Jacinta Menezes
 *
 * The final paper describes the experimental methods and reported results.
 * This controller was used with different parameter settings during testing;
 * a saved parameter value represents one trial configuration, not the full sweep.
 *
 * Repository defaults below select the paper's best tested base-incline condition:
 * 6 Hz, 3-degree commanded amplitude, and 1.4 seconds of vibration.
 * The supplied snapshot had duration=0.0, frequency=0.0, and amplitude=5.0.
 * Only documentation and those three default values have been changed here.
 * No new on-robot validation has been performed for this edited copy.
 *
 * Arduino sketch layout: Project/Project.ino (folder and filename must match).
 */

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

// Assembly-specific mapping: verify physical motor IDs and calibration before use.
uint8_t IDs[]={3,2,30,4}; // Leg ID corresponding to [RF,LF,LR,RR] legs. 
uint8_t Directions[]={1,0,0,1}; // Leg rotating direction corresponding to [RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; 
//Walk {0.5, 0.25, 0.75}
// Preserved original mapping: [RF, LF, LR, RR] phases = [0, 0.5, 0.25, 0.75].
// The paper lists [0, 0.25, 0.5, 0.75] without the same explicit leg ordering.
// Confirm the ordering from experiment records before changing these offsets.
float gait[]={0.5, 0.25, 0.75}; // (\phi_1, \phi_2, \phi_3) = [LF-RF,LR-RF,RR-RF]; 
float gait_deg[]={0,0,180,180};  //gait in deg
float leg_phase[4];
int Leg_zeroing_offset[]={52,69,64,55}; //zeroing calibration offset in deg

// --------------------
// EXPERIMENT CONFIGURATION
// --------------------
// Best tested condition in the final paper (base incline, BB-pellet substrate).
// Amplitude is the commanded sine amplitude in degrees, not peak-to-peak motion
// or a measurement of actual foot displacement. Re-upload after editing settings.
bool use_vibration = true;      // set to false for the no-vibration baseline
float vibrate_duration = 1.4;   // seconds; set to 0.0 for the no-vibration baseline
float vibrate_amplitude = 3.0;  // degrees; paper frequency-amplitude sweep: 2, 3, 4
float vibrate_frequency = 6.0;  // Hz; paper frequency-amplitude sweep: 6, 8, 10

// Other experiments reported in the paper:
// - Duration sweep: 0.2 to 1.4 s in 0.2 s steps, at 6 or 10 Hz and 2.5 degrees.
// - Frequency/amplitude sweep: duration fixed at 1.4 s.
// - Slope sweep: amplitude 3 degrees and duration 1.4 s, varying frequency/slope.
// Change the applicable variables for each condition and record every trial.
//
// No-vibration baseline: set BOTH use_vibration=false and vibrate_duration=0.0.
// Frequency may also be set to 0.0 for clarity; amplitude is then unused.
// Disabling only use_vibration does not remove the allocated vibration interval.
//
// Timing requirement: 0 <= vibrate_duration < clock_period * (1 - d_c).
// With period=4.0 s and d_c=0.45, the upper bound is 2.2 s (exclusive).
// This is a mathematical bound, not a demonstrated hardware operating limit.
// Settings remain constant during each run; this sketch does not automate sweeps.

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

// Legacy threshold declarations below are unused by the current loop.
// Actual mode switching uses the hard-coded conditions in loop().
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

// Vibration takes time from fast swing; slow stance retains d_c * clock_period.
// Keep the denominator positive by checking the configuration above.
// This function does not enforce valid parameter bounds.
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

// Calculate fast-swing, pre-stance vibration, and fixed-duration stance boundaries.
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
// Output fields: cycle-relative seconds, leg index, desired angle in degrees.
// This is commanded motion before applying the calibration offset, not measured
// robot displacement/slip. Extra loop debug messages are mixed into the stream.
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
  // On Uno/Mega, DEBUG_SERIAL uses SoftwareSerial pins 7/8 rather than USB Serial.
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Torque is enabled during setup; motion begins automatically in loop().
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
// Nominal scheduling interval only; printing, bus traffic, and delays add latency.
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
       DEBUG_SERIAL.print("Leg \t");
       DEBUG_SERIAL.println(i);
      desired_pos = desired_pos + Leg_zeroing_offset[i];
      
      // FIX 2: WRAP IS UNCOMMENTED! This is vital so the offset doesn't break the dead-zone logic
      desired_pos = wrap360(desired_pos);

      if (in_dead_zone[i]==0){
         DEBUG_SERIAL.print("In deadzone \t");
        if (desired_pos<300) {
          DEBUG_SERIAL.print("desired_pos<300 \t");
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        } else {
          DEBUG_SERIAL.print("else \t");
          in_dead_zone[i]=1;

          float present_speed=dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          
          delay(10);
          if(Directions[i]==0)
          {
            DEBUG_SERIAL.print("Directions[i]==0 \t");
            dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
          }
          else{
            DEBUG_SERIAL.print("Else Directions[i]==0 \t");
            dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
          }
        }
      }
      else {
        DEBUG_SERIAL.print("Elsr In deadzone \t");
        int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        bool emerged = false;
        
        // Retain original wait-for-clock logic for crossing the position blind zone.
        // This variant has no dead-zone timeout.
        if(Directions[i]==0) {
            DEBUG_SERIAL.print("Elsr In deadzone Directions[i]==0 \t");
            emerged = (current_pos > 5 && current_pos < 90);
        } else {
          DEBUG_SERIAL.print("Elsr In deadzone current_pos < 295 \t");
            emerged = (current_pos < 295 && current_pos > 210);
        }
          //print_position(i,desired_pos);
        if (emerged) {
           DEBUG_SERIAL.print("emerged \t");
            if (desired_pos >= 300) {
               DEBUG_SERIAL.print("els e desired_pos >= 300 \t");
                // The servo raced ahead of the clock! Stop it and wait.
                if (Directions[i] == 0) 
                {
                  DEBUG_SERIAL.print("emerged Directions[i] == 0 \t");
                  dxl.setGoalVelocity(IDs[i], 0);
                }
                else{
                   DEBUG_SERIAL.print("else emerged Directions[i] == 0 \t");
                   dxl.setGoalVelocity(IDs[i], 1024);
                }
            } else {
                // Clock has caught up! Hand control cleanly back to Position Mode.
                DEBUG_SERIAL.print("in else 2 \t");
                in_dead_zone[i]=0;
                dxl.torqueOff(IDs[i]);
                dxl.setOperatingMode(IDs[i], OP_POSITION);
                dxl.torqueOn(IDs[i]);
                delay(10);
                dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
            }
        } else {
            // Still in the blind spot, keep spinning to cross it
            DEBUG_SERIAL.print("in else 3 \t");
            if (Directions[i]==0)
            {
                DEBUG_SERIAL.print("3 Directions[i]==0 \t");
                dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
            }
            else{
              DEBUG_SERIAL.print("3 setGoalVelocity \t");
              dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
            }
        }
      }
    }
  }
}