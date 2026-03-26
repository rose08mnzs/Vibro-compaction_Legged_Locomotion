

#include <DynamixelShield.h>
#include <stdlib.h>
#include <math.h>

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

uint8_t IDs[]={2,3,4,30}; // Leg ID corresponding to [RF,LF,LR,RR] legs.  if not changed, leg RF= ID 1, leg LF= ID 2, leg LR= ID 3, leg RR= ID 4; 
uint8_t Directions[]={0,1,1,0}; // Leg rotating direction corresponding to [RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; If not changed, leg RF and leg RR will rotate CCW, and leg LF and leg LR will rotate CW; 
float gait[]={0.5,0.25,0.75}; // (\phi_1, \phi_2, \phi_3) = [LF-RF,LR-RF,RR-RF]; 
float gait_deg[]={0,0,180,180};  //gait in deg; [RF,LF,LR,RR]; the value for RF will always be 0, the value for LF= 360*\phi_1, LR=360*\phi_2, RR=360*\phi_3; calculated in translate_gait_deg();

int Leg_zeroing_offset[]={60,60,60,60}; //zeroing calibration offset in deg; leg angular position, ϕ, is defined as the angle measured clockwise about the axle from the upward vertical to the leg position, in radians. A zeroing calibration offset is needed because the servo’s zero position is not vertically downward and there is the offset between the servo-leg connector and the servo.  
                                        //by default is 60, which means when position command is 0, all legs should be pointing vertically upward.  

// NEW FOR PROJECT
float vibrate_duration = 0.5; // seconds to pause and vibrate
float vibrate_amplitude = 5.0; // degrees of amplitude for the vibration
float vibrate_frequency = 10.0; // Hz (cycles per second)
float time_vibrate_end; // will be calculated in clock_init()
// END FOR PROJECT
//clock parameters belows need to be calculated in clock_init()///////////////////////////////////////////////
float time_slow_start=1; // in seconds, time after the start of the period that the the leg enters the slow phase.
float time_slow_end=3; // in seconds, time after the start of the period that the the leg exits the slow phase.
float degree_slow_start=150; //in deg, the slow phase starting position
float degree_slow_end=210; //in deg, the slow phase starting position, if degree_slow_start=150 and degree_slow_end=210, this means whenever the leg's position is between 150 and 210, it will be in the slow phase.  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//You can change the desired Buehler clock parameters here

// float phi_s=0.85; //in rad, ϕ_s is the angular extent of the slow phase 
// float phi_0=3.27; //in rad, ϕ_0 is the center of the slow phase  
// float d_c=0.56;//d_c is the duty factor of the slow phase (i.e. fraction of the period spent in the slow phase).  
// float clock_period=4; //in seconds, time to complete 1 rotation

float phi_s=1.1; //in rad, ϕ_s is the angular extent of the slow phase
float phi_0=2.64; //in rad, ϕ_0 is the center of the slow phase 
float d_c=0.45;//d_c is the duty factor of the slow phase (i.e. fraction of the period spent in the slow phase). 
float clock_period=4; //in seconds, time to complete 1 rotation

const float RAD2DEG = 180/3.1415;

//change code to return desired speed in slow phase
float omega_slow(){ 
  return RAD2DEG* phi_s/(d_c*clock_period); //FIXME ;  //return desired leg speed in slow phase in deg/s
}


//change code to return desired speed in fast phase
float omega_fast(){  
    
  
  return RAD2DEG * (2*3.14 - phi_s)/((1-d_c)*clock_period); //FIXME ; //return desired leg speed in fast phase in deg/s
}


void clock_init() {
  float slow_start_deg = (phi_0 - phi_s/2) * RAD2DEG;
  float slow_end_deg   = (phi_0 + phi_s/2) * RAD2DEG;

  // 1. Fast phase ends here
  time_slow_start   = slow_start_deg / omega_fast();        

  // 2. Vibration phase ends here (New!)
  // NEW FOR PROJECT
  time_vibrate_end  = time_slow_start + vibrate_duration; 

  // 3. Slow phase ends here (Pushed back by vibration duration)
  time_slow_end     = time_vibrate_end + (d_c * (clock_period - vibrate_duration));
  // END FOR PROJECT
  degree_slow_start = slow_start_deg;
  degree_slow_end   = slow_end_deg;
}

float get_desired_angle(int leg, long elapsed) { 
    float phase[4] = {0.0f, gait[0], gait[1], gait[2]}; 
    float time_sec = fmod(elapsed/1000.0f + phase[leg]*clock_period, clock_period);
    if (time_sec < 0) time_sec += clock_period;

    float angle;
    
    // Phase 1: Fast Phase (Swing)
    if(time_sec < time_slow_start){
      angle = omega_fast() * time_sec;
    }
    
    // Phase 2: Vibrate Phase (Pre-Stance Compaction)
    // NEW FOR PROJECT
    else if (time_sec < time_vibrate_end){
      // Calculate how far into the vibration phase we are
      float t_vib = time_sec - time_slow_start; 
      
      // Hold at the start position, but superimpose a high-frequency sine wave
      angle = degree_slow_start + vibrate_amplitude * sin(2 * 3.14159 * vibrate_frequency * t_vib);
    }
    // END FOR PROJECT
    
    // Phase 3: Slow Phase (Stance/Push)
    // NEW FOR PROJECT (PUSHED THE SLOW PHASE BACK)
    else if (time_sec < time_slow_end){
      // Resume slow phase smoothly from where we paused
      angle = degree_slow_start + omega_slow() * (time_sec - time_vibrate_end);
    }
    // END FOR PROJECT
    // Phase 4: Fast Phase (Return to top)
    else{
      angle = degree_slow_end + omega_fast() * (time_sec - time_slow_end) ;
    }
    
    float desired_angle = angle + Leg_zeroing_offset[leg];
    desired_angle = fmod(desired_angle, 360.0);
    return desired_angle; 
}
// print desired motor position and associated elapsed time to debug serial
void print_position(long t, int leg, float desired_pos){
     
  // Serial.print("Motor ");
  DEBUG_SERIAL.print(leg);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(t);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println(desired_pos);        
  return 0;
}


void translate_gait_deg(){ //calculate the value of the array 'gait_deg[]' 
  //notice the unit of 'gait_deg[]' is in deg and has 4 elements
  gait_deg[0]=0;  //RF
  gait_deg[1]=gait[0]*360; //LF
  gait_deg[2]=gait[1]*360; //LB
  gait_deg[3]=gait[2]*360; //RB
  
   return;
}

int dead_zone_speed_tuning=-10; //adjustment to tune deadzone speed, MAGIC Variable as we are using position control outside the deadzone and speed control inside deadzone so the speed might be different; this variable is manually selected from observation, and it's ok that it's not working well.  
//int different_direction_offset=-110; //adjustment to compensate position offset between 2 legs with different rotating direction. Another MAGIC variable that requires manual observation. 


////////////////////////////////////////////// The code below should require none or minimal changes /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


long start;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("HELLO WORLD!");
  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  bool ok;
  for(int i=0; i<total_legs;i++)
  {
    ok = dxl.ping(IDs[i]);
  if (!ok) {
    DEBUG_SERIAL.println("PING FAILED: no reply (ID/protocol/baud/wiring/power)");
    while (1);
  }
  delay(1000);
  }
  // Turn off torque when configuring items in EEPROM area
  for (int i=0;i<=0;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
  clock_init();
  translate_gait_deg(); 
}


long last_time=0;
int time_step=100;
bool in_dead_zone[]={0,0,0,0}; //0=not, 1=in



void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  long elapsed = millis() - start;
  
  
  if (elapsed-last_time>time_step){
    last_time=elapsed;
    for (int i=0;i<total_legs;i++){
      float desired_pos=get_desired_angle(i,elapsed);
      if (Directions[i]==1) desired_pos=360.0-desired_pos;
        
      print_position(elapsed, i,desired_pos);
      
      if (in_dead_zone[i]==0){

        
        if (desired_pos<300)
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        else{
          in_dead_zone[i]=1;

          float present_speed=dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          if(Directions[i]==0)
            dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
          else
            dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
          
        }
        delay(10);   
      }
      else{
        int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        bool flag_temp=0;
        
        
        if(Directions[i]==0)
            flag_temp=current_pos>20;
         else
            flag_temp=current_pos<280;
          
        
        if (flag_temp && desired_pos>0 && desired_pos<300){
          in_dead_zone[i]=0;
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_POSITION);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        }
        
      }
    }
  }
  
}
