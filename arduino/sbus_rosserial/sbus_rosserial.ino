#include <sbus.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

//===================================================================================================================

// used pins
#define SBUS_PIN 4   // D3
SBUS sbus;

int data[17];
int ch_1_min = 985;
int ch_2_min = 985;
int ch1;
int ch2;
int switch_1;
int switch_2;

int auto_threshold = 1000;
int tx_threshold = 1500;
int stop_threshold = 2000;
int last_value_1 = 0;
int last_value_2 = 0;

int last_switch_1 = 0;
int last_switch_2 = 0;

//===================================================================================================================

float vel_forward;
float angular_speed;

ros::NodeHandle_<ArduinoHardware, 1, 2, 32, 512> nh;

geometry_msgs::Twist cmd_vel;
diagnostic_msgs::KeyValue rc_status;

//ros::Publisher pub_cmd("/Diff_Drive/diff_drive_controller/cmd_vel", &cmd_vel);
ros::Publisher rc_cmd("rc_controller/cmd_vel", &cmd_vel);
ros::Publisher rc_diag("diagnostics/rc_status", &rc_status);
uint32_t seq;

//===================================================================================================================

void setup() {
  sbus.begin(SBUS_PIN, sbusBlocking);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rc_cmd);
  nh.advertise(rc_diag);
  seq = 0;
}

//===================================================================================================================

void loop() {
  while (true) {
    if (!sbus.waitFrame()) {
      ;
    }
  
    else {
      ch1 = sbus.getChannel(1);
      ch2 = sbus.getChannel(2);
      switch_1 = sbus.getChannel(5);
      switch_2 = sbus.getChannel(8);
      if ((abs(switch_1-last_switch_1)>100) && (abs(switch_2-last_switch_2)>100)) {
        last_switch_1 = switch_1;
        last_switch_2 = switch_2;
        continue;
      }
  
      
      if (sbus.signalLossActive() or sbus.failsafeActive()) {
        rc_status.key = "SIGNAL_LOSS";
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        rc_cmd.publish(&cmd_vel);
      }
      
  
        //rc_status.key = "RUNNING";
  
        if (abs(switch_1 - auto_threshold) < 250 && abs(switch_2 - auto_threshold) < 250) {
        }
        
        else if (abs(switch_1 - tx_threshold) < 10 && abs(switch_2 - tx_threshold) < 10) {
          if ((ch1 > ch_1_min) && (ch2 > ch_2_min) && (abs(ch1-last_value_1)<200) && (abs(ch2-last_value_2)<200)){
            vel_forward = (float)round(map(ch1, 1000, 2000, 0 , 2000) / 25) / 40;
            angular_speed = -(float)round(map(ch2, 1000, 2000, -30, 30) / 5) / 10;
            
          }      
          cmd_vel.linear.x = vel_forward;
          cmd_vel.angular.z = angular_speed;
          rc_cmd.publish(&cmd_vel); 
          last_value_1 = ch1;
          last_value_2 = ch2;   
          
        }
  
        else if (abs(switch_1 - stop_threshold) < 250 && abs(switch_2 - stop_threshold) < 250) {
          cmd_vel.linear.x  = 0;
          cmd_vel.angular.z = 0;
          rc_cmd.publish(&cmd_vel);
  
        }
      }
      //rc_diag.publish(&rc_status);
      
    //}
    delay(20);
    nh.spinOnce();
  }
}
