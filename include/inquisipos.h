
#ifndef INQUISIPOD_H
#define INQUISIPOD_H

void setServo(int servonum, int position);
void transactServos();
void commitServos();
void save_trims();
void erase_trims();
void setLeg(int legmask, int hip_pos, int knee_pos, int adj);
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw);
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw, int leanangle);
void setHipRaw(int leg, int pos);
void setHip(int leg, int pos);
void setHip(int leg, int pos, int adj);
void setKnee(int leg, int pos);
void setGrip(int elbow, int claw);
void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod);
void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod, int leanangle);
void stand();
void stand_90_degrees();
void laydown();
void tiptoes();
void wave(int dpad);
void gait_sidestep(int left, long timeperiod);
void griparm_mode(char dpad);
void fight_mode(char dpad, int mode, long timeperiod);
void gait_command(int gaittype, int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, int leanangle, int timeperiod);
void gait_tripod(int reverse, int hipforward, int hipbackward, 
          int kneeup, int kneedown, long timeperiod);
void gait_tripod(int reverse, int hipforward, int hipbackward, 
                int kneeup, int kneedown, long timeperiod, int leanangle);
void gait_tripod_scamper(int reverse, int turn);
void gait_ripple(int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod);
void gait_ripple(int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod, int leanangle);
void random_gait(int timingfactor);
void foldup();
void dance_dab(int timingfactor);
void flutter();
void dance_ballet(int dpad);
void dance_hands(int dpad);
void dance(int legs_up, int submode, int timingfactor);
void boogie_woogie(int legs_flat, int submode, int timingfactor);
void checkForCrashingHips();
void attach_all_servos();
void detach_all_servos();
void resetServoDriver();
void checkForServoSleep();
void beep(int f, int t);
void beep(int f);
void sendSensorData();
uint32_t bluewriteword(int w);
uint32_t readUltrasonic();
void sendSensorData();
void packetErrorChirp(char c);
void processPacketData();
int receiveDataHandler();
void dumpPacket();
void printTaskInfo(xTaskHandle xTaskToQuery);


#endif