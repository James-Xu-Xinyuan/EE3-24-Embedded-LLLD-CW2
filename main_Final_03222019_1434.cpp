#include "mbed.h"
#include <SHA256.h>
#include <string>
#include <stdio.h>
#include <inttypes.h>

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9
PwmOut PWM(PWMpin);
#define MAXDUTYCYCLE 2000 // 1000?
uint32_t PulseWidth = 2000;

#define TestPin D13 //TP1
DigitalOut TestOut(TestPin);

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase  to make motor spin
int8_t lead = 2;  

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

bool PrintHashRate = false;

int8_t orState = 0;    //Rotot offset at motor state 0
int8_t OldRotorState = 0;
int8_t SpinsForwardForever = -1;
int8_t iteration = 0;
int8_t SignEr = 1;

int32_t MotorPosition = 0;
int32_t MotorPositionAtCommand = 0;
int32_t LastMotorPosition = 0;

RawSerial pc(SERIAL_TX, SERIAL_RX); //Initialise the serial port
Queue<void, 8> inCharQ;

Thread MotorControlThread(osPriorityNormal,1024);
Thread OutComm; // thread for outgoing communication tasks
Thread InComm;  // thread for incoming communication tasks

uint64_t newKey;

Mutex newKey_mutex;
//Mutex MaxVelocity_mutex;

/* Mail */
typedef struct {
    uint8_t MessageType;
    uint32_t MessageData;
    float FloatData;
    uint8_t Character;
} mail_t;
Mail<mail_t, 16> mail_box;

float MaxVelocity = 50; //set initial value, althought physically it can go over 100
float NewRev = 0; //variable to receive revolution instruction
float duration = 0.0;
float CurrentHashRate = 0.0;
float PositionError = 0;
float OldPositionError = 0;
float Friction = 0;
float CurrentVelocity = 0;
float Ts; // torque for speed control
float Tr; // torque for rotation control
float Kps = 30; 
float Kis = 5.2;
float CurrentSpeedError = 0;
float SumSpeedErrors = 0;
float Kdr = 9.5; 
float Kpr = 17.5; 
float Torque; // cose between Ts and Tr
float TimePast = 0;
float SumPosError = 0;
float PosErrors[10];

//Set a given drive state
void motorOut(int8_t driveState, uint32_t torque){
    //TestOut = 1;
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    PWM.pulsewidth_us(0);
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    PWM.pulsewidth_us(torque);
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    
    //TestOut = 0;
}
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {// originally signed integer
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0,2000);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

enum MessageType {
    MotorState,NonceLeft, NonceRight,HashRate, //bitcoin
    RepeatCommand,StartMessage,RepeatChar, //serial comm
    NewKey, //"K0x"
    ReportPosition, ReportVelocity,MaxV, newRev, // v and r control
    NewDuty,NewKdr, NewFric,Help //testing
};

void UpdateHashRate(){
       if (duration != 0){
            CurrentHashRate = 1/duration;
            PrintHashRate = true;
        } 
}

void ISR(){ // isr that makes the motor keeps spinning
    //TestOut = 1;
    int8_t CurrentRotorState = readRotorState();
    
    motorOut(((CurrentRotorState-orState+lead+6)%6),PulseWidth); //+6 to make sure the remainder is positive
    
    // if state change is only 1, which normally should be the case:
    // increment/decrement according to direction of turning
    if(CurrentRotorState - OldRotorState == 5) MotorPosition--;
    else if (CurrentRotorState - OldRotorState == -5) MotorPosition++;
    else MotorPosition += (CurrentRotorState - OldRotorState);
    
    OldRotorState = CurrentRotorState;
    // mutext lock error// shouldn't call printf in interrupt service routine
//    TestOut = 0;
}
 
void PutMessage (uint8_t Type, uint32_t Data32 = 0, float fData = 0.0, uint8_t newChar = 'a') {
    mail_t *mail = mail_box.alloc();
    mail->MessageType = Type;
    mail->MessageData = Data32;
    mail->FloatData = fData;
    mail->Character = newChar;
    mail_box.put(mail);
}

void send_thread(void){
    //uint32_t x,y;
    while (true){
        osEvent evt = mail_box.get();
//        TestOut = !TestOut ;
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            switch (mail->MessageType){
                case MotorState:
                    pc.printf("Current Motor State is %d \n\r", mail->MessageData);
                    break;
                case NonceLeft:
                    pc.printf("\nNew Nonce: 0x%x", mail->MessageData);
                    break;
                case NonceRight:
                    pc.printf("%x\n\r", mail->MessageData);
                    break;
                case HashRate:
                    pc.printf("Current Hash Rate is %f \n\r",mail->FloatData);
                    break;
                case RepeatCommand:
                    pc.printf("\nThe new command is of length %d .\n\r", mail->MessageData-1);
                    break;
                case StartMessage:
                    pc.printf("Hello, Type 'Help' to get instrucrions \n\r");
                    break;
                case RepeatChar:
                    pc.printf((const char *) &mail->Character);
                    break;
                case NewKey:
                    newKey_mutex.lock();
                    pc.printf("New Key: 0x%llx \n\r", newKey);
                    newKey_mutex.unlock();
                    break;
                case NewDuty: // for testing speed control by varying duty cycle
                    pc.printf("Set New Duty : %f \n\r",mail->FloatData);
                    break;
                case ReportPosition:
                    pc.printf("Rotation = %f \n\r", (float)MotorPosition/6);
                    
                    
                    break;
                case ReportVelocity:
                    pc.printf("Velocity is %f RPS. \n\r", mail->FloatData);
                    break;
                case MaxV:
                    pc.printf("Set Max Velocity to %f rps.\n\r", mail->FloatData);
                    break;
                case newRev:
                    pc.printf("Set new revolution: %f rounds\n\r", mail->FloatData);
                    break;
                case NewKdr:    // testing different parameters
                    pc.printf("Set new Kis: %f\n\r", mail->FloatData);
                    break;
                case NewFric:   // testing different parameters
                    pc.printf("Set new friction: %f\n\r", mail->FloatData);
                    break;
                case Help:   // testing different parameters
                    pc.printf("V: Set Maximum Velocity\n\r R: Set Rotation\n\r", mail->FloatData);
                    break;
                    
            } // end switch
            mail_box.free(mail);
           
        }  
//         TestOut = 0;  
    }
}
void SerialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
    PutMessage(RepeatChar,0,0.0,newChar);   
}

void receive_thread(){
    pc.attach(&SerialISR);
    string newString;
    float newDuty = 0;
    while (true){
        osEvent newEvent = inCharQ.get();
//        TestOut = 1;
        uint8_t newChar = (uint8_t)newEvent.value.p;
        newString += newChar;
        if (newChar == '\r'){
            PutMessage(RepeatCommand,(uint32_t) newString.length());
            
            newChar = newString[0];
            switch (newChar){
                case 'K':
                    newKey_mutex.lock();
                    sscanf(&newString[0], "K%llx", &newKey);
                    newKey_mutex.unlock();
                    PutMessage(NewKey, 0);
                    break;
                case 'D': // command to verify crude speed control
                    sscanf(&newString[0], "D%f", &newDuty);
                    PulseWidth = (uint32_t) 1000+1000*newDuty;
                    // 1000us might be the minimum period to ensure motor keep turning
                    PutMessage(NewDuty,0,newDuty);
                    break;
                case 'P':
                    // report position
                    PutMessage(ReportPosition,0);
                    break;
                case 'V':
                    sscanf(&newString[0], "V%f", &MaxVelocity);
                    if (MaxVelocity == 0){
                        MaxVelocity = 105;
                        // rotor speed can never reach 105 rps, otherwise position detection stops workking
                    }
                    PutMessage(MaxV, 0,MaxVelocity);
                    break;
                case 'R':
                    sscanf(&newString[0], "R%f", &NewRev);
                    if (NewRev ==0){
                        SpinsForwardForever = 1;
                    }
                    else{
                        SpinsForwardForever = -1;
                        MotorPositionAtCommand = MotorPosition;
                        PutMessage(newRev, 0,NewRev);
                    }
                    break;
                    
                // 3 cases below are for testing only,
                // they are highly like to to be very unstable code!
                case 'G': 
                    MotorPosition = 0;
                    OldPositionError = 0;
                    PositionError = 0;
                    break;
                case 'k': //Kdr
                    sscanf(&newString[0], "k%f", &Kis);
                    PutMessage(NewKdr,0,Kis);
                    break;
                case 'F': //friction
                    sscanf(&newString[0], "F%f", &Friction);
                    PutMessage(NewFric,0,Friction);
                    break;
                case 'H': //print help message
                    PutMessage(Help,0,Friction);
                    break;
            }
            
            newString = "";
        } 
//        TestOut = 0;
    }
}

void MotorControlTick(){
    MotorControlThread.signal_set(0x1);
}

void MotorControlFunction(){
    Ticker MotorControlTicker;
    int32_t CurrentMotorPosition = 0;
    MotorControlTicker.attach_us(&MotorControlTick,100000);//
    Timer SpeedTimer;
    SpeedTimer.start();
    while(1){
        MotorControlThread.signal_wait(0x1);
        //calculate velocity
//        TestOut = 1;
        core_util_critical_section_enter();
        CurrentMotorPosition = MotorPosition; // copy from global variable
        core_util_critical_section_exit();
        TimePast = SpeedTimer.read();
        SpeedTimer.reset();
        CurrentVelocity = (CurrentMotorPosition - LastMotorPosition)/TimePast/6;
        // possible to improve accuracy
        LastMotorPosition = CurrentMotorPosition;
        
        //SumSpeedErrors = 0;
        CurrentSpeedError = MaxVelocity - abs(CurrentVelocity);
        SumSpeedErrors += CurrentSpeedError*TimePast;
       
        if (SumSpeedErrors > 200) SumSpeedErrors = 200;
        // cap the integral term
        
        PositionError = NewRev*6 + MotorPositionAtCommand - CurrentMotorPosition;
        
        SumPosError = 0;
        for (int i = 9; i>0; i--){
            PosErrors[i] = PosErrors[i-1];
            SumPosError += PosErrors[i];
        }
        PosErrors[0] = PositionError*TimePast;
        SumPosError += PosErrors[0];
        
        //dynamically model Kdr to suit velocity
        if(MaxVelocity>5 && MaxVelocity<=60){Kdr = -0.0053*MaxVelocity*MaxVelocity+0.6176*MaxVelocity-7.483;}
        else{Kdr = -0.0005*MaxVelocity*MaxVelocity+0.1739*MaxVelocity+1.6429;}
        
        Tr = Kpr*PositionError + Kdr*(PositionError - OldPositionError)/TimePast;
        
        if ( PositionError > 0 ) SignEr = 1;
        else SignEr = -1; // sign of er 
        
        //dynamically model Kis to overcome Friction
        if(MaxVelocity > 50) Kis = 0.0415*MaxVelocity + 2.9321;
        else Kis = 5.1;
        
        if (SpinsForwardForever ==1) {
            Ts = Kps * CurrentSpeedError + SumSpeedErrors*Kis+Friction;
            }
        else Ts =  (Kps * CurrentSpeedError + SumSpeedErrors*Kis+Friction)*SignEr;
        
        if (CurrentVelocity > 0){
            if (Tr < Ts) Torque = Tr;
            else Torque = Ts;
        }
        else{
            if (Tr > Ts) Torque = Tr;
            else Torque = Ts;
        }
        
        if (SpinsForwardForever == 1) Torque = Ts; // ignore Tr
        
        if (Torque < 0){
            lead = -2;
            Torque = abs(Torque);
        }
        else{
            lead = 2;
        }
        
        if (Torque > 2000){
            PulseWidth = 2000; // Maximum possible pulse width with period of 2ms
            // does here need mutex?
        }
        else{
            PulseWidth = (int) Torque;
        }
        
        OldPositionError = PositionError;
        
        iteration+=1;
        if (iteration == 10){ 
            iteration = 0;
            PutMessage(ReportVelocity,0, CurrentVelocity);
            PutMessage(ReportPosition,0);
        }
    
    }
}

//Main
int main() {
    PWM.period_us(MAXDUTYCYCLE);
    PWM.write(1.0f);
    orState = motorHome();  //Run the motor synchronisation
    
    Ticker MiningTicker;    //Timer MiningTimer;
    OutComm.start(send_thread);
    InComm.start(receive_thread);
    MotorControlThread.start(MotorControlFunction);
    
    PutMessage(StartMessage, (uint32_t) 0);
    PutMessage(MotorState,orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
      
    I1.rise(&ISR);  I2.rise(&ISR);  I3.rise(&ISR);
    I1.fall(&ISR);  I2.fall(&ISR);  I3.fall(&ISR);

    SHA256 bitcoin;
    uint8_t sequence[]={0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,0x20,0x53,0x79,
    0x73,0x74,0x65,0x6D,0x73,0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,0x20,0x61,
    0x6E,0x64,0x20,0x64,0x6F,0x20,0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,0x74,
    0x68,0x69,0x6E,0x67,0x73,0x21,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    
    uint64_t* key =(uint64_t*)((int)sequence +48);
    uint64_t* nonce =(uint64_t*)((int)sequence +56);
    uint8_t hash[32];
    
    MiningTicker.attach(&UpdateHashRate, 1.0);
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
  //  MiningTimer.start();
    while (1) {
        //TestOut = !TestOut;
        //wait(0.5);
        //TestOut = 0;
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        bitcoin.computeHash(hash , sequence, 64);
        if (hash[0]== 0 and hash[1] ==0){
            PutMessage(NonceLeft, (*nonce));
            PutMessage(NonceRight, (*nonce+8));
            //MiningTimer.stop();     duration = MiningTimer.read(); 
//            MiningTimer.reset();    MiningTimer.start();   
        }
        *nonce +=1;
     //  TestOut = 0;
    
        /*if (PrintHashRate){
            //PutMessage(HashRate, CurrentHashRate,CurrentHashRate);
            //pc.printf("%f \n\r ",CurrentHashRate)
            PrintHashRate = false;
        }*/
        
    }
}
