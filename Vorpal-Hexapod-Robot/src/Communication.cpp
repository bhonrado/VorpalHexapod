#include "Communication.h"

unsigned char packetData[MAXPACKETDATA];
unsigned int packetLength = 0;
unsigned int packetLengthReceived = 0;
int packetState = P_WAITING_FOR_HEADER;

SoftwareSerial BlueTooth(3,2);  // Bluetooth pins: TX=3=Yellow wire,  RX=2=Green wire

byte lastCmd = 's';
byte priorCmd = 0;

// write out a word, high byte first, and return checksum of two individual bytes
unsigned int bluewriteword(int w) {
  unsigned int h = highByte(w);
  BlueTooth.write(h);
  unsigned int l = lowByte(w);
  BlueTooth.write(l);
  return h+l;
}


void packetErrorChirp(char c) {
  beep(70,8);
  Serial.print(" BTER:"); Serial.print(packetState); Serial.print(c);
  //Serial.print("("); Serial.print(c,DEC); Serial.print(")");
  Serial.print("A"); Serial.println(BlueTooth.available());
  packetState = P_WAITING_FOR_HEADER; // reset to initial state if any error occurs
}


int receiveDataHandler() {

  while (BlueTooth.available() > 0) {
    unsigned int c = BlueTooth.read();

    // uncomment the following lines if you're doing some serious packet debugging, but be aware this will take up so
    // much time you will drop some data. I would suggest slowing the gamepad/scratch sending rate to 4 packets per
    // second or slower if you want to use this.
#if 0
unsigned long m = millis();
//Serial.print(m);
Serial.print("'"); Serial.write(c); Serial.print("' ("); Serial.print((int)c); 
//Serial.print(")S="); Serial.print(packetState); Serial.print(" a="); Serial.print(BlueTooth.available()); Serial.println("");
//Serial.print(m);
Serial.println("");
#endif
    
    switch (packetState) {
      case P_WAITING_FOR_HEADER:
        if (c == 'V') {
          packetState = P_WAITING_FOR_VERSION;
          //Serial.print("GOT V ");
        } else if (c == '@') {  // simplified mode, makes it easier for people to write simple apps to control the robot
          packetState = P_SIMPLE_WAITING_FOR_DATA;
          packetLengthReceived = 0; // we'll be waiting for exactly 3 bytes like 'D1b' or 'F3s'
          //Serial.print("GOT @");
        } else {
          // may as well flush up to the next header
          int flushcount = 0;
          while (BlueTooth.available()>0 && (BlueTooth.peek() != 'V') && (BlueTooth.peek() != '@')) {
            BlueTooth.read(); // toss up to next possible header start
            flushcount++;
          }
          Serial.print("F:"); Serial.print(flushcount);
          packetErrorChirp(c);
        }
        break;
      case P_WAITING_FOR_VERSION:
        if (c == '1') {
          packetState = P_WAITING_FOR_LENGTH;
          //Serial.print("GOT 1 ");
        } else if (c == 'V') {
          // this can actually happen if the checksum was a 'V' and some noise caused a
          // flush up to the checksum's V, that V would be consumed by state WAITING FOR HEADER
          // leaving the real 'V' header in position 2. To avoid an endless loop of this situation
          // we'll simply stay in this state (WAITING FOR VERSION) if we see a 'V' in this state.

          // do nothing here
        } else {
          packetErrorChirp(c);
          packetState = P_WAITING_FOR_HEADER; // go back to looking for a 'V' again
        }
        break;
      case P_WAITING_FOR_LENGTH:
        { // need scope for local variables
            packetLength = c;
            if (packetLength > MAXPACKETDATA) {
              // this can happen if there's either a bug in the gamepad/scratch code, or if a burst of
              // static happened to hit right when the length was being transmitted. In either case, this
              // packet is toast so abandon it.
              packetErrorChirp(c);
              Serial.print("Bad Length="); Serial.println(c);
              packetState = P_WAITING_FOR_HEADER;
              return 0;
            }
            packetLengthReceived = 0;
            packetState = P_READING_DATA;

            //Serial.print("L="); Serial.println(packetLength);
        }
        break;
      case P_READING_DATA:
        if (packetLengthReceived >= MAXPACKETDATA) {
          // well this should never, ever happen but I'm being paranoid here.
          Serial.println("ERROR: PacketDataLen out of bounds!");
          packetState = P_WAITING_FOR_HEADER;  // abandon this packet
          packetLengthReceived = 0;
          return 0;
        }
        packetData[packetLengthReceived++] = c;
        if (packetLengthReceived == packetLength) {
          packetState = P_WAITING_FOR_CHECKSUM;
        }
        //Serial.print("CHAR("); Serial.print(c); Serial.print("/"); Serial.write(c); Serial.println(")");
        break;

      case P_WAITING_FOR_CHECKSUM:

        {
          unsigned int sum = packetLength;  // the length byte is part of the checksum
          for (unsigned int i = 0; i < packetLength; i++) {
            // uncomment the next line if you need to see the packet bytes
            //Serial.print(packetData[i]);Serial.print("-");
            sum += packetData[i];
          }
          sum = (sum % 256);

          if (sum != c) {
            packetErrorChirp(c);
            Serial.print("CHECKSUM FAIL "); Serial.print(sum); Serial.print("!="); Serial.print((int)c);
            Serial.print(" len=");Serial.println(packetLength);
            packetState = P_WAITING_FOR_HEADER;  // giving up on this packet, let's wait for another
          } else {
            LastValidReceiveTime = millis();  // set the time we received a valid packet
            processPacketData();
            packetState = P_WAITING_FOR_HEADER;
            //dumpPacket();   // comment this line out unless you are debugging packet transmissions
            return 1; // new data arrived!
          }
        }
        break;

        case P_SIMPLE_WAITING_FOR_DATA:
          packetData[packetLengthReceived++] = c;
          if (packetLengthReceived == 3) {
              // at this point, we're done no matter whether the packet is good or not
              // so might as well set the new state right up front
              packetState = P_WAITING_FOR_HEADER;
              
             // this simple mode consists of an at-sign followed by three letters that indicate the
             // button and mode, such as: @W2f means walk mode two forward. As such, there is no
             // checksum, but we can be pretty sure it's valid because there are strong limits on what
             // each letter can be. The following large conditional tests these constraints
             if ( (packetData[0] != 'W' && packetData[0] != 'D' && packetData[0] != 'F') ||
                    (packetData[1] != '1' && packetData[1] != '2' && packetData[1] != '3' && packetData[1] != '4') ||
                    (packetData[2] != 'f' && packetData[2] != 'b' && packetData[2] != 'l' && packetData[2] != 'r' && 
                       packetData[2] != 'w' && packetData[2] != 's')) {
  
                        // packet is bad, just toss it.
                        return 0;
            } else {
              // we got a good combo of letters in simplified mode
              processPacketData();
              return 1;
            }
          }
          //Serial.print("CHAR("); Serial.print(c); Serial.print("/"); Serial.write(c); Serial.println(")");
          break;
    }
  }

  return 0; // no new data arrived
}

void dumpPacket() { // this is purely for debugging, it can cause timing problems so only use it for debugging
  Serial.print("DMP:");
  for (unsigned int i = 0; i < packetLengthReceived; i++) {
    Serial.write(packetData[i]); Serial.print("("); Serial.print(packetData[i]); Serial.print(")");
  }
  Serial.println("");
}


void processPacketData() {
  unsigned int i = 0;
  while (i < packetLengthReceived) {
    switch (packetData[i]) {
      case 'W': 
      case 'F':
      case 'D':
        // gamepad mode change
        if (i <= packetLengthReceived - 3) {

          mode = packetData[i];
          submode = packetData[i+1];
          lastCmd = packetData[i+2];
          //Serial.print("GP="); Serial.write(mode);Serial.write(submode);Serial.write(lastCmd);Serial.println("");
          i += 3; // length of mode command is 3 bytes
          continue;
        } else {
          // this is an error, we got a command that was not complete
          // so the safest thing to do is toss the entire packet and give an error
          // beep
          beep(BF_ERROR, BD_MED);
          Serial.println("PKERR:M:Short");
          return;  // stop processing because we can't trust this packet anymore
        }
        break;
      case 'B':   // beep
        if (i <= packetLengthReceived - 5) {
            int honkfreq = word(packetData[i+1],packetData[i+2]);
            int honkdur = word(packetData[i+3],packetData[i+4]);
            // eventually we should queue beeps so scratch can issue multiple tones
            // to be played over time.
            if (honkfreq > 0 && honkdur > 0) {
              Serial.println("Beep Command");
              beep(honkfreq, honkdur);    
            }  
            i += 5; // length of beep command is 5 bytes
        } else {
          // again, we're short on bytes for this command so something is amiss
          beep(BF_ERROR, BD_MED);
          Serial.print("PKERR:B:Short:");Serial.print(i);Serial.print(":");Serial.println(packetLengthReceived);
          return;  // toss the rest of the packet
        }
        break;
        
      case 'R': // Raw Servo Move Command (from Scratch most likely)

#define RAWSERVOPOS 0
#define RAWSERVOADD 1
#define RAWSERVOSUB 2
#define RAWSERVONOMOVE 255
#define RAWSERVODETACH 254
        // Raw servo command is 18 bytes, command R, second byte is type of move, next 16 are all the servo ports positions
        // note: this can move more than just leg servos, it can also access the four ports beyond the legs so
        // you could make active attachments with servo motors, or you could control LED light brightness, etc.
        // move types are: 0=set to position, 1=add to position, 2=subtract from position
        // the 16 bytes of movement data is either a number from 1 to 180 meaning a position, or the
        // number 255 meaning "no move, stay at prior value", or 254 meaning "cut power to servo"
        if (i <= packetLengthReceived - 18) {
            //Serial.println("Got Raw Servo with enough bytes left");
            int movetype = packetData[i+1];
            //Serial.print(" Movetype="); Serial.println(movetype);
            for (int servo = 0; servo < 16; servo++) {
              int pos = packetData[i+2+servo];
              if (pos == RAWSERVONOMOVE) {
                //Serial.print("Port "); Serial.print(servo); Serial.println(" NOMOVE");
                continue;
              }
              if (pos == RAWSERVODETACH) {
                    servoDriver.setPin(servo,0,false); // stop pulses which will quickly detach the servo
                    //Serial.print("Port "); Serial.print(servo); Serial.println(" detached");
                    continue;
              }
              if (movetype == RAWSERVOADD) {
                pos += ServoPos[servo];
              } else if (movetype == RAWSERVOSUB) {
                pos = ServoPos[servo] - pos;
              }
              pos = constrain(pos,0,180);
              //Serial.print("Servo "); Serial.print(servo); Serial.print(" pos "); Serial.println(pos);
              ServoPos[servo] = pos;
            }
            checkForCrashingHips();  // make sure the user didn't do something silly
            for (int servo = 0; servo < 12; servo++) {
               setServo(servo, ServoPos[servo]);               
            }
            i += 18; // length of raw servo move is 18 bytes
            mode = MODE_LEG;  // suppress auto-repeat of gamepad commands when this is in progress
            startedStanding = -1; // don't allow sleep mode while this is running
        } else {
          // again, we're short on bytes for this command so something is amiss
          beep(BF_ERROR, BD_MED);
          Serial.print("PKERR:R:Short:");Serial.print(i);Serial.print(":");Serial.println(packetLengthReceived);
          return;  // toss the rest of the packet
        }
        break;

     case 'G': // Gait command (coming from Scratch most likely). This command is always 10 bytes long
               // params: literal 'G', 
               //         Gait type: 0=tripod, 1=turn in place CW from top, 2=ripple, 3=sidestep
               //         reverse direction(0 or 1)
               //         hipforward (angle)
               //         hipbackward (angle), 
               //         kneeup (angle)
               //         kneedown(angle)
               //         lean (angle)    make the robot body lean forward or backward during gait, adjusts front and back legs
               //         cycle time (2 byte unsigned long)  length of time a complete gait cycle should take, in milliseconds
              if (i <= packetLengthReceived - 10) {
                 LastGgaittype = packetData[i+1];
                 LastGreverse = packetData[i+2];
                 LastGhipforward = packetData[i+3];
                 LastGhipbackward = packetData[i+4];
                 LastGkneeup = packetData[i+5];
                 LastGkneedown = packetData[i+6];
                 int lean = packetData[i+7];
                 LastGtimeperiod = word(packetData[i+8], packetData[i+9]);

                 LastGleanangle = constrain(lean-70,-70,70);  // lean comes in from 0 to 60, but we need to bring it down to the range -30 to 30

                 gait_command(LastGgaittype, LastGreverse, LastGhipforward, LastGhipbackward, LastGkneeup, LastGkneedown, LastGleanangle, LastGtimeperiod);

                 i += 10;  // length of command
                 startedStanding = -1; // don't sleep the legs during this command
              } else {
                  // again, we're short on bytes for this command so something is amiss
                  beep(BF_ERROR, BD_MED);
                  Serial.println("PKERR:G:Short");
                  return;  // toss the rest of the packet                
              }
              break;

      case 'L': // leg motion command (coming from Scratch most likely). This command is always 5 bytes long
        if (i <= packetLengthReceived - 5) {
           unsigned int knee = packetData[i+2];
           unsigned int hip = packetData[i+3];
           if (knee == 255) {
              knee = NOMOVE;
              Serial.println("KNEE NOMOVE");
           }
           if (hip == 255) {
            hip = NOMOVE;
            Serial.println("HIP NOMOVE");
           }
           unsigned int legmask = packetData[i+1];
           int raw = packetData[i+4];
           Serial.print("SETLEG:"); Serial.print(legmask,DEC); Serial.print("/");Serial.print(knee);
           Serial.print("/"); Serial.print(hip); Serial.print("/"); Serial.println(raw,DEC);
           setLeg(legmask, knee, hip, 0, raw);
           mode = MODE_LEG;   // this stops auto-repeat of gamepad mode commands
           i += 5;  // length of leg command
           startedStanding = -1; // don't sleep the legs when a specific LEG command was received
           if (ServosDetached) { // wake up any sleeping servos
            attach_all_servos();
           }
           break;
        } else {
          // again, we're short on bytes for this command so something is amiss
          beep(BF_ERROR, BD_MED);
          Serial.println("PKERR:L:Short");
          return;  // toss the rest of the packet
        }
        break;

      case 'T': // Trim command

      // The trim command is always just a single operator, either a DPAD button (f, b, l, r, s, w) or the 
      // special values S (save), E (erase), P (toggle pose), or R (reset temporarily to untrimmed stance).
      // The meanings are:
      // f    Raise current knee 1 microsecond
      // b    Lower current knee 1 microsecond
      // l    Move current hip clockwise
      // r    Move current hip counterclockwise
      // w    Move to next leg, the leg will twitch to indicate
      // s    Do nothing, just hold steady
      // S    Save all the current trim values
      // P    Toggle the pose between standing and adjust mode
      // R    Show untrimmed stance in the current pose
      // E    Erase all the current trim values
        if (i <= packetLengthReceived - 2) {
          
            unsigned int command = packetData[i+1];
            
            Serial.print("Trim Cmd: "); Serial.write(command); Serial.println("");

           i += 2;  // length of trim command
           startedStanding = -1; // don't sleep the legs when a specific LEG command was received
           mode = MODE_LEG;
           if (ServosDetached) { // wake up any sleeping servos
            attach_all_servos();
           }

          TrimInEffect = 1;   // by default we'll show trims in effect
          
           // Interpret the command received
           switch (command) {
            case 'f':
            case 'b':
              ServoTrim[TrimCurLeg+NUM_LEGS] = constrain(ServoTrim[TrimCurLeg+NUM_LEGS]+((command=='b')?-1:1),0,255);
              beep(300,30);
              break;

            case 'l':
            case 'r':
              ServoTrim[TrimCurLeg] = constrain(ServoTrim[TrimCurLeg]+((command=='r')?-1:1),0,255);
              beep(500,30);
              break;
                          
            case 'w':
              TrimCurLeg = (TrimCurLeg+1)%NUM_LEGS;
              setKnee(TrimCurLeg, 120);
              beep(100, 30);
              delay(500);  // twitch the leg up to give the user feedback on what the new leg being trimmed is
                           // this delay also naturally debounces this command a bit
              break;
            case 'R':
              TrimInEffect = 0;
              beep(100,30);
              break;
            case 'S':
              save_trims();
              beep(800,1000);
              delay(500);
              break;
            case 'P':
              TrimPose = 1 - TrimPose;  // toggle between standing (0) and adjust mode (1)
              beep(500,30);
              break;
            case 'E':
              erase_trims();
              beep(1500,1000);
              break;
              
            default:
            case 's':
              // do nothing.
              break;
           }

           // now go ahead and implement the trim settings to display the result
           for (int i = 0; i < NUM_LEGS; i++) {
            setHip(i, HIP_NEUTRAL);
            if (TrimPose == 0) {
              setKnee(i, KNEE_STAND);
            } else {
              setKnee(i, KNEE_NEUTRAL);
            }
           }
           break;
        } else {
          // again, we're short on bytes for this command so something is amiss
          beep(BF_ERROR, BD_MED);
          Serial.println("PKERR:T:Short");
          return;  // toss the rest of the packet
        }
        break;

        case 'P': // Pose command (from Scratch) sets all 12 robot leg servos in a single command
                  // special value of 255 means no change from prior commands, 254 means power down the servo
                  // This command is 13 bytes long: "P" then 12 values to set servo positions, in order from servo 0 to 11

            if (ServosDetached) { // wake up any sleeping servos
             attach_all_servos();
            }
            if (i <= packetLengthReceived - 13) {
              for (int servo = 0; servo < 12; servo++) {
                 unsigned int position = packetData[i+1+servo];
                 if (position < 0) {
                  position = 0;
                 } else if (position > 180 && position < 254) {
                  position = 180;
                 }
                 if (position < 254) {
                  ServoPos[servo] = position;

                  //Serial.print("POSE:servo="); Serial.print(servo); Serial.print(":pos="); Serial.println(position);
                 } else if (position == 254) {
                    // power down this servo
                    servoDriver.setPin(servo,0,false); // stop pulses which will quickly detach the servo
                    //Serial.print("POSE:servo="); Serial.print(servo); Serial.println(":DETACHED");
                 } else {
                    //Serial.print("POSE:servo="); Serial.print(servo); Serial.println(":pos=unchanged");
                 }
              }
              checkForCrashingHips();
              for (int servo = 0; servo < 12; servo++) {
                 setServo(servo, ServoPos[servo]);               
              }

              mode = MODE_LEG;   // this stops auto-repeat of gamepad mode commands
             
              i += 13;  // length of pose command
              startedStanding = -1; // don't sleep the legs when a specific LEG command was received

              break;
            } else {
              // again, we're short on bytes for this command so something is amiss
              beep(BF_ERROR, BD_MED);
              Serial.println("PKERR:P:Short");
              return;  // toss the rest of the packet
            }
            break;  // I don't think we can actually get here.

      case 'S':   // sensor request
        // CMUCam seems to require it's own power supply so for now we're not doing that, will get it
        // figured out by the time KS shipping starts.
        i++;  // right now this is a single byte command, later we will take options for which sensors to send
        sendSensorData();
        //////////////// TEMPORARY CODE ////////////////////
        // chirp at most once per second if sending sensor data, this is helpful for debugging
        if (0) {
          unsigned long t = millis()%1000;
          if (t < 110) {
            beep(2000,20);
          }
        }
        ////////////////////////////////////////////////////
        break;
      default:
          Serial.print("PKERR:BadSW:"); Serial.print(packetData[i]); 
          Serial.print("i=");Serial.print(i);Serial.print(" RCV="); Serial.println(packetLengthReceived);
          beep(BF_ERROR, BD_MED);
          return;  // something is wrong, so toss the rest of the packet
    }
  }
}


void sendSensorData() {

  unsigned int ultra = readUltrasonic(); // this delays us 20 milliseconds but we should still be well within timing constraints
  //uint16_t blocks = CmuCam5.getBlocks(1); // just return the largest object for now
  int blocks = 0; // comment out cmucam for now
  
  BlueTooth.print("V");
  BlueTooth.print("1");
  int length = 8;  //+blocks?10:0; // if there is a cmucam block, we need to add 10 more bytes of data
  unsigned int checksum = length;
  BlueTooth.write(length);
  //////////////////for testing only////////////////////////////////
  //int testword = 567; // for testing we will for now hard code the first sensor to a fixed value
  //checksum += bluewriteword(testword);
  //checksum += bluewriteword(testword);
  //checksum += bluewriteword(testword);
  /////////////////////////////////////////////////////////////////
  checksum += bluewriteword(analogRead(A3));
  checksum += bluewriteword(analogRead(A6));
  checksum += bluewriteword(analogRead(A7));
  checksum += bluewriteword(ultra);
  if (blocks > 0) {
    //checksum += bluewriteword(CmuCam5.blocks[0].signature);
    //checksum += bluewriteword(CmuCam5.blocks[0].x);
    //checksum += bluewriteword(CmuCam5.blocks[0].y);
    //checksum += bluewriteword(CmuCam5.blocks[0].width);
    //checksum += bluewriteword(CmuCam5.blocks[0].height);
  }
  
  checksum = (checksum%256);
  BlueTooth.write(checksum); // end with checksum of data and length   
  //Serial.println("Sens");

  startedStanding = millis(); // sensor commands are coming from scratch so suppress sleep mode if this comes in

}