#include <SoftwareSerial.h>
#include <IRremote.h>
#include <Arduino.h>
#include <IRremote.hpp>
// #include "PinDefinitionsAndMore.h"


const int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;

class Motor {
  private:
    int pinLeft;
    int pinRight;
    String name;
  
  public:
    Motor(int pLeft, int pRight, String n) {
      pinLeft = pLeft;
      pinRight = pRight;
      name = n;
      // establish motor pins as output
      pinMode(pinLeft, OUTPUT);
      pinMode(pinRight, OUTPUT);
      stop();
    }

    turnOnLeft() {
      digitalWrite(pinRight, LOW);
      digitalWrite(pinLeft, HIGH);
    }

    turnOnRight() {
      digitalWrite(pinLeft, LOW);
      digitalWrite(pinRight, HIGH);
    }

    stop() {
      digitalWrite(pinLeft, HIGH);
      digitalWrite(pinRight, HIGH);
    }
};


class MotorController {
  private:
    Motor motor1;
    Motor motor2;
  
  public:
    MotorController(Motor m1, Motor m2) : motor1(m1), motor2(m2) {
      motor1.stop();
      motor2.stop();
    }

    void controlMotor(String motorName, String command) {
      if (motorName == "motor1") {
        if (command == "left") motor1.turnOnLeft();
        else if (command == "right") motor1.turnOnRight();
        else motor1.stop();
      }
      else if (motorName == "motor2") {
        if (command == "left") motor2.turnOnLeft();
        else if (command == "right") motor2.turnOnRight();
        else motor2.stop();
      }
    }
};



// class BluetoothController {
//   private:
//     SoftwareSerial bluetooth;
//     MotorController& motorController;
  
//   public:
//     BluetoothController(int txPin, int rxPin, MotorController& mController) : motorController(mController), bluetooth(SoftwareSerial(txPin, rxPin)) {}

//     void begin(long baudRate) {
//       bluetooth.begin(baudRate); // this actually starts the bluetooth communication
//     }

//     void update() {
//       if (bluetooth.available()) {
//         String command = bluetooth.readStringUntil('\n');
//         Serial.println(command);  
//         executeCommand(command);
//       }
//     }

//     void executeCommand(String command) {
//       if (command == "M1_UP") {
//         motorController.controlMotor("motor1", "up");
//       }
//       else if (command == "M1_DOWN") {
//         motorController.controlMotor("motor1", "down");
//       }
//       else if (command == "M1_STOP") {
//         motorController.controlMotor("motor1", "stop");
//       }
//       else if (command == "M2_UP") {
//         motorController.controlMotor("motor2", "up");
//       } 
//       else if (command == "M2_DOWN") {
//         motorController.controlMotor("motor2", "down");
//       }
//       else if (command == "M2_STOP") {
//         motorController.controlMotor("motor2", "stop");
//       }
//       else {
//         Serial.println("Some bluetooth connection came in that was weird");
//         return;
//       }
//     }
// };

Motor motor1(2, 3, "motor1");
Motor motor2(5, 6, "motor2");

MotorController motorController(motor1, motor2);

class IRReceiver {
  private:
    int irReceivePin;
  public:
    IRReceiver(int irPin): irReceivePin(irPin) {}

    void begin() {
      Serial.begin(9600);
      while (!Serial)
          ;
      // Just to know which program is running on my Arduino
      Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

      // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
      IrReceiver.begin(irReceivePin, true);

      Serial.print(F("Ready to receive IR signals of protocols: "));
      printActiveIRProtocols(&Serial);
      Serial.println(F("at pin "));
      Serial.println(irReceivePin);
    }

    void update() {
      if (IrReceiver.decode()) {
        /*
         * Check the code type and see if it is a known structure
         */
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
            // We have an unknown protocol here, print extended info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
            IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
        } else {
            IrReceiver.resume(); // Early enable receiving of the next IR frame
            // IrReceiver.printIRResultShort(&Serial);
            // IrReceiver.printIRSendUsage(&Serial);

            // Get the command from the decoded result
            unsigned long command = IrReceiver.decodedIRData.command;

            switch (command) {
              case 0xC:
                Serial.println("Printing 0xC or number 1");
                motorController.controlMotor("motor1", "left");
                break;
              case 0x18:
                Serial.println("Printing 0x18 or number 2");
                motorController.controlMotor("motor1", "stop");
                break;
              case 0x5E:
                Serial.println("Printing 0x5E or number 3");
                motorController.controlMotor("motor1", "right");
                break;
              case 0x8:
                Serial.println("Printing 0x8 or number 4");
                motorController.controlMotor("motor2", "left");
                break;
              case 0x1C:
                Serial.println("Printing 0x1C or number 5");
                motorController.controlMotor("motor2", "stop");
                break;
              case 0x5A:
                Serial.println("Button 6 pressed");
                motorController.controlMotor("motor2", "right");
                break;
              case 0x42:
                Serial.println("Button 7 pressed");
                motorController.controlMotor("motor1", "left");
                motorController.controlMotor("motor2", "left");
                break;
              case 0x52:
                Serial.println("Button 8 pressed");
                motorController.controlMotor("motor1", "stop");
                motorController.controlMotor("motor2", "stop");
                break;
              case 0x4A:
                Serial.println("Button 9 pressed");
                motorController.controlMotor("motor1", "right");
                motorController.controlMotor("motor2", "right");
                break;
              default:
                Serial.println("Got a command that was in the protocol but not a 1-9 number");
            };
        }
        Serial.println();
      }
      else {
        // Serial.println("Didn't receive anything on the RF signal");
        // delay(1000);
      }
    }
};

IRReceiver irr(11);
// BluetoothController bluetoothController(10,11, motorController);

void setup() {
  irr.begin();
  // motorController.controlMotor("motor1", "stop");
  // motorController.controlMotor("motor2", "stop");
  // delay(3000);
  // motorController.controlMotor("motor1", "left");
  // motorController.controlMotor("motor2", "left");
  // delay(2000);
  // motorController.controlMotor("motor1", "stop");
  // motorController.controlMotor("motor2", "stop");
  // delay(2000);
  // motorController.controlMotor("motor1", "right");
  // motorController.controlMotor("motor2", "right");
}

void loop() {
  irr.update();
}
