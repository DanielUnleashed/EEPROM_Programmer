#include <Arduino.h>

/* 
  This program will control the programming and subsequent check of an AT28C64 EEPROM.
  It stores 8KB. The data will be passed through the Serial Port to the Arduino and it will
  store it inside the EEPROM.

  For the constrol of the multiple ports it will be used two cascaded SN74HC595 shift registers.
*/

/* The shift register controls:
  1st register: Qa = #CE  (Chip enable)
                Qb = #OE  (Output enable)
                Qc = x    (None)
                Qd = A0   (adresses of eeprom)
                ...
                Qh = A4

  2nd register: Qa = A5
                ...
                Qh = A12
*/

#define SRLCLK 2
#define RCLK 3
#define SER 4

#define _WE 10
const uint8_t IO_Ports[8] = {13, A1, A2, 5, 6, 7, 8, 9};

#define SERIAL_BLOCK_SIZE 60

void EEPROM_setAddress(uint16_t address, bool _outputEnable, bool _chipEnable = false){
  if(address >= 0x2000){
    Serial.println("ERROR: address is bigger than maximum 0x2000");
    return;
  }
  shiftOut(SER, SRLCLK, MSBFIRST, address>>5);
  shiftOut(SER, SRLCLK, MSBFIRST, (address<<3) | (_outputEnable<<1) | _chipEnable);
  
  digitalWrite(RCLK, LOW);
  digitalWrite(RCLK, HIGH);
  digitalWrite(RCLK, LOW);
}

/*
  Set as INPUT for read mode.
  Set as OUTPUT for write mode.
*/
void setIOPorts(uint8_t mode){
  static uint8_t currentMode = 0xFF;
  if(mode == currentMode) return;

  EEPROM_setAddress(0, true, true);
  for(uint8_t i = 0; i < 8; i++) pinMode(IO_Ports[i], mode);
  currentMode = mode;
}

uint8_t EEPROM_readByte(uint16_t add){
  EEPROM_setAddress(add, false);
  
  uint8_t data = 0;
  for(uint8_t i = 0; i < 8; i++) data = data | (digitalRead(IO_Ports[i]) << i);

  return data;
}

void EEPROM_read(uint16_t from, uint16_t to){
  if(from > to){
    uint16_t temp = from;
    from = to;
    to = temp;
  }

  setIOPorts(INPUT);

  Serial.print("reading");

  for(uint16_t add = from; add <= to; add++){
    char out[2];
    sprintf(out, "%02X", EEPROM_readByte(add));
    Serial.print(out);
  }

  Serial.print("done");
  EEPROM_setAddress(0, true, true);
}

void EEPROM_writeByte(uint16_t address, uint8_t data){
  setIOPorts(INPUT);
  if(EEPROM_readByte(address) == data) return;

  setIOPorts(OUTPUT);
  EEPROM_setAddress(address, true);
  
  for(uint8_t i = 0; i < 8; i++) digitalWrite(IO_Ports[i], (data>>i)&1);
  
  digitalWrite(_WE, LOW);
  delayMicroseconds(1);
  digitalWrite(_WE, HIGH);
  delay(10);
}

void EEPROM_write(uint16_t from, uint16_t to){
  Serial.print("writing");

  uint16_t dir = from;
  while(dir < to){
    int remaindingBytes = to-dir;
    if(remaindingBytes < SERIAL_BLOCK_SIZE){
      while(Serial.available() < remaindingBytes){}
    }else{
      while(Serial.available() < SERIAL_BLOCK_SIZE){}
    }

    int cycles = remaindingBytes<SERIAL_BLOCK_SIZE ? remaindingBytes : SERIAL_BLOCK_SIZE;
    for(uint8_t i = 0; i < cycles; i++){
      EEPROM_writeByte(dir, Serial.read());
      dir++;
    }

    if((dir-from)%SERIAL_BLOCK_SIZE == 0) Serial.print("ok");
  }

  Serial.print("done");
  EEPROM_setAddress(0, true, true);
}

void EEPROM_erase(){
  Serial.print("erasing");

  for(uint16_t i = 0; i < 0x2000; i++){
    if(i%0x80 == 0){
      uint32_t progress = i*100/0x2000;
      Serial.print("Erased ");
      Serial.print(i, HEX);
      Serial.println(" (" + String(progress) + " %)");
    }
    
    setIOPorts(INPUT);
    if(EEPROM_readByte(i) == 0x00) continue;

    setIOPorts(OUTPUT);
    EEPROM_writeByte(i, 0x00);
  }

  Serial.print("done");
  EEPROM_setAddress(0, true, true);
}

void setup() {
  Serial.begin(115200);
  
  digitalWrite(SRLCLK, LOW);
  digitalWrite(RCLK, LOW);
  digitalWrite(SER, LOW);

  pinMode(SRLCLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SER, OUTPUT);

  digitalWrite(_WE, HIGH);
  pinMode(_WE, OUTPUT);

  EEPROM_setAddress(0, true, true);

  Serial.print("bootup");
}

String inputCommand = "";

bool executingCommand = false;

#define WRITE_COMMAND 0
#define READ_COMMAND 1
#define ERASE_COMMAND 2
#define WRITE_BYTE_COMMAND 3
#define READ_BYTE_COMMAND 4
#define NONE_COMMAND 0xFF
uint8_t commandID = NONE_COMMAND;

#define MAX_PARAMETERS 3
String parameters[MAX_PARAMETERS];

void loop() {
  if(Serial.available() > 0){
    char c = Serial.read();
    if(!executingCommand){
      inputCommand += c;
      if(c == '\n'){
        executingCommand = true;
        int index = inputCommand.indexOf("_");
        String commandString = "";
        if(index == -1){ //Method without parameters
          commandString = inputCommand.substring(0, inputCommand.length()-1);
        }else{
          commandString = inputCommand.substring(0, index);
          
          // Fetch parameters
          String parametersString = inputCommand.substring(index+1, inputCommand.length()-1);
          for(uint8_t i = 0; i < MAX_PARAMETERS; i++){
            int indexOfSpace = parametersString.indexOf(" ");
            if(indexOfSpace == -1){
              parameters[i] = parametersString;
              break;
            }
            parameters[i] = parametersString.substring(0, indexOfSpace);
            parametersString = parametersString.substring(indexOfSpace+1, parametersString.length());
          }
        }
        
        //Serial.println("Received command: " + commandString + " Params: " + parametersString);

        if(commandString.equals("write")) commandID = WRITE_COMMAND;
        else if(commandString.equals("read")) commandID = READ_COMMAND;
        else if(commandString.equals("erase")) commandID = ERASE_COMMAND;
        else if(commandString.equals("wb")) commandID = WRITE_BYTE_COMMAND;
        else if(commandString.equals("rb")) commandID = READ_BYTE_COMMAND;
        else{
          commandID = NONE_COMMAND;
          return;
        }
      }
    }
  }

  if(executingCommand){ // Header ended.
      if(commandID == WRITE_COMMAND){
        EEPROM_write(parameters[0].toInt(), parameters[1].toInt());
      }else if(commandID == READ_COMMAND){
        EEPROM_read(parameters[0].toInt(), parameters[1].toInt());
      }else if(commandID == ERASE_COMMAND){
        EEPROM_erase();
      }else if(commandID == WRITE_BYTE_COMMAND){
        setIOPorts(OUTPUT);
        uint16_t dir = parameters[0].toInt();
        for(char c : parameters[1]){
          if(c == ',') continue;
          Serial.print("Writing: " + String(c) +" (0x");
          Serial.print(c, HEX);
          Serial.println(") to " + String(dir));
          EEPROM_writeByte(dir++, c);
        }
      }else if(commandID == READ_BYTE_COMMAND){
        setIOPorts(INPUT);
        Serial.println(EEPROM_readByte(parameters[0].toInt()));
      }

      inputCommand = "";
      commandID = NONE_COMMAND;
      executingCommand = false;
      // Erase parameters array?
  }

}