#include <TEA5767.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1015.h>

/* Transmitter */
Adafruit_MCP4725 dac;

#define baud 250
#define defaultFreq 1700
#define freq0 500
#define freq1 750
#define freq2 1000
#define freq3 1250
const uint16_t S_DAC[4] = {2047, 4095, 2047, 0};
int delay0, delay1, delay2, delay3;
char inData[30];

/* Receiver */
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#define r_slope 30

int keep = 0;
int sum = 0;
int max = 0;
int prev = 0;
int count = 0;
int nub = 0;
int check = false, first = true;
uint8_t dis = 0;
unsigned long times;
int eiei = 3975;
TEA5767 radio = TEA5767();

float frequency = 104.7;//98.2;//102.8 //Enter your own Frequency

/* Make Data */
uint8_t chars[29];
uint8_t charsLen;
uint8_t color[16];
uint8_t width[16];
uint16_t height[16];
uint8_t dataT[2][25];
uint8_t receive[29];
uint8_t receiveLen;

char pint[200];

void setCharsZero() {
  for (int i = 0; i < 29; i++) {
    chars[i] = 0;
  }
  charsLen = 0;
}

void setReceiveZero() {
  for (int i = 0; i < 29; i++) {
    receive[i] = 0;
  }
  receiveLen = 0;
}

void setDataTZero() {
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 25; j++) {
      dataT[i][j] = 0;
    }
  }
}

void setDataZero() {
  for (int i = 0; i < 16; i++) {
    color[i] = 0;
    width[i] = 0;
    height[i] = 0;
  }
}

void encoder() {
  setDataTZero();
  for (int i = 0; i < 2; i++) {
    uint8_t remain = 0;
    for (int j = 0; j < 8; j++) {
      dataT[i][j * 3] = height[j + (8 * i)] & 0xFF;
      dataT[i][(j * 3) + 1] = width[j + (8 * i)];
      dataT[i][(j * 3) + 2] = color[j + (8 * i)];
      remain <<= 1;
      remain |= (height[j + (8 * i)] & 0x100) >> 8;
    }
    dataT[i][24] = remain;
  }
}

void decoder() {
  setDataZero();
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 8; j++) {
      height[j + (8 * i)] = dataT[i][j * 3] | (((dataT[i][24] & (1 << (7 - j))) >> (7 - j)) << 8);
      width[j + (8 * i)] = dataT[i][(j * 3) + 1];
      color[j + (8 * i)] = dataT[i][(j * 3) + 2];
    }
  }
}

void printCode() {
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 25; i++) {
      if (i % 5 == 0)
        Serial.println();
      Serial.print(dataT[j][i], BIN);
      if (dataT[j][i] > 127)
        Serial.print("\t");
      else
        Serial.print("\t\t");
    }
    Serial.println();
  }
}

void generateCode() {
  for (int i = 0; i < 16; i++) {
    height[i] = 20 * i;
    width[i] = 15 * i;
    color[i] = 16 * i;
    sprintf(pint, "%X %X %X", height[i], width[i], color[i]);
    Serial.println(pint);
  }
}

uint8_t makeSum(uint8_t header, uint8_t num) {
  uint16_t sum = header;
  for (int i = 0; i < 25; i++) {
    sum += dataT[num][i];
  }
  while ((sum & 0xFF00) != 0) {
    sum = (sum & 0x00FF) + (sum >> 8);
  }
  return ~(0x00FF & sum);
}

bool checkSum() {
  uint16_t sum = 0;
  for (int i = 1; i < 27; i++) {
    sum += receive[i];
  }
  while ((sum & 0xFF00) != 0) {
    sum = (sum & 0x00FF) + (sum >> 8);
  }
  sum += receive[27];
  while ((sum & 0xFF00) != 0) {
    sum = (sum & 0x00FF) + (sum >> 8);
  }
  return (sum == 0xFF) ? true : false;
}

uint8_t makeParity(uint8_t header) {
  uint8_t parity = 0;
  for (int i = 0; i < 7; i++) {
    parity ^= (header & (1 << (7 - i))) >> (7 - i);
  }
  header |= parity;
  return header;
}

bool checkParity(uint8_t header) {
  uint8_t parity = 0;
  for (int i = 0; i < 8; i++) {
    parity ^= header & 1;
    header >>= 1;
  }
  return (parity) ? false : true;
}

void makeData(String header, uint8_t seq_No, uint8_t size_No, uint8_t num) {
  setCharsZero();
  //Serial.println(header);
  if (header == "request") {
    chars[0] = B10011001;
    chars[1] = B00000000 + (seq_No << 4) + (size_No << 1);
    chars[1] = makeParity(chars[1]);
    chars[2] = B10011001;
    charsLen = 3;
  }
  else if (header == "ACK") {
    chars[0] = B10011001;
    chars[1] = B01000000 + (seq_No << 4);
    chars[1] = makeParity(chars[1]);
    chars[2] = B10011001;
    charsLen = 3;
  }
  else if (header == "NAK") {
    chars[0] = B10011001;
    chars[1] = B01100000 + (seq_No << 4);
    chars[1] = makeParity(chars[1]);
    chars[2] = B10011001;
    charsLen = 3;
  }
  else if (header == "ask") {
    chars[0] = B10011001;
    chars[1] = B10000000 + (seq_No << 4);
    chars[1] = makeParity(chars[1]);
    chars[2] = B10011001;
  }
  else if (header == "processing") {
    chars[0] = B10011001;
    chars[1] = B10100000 + (seq_No << 4);
    chars[1] = makeParity(chars[1]);
    chars[2] = B10011001;
    charsLen = 3;
  }
  else if (header == "detect") {
    chars[0] = B10011001;
    chars[1] = B11000000 + (seq_No << 4) + (size_No << 1);
    chars[1] = makeParity(chars[1]);
    chars[2] = B10011001;
    charsLen = 3;
  }
  else if (header == "data") {
    chars[0] = B10011001;
    chars[1] = B00100000 + (seq_No << 4) + (size_No << 1);
    chars[1] = makeParity(chars[1]);
    if (size_No == 6) {
      for (int i = 0; i < 26; i++) {
        chars[i + 2] = dataT[num][i];
      }
      chars[27] = makeSum(chars[1], num);
      chars[28] = B10011001;
      charsLen = 29;
    }
    else {
      chars[2] = B10011001;
      charsLen = 3;
    }
  }
  else {
    Serial.println("ERROR makeData");
    return;
  }
}

void printBinary() {
  for (int i = 0; i < charsLen; i++) {
    if (i % 4 == 0)
      Serial.println();
    Serial.print(chars[i], BIN);
    if (chars[i] > 127)
      Serial.print("\t");
    else
      Serial.print("\t\t");
  }
}

/* Flow Control */
int state = 0;
uint8_t seq = 0;
uint8_t numdata = 0;
String receiveText = "";
String input = "";
uint8_t count16bit = 0;
#define timeout 1000

bool getSignal(int total , bool enable) {
  setReceiveZero();
  unsigned long starto = millis();
  while (1) {
    int tmp = analogRead(A0);
    //Serial.println(tmp);
    if (tmp - prev > r_slope && check == false && tmp > 420) {
      max = 420;
      check = true;
    }
    if (tmp > max) {
      max = tmp;
    }
    //Serial.println(tmp);
    if (first && tmp > 450 && check == true) {
      times = micros();
      first = false;
      count = 0;
    }
    if (max - tmp > r_slope) {
      //Serial.println(tmp);
      if (check) {
        if (490 < max && max < 1024) {
          count++;
          //Serial.println(count);
          //delayMicroseconds(300);
        }
      }
      check = false;
    }
    if (micros() - times > eiei && !first) {

      //Serial.print(nub);
      //Serial.println(count);
      if (count < 2)
      {
        count = 0;
        first = true;
        dis = 0;
        nub = 0;
      }
      else {
        times = micros();
        nub++;
        keep = count - 2;
        //Serial.println(keep);
        dis >>= 2;
        dis |= (keep << 6) ;
        //Serial.println(dis,BIN);
        //Serial.println();
        count = 0;
        if (nub == 4) {
          //Serial.println(dis, BIN);
          nub = 0;
          receive[receiveLen] = dis;
          dis = 0;
          keep = 0;
          first = true;
          receiveLen++;
          delayMicroseconds(4800);
        }
      }
    }
    prev = tmp;
    if (receiveLen == total) {
      return true;
    }
    if (enable && millis() - starto > timeout) {
      return false;
    }
  }
}

void sendSignal() {
  for (int i = 0; i < charsLen; i++) {
    for (int k = 7; k >= 0; k -= 2) {
      int tmp = chars[i] & 3;
      //Serial.println(tmp);
      if (tmp == 0) {
        for (int sl = 0; sl < freq0 / baud; sl++) {
          //Serial.println("0");
          for (int s = 0; s < 4; s++) {
            dac.setVoltage(S_DAC[s], false);
            delayMicroseconds(delay0);
          }
        }
      }
      else if (tmp == 1) {
        for (int sl = 0; sl < freq1 / baud; sl++) {
          //Serial.println("1");
          for (int s = 0; s < 4; s++) {
            dac.setVoltage(S_DAC[s], false);
            delayMicroseconds(delay1);
          }
        }
      }
      else if (tmp == 2) {
        for (int sl = 0; sl < freq2 / baud; sl++) {
          //Serial.println("2");
          for (int s = 0; s < 4; s++) {
            dac.setVoltage(S_DAC[s], false);
            delayMicroseconds(delay2);
          }
        }
      }
      else {
        for (int sl = 0; sl < freq3 / baud; sl++) {
          //Serial.println("3");
          for (int s = 0; s < 4; s++) {
            dac.setVoltage(S_DAC[s], false);
            delayMicroseconds(delay3);
          }
        }
      }
      chars[i] >>= 2;
    }
    dac.setVoltage(2047, false);
    delayMicroseconds(5000);
  }
  dac.setVoltage(2047, false);
}

void translateData() {
  receiveText = "";
  if (receive[1] < 32) {
    if ((receive[1]&B00001110) == 0) {
      receiveText = "request scan";
    }
    else if (receive[1]&B00000010) {
      receiveText = "request left";
    }
    else if (receive[1]&B00000100) {
      receiveText = "request mid";
    }
    else if (receive[1]&B00001000) {
      receiveText = "request right";
    }
    else {
      receiveText = "ERROR";
    }
  }
  else if ((receive[1]&B11100000) == B00100000) {
    if ((receive[1]&B00001110) == 0) {
      receiveText = "Data top";
    }
    else if ((receive[1]&B00001110) == B00000010) {
      receiveText = "Data left";
    }
    else if ((receive[1]&B00001110) == B00000100) {
      receiveText = "Data bottom";
    }
    else if ((receive[1]&B00001110) == B00000110) {
      receiveText = "Data right";
    }
    else if ((receive[1]&B00001110) == B00001000) {
      receiveText = "Data upper";
    }
    else if ((receive[1]&B00001110) == B00001010) {
      receiveText = "Data lower";
    }
    else if ((receive[1]&B00001110) == B00001100) {
      receiveText = "Data 16 bit";
    }
    else {
      receiveText = "ERROR";
    }
  }
  else if ((receive[1]&B11100000) == B01000000) {
    receiveText = "ACK";
  }
  else if ((receive[1]&B11100000) == B01100000) {
    receiveText = "NAK";
  }
  else if ((receive[1]&B11100000) == B10000000) {
    receiveText = "ask";
  }
  else if ((receive[1]&B11100000) == B10100000) {
    receiveText = "processing";
  }
  else if ((receive[1]&B11100000) == B11000000) {
    if (receive[1]&B00000010) {
      receiveText = "detect left";
    }
    else if (receive[1]&B00000100) {
      receiveText = "detect mid";
    }
    else if (receive[1]&B00001000) {
      receiveText = "detect right";
    }
    else {
      receiveText = "ERROR";
    }
  }
  else {
    receiveText = "ERROR";
  }
}

void receiveSerial() {
  String a = "";
  String b = "";
  String c = "";
  while (Serial.available() >= 9) {
    for (int i = 0; i < 3; i++) {
      a += Serial.read() - '0';
    }
    for (int i = 0; i < 3; i++) {
      b += Serial.read() - '0';
    }
    for (int i = 0; i < 3; i++) {
      c += Serial.read() - '0';
    }
    height[numdata] = a.toInt();
    width[numdata] = b.toInt();
    color[numdata] = c.toInt();
    numdata++;
    a = "";
    b = "";
    c = "";
  }
}

bool checkSeq() {
  if (((receive[1]&B00010000) >> 4) == seq)
    return true;
  else
    return false;
}

bool checkFlag() {
  if (receive[0] == B10011001 && receive[receiveLen - 1] == B10011001)
    return true;
  else
    return false;
}
/*Servo*/
#define servoBase 9
#include <Servo.h>
Servo servo;
float pos;

void goTo(float angle) {
  if (pos > angle)
  {
    for (; pos >= angle; pos--)
    {
      servo.write(pos);
      delay(20);
    }
  }
  else
  {
    for (; pos <= angle; pos++)
    {
      servo.write(pos);
      delay(20);
    }
  }
}

void setup() {
  /* Transmitter */
  Serial.begin(115200);
  dac.begin(0x64);
  delay0 = 1000000 / freq0 / 4 - 1000000 / defaultFreq / 4;
  delay1 = 1000000 / freq1 / 4 - 1000000 / defaultFreq / 4;
  delay2 = 1000000 / freq2 / 4 - 1000000 / defaultFreq / 4;
  delay3 = 1000000 / freq3 / 4 - 1000000 / defaultFreq / 4;

  /* Receiver */
  Wire.begin();
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  //Set Frequency to start with
  radio.setFrequency(frequency);
  Serial.flush();

  /*Servo*/
  servo.attach(servoBase);

  delay(100);
}

void loop() {
  if (state == 0) {
    getSignal(3, false);
    Serial.println(seq);
    //    Serial.println(receive[0],BIN);
    //    Serial.println(receive[1],BIN);
    //    Serial.println(receive[2],BIN);
    if (checkFlag) {
      if (checkParity(receive[1])) {
        if (checkSeq()) {
          translateData();
          if (receiveText == "request scan") {
            Serial.println("receive scan");
            seq = (seq + 1) % 2;
            state = 1;
            makeData("ACK", seq, 0, 0);
            printBinary();
            delay(40);
            sendSignal();
            Serial.println("send ACK");
          }
          else if (receiveText == "request left") {
            seq = (seq + 1) % 2;
            state = 10;
            numdata = 0;
            makeData("ACK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
          else if (receiveText == "request mid") {
            seq = (seq + 1) % 2;
            state = 13;
            numdata = 0;
            makeData("ACK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
          else if (receiveText == "request right") {
            seq = (seq + 1) % 2;
            state = 16;
            numdata = 0;
            makeData("ACK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
          else {
            Serial.println("Data ERROR(Non request)");
            makeData("NAK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.println("ERROR Seq");
          makeData("NAK", seq, 0, 0);
          delay(80);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Parity");
        makeData("NAK", seq, 0, 0);
        delay(80);
        sendSignal();
      }
    }
    else {
      Serial.println("ERROR Flag");
      makeData("NAK", seq, 0, 0);
      delay(80);
      sendSignal();
    }
  }
  else if (state == 1) {
    goTo(43);
    delay(2000);
    Serial.println("R");
    while (1) {
      getSignal(3, false);
      if (Serial.available() != 0) {
        delay(20);
        input = Serial.readString();
      }
      //    Serial.println(receive[0],BIN);
      //    Serial.println(receive[1],BIN);
      //    Serial.println(receive[2],BIN);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ask") {
              Serial.println("receive ask");
              if (input == "TOP" || input == "RIG" || input == "LEF" || input == "BOT" || input == "UPP" || input == "LOW") {
                Serial.println("Data");
                seq = (seq + 1) % 2;
                state = 2;
                makeData("detect", seq, 4, 0);
                printBinary();
                delay(40);
                sendSignal();
                break;
              }
              else if (input == "UNK") {
                Serial.println("Unknown");
                input = "";
                break;
              }
              else if (input == "") {
                Serial.println("Processing");
                seq = (seq + 1) % 2;
                makeData("processing", seq, 0, 0);
                printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("ERROR Data");
                input = "";
                break;
              }
            }
            else {
              Serial.println("Data ERROR(Non request)");
              makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.println("ERROR Seq");
            /*makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();*/
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
  }
  else if (state == 2) {
    getSignal(3, false);
    //    Serial.println(receive[0],BIN);
    //    Serial.println(receive[1],BIN);
    //    Serial.println(receive[2],BIN);
    if (checkFlag) {
      if (checkParity(receive[1])) {
        if (checkSeq()) {
          translateData();
          if (receiveText == "ACK") {
            Serial.println("receive ACK");
            if (input == "TOP") {
              Serial.println("send top");
              seq = (seq + 1) % 2;
              makeData("data", seq, 0, 0);
              //printBinary();
              delay(40);
              sendSignal();
              state = 3;
            }
            else if (input == "LEF") {
              Serial.println("send left");
              seq = (seq + 1) % 2;
              state = 3;
              makeData("data", seq, 1, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "BOT") {
              Serial.println("send bottom");
              seq = (seq + 1) % 2;
              state = 3;
              makeData("data", seq, 2, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "RIG") {
              Serial.println("send right");
              seq = (seq + 1) % 2;
              state = 3;
              makeData("data", seq, 3, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "UPP") {
              Serial.println("send upper");
              seq = (seq + 1) % 2;
              state = 3;
              makeData("data", seq, 4, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LOW") {
              Serial.println("send lower");
              seq = (seq + 1) % 2;
              state = 3;
              makeData("data", seq, 5, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else {
              Serial.println("data ERROR");
            }
          }

          else {
            Serial.println("Data ERROR(Non request)");
            makeData("NAK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.println("ERROR Seq");
          makeData("detect", seq, 4, 0);
          printBinary();
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Parity");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("ERROR Flag");
      makeData("NAK", seq, 0, 0);
      delay(40);
      sendSignal();
    }
  }
  else if (state == 3) {
    delay(20);
    if (getSignal(3, true)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              state = 4;
              input = "";
              seq = (seq + 1) % 2;
            }
            else {
              if (input == "TOP") {
                Serial.println("send top");
                makeData("data", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "LEF") {
                Serial.println("send left");
                makeData("data", seq, 1, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "BOT") {
                Serial.println("send bottom");
                makeData("data", seq, 2, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "RIG") {
                Serial.println("send right");
                makeData("data", seq, 3, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "UPP") {
                Serial.println("send upper");
                makeData("data", seq, 4, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "LOW") {
                Serial.println("send lower");
                makeData("data", seq, 5, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("data ERROR");
              }
            }
          }
          else {
            if (input == "TOP") {
              Serial.println("send top");
              makeData("data", seq, 0, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LEF") {
              Serial.println("send left");
              makeData("data", seq, 1, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "BOT") {
              Serial.println("send bottom");
              makeData("data", seq, 2, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "RIG") {
              Serial.println("send right");
              makeData("data", seq, 3, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "UPP") {
              Serial.println("send upper");
              makeData("data", seq, 4, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LOW") {
              Serial.println("send lower");
              makeData("data", seq, 5, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else {
              Serial.println("data ERROR");
            }
          }
        }
        else {
          if (input == "TOP") {
            Serial.println("send top");
            makeData("data", seq, 0, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "LEF") {
            Serial.println("send left");
            makeData("data", seq, 1, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "BOT") {
            Serial.println("send bottom");
            makeData("data", seq, 2, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "RIG") {
            Serial.println("send right");
            makeData("data", seq, 3, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "UPP") {
            Serial.println("send upper");
            makeData("data", seq, 4, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "LOW") {
            Serial.println("send lower");
            makeData("data", seq, 5, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else {
            Serial.println("data ERROR");
          }
        }
      }
      else {
        if (input == "TOP") {
          Serial.println("send top");
          makeData("data", seq, 0, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "LEF") {
          Serial.println("send left");
          makeData("data", seq, 1, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "BOT") {
          Serial.println("send bottom");
          makeData("data", seq, 2, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "RIG") {
          Serial.println("send right");
          makeData("data", seq, 3, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "UPP") {
          Serial.println("send upper");
          makeData("data", seq, 4, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "LOW") {
          Serial.println("send lower");
          makeData("data", seq, 5, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else {
          Serial.println("data ERROR");
        }
      }
    }
    else {
      if (input == "TOP") {
        Serial.println("send top");
        makeData("data", seq, 0, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "LEF") {
        Serial.println("send left");
        makeData("data", seq, 1, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "BOT") {
        Serial.println("send bottom");
        makeData("data", seq, 2, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "RIG") {
        Serial.println("send right");
        makeData("data", seq, 3, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "UPP") {
        Serial.println("send upper");
        makeData("data", seq, 4, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "LOW") {
        Serial.println("send lower");
        makeData("data", seq, 5, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else {
        Serial.println("data ERROR");
      }
    }
  }
  else if (state == 4) {
    goTo(88);
    delay(2000);
    Serial.println("M");
    while (1) {
      getSignal(3, false);
      if (Serial.available() != 0) {
        delay(20);
        input = Serial.readString();
      }
      //    Serial.println(receive[0],BIN);
      //    Serial.println(receive[1],BIN);
      //    Serial.println(receive[2],BIN);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ask") {
              Serial.println("receive ask");
              if (input == "TOP" || input == "RIG" || input == "LEF" || input == "BOT" || input == "UPP" || input == "LOW") {
                Serial.println("Data");
                seq = (seq + 1) % 2;
                state = 5;
                makeData("detect", seq, 2, 0);
                //printBinary();
                delay(40);
                sendSignal();
                break;
              }
              else if (input == "UNK") {
                Serial.println("Unknown");
                input = "";
                break;
              }
              else if (input == "") {
                Serial.println("Processing");
                seq = (seq + 1) % 2;
                makeData("processing", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("ERROR Data");
                input = "";
                break;
              }
            }
            else {
              Serial.println("Data ERROR(Non request)");
              makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.println("ERROR Seq");
            /*makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();*/
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
  }
  else if (state == 5) {
    getSignal(3, false);
    //    Serial.println(receive[0],BIN);
    //    Serial.println(receive[1],BIN);
    //    Serial.println(receive[2],BIN);
    if (checkFlag) {
      if (checkParity(receive[1])) {
        if (checkSeq()) {
          translateData();
          if (receiveText == "ACK") {
            Serial.println("receive ACK");
            if (input == "TOP") {
              Serial.println("send top");
              seq = (seq + 1) % 2;
              makeData("data", seq, 0, 0);
              //printBinary();
              delay(40);
              sendSignal();
              state = 6;
            }
            else if (input == "LEF") {
              Serial.println("send left");
              seq = (seq + 1) % 2;
              state = 6;
              makeData("data", seq, 1, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "BOT") {
              Serial.println("send bottom");
              seq = (seq + 1) % 2;
              state = 6;
              makeData("data", seq, 2, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "RIG") {
              Serial.println("send right");
              seq = (seq + 1) % 2;
              state = 6;
              makeData("data", seq, 3, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "UPP") {
              Serial.println("send upper");
              seq = (seq + 1) % 2;
              state = 6;
              makeData("data", seq, 4, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LOW") {
              Serial.println("send lower");
              seq = (seq + 1) % 2;
              state = 6;
              makeData("data", seq, 5, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else {
              Serial.println("data ERROR");
            }
          }

          else {
            Serial.println("Data ERROR(Non request)");
            makeData("NAK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.println("ERROR Seq");
          makeData("detect", seq, 2, 0);
          printBinary();
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Parity");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("ERROR Flag");
      makeData("NAK", seq, 0, 0);
      delay(40);
      sendSignal();
    }
  }
  else if (state == 6) {
    delay(20);
    if (getSignal(3, true)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              state = 7;
              input = "";
              seq = (seq + 1) % 2;
            }
            else {
              if (input == "TOP") {
                Serial.println("send top");
                makeData("data", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "LEF") {
                Serial.println("send left");
                makeData("data", seq, 1, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "BOT") {
                Serial.println("send bottom");
                makeData("data", seq, 2, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "RIG") {
                Serial.println("send right");
                makeData("data", seq, 3, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "UPP") {
                Serial.println("send upper");
                makeData("data", seq, 4, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "LOW") {
                Serial.println("send lower");
                makeData("data", seq, 5, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("data ERROR");
              }
            }
          }
          else {
            if (input == "TOP") {
              Serial.println("send top");
              makeData("data", seq, 0, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LEF") {
              Serial.println("send left");
              makeData("data", seq, 1, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "BOT") {
              Serial.println("send bottom");
              makeData("data", seq, 2, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "RIG") {
              Serial.println("send right");
              makeData("data", seq, 3, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "UPP") {
              Serial.println("send upper");
              makeData("data", seq, 4, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LOW") {
              Serial.println("send lower");
              makeData("data", seq, 5, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else {
              Serial.println("data ERROR");
            }
          }
        }
        else {
          if (input == "TOP") {
            Serial.println("send top");
            makeData("data", seq, 0, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "LEF") {
            Serial.println("send left");
            makeData("data", seq, 1, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "BOT") {
            Serial.println("send bottom");
            makeData("data", seq, 2, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "RIG") {
            Serial.println("send right");
            makeData("data", seq, 3, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "UPP") {
            Serial.println("send upper");
            makeData("data", seq, 4, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "LOW") {
            Serial.println("send lower");
            makeData("data", seq, 5, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else {
            Serial.println("data ERROR");
          }
        }
      }
      else {
        if (input == "TOP") {
          Serial.println("send top");
          makeData("data", seq, 0, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "LEF") {
          Serial.println("send left");
          makeData("data", seq, 1, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "BOT") {
          Serial.println("send bottom");
          makeData("data", seq, 2, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "RIG") {
          Serial.println("send right");
          makeData("data", seq, 3, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "UPP") {
          Serial.println("send upper");
          makeData("data", seq, 4, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "LOW") {
          Serial.println("send lower");
          makeData("data", seq, 5, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else {
          Serial.println("data ERROR");
        }
      }
    }
    else {
      if (input == "TOP") {
        Serial.println("send top");
        makeData("data", seq, 0, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "LEF") {
        Serial.println("send left");
        makeData("data", seq, 1, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "BOT") {
        Serial.println("send bottom");
        makeData("data", seq, 2, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "RIG") {
        Serial.println("send right");
        makeData("data", seq, 3, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "UPP") {
        Serial.println("send upper");
        makeData("data", seq, 4, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "LOW") {
        Serial.println("send lower");
        makeData("data", seq, 5, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else {
        Serial.println("data ERROR");
      }
    }
  }
  else if (state == 7) {
    goTo(136);
    delay(2000);
    Serial.println("L");
    while (1) {
      getSignal(3, false);
      if (Serial.available() != 0) {
        delay(20);
        input = Serial.readString();
      }
      //    Serial.println(receive[0],BIN);
      //    Serial.println(receive[1],BIN);
      //    Serial.println(receive[2],BIN);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ask") {
              Serial.println("receive ask");
              if (input == "TOP" || input == "RIG" || input == "LEF" || input == "BOT" || input == "UPP" || input == "LOW") {
                Serial.println("Data");
                seq = (seq + 1) % 2;
                state = 8;
                makeData("detect", seq, 1, 0);
                //printBinary();
                delay(40);
                sendSignal();
                break;
              }
              else if (input == "UNK") {
                Serial.println("Unknown");
                input = "";
                break;
              }
              else if (input == "") {
                Serial.println("Processing");
                seq = (seq + 1) % 2;
                makeData("processing", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("ERROR Data");
                input = "";
                break;
              }
            }
            else {
              Serial.println("Data ERROR(Non request)");
              makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.println("ERROR Seq");
            /*makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();*/
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
  }
  else if (state == 8) {
    getSignal(3, false);
    //    Serial.println(receive[0],BIN);
    //    Serial.println(receive[1],BIN);
    //    Serial.println(receive[2],BIN);
    if (checkFlag) {
      if (checkParity(receive[1])) {
        if (checkSeq()) {
          translateData();
          if (receiveText == "ACK") {
            Serial.println("receive ACK");
            if (input == "TOP") {
              Serial.println("send top");
              seq = (seq + 1) % 2;
              makeData("data", seq, 0, 0);
              //printBinary();
              delay(40);
              sendSignal();
              state = 9;
            }
            else if (input == "LEF") {
              Serial.println("send left");
              seq = (seq + 1) % 2;
              state = 9;
              makeData("data", seq, 1, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "BOT") {
              Serial.println("send bottom");
              seq = (seq + 1) % 2;
              state = 9;
              makeData("data", seq, 2, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "RIG") {
              Serial.println("send right");
              seq = (seq + 1) % 2;
              state = 9;
              makeData("data", seq, 3, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "UPP") {
              Serial.println("send upper");
              seq = (seq + 1) % 2;
              state = 9;
              makeData("data", seq, 4, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LOW") {
              Serial.println("send lower");
              seq = (seq + 1) % 2;
              state = 9;
              makeData("data", seq, 5, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else {
              Serial.println("data ERROR");
            }
          }

          else {
            Serial.println("Data ERROR(Non request)");
            makeData("NAK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.println("ERROR Seq");
          makeData("detect", seq, 1, 0);
          printBinary();
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Parity");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("ERROR Flag");
      makeData("NAK", seq, 0, 0);
      delay(40);
      sendSignal();
    }
  }
  else if (state == 9) {
    delay(20);
    if (getSignal(3, true)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              state = 0;
              input = "";
              Serial.println("Hello");
              seq = (seq + 1) % 2;
            }
            else {
              if (input == "TOP") {
                Serial.println("send top");
                makeData("data", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "LEF") {
                Serial.println("send left");
                makeData("data", seq, 1, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "BOT") {
                Serial.println("send bottom");
                makeData("data", seq, 2, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "RIG") {
                Serial.println("send right");
                makeData("data", seq, 3, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "UPP") {
                Serial.println("send upper");
                makeData("data", seq, 4, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else if (input == "LOW") {
                Serial.println("send lower");
                makeData("data", seq, 5, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("data ERROR");
              }
            }
          }
          else {
            if (input == "TOP") {
              Serial.println("send top");
              makeData("data", seq, 0, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LEF") {
              Serial.println("send left");
              makeData("data", seq, 1, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "BOT") {
              Serial.println("send bottom");
              makeData("data", seq, 2, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "RIG") {
              Serial.println("send right");
              makeData("data", seq, 3, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "UPP") {
              Serial.println("send upper");
              makeData("data", seq, 4, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else if (input == "LOW") {
              Serial.println("send lower");
              makeData("data", seq, 5, 0);
              //printBinary();
              delay(40);
              sendSignal();
            }
            else {
              Serial.println("data ERROR");
            }
          }
        }
        else {
          if (input == "TOP") {
            Serial.println("send top");
            makeData("data", seq, 0, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "LEF") {
            Serial.println("send left");
            makeData("data", seq, 1, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "BOT") {
            Serial.println("send bottom");
            makeData("data", seq, 2, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "RIG") {
            Serial.println("send right");
            makeData("data", seq, 3, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "UPP") {
            Serial.println("send upper");
            makeData("data", seq, 4, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else if (input == "LOW") {
            Serial.println("send lower");
            makeData("data", seq, 5, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
          else {
            Serial.println("data ERROR");
          }
        }
      }
      else {
        if (input == "TOP") {
          Serial.println("send top");
          makeData("data", seq, 0, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "LEF") {
          Serial.println("send left");
          makeData("data", seq, 1, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "BOT") {
          Serial.println("send bottom");
          makeData("data", seq, 2, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "RIG") {
          Serial.println("send right");
          makeData("data", seq, 3, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "UPP") {
          Serial.println("send upper");
          makeData("data", seq, 4, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else if (input == "LOW") {
          Serial.println("send lower");
          makeData("data", seq, 5, 0);
          //printBinary();
          delay(40);
          sendSignal();
        }
        else {
          Serial.println("data ERROR");
        }
      }
    }
    else {
      if (input == "TOP") {
        Serial.println("send top");
        makeData("data", seq, 0, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "LEF") {
        Serial.println("send left");
        makeData("data", seq, 1, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "BOT") {
        Serial.println("send bottom");
        makeData("data", seq, 2, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "RIG") {
        Serial.println("send right");
        makeData("data", seq, 3, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "UPP") {
        Serial.println("send upper");
        makeData("data", seq, 4, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else if (input == "LOW") {
        Serial.println("send lower");
        makeData("data", seq, 5, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
      else {
        Serial.println("data ERROR");
      }
    }
  }
  else if (state == 10) {
    goTo(136);
    delay(2000);
    Serial.println("l");
    while (1) {
      getSignal(3, false);
      receiveSerial();
      //    Serial.println(receive[0],BIN);
      //    Serial.println(receive[1],BIN);
      //    Serial.println(receive[2],BIN);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ask") {
              Serial.println("receive ask");
              if (numdata == 16) {
                Serial.println("Data complete");
                seq = (seq + 1) % 2;
                state = 11;
                count16bit = 0;
                makeData("detect", seq, 1, 0);
                //printBinary();
                delay(40);
                sendSignal();
                encoder();
                break;
              }
              else if (numdata < 16 && numdata >= 0) {
                Serial.println("Processing");
                seq = (seq + 1) % 2;
                makeData("processing", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("ERROR Data");
              }
            }
            else {
              Serial.println("Data ERROR(Non request)");
              makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.println("ERROR Seq");
            /*makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();*/
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
  }
  else if (state == 11) {
    getSignal(3, false);
    //    Serial.println(receive[0],BIN);
    //    Serial.println(receive[1],BIN);
    //    Serial.println(receive[2],BIN);
    if (checkFlag) {
      if (checkParity(receive[1])) {
        if (checkSeq()) {
          translateData();
          if (receiveText == "ACK") {
            Serial.println("receive ACK");
            if (count16bit < 2) {
              Serial.print("send count16bit ");
              Serial.println(count16bit);
              seq = (seq + 1) % 2;
              makeData("data", seq, 6, count16bit);
              //printBinary();
              delay(40);
              sendSignal();
              count16bit++;
            }
            else {
              state = 12;
            }
          }
          else if (receiveText == "NAK") {
            Serial.print("send count16bit ");
            if (count16bit == 0) {
              Serial.println(count16bit);
              makeData("data", seq, 6, count16bit);
            }
            else {
              Serial.println(count16bit - 1);
              makeData("data", seq, 6, count16bit - 1);
            }
            //printBinary();
            delay(40);
            sendSignal();
          }
          else {
            Serial.println("Data ERROR(Non request)");
            makeData("NAK", seq, 0, 0);
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.println("ERROR Seq");
          makeData("detect", seq, 1, 0);
          delay(40);
          sendSignal();
          //seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR Parity");
        makeData("NAK", seq, 0, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("ERROR Flag");
      makeData("NAK", seq, 0, 0);
      //printBinary();
      delay(40);
      sendSignal();
    }
  }
  else if (state == 12) {
    delay(20);
    if (getSignal(3, true)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              state = 0;
              count16bit = 0;
              seq = (seq + 1) % 2;
              //printBinary();
            }
            else {
              Serial.print("send count16bit ");
              if (count16bit == 0) {
                Serial.println(count16bit);
                makeData("data", seq, 6, count16bit);
              }
              else {
                Serial.println(count16bit - 1);
                makeData("data", seq, 6, count16bit - 1);
              }
              //printBinary();
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.print("send count16bit ");
            if (count16bit == 0) {
              Serial.println(count16bit);
              makeData("data", seq, 6, count16bit);
            }
            else {
              Serial.println(count16bit - 1);
              makeData("data", seq, 6, count16bit - 1);
            }
            //printBinary();
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.print("send count16bit ");
          if (count16bit == 0) {
            Serial.println(count16bit);
            makeData("data", seq, 6, count16bit);
          }
          else {
            Serial.println(count16bit - 1);
            makeData("data", seq, 6, count16bit - 1);
          }
          //printBinary();
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.print("send count16bit ");
        if (count16bit == 0) {
          Serial.println(count16bit);
          makeData("data", seq, 6, count16bit);
        }
        else {
          Serial.println(count16bit - 1);
          makeData("data", seq, 6, count16bit - 1);
        }
        //printBinary();
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("time out");
      Serial.print("send count16bit ");
      if (count16bit == 0) {
        Serial.println(count16bit);
        makeData("data", seq, 6, count16bit);
      }
      else {
        Serial.println(count16bit - 1);
        makeData("data", seq, 6, count16bit - 1);
      }
      //printBinary();
      delay(40);
      sendSignal();
    }
  }
  else if (state == 13) {
    goTo(88);
    delay(2000);
    Serial.println("m");
    while (1) {
      getSignal(3, false);
      receiveSerial();
      //    Serial.println(receive[0],BIN);
      //    Serial.println(receive[1],BIN);
      //    Serial.println(receive[2],BIN);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ask") {
              Serial.println("receive ask");
              if (numdata == 16) {
                Serial.println("Data complete");
                seq = (seq + 1) % 2;
                state = 14;
                count16bit = 0;
                makeData("detect", seq, 2, 0);
                //printBinary();
                delay(40);
                sendSignal();
                encoder();
                break;
              }
              else if (numdata < 16 && numdata >= 0) {
                Serial.println("Processing");
                seq = (seq + 1) % 2;
                makeData("processing", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("ERROR Data");
              }
            }
            else {
              Serial.println("Data ERROR(Non request)");
              makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.println("ERROR Seq");
            /*makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();*/
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
  }
  else if (state == 14) {
    getSignal(3, false);
    //    Serial.println(receive[0],BIN);
    //    Serial.println(receive[1],BIN);
    //    Serial.println(receive[2],BIN);
    if (checkFlag) {
      if (checkParity(receive[1])) {
        if (checkSeq()) {
          translateData();
          if (receiveText == "ACK") {
            Serial.println("receive ACK");
            if (count16bit < 2) {
              Serial.print("send count16bit ");
              Serial.println(count16bit);
              seq = (seq + 1) % 2;
              makeData("data", seq, 6, count16bit);
              //printBinary();
              delay(40);
              sendSignal();
              count16bit++;
              Serial.println(count16bit);
            }
            else {
              state = 15;
            }
          }
          else if (receiveText == "NAK") {
            Serial.print("send count16bit ");
            if (count16bit == 0) {
              Serial.println(count16bit);
              makeData("data", seq, 6, count16bit);
            }
            else {
              Serial.println(count16bit - 1);
              makeData("data", seq, 6, count16bit - 1);
            }
            //printBinary();
            delay(40);
            sendSignal();
          }
          else {
            Serial.println("Data ERROR(Non request)");
            makeData("NAK", seq, 0, 0);
            //printBinary();
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.println("ERROR Seq");
          makeData("detect", seq, 2, 0);
          delay(40);
          sendSignal();
          //seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR Parity");
        makeData("NAK", seq, 0, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("ERROR Flag");
      makeData("NAK", seq, 0, 0);
      //printBinary();
      delay(40);
      sendSignal();
    }
  }
  else if (state == 15) {
    delay(20);
    if (getSignal(3, true)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              state = 0;
              count16bit = 0;
              seq = (seq + 1) % 2;
              //printBinary();
            }
            else {
              Serial.print("send count16bit ");
              if (count16bit == 0) {
                Serial.println(count16bit);
                makeData("data", seq, 6, count16bit);
              }
              else {
                Serial.println(count16bit - 1);
                makeData("data", seq, 6, count16bit - 1);
              }
              //printBinary();
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.print("send count16bit ");
            if (count16bit == 0) {
              Serial.println(count16bit);
              makeData("data", seq, 6, count16bit);
            }
            else {
              Serial.println(count16bit - 1);
              makeData("data", seq, 6, count16bit - 1);
            }
            //printBinary();
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.print("send count16bit ");
          if (count16bit == 0) {
            Serial.println(count16bit);
            makeData("data", seq, 6, count16bit);
          }
          else {
            Serial.println(count16bit - 1);
            makeData("data", seq, 6, count16bit - 1);
          }
          //printBinary();
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.print("send count16bit ");
        if (count16bit == 0) {
          Serial.println(count16bit);
          makeData("data", seq, 6, count16bit);
        }
        else {
          Serial.println(count16bit - 1);
          makeData("data", seq, 6, count16bit - 1);
        }
        //printBinary();
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.print("time out");
      Serial.print("send count16bit ");
      if (count16bit == 0) {
        Serial.println(count16bit);
        makeData("data", seq, 6, count16bit);
      }
      else {
        Serial.println(count16bit - 1);
        makeData("data", seq, 6, count16bit - 1);
      }
      //printBinary();
      delay(40);
      sendSignal();
    }
  }
  else if (state == 16) {
    goTo(43);
    delay(2000);
    Serial.println("r");
    while (1) {
      getSignal(3, false);
      receiveSerial();
      //    Serial.println(receive[0],BIN);
      //    Serial.println(receive[1],BIN);
      //    Serial.println(receive[2],BIN);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ask") {
              Serial.println("receive ask");
              if (numdata == 16) {
                Serial.println("Data complete");
                seq = (seq + 1) % 2;
                state = 17;
                count16bit = 0;
                makeData("detect", seq, 4, 0);
                //printBinary();
                delay(40);
                sendSignal();
                encoder();
                break;
              }
              else if (numdata < 16 && numdata >= 0) {
                Serial.println("Processing");
                seq = (seq + 1) % 2;
                makeData("processing", seq, 0, 0);
                //printBinary();
                delay(40);
                sendSignal();
              }
              else {
                Serial.println("ERROR Data");
              }
            }
            else {
              Serial.println("Data ERROR(Non request)");
              makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.println("ERROR Seq");
            /*makeData("NAK", seq, 0, 0);
              delay(40);
              sendSignal();*/
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        delay(40);
        sendSignal();
      }
    }
  }
  else if (state == 17) {
    getSignal(3, false);
    //    Serial.println(receive[0],BIN);
    //    Serial.println(receive[1],BIN);
    //    Serial.println(receive[2],BIN);
    if (checkFlag) {
      if (checkParity(receive[1])) {
        if (checkSeq()) {
          translateData();
          if (receiveText == "ACK") {
            Serial.println("receive ACK");
            if (count16bit < 2) {
              Serial.print("send count16bit ");
              Serial.println(count16bit);
              seq = (seq + 1) % 2;
              makeData("data", seq, 6, count16bit);
              //printBinary();
              delay(40);
              sendSignal();
              count16bit++;
            }
            else {
              state = 18;
            }
          }
          else if (receiveText == "NAK") {
            Serial.print("send count16bit ");
            if (count16bit == 0) {
              Serial.println(count16bit);
              makeData("data", seq, 6, count16bit);
            }
            else {
              Serial.println(count16bit - 1);
              makeData("data", seq, 6, count16bit - 1);
            }
            //printBinary();
            delay(40);
            sendSignal();
          }
          else {
            Serial.println("Data ERROR(Non request)");
            Serial.print("send count16bit ");
            if (count16bit == 0) {
              Serial.println(count16bit);
              makeData("data", seq, 6, count16bit);
            }
            else {
              Serial.println(count16bit - 1);
              makeData("data", seq, 6, count16bit - 1);
            }
            //printBinary();
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.println("ERROR Seq");
          makeData("detect", seq, 4, 0);
          delay(40);
          sendSignal();
          //seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR Parity");
        makeData("NAK", seq, 0, 0);
        //printBinary();
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("ERROR Flag");
      makeData("NAK", seq, 0, 0);
      //printBinary();
      delay(40);
      sendSignal();
    }
  }
  else if (state == 18) {
    delay(20);
    if (getSignal(3, true)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              state = 0;
              count16bit = 0;
              seq = (seq + 1) % 2;
            }
            else {
              Serial.print("send count16bit ");
              if (count16bit == 0) {
                Serial.println(count16bit);
                makeData("data", seq, 6, count16bit);
              }
              else {
                Serial.println(count16bit - 1);
                makeData("data", seq, 6, count16bit - 1);
              }
              //printBinary();
              delay(40);
              sendSignal();
            }
          }
          else {
            Serial.print("send count16bit ");
            if (count16bit == 0) {
              Serial.println(count16bit);
              makeData("data", seq, 6, count16bit);
            }
            else {
              Serial.println(count16bit - 1);
              makeData("data", seq, 6, count16bit - 1);
            }
            //printBinary();
            delay(40);
            sendSignal();
          }
        }
        else {
          Serial.print("send count16bit ");
          seq = (seq + 1) % 2;
          if (count16bit == 0) {
            Serial.println(count16bit);
            makeData("data", seq, 6, count16bit);
          }
          else {
            Serial.println(count16bit - 1);
            makeData("data", seq, 6, count16bit - 1);
          }
          //printBinary();
          delay(40);
          sendSignal();
        }
      }
      else {
        Serial.print("send count16bit ");
        if (count16bit == 0) {
          Serial.println(count16bit);
          makeData("data", seq, 6, count16bit);
        }
        else {
          Serial.println(count16bit - 1);
          makeData("data", seq, 6, count16bit - 1);
        }
        //printBinary();
        delay(40);
        sendSignal();
      }
    }
    else {
      Serial.println("time out");
      Serial.print("send count16bit ");
      if (count16bit == 0) {
        Serial.println(count16bit);
        makeData("data", seq, 6, count16bit);
      }
      else {
        Serial.println(count16bit - 1);
        makeData("data", seq, 6, count16bit - 1);
      }
      //printBinary();
      delay(40);
      sendSignal();
    }
  }
}
