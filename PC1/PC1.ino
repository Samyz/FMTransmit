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
#define r_slope 15
int keep = 0;
int min = 320;
int prev = 0;
int count = 0;
int nub = 0;
int check = false, first = true;
uint8_t dis = 0;
unsigned long times;
int eiei = 4000;
TEA5767 radio = TEA5767();

float frequency = 91.8;//87.8;//98.2;//102.8 //Enter your own Frequency

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
    charsLen = 3;
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
String receiveText;
String dataLeft = "";
String dataMid = "";
String dataRight = "";
unsigned long stateTime;
int count16bit = 0;
bool jump = false;
#define timeout 1500

bool getSignal(int total) {
  setReceiveZero();
  delay(20);
  unsigned long starto = millis();
  while (1) {
    int tmp = analogRead(A0);
    if ((prev - tmp > 6) && check == false && tmp < 350) {
      min = 350;
      //Serial.print("HI");
      check = true;
    }
    if (tmp < min) {
      min = tmp;
    }
    //Serial.println(tmp);
    if (first && tmp < 350 && check == true) {
      times = micros();
      //Serial.print("Hello");
      first = false;
      count = 0;
    }
    if (tmp - min > r_slope) {
      //Serial.println(tmp);
      //Serial.println(min);
      if (check) {
        if (200 < min && min < 340) {
          count++;
          min = 350;
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
          //Serial.print((char)dis);
          receive[receiveLen] = dis;
          //Serial.println(receive[receiveLen], BIN);
          nub = 0;
          dis = 0;
          keep = 0;
          first = true;
          receiveLen++;
        }
      }
    }
    prev = tmp;
    if (receiveLen == total) {
      break;
    }
    if (millis() - starto > timeout) {
      return false;
    }
  }
  return true;
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

void printPicture() {
  Serial.println();
  Serial.println("-------------------------Picture-------------------------");
  Serial.println("\tLeft\tMid\tRight");
  //sprintf(pint, "\t%s\t%s\t%s", dataLeft, dataMid, dataRight);
  Serial.print("\t");
  Serial.print(dataLeft);
  Serial.print("\t");
  Serial.print(dataMid);
  Serial.print("\t");
  Serial.println(dataRight);
  Serial.println("---------------------------------------------------------");
  Serial.println();
}

void print16bit(int mode) {
  Serial.println();
  Serial.println("-------------------------Data-------------------------");
  Serial.print("                         ");
  if (mode == 0)
    Serial.println(dataLeft);
  else if (mode == 1)
    Serial.println(dataMid);
  else
    Serial.println(dataRight);
  Serial.println("\theight\twidth\tcolor");
  for (int i = 0; i < 16; i++) {
    sprintf(pint, "\t%d\t%d\t%d", height[i], width[i], color[i]);
    Serial.println(pint);
  }
  Serial.println("------------------------------------------------------");
  Serial.println();
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

  delay(100);

}

void loop() {
  if (state == 0) {
    Serial.println("Hello");
    while (Serial.available() < 2) {
      //Serial.println(seq);
      delay(40);
      if (getSignal(29)) {
        seq = (seq + 1) % 2;
        makeData("ACK", seq, 0, 0);
        //printBinary();
        Serial.println("Send ACK");
        sendSignal();
        seq = (seq + 1) % 2;
        delay(20);
      }
    }
    delay(20);
    String input = Serial.readString();
    if (input == "start") {
      state = 1;
    }
    else if (input == "bottom" || input == "top" || input == "left" || input == "right" || input == "upper" || input == "lower") {
      if (dataLeft != "" && dataMid != "" && dataRight != "") {
        if (input == dataLeft) {
          state = 2;
        }
        else if (input == dataMid) {
          state = 3;
        }
        else if (input == dataRight) {
          state = 4;
        }
        else {
          Serial.println("ERROR Input Picture");
        }
      }
      else {
        Serial.println("ERROR Input Picture");
      }
    }
    else {
      Serial.println("ERROR Input Picture");
    }
  }
  else if (state == 1) {
    stateTime = millis();
    makeData("request", seq, 0, 0);
    //printBinary();
    Serial.println("Send request");
    delay(20);
    sendSignal();
    seq = (seq + 1) % 2;
    delay(20);
    if (getSignal(3)) {
      Serial.println(receive[0]);
      Serial.println(receive[1]);
      Serial.println(receive[2]);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              Serial.println("Receive ACK");
              state = 5;
            }
            else if (receiveText == "NAK") {
              Serial.println("Receive NAK");
              seq = (seq + 1) % 2;
            }
            else {
              Serial.println("Data ERROR(Non ACK)");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR SEQ");
            //seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR Flag");
        seq = (seq + 1) % 2;
      }
    }
    else {
      Serial.println("ERROR timeout");
      seq = (seq + 1) % 2;
    }
    while (millis() - stateTime < 2000 && state == 1);
  }
  else if (state == 2) {
    stateTime = millis();
    makeData("request", seq, 1, 0);
    //printBinary();
    Serial.println("Send request Left");
    sendSignal();
    seq = (seq + 1) % 2;
    delay(20);
    if (getSignal(3)) {
      //Serial.println(receive[0]);
      //Serial.println(receive[1]);
      //Serial.println(receive[2]);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              Serial.println("Receive ACK");
              state = 11;
            }
            else if (receiveText == "NAK") {
              Serial.println("Receive NAK");
              seq = (seq + 1) % 2;
            }
            else {
              Serial.println("Data ERROR(Non ACK)");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR SEQ");
            //seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR Flag");
        seq = (seq + 1) % 2;
      }
    }
    else {
      Serial.println("ERROR timeout");
      seq = (seq + 1) % 2;
    }
    while (millis() - stateTime < 2000 && state == 2);
  }
  else if (state == 3) {
    stateTime = millis();
    makeData("request", seq, 2, 0);
    //printBinary();
    Serial.println("Send request Mid");
    sendSignal();
    seq = (seq + 1) % 2;
    delay(20);
    if (getSignal(3)) {
      //Serial.println(receive[0]);
      //Serial.println(receive[1]);
      //Serial.println(receive[2]);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              Serial.println("Receive ACK");
              state = 13;
            }
            else if (receiveText == "NAK") {
              Serial.println("Receive NAK");
              seq = (seq + 1) % 2;
            }
            else {
              Serial.println("Data ERROR(Non ACK)");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR SEQ");
            //seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR Flag");
        seq = (seq + 1) % 2;
      }
    }
    else {
      Serial.println("ERROR timeout");
      seq = (seq + 1) % 2;
    }
    while (millis() - stateTime < 2000 && state == 3);
  }
  else if (state == 4) {
    stateTime = millis();
    makeData("request", seq, 4, 0);
    //printBinary();
    Serial.println("Send request Right");
    sendSignal();
    seq = (seq + 1) % 2;
    delay(20);
    if (getSignal(3)) {
      //Serial.println(receive[0]);
      //Serial.println(receive[1]);
      //Serial.println(receive[2]);
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "ACK") {
              Serial.println("Receive ACK");
              state = 15;
            }
            else if (receiveText == "NAK") {
              Serial.println("Receive NAK");
              seq = (seq + 1) % 2;
            }
            else {
              Serial.println("Data ERROR(Non ACK)");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR SEQ");
            //seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Parity");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR Flag");
        seq = (seq + 1) % 2;
      }
    }
    else {
      Serial.println("ERROR timeout");
      seq = (seq + 1) % 2;
    }
    while (millis() - stateTime < 2000 && state == 4);
  }
  else if (state == 5) {
    delay(2000);
    while (1) {
      stateTime = millis();
      makeData("ask", seq, 0, 0);
      //printBinary();
      Serial.println("Send ask");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
      if (getSignal(3)) {
        //Serial.println(receive[0],BIN);
        //Serial.println(receive[1],BIN);
        //Serial.println(receive[2],BIN);
        if (checkFlag) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              Serial.println(receiveText);
              if (receiveText == "processing") {
                Serial.println("Receive processing");
              }
              else if (receiveText == "detect right") {
                Serial.println("Receive detect right");
                state = 6;
                break;
              }
              else {
                Serial.println("Data ERROR(Non processing/detect)");
                seq = (seq + 1) % 2;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR Parity");
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Flag");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR timeout");
        seq = (seq + 1) % 2;
        delay(1500);
      }
      while (millis() - stateTime < 2000 && state == 5);
    }
  }
  else if (state  == 6) {
    stateTime = millis();
    if (!jump) {
      makeData("ACK", seq, 0, 0);
      //printBinary();
      Serial.println("Send ACK");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
    }
    if (getSignal(3)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "Data top") {
              Serial.println("Receive Data top");
              dataRight = "top";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 7;
              jump = false;
            }
            else if (receiveText == "Data bottom") {
              Serial.println("Receive Data bottom");
              dataRight = "bottom";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 7;
              jump = false;
            }
            else if (receiveText == "Data left") {
              Serial.println("Receive Data left");
              dataRight = "left";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 7;
              jump = false;
            }
            else if (receiveText == "Data right") {
              Serial.println("Receive Data right");
              dataRight = "right";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 7;
              jump = false;
            }
            else if (receiveText == "Data upper") {
              Serial.println("Receive Data upper");
              dataRight = "upper";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 7;
              jump = false;
            }
            else if (receiveText == "Data lower") {
              Serial.println("Receive Data lower");
              dataRight = "lower";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 7;
              jump = false;
            }
            else {
              Serial.println("Data ERROR(Non Data)");
              makeData("NAK", seq, 0, 0);
              Serial.println("Send NAK");
              sendSignal();
              jump = true;
            }
          }
          else {
            Serial.println("ERROR SEQ");
            makeData("NAK", seq, 0, 0);
            Serial.println("Send NAK");
            sendSignal();
            jump = true;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          Serial.println("Send NAK");
          sendSignal();
          jump = true;
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        Serial.println("Send NAK");
        sendSignal();
        jump = true;
      }
    }
    else {
      Serial.println("ERROR timeout");
      jump = false;
    }
    while (millis() - stateTime < 2000 && state == 6);
  }
  else if (state == 7) {
    delay(2000);
    while (1) {
      stateTime = millis();
      makeData("ask", seq, 0, 0);
      //printBinary();
      Serial.println("Send ask");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
      if (getSignal(3)) {
        //Serial.println(receive[0]);
        //Serial.println(receive[1]);
        //Serial.println(receive[2]);
        if (checkFlag) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "processing") {
                Serial.println("Receive processing");
              }
              else if (receiveText == "detect mid") {
                Serial.println("Receive detect mid");
                state = 8;
                break;
              }
              else {
                Serial.println("Data ERROR(Non processing/detect)");
                seq = (seq + 1) % 2;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR Parity");
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Flag");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR timeout");
        seq = (seq + 1) % 2;
        delay(1500);
      }
      while (millis() - stateTime < 2000 && state == 7);
    }
  }
  else if (state  == 8) {
    stateTime = millis();
    if (!jump) {
      makeData("ACK", seq, 0, 0);
      //printBinary();
      Serial.println("Send ACK");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
    }
    if (getSignal(3)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "Data top") {
              Serial.println("Receive Data top");
              dataMid = "top";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 9;
              jump = false;
            }
            else if (receiveText == "Data bottom") {
              Serial.println("Receive Data bottom");
              dataMid = "bottom";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 9;
              jump = false;
            }
            else if (receiveText == "Data left") {
              Serial.println("Receive Data left");
              dataMid = "left";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 9;
              jump = false;
            }
            else if (receiveText == "Data right") {
              Serial.println("Receive Data right");
              dataMid = "right";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 9;
              jump = false;
            }
            else if (receiveText == "Data upper") {
              Serial.println("Receive Data upper");
              dataMid = "upper";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 9;
              jump = false;
            }
            else if (receiveText == "Data lower") {
              Serial.println("Receive Data lower");
              dataMid = "lower";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(40);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 9;
              jump = false;
            }
            else {
              Serial.println("Data ERROR(Non Data)");
              makeData("NAK", seq, 0, 0);
              Serial.println("Send NAK");
              sendSignal();
              jump = true;
            }
          }
          else {
            Serial.println("ERROR SEQ");
            makeData("NAK", seq, 0, 0);
            Serial.println("Send NAK");
            sendSignal();
            jump = true;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          Serial.println("Send NAK");
          sendSignal();
          jump = true;
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        Serial.println("Send NAK");
        sendSignal();
        jump = true;
      }
    }
    else {
      Serial.println("ERROR timeout");
      jump = false;
    }
    while (millis() - stateTime < 2000 && state == 8);
  }
  else if (state == 9) {
    delay(2000);
    while (1) {
      stateTime = millis();
      makeData("ask", seq, 0, 0);
      //printBinary();
      Serial.println("Send ask");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
      if (getSignal(3)) {
        //Serial.println(receive[0]);
        //Serial.println(receive[1]);
        //Serial.println(receive[2]);
        if (checkFlag) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "processing") {
                Serial.println("Receive processing");
              }
              else if (receiveText == "detect left") {
                Serial.println("Receive detect left");
                state = 10;
                break;
              }
              else {
                Serial.println("Data ERROR(Non processing/detect)");
                seq = (seq + 1) % 2;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR Parity");
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Flag");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR timeout");
        seq = (seq + 1) % 2;
        delay(1500);
      }
      while (millis() - stateTime < 2000 && state == 9);
    }
  }
  else if (state  == 10) {
    stateTime = millis();
    if (!jump) {
      makeData("ACK", seq, 0, 0);
      //printBinary();
      Serial.println("Send ACK");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
    }
    if (getSignal(3)) {
      if (checkFlag) {
        if (checkParity(receive[1])) {
          if (checkSeq()) {
            translateData();
            if (receiveText == "Data top") {
              Serial.println("Receive Data top");
              dataLeft = "top";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(80);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 0;
              jump = false;
              printPicture();
            }
            else if (receiveText == "Data bottom") {
              Serial.println("Receive Data bottom");
              dataLeft = "bottom";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(80);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 0;
              jump = false;
              printPicture();
            }
            else if (receiveText == "Data left") {
              Serial.println("Receive Data left");
              dataLeft = "left";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(80);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 0;
              jump = false;
              printPicture();
            }
            else if (receiveText == "Data right") {
              Serial.println("Receive Data right");
              dataLeft = "right";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(80);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 0;
              jump = false;
              printPicture();
            }
            else if (receiveText == "Data upper") {
              Serial.println("Receive Data upper");
              dataLeft = "upper";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(80);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 0;
              jump = false;
              printPicture();
            }
            else if (receiveText == "Data lower") {
              Serial.println("Receive Data lower");
              dataLeft = "lower";
              makeData("ACK", seq, 0, 0);
              //printBinary();
              Serial.println("Send ACK");
              delay(80);
              sendSignal();
              seq = (seq + 1) % 2;
              state = 0;
              jump = false;
              printPicture();
            }
            else {
              Serial.println("Data ERROR(Non Data)");
              makeData("NAK", seq, 0, 0);
              Serial.println("Send NAK");
              sendSignal();
              jump = true;
            }
          }
          else {
            Serial.println("ERROR SEQ");
            makeData("NAK", seq, 0, 0);
            Serial.println("Send NAK");
            sendSignal();
            jump = true;
          }
        }
        else {
          Serial.println("ERROR Parity");
          makeData("NAK", seq, 0, 0);
          Serial.println("Send NAK");
          sendSignal();
          jump = true;
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        Serial.println("Send NAK");
        sendSignal();
        jump = true;
      }
    }
    else {
      Serial.println("ERROR timeout");
      jump = false;
    }
    while (millis() - stateTime < timeout && state == 10);
  }
  else if (state == 11) {
    delay(2000);
    while (1) {
      stateTime = millis();
      makeData("ask", seq, 0, 0);
      //printBinary();
      Serial.println("Send ask");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
      if (getSignal(3)) {
        //Serial.println(receive[0]);
        //Serial.println(receive[1]);
        //Serial.println(receive[2]);
        if (checkFlag) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "processing") {
                Serial.println("Receive processing");
              }
              else if (receiveText == "detect left") {
                Serial.println("Receive detect left");
                state = 12;
                count16bit = 0;
                break;
              }
              else {
                Serial.println("Data ERROR(Non processing/detect)");
                seq = (seq + 1) % 2;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR Parity");
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Flag");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR timeout");
        seq = (seq + 1) % 2;
        delay(500);
      }
      while (millis() - stateTime < 2000 && state == 11);
    }
  }
  else if (state  == 12) {
    stateTime = millis();
    if (!jump) {
      makeData("ACK", seq, 0, 0);
      //printBinary();
      Serial.println("Send ACK");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
    }
    if (getSignal(29)) {
      if (checkFlag) {
        if (checkSum) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "Data 16 bit") {
                Serial.println("Receive Data 16 bit");
                for (int i = 0; i < 25; i++) {
                  dataT[count16bit][i] = receive[i + 2];
                }
                Serial.println("-------------data-----------");
                for (int i=0;i<25;i++){
                  Serial.println(dataT[count16bit][i],BIN);
                }
                Serial.println("-------------data-----------");
                count16bit++;
                makeData("ACK", seq, 0, 0);
                //printBinary();
                Serial.println("Send ACK");
                delay(40);
                sendSignal();
                seq = (seq + 1) % 2;
                jump = false;
                if (count16bit == 2) {
                  state = 0;
                  decoder();
                  print16bit(0);
                }
                else {
                  state = 12;
                  jump = true;
                }
              }
              else {
                Serial.println("Data ERROR(Non Data)");
                makeData("NAK", seq, 0, 0);
                Serial.println("Send NAK");
                sendSignal();
                jump = true;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              makeData("NAK", seq, 0, 0);
              Serial.println("Send NAK");
              sendSignal();
              jump = true;
            }
          }
          else {
            Serial.println("ERROR Parity");
            makeData("NAK", seq, 0, 0);
            Serial.println("Send NAK");
            sendSignal();
            jump = true;
          }
        }
        else {
          Serial.println("ERROR checkSUM");
          makeData("NAK", seq, 0, 0);
          Serial.println("Send NAK");
          sendSignal();
          jump = true;
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        Serial.println("Send NAK");
        sendSignal();
        jump = true;
      }
    }
    else {
      Serial.println("ERROR timeout");
      jump = false;
    }
    while (millis() - stateTime < 2000 && state == 12);
  }
  else if (state == 13) {
    delay(2000);
    while (1) {
      stateTime = millis();
      makeData("ask", seq, 0, 0);
      //printBinary();
      Serial.println("Send ask");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
      if (getSignal(3)) {
        //Serial.println(receive[0]);
        //Serial.println(receive[1]);
        //Serial.println(receive[2]);
        if (checkFlag) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "processing") {
                Serial.println("Receive processing");
              }
              else if (receiveText == "detect mid") {
                Serial.println("Receive detect mid");
                state = 14;
                count16bit = 0;
                break;
              }
              else {
                Serial.println("Data ERROR(Non processing/detect)");
                seq = (seq + 1) % 2;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR Parity");
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Flag");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR timeout");
        seq = (seq + 1) % 2;
        delay(500);
      }
      while (millis() - stateTime < 2000 && state == 13);
    }
  }
  else if (state  == 14) {
    stateTime = millis();
    if (!jump) {
      makeData("ACK", seq, 0, 0);
      //printBinary();
      Serial.println("Send ACK");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
    }
    if (getSignal(29)) {
      if (checkFlag) {
        if (checkSum) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "Data 16 bit") {
                Serial.println("Receive Data 16 bit");
                for (int i = 0; i < 25; i++) {
                  dataT[count16bit][i] = receive[i + 2];
                }
                Serial.println("-------------data-----------");
                for (int i=0;i<25;i++){
                  Serial.println(dataT[count16bit][i],BIN);
                }
                Serial.println("-------------data-----------");
                count16bit++;
                makeData("ACK", seq, 0, 0);
                //printBinary();
                Serial.println("Send ACK");
                delay(40);
                sendSignal();
                seq = (seq + 1) % 2;
                jump = false;
                if (count16bit == 2) {
                  state = 0;
                  decoder();
                  print16bit(1);
                }
                else {
                  state = 14;
                  jump = true;
                }
              }
              else {
                Serial.println("Data ERROR(Non Data)");
                makeData("NAK", seq, 0, 0);
                Serial.println("Send NAK");
                sendSignal();
                jump = true;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              makeData("NAK", seq, 0, 0);
              Serial.println("Send NAK");
              sendSignal();
              jump = true;
            }
          }
          else {
            Serial.println("ERROR Parity");
            makeData("NAK", seq, 0, 0);
            Serial.println("Send NAK");
            sendSignal();
            jump = true;
          }
        }
        else {
          Serial.println("ERROR checkSUM");
          makeData("NAK", seq, 0, 0);
          Serial.println("Send NAK");
          sendSignal();
          jump = true;
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        Serial.println("Send NAK");
        sendSignal();
        jump = true;
      }
    }
    else {
      Serial.println("ERROR timeout");
      jump = false;
    }
    while (millis() - stateTime < 2000 && state == 14);
  }
  else if (state == 15) {
    delay(2000);
    while (1) {
      stateTime = millis();
      makeData("ask", seq, 0, 0);
      //printBinary();
      Serial.println("Send ask");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
      if (getSignal(3)) {
        //Serial.println(receive[0]);
        //Serial.println(receive[1]);
        //Serial.println(receive[2]);
        if (checkFlag) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "processing") {
                Serial.println("Receive processing");
              }
              else if (receiveText == "detect right") {
                Serial.println("Receive detect right");
                state = 16;
                count16bit = 0;
                break;
              }
              else {
                Serial.println("Data ERROR(Non processing/detect)");
                seq = (seq + 1) % 2;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              seq = (seq + 1) % 2;
            }
          }
          else {
            Serial.println("ERROR Parity");
            seq = (seq + 1) % 2;
          }
        }
        else {
          Serial.println("ERROR Flag");
          seq = (seq + 1) % 2;
        }
      }
      else {
        Serial.println("ERROR timeout");
        seq = (seq + 1) % 2;
        delay(500);
      }
      while (millis() - stateTime < 2000 && state == 15);
    }
  }
  else if (state  == 16) {
    stateTime = millis();
    if (!jump) {
      makeData("ACK", seq, 0, 0);
      //printBinary();
      Serial.println("Send ACK");
      sendSignal();
      seq = (seq + 1) % 2;
      delay(20);
    }
    if (getSignal(29)) {
      if (checkFlag) {
        if (checkSum) {
          if (checkParity(receive[1])) {
            if (checkSeq()) {
              translateData();
              if (receiveText == "Data 16 bit") {
                Serial.println("Receive Data 16 bit");
                for (int i = 0; i < 25; i++) {
                  dataT[count16bit][i] = receive[i + 2];
                }
                Serial.println("-------------data-----------");
                for (int i=0;i<25;i++){
                  Serial.println(dataT[count16bit][i],BIN);
                }
                Serial.println("-------------data-----------");
                count16bit++;
                makeData("ACK", seq, 0, 0);
                //printBinary();
                Serial.println("Send ACK");
                delay(40);
                sendSignal();
                seq = (seq + 1) % 2;
                jump = false;
                if (count16bit == 2) {
                  state = 0;
                  decoder();
                  print16bit(2);
                }
                else {
                  state = 16;
                  jump = true;
                }
              }
              else {
                Serial.println("Data ERROR(Non Data)");
                makeData("NAK", seq, 0, 0);
                Serial.println("Send NAK");
                sendSignal();
                jump = true;
              }
            }
            else {
              Serial.println("ERROR SEQ");
              makeData("NAK", seq, 0, 0);
              Serial.println("Send NAK");
              sendSignal();
              jump = true;
            }
          }
          else {
            Serial.println("ERROR Parity");
            makeData("NAK", seq, 0, 0);
            Serial.println("Send NAK");
            sendSignal();
            jump = true;
          }
        }
        else {
          Serial.println("ERROR checkSUM");
          makeData("NAK", seq, 0, 0);
          Serial.println("Send NAK");
          sendSignal();
          jump = true;
        }
      }
      else {
        Serial.println("ERROR Flag");
        makeData("NAK", seq, 0, 0);
        Serial.println("Send NAK");
        sendSignal();
        jump = true;
      }
    }
    else {
      Serial.println("ERROR timeout");
      jump = false;
    }
    while (millis() - stateTime < 2000 && state == 16);
  }
}
