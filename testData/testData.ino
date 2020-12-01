uint8_t chars[29];
uint8_t charsLen;
uint8_t color[16];
uint8_t width[16];
uint16_t height[16];
uint8_t dataT[2][25];
uint8_t receive[29];

char pint[200];

void setCharsZero(){
  for (int i=0;i<29;i++){
    chars[i] = 0;
  }
  charsLen = 0;
}

void setDataTZero(){
  for (int i=0;i<2;i++){
    for (int j=0;j<25;j++){
      dataT[i][j] = 0;
    }
  }
}

void setDataZero(){
  for (int i=0;i<16;i++){
    color[i] = 0;
    width[i] = 0;
    height[i] = 0;
  }
}

void encoder(){
  for (int i=0;i<2;i++){
    uint8_t remain = 0;
    for (int j=0;j<8;j++){
      dataT[i][j*3] = height[j+(8*i)] & 0xFF;
      dataT[i][(j*3)+1] = width[j+(8*i)];
      dataT[i][(j*3)+2] = color[j+(8*i)];
      remain <<= 1;
      remain |= (height[j+(8*i)] & 0x100) >> 8;
    }
    dataT[i][24] = remain;
  }
}

void printCode(){
  for (int j=0;j<2;j++){
  for (int i=0;i<25;i++){
    if (i%5==0)
      Serial.println();
    Serial.print(dataT[j][i], BIN);
    if(dataT[j][i]>127)
      Serial.print("\t");
    else
      Serial.print("\t\t");
  }
  Serial.println();
  }
}

void generateCode(){
  for (int i=0;i<16;i++){
    height[i] = 20*i;
    width[i] = 15*i;
    color[i] = 16*i;
    sprintf(pint,"%X %X %X",height[i],width[i],color[i]);
    Serial.println(pint);
  }
}

uint8_t makeSum(uint8_t header,uint8_t num){
  uint16_t sum = header;
  for (int i=0;i<25;i++){
    sum += dataT[num][i];
  }
  while((sum&0xFF00) != 0){
    sum = (sum&0x00FF) + (sum>>8);
  }
  return ~(0x00FF&sum);
}

bool checkSum(){
  uint16_t sum = 0;
  for (int i=1;i<27;i++){
    sum += receive[i];
  }
  while((sum&0xFF00) != 0){
    sum = (sum&0x00FF) + (sum>>8);
  }
  sum += receive[27];
  while((sum&0xFF00) != 0){
    sum = (sum&0x00FF) + (sum>>8);
  }
  return (sum == 0xFF) ? true:false;
}

uint8_t makeParity(uint8_t header){
  uint8_t parity = 0;
  for (int i=0; i<7; i++){
    parity ^= (header & (1 << (7 - i))) >> (7 - i);
  }
  header |= parity;
  return header;
}

bool checkParity(uint8_t header){
  uint8_t parity = 0;
  for (int i=0;i<8;i++){
    parity ^= header & 1;
    header >>= 1;
  }
  return (parity) ? false : true;
}
void makeData(String header, uint8_t seq_No, uint8_t size_No, uint8_t num){
  setCharsZero();
  Serial.println(header);
  if(header == "request"){
      chars[0] = B10011001;
      chars[1] = B00000000 + (seq_No << 4) + (size_No << 1);
      chars[1] = makeParity(chars[1]);
      chars[2] = B10011001;
      charsLen = 3;
  }
  else if(header == "ACK"){
      chars[0] = B10011001;
      chars[1] = B01000000 + (seq_No << 4);
      chars[1] = makeParity(chars[1]);
      chars[2] = B10011001;
      charsLen = 3;
  }
  else if(header == "NAK"){
      chars[0] = B10011001;
      chars[1] = B01100000 + (seq_No << 4);
      chars[1] = makeParity(chars[1]);
      chars[2] = B10011001;
      charsLen = 3;
  }
  else if(header == "ask"){
      chars[0] = B10011001;
      chars[1] = B10000000 + (seq_No << 4);
      chars[1] = makeParity(chars[1]);
      chars[2] = B10011001;
  }
  else if(header == "processing"){
      chars[0] = B10011001;
      chars[1] = B10100000 + (seq_No << 4);
      chars[1] = makeParity(chars[1]);
      chars[2] = B10011001;
      charsLen = 3;
  }
  else if(header == "detect"){
      chars[0] = B10011001;
      chars[1] = B11000000 + (seq_No << 4) + (size_No << 1);
      chars[1] = makeParity(chars[1]);
      chars[2] = B10011001;
      charsLen = 3;
  }
  else if(header == "data"){
      chars[0] = B10011001;
      chars[1] = B00100000 + (seq_No << 4) + (size_No << 1);
      chars[1] = makeParity(chars[1]);
      if(size_No == 6){
        for (int i=0;i<26;i++){
          chars[i+2] = dataT[num][i];
        }
        chars[27] = makeSum(chars[1], num);
        chars[28] = B10011001;
        charsLen = 29;
      }
      else{
        chars[2] = B10011001;
        charsLen = 3;
      }
  }
  else{
      Serial.println("ERROR makeData");
      return;
  }
}
void printBinary(){
  for(int i=0;i<charsLen;i++){
    if (i%4==0)
      Serial.println();
    Serial.print(chars[i], BIN);
    if(chars[i]>127)
      Serial.print("\t");
    else
      Serial.print("\t\t");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.flush();
  uint8_t count = makeParity(B10111000);
  /*Serial.println(count, BIN);
  if(checkParity(count)){
    Serial.println("eiei");
  }
  else{
    Serial.println("ohoh");
  }
  if(checkParity(count+1)){
    Serial.println("eiei");
  }
  else{
    Serial.println("ohoh");
  }*/
  generateCode();
  encoder();
  printCode();
  makeData("data",1,6,0);
  printBinary();
  for (int i=0;i<28;i++){
    receive[i] = chars[i];
  }
  if(checkSum()){
    Serial.println("eiei");
  }
  else{
    Serial.println("ohoh");
  }
}

void loop() {/*
  if(Serial.available()){
  setDataZero();
  String a = Serial.readString();
  Serial.println("receive string");
  Serial.println(a);
  uint8_t b;
  b = 15;
  Serial.println("receive int");
  for (int i=0;i<16;i++){
    if(i<10)
      data[i] = 153;
    else
      data[i] = 3;
    Serial.print("receive int ");
    Serial.println(i);
    Serial.println(data[i]);
  }
  dataLen = 16;
  Serial.println("eiei");
  makeData(a,b);
  Serial.println("finish");
  Serial.println(charsLen);
  printBinary();
  uint16_t c=0;
  for(int i=0;i<dataLen;i++){
    c+=data[i];
    Serial.println(c,BIN);
  }
  c+=B10001111;
  c+=checksum(B10001111);
  Serial.println(c,BIN);
  while((c&0xFF00) != 0){
    c = (c&0x00FF) + (c>>8);
  }
  Serial.println(c&0x00FF,BIN);
  }*/
}
