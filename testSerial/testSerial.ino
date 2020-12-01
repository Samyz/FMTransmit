uint16_t height[16];
uint8_t width[16];
uint8_t color[16];
int numdata = 0;

void receiveSerial() {
  String a = "";
  String b = "";
  String c = "";
  while (Serial.available() >= 9) {
    for (int i = 0; i < 3; i++) {
      a += Serial.read() - '0';
      Serial.println(a);
    }
    for (int i = 0; i < 3; i++) {
      b += Serial.read() - '0';
      Serial.println(b);
    }
    for (int i = 0; i < 3; i++) {
      c += Serial.read() - '0';
      Serial.println(c);
    }
    Serial.print("a");
    Serial.println(a[0]);
    Serial.print("b");
    Serial.println(b[0]);
    Serial.print("c");
    Serial.println(c);
    height[numdata] = a.toInt();
    width[numdata] = b.toInt();
    color[numdata] = c.toInt();
    Serial.print("a");
    Serial.println(height[numdata]);
    Serial.print("b");
    Serial.println(width[numdata]);
    Serial.print("c");
    Serial.println(color[numdata]);
    numdata++;
    a = "";
    b = "";
    c = "";
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  receiveSerial();

}
