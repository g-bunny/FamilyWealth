#include <LiquidCrystal.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_REALACCEL
LiquidCrystal lcd(12,11,5,4,3,2);



//---------------------------------------------- OUR CODE -------------------------------------------------//
boolean hasMonster = false;
boolean hasTreasure = false;
boolean newGame = true;
boolean navigating = false;
boolean fightState = false;

const int upPin = A0;
const int downPin = A2;
const int leftPin = A1;
const int rightPin = 10;
const int attackPin = 9;
const int regenPin = A4;

const int textDelay = 3000;
const int winningLootCount = 100;

int attackRate = 1; //how fast player health decreases

int monsterHealth = 100;
const int monsterMaxHealth = 100;
int playerHealth = 100;
const int playerMaxHealth = 100;
int playerLoot = 0;

int upState = 0;
int downState = 0;
int leftState = 0;
int rightState = 0;
int attackState = 0;
int regenState = 0;

int attackCounter = 0;
int regenCounter = 0;

int lastUpState = LOW;
int lastDownState = LOW;
int lastLeftState = LOW;
int lastRightState = LOW;
int lastAttackState = LOW;
int lastRegenState = LOW;

long lastDebounceTimeA = 0;
long lastDebounceTimeR = 0;
long debounceDelay = 50;

int dataIn = 6;
int load = 7;
int clock = 13;
boolean theMap[8][8];
int treasureRate = 625; // Out of 10,000
int playerX = 4;    //Player start X position
int playerY = 4;    //Player start Y position
int accelRate = 0;
int monsterLootValue = 0;
const long interval = 250;
unsigned long previousMillis = 0;
int ledState = LOW;
int maxInUse = 1;    //change this variable to set how many MAX7219's you'll use
int e = 0;           // just a variable

//-------------------------------------------------------------------------------------------------------------//

//---------------------------------------------- NOT OUR CODE -------------------------------------------------//

// define max7219 registers
byte max7219_reg_noop        = 0x00;
byte max7219_reg_digit0      = 0x01;
byte max7219_reg_digit1      = 0x02;
byte max7219_reg_digit2      = 0x03;
byte max7219_reg_digit3      = 0x04;
byte max7219_reg_digit4      = 0x05;
byte max7219_reg_digit5      = 0x06;
byte max7219_reg_digit6      = 0x07;
byte max7219_reg_digit7      = 0x08;
byte max7219_reg_decodeMode  = 0x09;
byte max7219_reg_intensity   = 0x0a;
byte max7219_reg_scanLimit   = 0x0b;
byte max7219_reg_shutdown    = 0x0c;
byte max7219_reg_displayTest = 0x0f;
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;    
//-------------------------------------------------------------------------------------------------------------//
















//---------------------------------------------- OUR FUNCTIONS -------------------------------------------------//

void drawMap() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    theMap[playerX][playerY] = !theMap[playerX][playerY];
  }
  for (int mapX = 0; mapX <= 7; mapX++) {
    double binary = 0;
    for (int mapY = 0; mapY <=7; mapY++) {
      if (theMap[mapX][mapY]) { 
        binary = binary + pow(2,mapY);        
      }
    }    
    if (binary > 3) {  //Dirty method to fix int() casting/conversion error
      binary+=1;
    }
    maxSingle (mapX+1, binary);
  }  
}

void newMap() {
  for (int x = 0; x <=7; x++) {
    for (int y = 0; y <=7; y ++) {
      long randomTreasure = random(10000);
      if (randomTreasure < treasureRate) {
        theMap[x][y] = true;
      } 
      else {
        theMap[x][y] = false;
      }
    }
  }
}

void setup () {

  newMap();
  pinMode(dataIn, OUTPUT);
  pinMode(clock,  OUTPUT);
  pinMode(load,   OUTPUT);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("HI"); 

//-------------------------------------------------------------------------------------------------------------//

//---------------------------------------------- NOT OUR CODE -------------------------------------------------//
  //initiation of the max 7219
  maxAll(max7219_reg_scanLimit, 0x07);      
  maxAll(max7219_reg_decodeMode, 0x00);  // using an led matrix (not digits)
  maxAll(max7219_reg_shutdown, 0x01);    // not in shutdown mode
  maxAll(max7219_reg_displayTest, 0x00); // no display test
  for (e=1; e<=8; e++) {    // empty registers, turn all LEDs off 
    maxAll(e,0);
  }
  maxAll(max7219_reg_intensity, 0x0f & 0x0f);    // the first 0x0f is the value you can set
  // range: 0x00 to 0x0f
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
//-------------------------------------------------------------------------------------------------------------//

//---------------------------------------------- OUR CODE -------------------------------------------------//
  pinMode(upPin, INPUT_PULLUP);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  pinMode(attackPin, INPUT_PULLUP);
  pinMode(regenPin, INPUT_PULLUP);
  pinMode (A3, OUTPUT);
}  

void loop () {
  lcd.setCursor(0,0);
  lcd.print("Monster Loot:");
  lcd.setCursor(13,0);
  lcd.print(monsterLootValue);

  if (newGame == true) {
    story();
    newGame == false;
  }
//-------------------------------------------------------------------------------------------------------------//

//---------------------------------------------- NOT OUR CODE -------------------------------------------------//
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
//-------------------------------------------------------------------------------------------------------------//

//---------------------------------------------- OUR CODE -------------------------------------------------//
//    if(digitalRead(UpPin) == LOW){
//      Serial.print("1");
//    }
//    else  {
//      Serial.print("0");
//    }
//    Serial.print(",");
//    
//    if(digitalRead(DownPin) == LOW){
//      Serial.print("1");
//    }
//    else  {
//      Serial.print("0");
//    }
//    Serial.print(",");
//        
//    if (digitalRead(RightPin) == LOW) {
//      Serial.print("1");
//    } 
//    else  {
//      Serial.print("0");
//    }
//    Serial.print(",");
//    if (digitalRead(LeftPin) == LOW) {
//      Serial.print("1");
//    } 
//    else  {
//      Serial.print("0");
//    }
//    Serial.print(",");
//    if (digitalRead(AttackPin) == LOW) {
//      Serial.print("1");
//    } 
//    else  {
//      Serial.print("0");
//    }
//    Serial.print(",");
//        if (digitalRead(RegenPin) == LOW) {
//      Serial.print("1");
//    } 
//    else  {
//      Serial.print("0");
//    }
//    Serial.println(",");
//    Serial.println("accelRate");
//    Serial.println(accelRate);
//    Serial.println("monsterLoot");
//    Serial.println(monsterLootValue);      

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.print(aaReal.z);
    Serial.print("\t");
    
#endif
    insideLoop();
  }
}

void fightmode() {
  monsterLootCheck();
  drawHealth();
  playerHealth = playerHealth - attackRate;

  int lootValue = monsterLootValue;

  int readingA = digitalRead(attackPin);
  if (readingA != lastAttackState) {
    lastDebounceTimeA = millis();
  }
  if ((millis() - lastDebounceTimeA) > debounceDelay) {
    if (readingA != attackState) {
      attackState = readingA;
      if (attackState == HIGH) {
        attackCounter++;
        Serial.print("Attacks: ");
        Serial.println(attackCounter);
      }
    }
  }
  lastAttackState = readingA;

  //READING REGEN BUTTON
  int readingReg = digitalRead(regenPin);
  if (readingReg != lastRegenState) {
    lastDebounceTimeR = millis();
  }
  if ((millis() - lastDebounceTimeR) > debounceDelay) {
    if (readingReg != regenState) {
      regenState = readingReg;
      if (regenState == HIGH) {
        regenCounter++;
        Serial.print("Regens: ");
        Serial.println(regenCounter);
      }
    }
  }
  lastRegenState = readingReg;

  //UPDATE DISPLAY
  playerHealth = playerHealth + regenCounter;
  monsterHealth = monsterHealth - attackCounter;

  if (playerHealth <= 0 && monsterHealth > 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You died. Potham");
    lcd.setCursor(0, 1);
    lcd.print("is going down.");
//    delay(textDelay);
    lcd.clear();
  }

  else if (monsterHealth <= 0 && playerHealth > 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You killed it!!");
    lcd.setCursor(0, 1);
    lcd.print("Loot: ");
    lcd.setCursor(7, 1);
    lcd.print(lootValue);
//    delay(textDelay);
    lcd.clear();
    playerLoot = playerLoot + lootValue;
  }

  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You   || Monster");
    lcd.setCursor(2, 1);
    lcd.print(playerHealth);
    lcd.setCursor(12, 1);
    lcd.print(monsterHealth);
  }
}

void story() {
  downState = digitalRead(downPin);
  if (downState != lastDownState) {
    if (downState == HIGH) {
      lcd.clear();
      //LOADMAP???
    }
    //    delay(50);
  }
  lastDownState = downState;
}

void monsterLootCheck(){
  if (aaReal.x > 900 || aaReal.x < -900 || aaReal.y > 1000 || aaReal.y < -1000 || aaReal.z > 2000 || aaReal.z < -2000){
    accelRate = 1;
    monsterLootValue += 1;
  } 
  else {
    accelRate = 0;
  }
}

void drawHealth() {
  int playerBar = map(playerHealth, 0, playerMaxHealth, 0, 8);
  int monsterBar = map(monsterHealth, 0, monsterMaxHealth, 0, 8);
  double playerBinary = 0;
  double monsterBinary = 0;
  for (int i=0;i<=playerBar;i++) {
    playerBinary = playerBinary + pow(2, i);
  }
  if (playerBinary > 3) {  //Dirty method to fix int() casting/conversion error
    playerBinary+=1;
  }
  for (int i=0;i<monsterBar;i++) {
    monsterBinary = monsterBinary + pow(2, i);
  }
  if (monsterBinary > 3) {  //Dirty method to fix int() casting/conversion error
    monsterBinary+=1;
  }
  maxSingle (1, monsterBinary);
  maxSingle (2, monsterBinary);
  maxSingle (7, playerBinary);
  maxSingle (8, playerBinary);
}

void insideLoop() {
  //tempLoop();
  Serial.print("Monster Health: ");
  Serial.println(monsterHealth);
  if (monsterHealth == 0) {
    navigationButtons();
    drawMap(); 
  } else {
    //fightmode();
    drawHealth();    
  }
}

void tempLoop() {
  //single LED
    if (accelRate ==1){
      digitalWrite(A3, HIGH);
    } 
    else {
      digitalWrite(A3, LOW);
    }
    if(digitalRead(10) == LOW){
    int monsterEncounter = random(0,2);
    drawMap();
    if (fightState) {
      fightmode();
      Serial.print( "Monster Health: ");
      Serial.print(monsterHealth);
      if (monsterHealth <= 0){
        fightState = false;
      }
    } else {
     drawMap(); 
    }
  }
  //
  else if (hasTreasure == true) {
    int roomTreasure = int(random(1, 10));
    lcd.setCursor(0, 0);
    lcd.print("A treasure box!");
    lcd.setCursor(0, 1);
    lcd.print(roomTreasure);
    lcd.setCursor(3, 1);
    lcd.print("gold bars found!");
  }

  else {
    lcd.setCursor(0, 0);
    lcd.print("Room is empty.");
    lcd.setCursor(0, 1);
    lcd.print("Go to next room.");
  }

  //show health and loot value in navigation
  if (navigating == true) {
    lcd.setCursor(0, 0);
    lcd.print("Use D-pad to");
    lcd.setCursor(0, 1);
    lcd.print("go to next room.");
//    delay(textDelay);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Your health:");
    lcd.setCursor(14, 0);
    lcd.print(playerHealth);
    lcd.setCursor(0, 1);
    lcd.print("Loot earned: ");
    lcd.setCursor(13, 1);
    lcd.print(playerLoot);
//    delay(textDelay);
    lcd.clear();
  }
}

void navigationButtons() {
    if(digitalRead(upPin)){
      upState = 1;
    } else {
      upState = 0;
    }
    if(digitalRead(downPin)){
      downState = 1;
    } else {
      downState = 0;
    }
    if(digitalRead(leftPin)){
      leftState = 1;
    } else {
      leftState = 0;
    }
    if(digitalRead(rightPin)){
      rightState = 1;
    } else {
      rightState = 0;
    }  
}

void movePlayer(){
  if(upState ==1){
    playerY -=1;
  }
  if(downState ==1){
    playerY +=1;
  }
  if(leftState == 1){
    playerX -= 1;
  }
  if(rightState == 1){
    playerX +=1;
  }
}


//-------------------------------------------------------------------------------------------------------------//

//---------------------------------------------- NOT OUR CODE -------------------------------------------------//












































void maxSingle( byte reg, byte col) {    
  //maxSingle is the "easy"  function to use for a single max7219

  digitalWrite(load, LOW);       // begin     
  putByte(reg);                  // specify register
  putByte(col);//((data & 0x01) * 256) + data >> 1); // put data   
  digitalWrite(load, LOW);       // and load da stuff
  digitalWrite(load,HIGH); 
}

void maxAll (byte reg, byte col) {    // initialize  all  MAX7219's in the system
  int c = 0;
  digitalWrite(load, LOW);  // begin     
  for ( c =1; c<= maxInUse; c++) {
    putByte(reg);  // specify register
    putByte(col);//((data & 0x01) * 256) + data >> 1); // put data
  }
  digitalWrite(load, LOW);
  digitalWrite(load,HIGH);
}

void maxOne(byte maxNr, byte reg, byte col) {    
  //maxOne is for addressing different MAX7219's, 
  //while having a couple of them cascaded

  int c = 0;
  digitalWrite(load, LOW);  // begin     

  for ( c = maxInUse; c > maxNr; c--) {
    putByte(0);    // means no operation
    putByte(0);    // means no operation
  }

  putByte(reg);  // specify register
  putByte(col);//((data & 0x01) * 256) + data >> 1); // put data 

  for ( c =maxNr-1; c >= 1; c--) {
    putByte(0);    // means no operation
    putByte(0);    // means no operation
  }

  digitalWrite(load, LOW); // and load da stuff
  digitalWrite(load,HIGH); 
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void putByte(byte data) {
  byte i = 8;
  byte mask;
  while(i > 0) {
    mask = 0x01 << (i - 1);      // get bitmask
    digitalWrite( clock, LOW);   // tick
    if (data & mask){            // choose bit
      digitalWrite(dataIn, HIGH);// send 1
    }//else{
    //      digitalWrite(dataIn, LOW); // send 0
    //    }
    digitalWrite(clock, HIGH);   // tock
    --i;                         // move to lesser bit
  }
}
