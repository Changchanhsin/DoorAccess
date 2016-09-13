#include <SPI.h>


////////// ////////// ////////// ////////// //////////
///  Pinout  //////// ////////// ////////// //////////

#define SPI_PIN_RST   9
#define SPI_PIN_SS   10   // Chip Select, always LOW
#define SPI_PIN_MOSI 11
#define SPI_PIN_MISO 12
#define SPI_PIN_SCK  13

#define OUTPUT_PIN_MAGENT      5  // LOW for DISABLE; HIGH for ENABLE
#define OUTPUT_PIN_REDLIGHT    3  // LOW for CLOSE; HIGH for OPEN
#define OUTPUT_PIN_GREENLIGHT  2  // LOW for CLOSE; HIGH for OPEN
#define OUTPUT_PIN_YELLOWLIGHT 4  // LOW for CLOSE; HIGH for OPEN

#define INPUT_PIN_OPEN       6  // LOW for CLOSE; HIGH for OPEN
#define INPUT_PIN_ADDUSER    7  // LOW for CLOSE; HIGH for OPEN
#define INPUT_PIN_DELETEUSER 8  // LOW for CLOSE; HIGH for OPEN


////////// ////////// ////////// ////////// //////////
///  MF522 Area begin  ///////// ////////// //////////

//data array maxium length
#define MAX_LEN 16

//MF522 command bits
#define PCD_IDLE 0x00       // NO action; cancel current commands
#define PCD_AUTHENT 0x0E    // verify password key
#define PCD_RECEIVE 0x08    // receive data
#define PCD_TRANSMIT 0x04   // send data
#define PCD_TRANSCEIVE 0x0C // send and receive data
#define PCD_RESETPHASE 0x0F // reset
#define PCD_CALCCRC 0x03    // CRC check and caculation

//Mifare_One card command bits
#define PICC_REQIDL 0x26    // Search the cards that not into sleep mode in the antenna area 
#define PICC_REQALL 0x52    // Search all the cards in the antenna area
#define PICC_ANTICOLL 0x93  // prevent conflict
#define PICC_SElECTTAG 0x93 // select card
#define PICC_AUTHENT1A 0x60 // verify A password key
#define PICC_AUTHENT1B 0x61 // verify B password key
#define PICC_READ 0x30      // read 
#define PICC_WRITE 0xA0     // write
#define PICC_DECREMENT 0xC0 // deduct value
#define PICC_INCREMENT 0xC1 // charge up value
#define PICC_RESTORE 0xC2   // Restore data into buffer
#define PICC_TRANSFER 0xB0  // Save data into buffer
#define PICC_HALT 0x50      // sleep mode

//THe mistake code that return when communicate with MF522
#define MI_OK       0
#define MI_NOTAGERR 1
#define MI_ERR      2

//------------------MFRC522 register ---------------
//Page 0:Command and Status
#define Reserved00    0x00 
#define CommandReg    0x01 
#define CommIEnReg    0x02 
#define DivlEnReg     0x03 
#define CommIrqReg    0x04 
#define DivIrqReg     0x05
#define ErrorReg      0x06 
#define Status1Reg    0x07 
#define Status2Reg    0x08 
#define FIFODataReg   0x09
#define FIFOLevelReg  0x0A
#define WaterLevelReg 0x0B
#define ControlReg    0x0C
#define BitFramingReg 0x0D
#define CollReg       0x0E
#define Reserved01    0x0F
//Page 1:Command 
#define Reserved10     0x10
#define ModeReg        0x11
#define TxModeReg      0x12
#define RxModeReg      0x13
#define TxControlReg   0x14
#define TxAutoReg      0x15
#define TxSelReg       0x16
#define RxSelReg       0x17
#define RxThresholdReg 0x18
#define DemodReg       0x19
#define Reserved11     0x1A
#define Reserved12     0x1B
#define MifareReg      0x1C
#define Reserved13     0x1D
#define Reserved14     0x1E
#define SerialSpeedReg 0x1F
//Page 2:CFG 
#define Reserved20        0x20 
#define CRCResultRegM     0x21
#define CRCResultRegL     0x22
#define Reserved21        0x23
#define ModWidthReg       0x24
#define Reserved22        0x25
#define RFCfgReg          0x26
#define GsNReg            0x27
#define CWGsPReg          0x28
#define ModGsPReg         0x29
#define TModeReg          0x2A
#define TPrescalerReg     0x2B
#define TReloadRegH       0x2C
#define TReloadRegL       0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F
//Page 3:TestRegister 
#define Reserved30      0x30
#define TestSel1Reg     0x31
#define TestSel2Reg     0x32
#define TestPinEnReg    0x33
#define TestPinValueReg 0x34
#define TestBusReg      0x35
#define AutoTestReg     0x36
#define VersionReg      0x37
#define AnalogTestReg   0x38
#define TestDAC1Reg     0x39 
#define TestDAC2Reg     0x3A 
#define TestADCReg      0x3B 
#define Reserved31      0x3C 
#define Reserved32      0x3D 
#define Reserved33      0x3E 
#define Reserved34      0x3F
//-----------------------------------------------

//4 bytes Serial number of card, the 5 bytes is verfiy bytes
byte serNum[5];

void MFRC522_begin(){
  SPI.begin();

  pinMode(SPI_PIN_SS, OUTPUT);   // Set digital pin 10 as OUTPUT to connect it to the RFID /ENABLE pin 
  digitalWrite(SPI_PIN_SS, LOW); // Activate the RFID reader
  pinMode(SPI_PIN_RST,OUTPUT);   // Set digital pin 5 , Not Reset and Power-down
  MFRC522_Init(); 
}

boolean MFRC522_getCardID(byte *cardID){
  byte status;
  byte str[MAX_LEN];

  status = MFRC522_Request(PICC_REQIDL, str); 
  if (status != MI_OK){
    //    Serial.println("not ready");
    return false;
  }
  //  Serial.println("ready");
  ShowCardType(str);
  status = MFRC522_Anticoll(str);
  MFRC522_Halt(); //command the card into sleep mode 
  if (status == MI_OK){
    memcpy(cardID, str, 5);
    ShowCardID(str);
    return true;
  }
  return false;
}

/*
 * Function：ShowCardID
 * Description：Show Card ID
 * Input parameter：ID string
 * Return：Null
 */
void ShowCardID(byte *id){
  int IDlen=4;
  for(int i=0; i<IDlen; i++){
    Serial.print(0x0F & (id[i]>>4), HEX);
    Serial.print(0x0F & id[i],HEX);
  }
  Serial.println("");
}

/*
 * Function：ShowCardType
 * Description：Show Card type
 * Input parameter：Type string
 * Return：Null
 */
void ShowCardType(byte *type){
  Serial.print("Card : ");
  if(type[0]==0x04&&type[1]==0x00) 
    Serial.print("MFOne-S50");
  else if(type[0]==0x02&&type[1]==0x00)
    Serial.print("MFOne-S70");
  else if(type[0]==0x44&&type[1]==0x00)
    Serial.print("MF-UltraLight");
  else if(type[0]==0x08&&type[1]==0x00)
    Serial.print("MF-Pro");
  else if(type[0]==0x44&&type[1]==0x03)
    Serial.print("MF Desire");
  else{
    Serial.print("Unknown");
    Serial.print(type[0],HEX);
    Serial.print(type[1],HEX);
  }
  Serial.print(", ");
}

/*
 * Function：Write_MFRC5200
 * Description：write a byte data into one register of MR RC522
 * Input parameter：addr--register address；val--the value that need to write in
 * Return：Null
 */
void Write_MFRC522(byte addr, byte val)
{
  digitalWrite(SPI_PIN_SS, LOW);
  //address format：0XXXXXX0
  SPI.transfer((addr<<1)&0x7E); 
  SPI.transfer(val);
  digitalWrite(SPI_PIN_SS, HIGH);
}

/*
 * Function：Read_MFRC522
 * Description：read a byte data into one register of MR RC522
 * Input parameter：addr--register address
 * Return：return the read value
 */
byte Read_MFRC522(byte addr){
  byte val;
  digitalWrite(SPI_PIN_SS, LOW);
  //address format：1XXXXXX0
  SPI.transfer(((addr<<1)&0x7E) | 0x80); 
  val = SPI.transfer(0x00);
  digitalWrite(SPI_PIN_SS, HIGH);
  return val; 
}

/*
 * Function：SetBitMask
 * Description：set RC522 register bit
 * Input parameter：reg--register address;mask--value
 * Return：null
 */
void SetBitMask(byte reg, byte mask){
  byte tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp | mask); // set bit mask
}

/*
 * Function：ClearBitMask
 * Description：clear RC522 register bit
 * Input parameter：reg--register address;mask--value
 * Return：null
 */
void ClearBitMask(byte reg, byte mask){
  byte tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp & (~mask)); // clear bit mask
}

/*
 * Function：AntennaOn
 * Description：Turn on antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter：null
 * Return：null
 */
void AntennaOn(void){
  byte temp = Read_MFRC522(TxControlReg);
  if (!(temp & 0x03))
    SetBitMask(TxControlReg, 0x03);
}

/*
 * Function：AntennaOff
 * Description：Turn off antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter：null
 * Return：null
 */
void AntennaOff(void){
  ClearBitMask(TxControlReg, 0x03);
}

/*
 * Function：ResetMFRC522
 * Description： reset RC522
 * Input parameter：null
 * Return：null
 */
void MFRC522_Reset(void){
  Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

/*
 * Function：InitMFRC522
 * Description：initilize RC522
 * Input parameter：null
 * Return：null
 */
void MFRC522_Init(void){
  digitalWrite(SPI_PIN_RST,HIGH);
  MFRC522_Reset();
  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  Write_MFRC522(TModeReg, 0x8D); //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  Write_MFRC522(TPrescalerReg, 0x3E); //TModeReg[3..0] + TPrescalerReg
  Write_MFRC522(TReloadRegL, 30); 
  Write_MFRC522(TReloadRegH, 0);
  Write_MFRC522(TxAutoReg, 0x40); //100%ASK
  Write_MFRC522(ModeReg, 0x3D); //CRC initilizate value 0x6363 ???
  AntennaOn(); //turn on antenna
}

/*
 * Function：MFRC522_Request
 * Description：Searching card, read card type
 * Input parameter：reqMode--search methods，
 * TagType--return card types
 * 0x4400 = Mifare_UltraLight
 * 0x0400 = Mifare_One(S50)
 * 0x0200 = Mifare_One(S70)
 * 0x0800 = Mifare_Pro(X)
 * 0x4403 = Mifare_DESFire
 * return：return MI_OK if successed
 */
byte MFRC522_Request(byte reqMode, byte *TagType){
  byte status; 
  word backBits; //the data bits that received
  Write_MFRC522(BitFramingReg, 0x07); //TxLastBists = BitFramingReg[2..0] ???
  TagType[0] = reqMode;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
  if ((status != MI_OK) || (backBits != 0x10))
    status = MI_ERR;
  return status;
}

/*
 * Function：MFRC522_ToCard
 * Description：communicate between RC522 and ISO14443
 * Input parameter：command--MF522 command bits
 * sendData--send data to card via rc522
 * sendLen--send data length 
 * backData--the return data from card
 * backLen--the length of return data
 * return：return MI_OK if successed
 */
byte MFRC522_ToCard(byte command, byte *sendData, byte sendLen, byte *backData, word *backLen){
  byte status = MI_ERR;
  byte irqEn = 0x00;
  byte waitIRq = 0x00;
  byte lastBits;
  byte n;
  word i;

  switch (command){
  case PCD_AUTHENT: //verify card password
    irqEn = 0x12;
    waitIRq = 0x10;
    break;
  case PCD_TRANSCEIVE: //send data in the FIFO
    irqEn = 0x77;
    waitIRq = 0x30;
    break;
  default:
    break;
  }

  Write_MFRC522(CommIEnReg, irqEn|0x80); //Allow interruption
  ClearBitMask(CommIrqReg, 0x80); //Clear all the interrupt bits
  SetBitMask(FIFOLevelReg, 0x80); //FlushBuffer=1, FIFO initilizate

  Write_MFRC522(CommandReg, PCD_IDLE); //NO action;cancel current command ???

  //write data into FIFO
  for (i=0; i<sendLen; i++)
    Write_MFRC522(FIFODataReg, sendData[i]); 

  //procceed it
  Write_MFRC522(CommandReg, command);
  if (command == PCD_TRANSCEIVE)
    SetBitMask(BitFramingReg, 0x80); //StartSend=1,transmission of data starts 

  //waite receive data is finished
  i = 2000; //i should adjust according the clock, the maxium the waiting time should be 25 ms???
  do{
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = Read_MFRC522(CommIrqReg);
    i--;
  }

  while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  ClearBitMask(BitFramingReg, 0x80); //StartSend=0

    if (i != 0){ 
    if(!(Read_MFRC522(ErrorReg) & 0x1B)){ //BufferOvfl Collerr CRCErr ProtecolErr
      status = MI_OK;
      if (n & irqEn & 0x01)
        status = MI_NOTAGERR; //?? 
      if (command == PCD_TRANSCEIVE){
        n = Read_MFRC522(FIFOLevelReg);
        lastBits = Read_MFRC522(ControlReg) & 0x07;
        if (lastBits)
          *backLen = (n-1)*8 + lastBits; 
        else
          *backLen = n*8; 
        if (n == 0)
          n = 1;
        if (n > MAX_LEN)
          n = MAX_LEN; 
        //read the data from FIFO
        for (i=0; i<n; i++)
          backData[i] = Read_MFRC522(FIFODataReg); 
      }
    }
    else
      status = MI_ERR; 
  }

  //SetBitMask(ControlReg,0x80); //timer stops
  //Write_MFRC522(CommandReg, PCD_IDLE); 
  return status;
}

/*
 * Function：MFRC522_Anticoll
 * Description：Prevent conflict, read the card serial number 
 * Input parameter：serNum--return the 4 bytes card serial number, the 5th byte is recheck byte
 * return：return MI_OK if successed
 */
byte MFRC522_Anticoll(byte *serNum){
  byte status;
  byte i;
  byte serNumCheck=0;
  word unLen;

  //ClearBitMask(Status2Reg, 0x08); //strSensclear
  //ClearBitMask(CollReg,0x80); //ValuesAfterColl
  Write_MFRC522(BitFramingReg, 0x00); //TxLastBists = BitFramingReg[2..0]

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

  if (status == MI_OK){
    //Verify card serial number
    for (i=0; i<4; i++)
      serNumCheck ^= serNum[i];
    if (serNumCheck != serNum[i])
      status = MI_ERR; 
  }
  //SetBitMask(CollReg, 0x80); //ValuesAfterColl=1
  return status;
} 

/*
 * Function：CalulateCRC
 * Description：Use MF522 to caculate CRC
 * Input parameter：pIndata--the CRC data need to be read，len--data length，pOutData-- the caculated result of CRC
 * return：Null
 */
void CalulateCRC(byte *pIndata, byte len, byte *pOutData){
  byte i, n;

  ClearBitMask(DivIrqReg, 0x04); //CRCIrq = 0
  SetBitMask(FIFOLevelReg, 0x80); //Clear FIFO pointer
  //Write_MFRC522(CommandReg, PCD_IDLE);

  //Write data into FIFO 
  for (i=0; i<len; i++)
    Write_MFRC522(FIFODataReg, *(pIndata+i)); 
  Write_MFRC522(CommandReg, PCD_CALCCRC);

  //waite CRC caculation to finish
  i = 0xFF;
  do{
    n = Read_MFRC522(DivIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x04)); //CRCIrq = 1

  //read CRC caculation result
  pOutData[0] = Read_MFRC522(CRCResultRegL);
  pOutData[1] = Read_MFRC522(CRCResultRegM);
}

/*
 * Function：MFRC522_Halt
 * Description：Command the cards into sleep mode
 * Input parameters：null
 * return：null
 */
void MFRC522_Halt(void){
  byte ret;
  word unLen;
  byte buff[4];

  buff[0] = PICC_HALT;
  buff[1] = 0;
  CalulateCRC(buff, 2, &buff[2]);

  ret = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}


////////// ////////// ////////// ////////// //////////
///  Main  ////////// ////////// ////////// //////////

#define STATUS_READY             0  // Lock, Red light, wait card
#define STATUS_ERRORCARD         1  // Lock, Red flashing, wait card
#define STATUS_UNLOCKING        10 // Card ok, unlock, green light, 3 seconds
#define STATUS_ADDUSER_READY    20 // Add user button pressed, unlock, quick green flashing and red light, wait card
#define STATUS_DELETEUSER_READY 30 // Delete user button pressed, unlock, quick red flashing and green light, wait card
#define STATUS_RESET_READY      90 // Add and delete user buttons pressed for 8 seconds, unlock, quick green and red flashing 3 seconds

// accessable cards list
byte accessableCards[20][5];
byte accessableCardsCount=0;
int currentStatus=STATUS_READY;
int subStatus;
int statusRed=LOW;
int statusGreen=LOW;

#define SETUPPIN(pin, mode, value)  pinMode(pin,mode)//;digitalWrite(pin,value);

#define OPEN_GREENLIGHT    digitalWrite(OUTPUT_PIN_GREENLIGHT, HIGH)
#define CLOSE_GREENLIGHT   digitalWrite(OUTPUT_PIN_GREENLIGHT, LOW)
#define OPEN_REDLIGHT      digitalWrite(OUTPUT_PIN_REDLIGHT, HIGH)
#define CLOSE_REDLIGHT     digitalWrite(OUTPUT_PIN_REDLIGHT, LOW)
#define OPEN_YELLOWLIGHT   digitalWrite(OUTPUT_PIN_YELLOWLIGHT, HIGH)
#define CLOSE_YELLOWLIGHT  digitalWrite(OUTPUT_PIN_YELLOWLIGHT, LOW)
#define DISABLE_MAGENT     digitalWrite(OUTPUT_PIN_MAGENT, HIGH)
#define ENABLE_MAGENT      digitalWrite(OUTPUT_PIN_MAGENT, LOW)

#define INACCESSABLE_CARD -1
#define NO_CARD_REGISTED  -2

#define ON  LOW
#define OFF HIGH
#define IS_ON(pin)  digitalRead(pin)==ON 
#define IS_OFF(pin) digitalRead(pin)==OFF

void setup(){
  Serial.begin(57600);
  MFRC522_begin();
  SETUPPIN(OUTPUT_PIN_MAGENT,      OUTPUT,       LOW );
  SETUPPIN(OUTPUT_PIN_REDLIGHT,    OUTPUT,       LOW );
  SETUPPIN(OUTPUT_PIN_GREENLIGHT,  OUTPUT,       LOW );
  SETUPPIN(OUTPUT_PIN_YELLOWLIGHT, OUTPUT,       LOW );
  SETUPPIN(INPUT_PIN_ADDUSER,      INPUT_PULLUP, HIGH);
  SETUPPIN(INPUT_PIN_DELETEUSER,   INPUT_PULLUP, HIGH);
  SETUPPIN(INPUT_PIN_OPEN,         INPUT_PULLUP, HIGH);
  CLOSE_YELLOWLIGHT;
  OPEN_REDLIGHT;
  CLOSE_GREENLIGHT;
  ENABLE_MAGENT;
}

void loop()
{
  byte cardID[5];
  int i;
  int d;

  switch(currentStatus){
  case STATUS_READY:
    CLOSE_YELLOWLIGHT;
    CLOSE_GREENLIGHT;
    OPEN_REDLIGHT;
    ENABLE_MAGENT;
    if(IS_ON(INPUT_PIN_OPEN)){
      Serial.println("open via button.");
      CLOSE_REDLIGHT;
      OPEN_GREENLIGHT;
      DISABLE_MAGENT;
      delay(3000);
      for(i=0;i<5;i++){
        CLOSE_GREENLIGHT;
        delay(200);
        OPEN_GREENLIGHT;
        delay(200);
      }
      CLOSE_GREENLIGHT;
      Serial.println("door closed.");   
    }
    // 等待卡片
    //    Serial.println("checking...");
    if(MFRC522_getCardID(cardID)){
      //      Serial.println("read a card");
      if(checkAccessable(cardID) != INACCESSABLE_CARD){
        Serial.println("open door");
        // 开锁，5秒不响应事件
        CLOSE_REDLIGHT;
        OPEN_GREENLIGHT;
        DISABLE_MAGENT;
        delay(3000);
        for(i=0;i<5;i++){
          CLOSE_GREENLIGHT;
          delay(200);
          OPEN_GREENLIGHT;
          delay(200);
        }
        CLOSE_GREENLIGHT;
        Serial.println("door closed.");   
      }
      else{
        Serial.println("forbidden!");
        // 闪烁3下，不响应事件
        CLOSE_GREENLIGHT;
        ENABLE_MAGENT;
        for(i=0;i<3;i++){
          CLOSE_REDLIGHT;
          delay(200);
          OPEN_REDLIGHT;
          delay(200);
        }
      }
    }
    else{
      //  Serial.println("no card");
      checkButtons(STATUS_READY);
      delay(100);
    }
    break;
  case STATUS_ADDUSER_READY:
    OPEN_YELLOWLIGHT;
    CLOSE_REDLIGHT;
    OPEN_GREENLIGHT;
    DISABLE_MAGENT;
    //    Serial.println("waiting for add card...");
    if(MFRC522_getCardID(cardID)){
      //      Serial.println("get a card.");
      if(checkAccessable(cardID)<0){
        accessableCards[accessableCardsCount][0]=cardID[0];
        accessableCards[accessableCardsCount][1]=cardID[1];
        accessableCards[accessableCardsCount][2]=cardID[2];
        accessableCards[accessableCardsCount][3]=cardID[3];
        accessableCardsCount++;
        Serial.print("add to list : ");
        Serial.println(accessableCardsCount);
        for(i=0;i<3;i++){
          OPEN_GREENLIGHT;
          delay(200);
          CLOSE_GREENLIGHT;
          delay(200);
        }
      }
      else{
        Serial.println("already added");
        for(i=0;i<3;i++){
          OPEN_REDLIGHT;
          delay(200);
          CLOSE_REDLIGHT;
          delay(200);
        }
      }
    }
    delay(100);
    checkButtons(STATUS_ADDUSER_READY);    
    break;
  case STATUS_DELETEUSER_READY:
    OPEN_YELLOWLIGHT;
    OPEN_REDLIGHT;
    CLOSE_GREENLIGHT;
    DISABLE_MAGENT;
    //    Serial.println("waiting for delete user...");
    if(MFRC522_getCardID(cardID)){
      d = checkAccessable(cardID);
      //      Serial.println("got a card");
      if(d>=0){
        Serial.print("delete from list : ");
        Serial.println(d);
        for(i=d;i<accessableCardsCount;i++){
          accessableCards[i][0]=accessableCards[i+1][0];
          accessableCards[i][1]=accessableCards[i+1][1];
          accessableCards[i][2]=accessableCards[i+1][2];
          accessableCards[i][3]=accessableCards[i+1][3];
        }
        accessableCardsCount--;
        for(i=0;i<3;i++){
          OPEN_GREENLIGHT;
          delay(200);
          CLOSE_GREENLIGHT;
          delay(200);
        }
      }
      else{
        Serial.println("already deleted");
        for(i=0;i<3;i++){
          OPEN_REDLIGHT;
          delay(200);
          CLOSE_REDLIGHT;
          delay(200);
        }
      }
    }
    delay(100);
    checkButtons(STATUS_DELETEUSER_READY);
    break;
  case STATUS_RESET_READY:
    OPEN_YELLOWLIGHT;
    OPEN_REDLIGHT;
    OPEN_GREENLIGHT;
    DISABLE_MAGENT;
    //    Serial.println("ready for reset...");
    if(subStatus>50){
      //      Serial.println("waitting for 10sec");
      accessableCardsCount=0;
      CLOSE_YELLOWLIGHT;
      CLOSE_REDLIGHT;
      CLOSE_GREENLIGHT;
      MFRC522_Init();
      delay(1000);
      currentStatus = STATUS_READY;
    }
    if(IS_ON(INPUT_PIN_ADDUSER) && IS_ON(INPUT_PIN_DELETEUSER)){
      Serial.println(subStatus);
      subStatus++;
    }
    else{
      currentStatus = STATUS_READY;
    }
    delay(100);
    break;
  }
}

int checkAccessable(byte *cardid){
  int i,j;

  if(accessableCardsCount==0){
    Serial.println("no card registed.");
    return NO_CARD_REGISTED;
  }
  for(i=0;i<accessableCardsCount;i++){
    Serial.print("list ");
    Serial.print(i);
    Serial.print(":");
    Serial.print(accessableCards[i][0],HEX);
    Serial.print(accessableCards[i][1],HEX);
    Serial.print(accessableCards[i][2],HEX);
    Serial.println(accessableCards[i][3],HEX);
    if(cardid[0]==accessableCards[i][0] &&
       cardid[1]==accessableCards[i][1] &&
       cardid[2]==accessableCards[i][2] &&
       cardid[3]==accessableCards[i][3]){
      Serial.println("accessable card.");
      return i;
    }
  }
  Serial.println("inaccessable card.");
  return INACCESSABLE_CARD;
}

void checkButtons(int cs){
  int cs_;
                                     //       ADD   DELETE
  if(IS_ON(INPUT_PIN_ADDUSER)){
    if(IS_ON(INPUT_PIN_DELETEUSER))
      cs_ = STATUS_RESET_READY;      //       ON    ON
    else
      cs_ = STATUS_ADDUSER_READY;    //       ON    OFF
  }else{
    if(IS_ON(INPUT_PIN_DELETEUSER))
      cs_ = STATUS_DELETEUSER_READY; //       OFF   ON
    else
      cs_ = STATUS_READY;            //       OFF   OFF
  }
  
  if (cs_!=cs){
    Serial.print("change status to : ");
    Serial.println(cs_);
    currentStatus = cs_;
    subStatus=0;
  }
}

