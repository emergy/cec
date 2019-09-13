#include <Arduino.h>
#include <IRremote.h>
#include <CEC_Device.h>

#define IN_LINE 4
#define OUT_LINE 5
#define LED_LINE 3
#define IR_LINE 7
#define DDC_LINE 6

//IRrecv irrecv(IR_LINE);
//decode_results results;

IRsend irsend;


// ugly macro to do debug printing in the OnReceive method
#define report(X) do { DbgPrint("report " #X "\n"); report ## X (); } while (0)

#define phy1 ((_physicalAddress >> 8) & 0xFF)
#define phy2 ((_physicalAddress >> 0) & 0xFF)

class MyCEC: public CEC_Device {
  public:
    MyCEC(int physAddr): CEC_Device(physAddr,IN_LINE,OUT_LINE) { }
    
    void reportStreamState() { unsigned char frame[3] = { 0x82, phy1, phy2 };       TransmitFrame(0x0F,frame,sizeof(frame)); } // report stream state (playing)
    void reportPhysAddr()    { unsigned char frame[4] = { 0x84, phy1, phy2, 0x04 }; TransmitFrame(0x0F,frame,sizeof(frame)); } // report physical address
    
    void reportPowerState()  { unsigned char frame[2] = { 0x90, 0x00 };             TransmitFrame(0x00,frame,sizeof(frame)); } // report power state (on)
    void reportCECVersion()  { unsigned char frame[2] = { 0x9E, 0x04 };             TransmitFrame(0x00,frame,sizeof(frame)); } // report CEC version (v1.3a)
    
    void reportOSDName()     { unsigned char frame[5] = { 0x47, 'H','T','P','C' };  TransmitFrame(0x00,frame,sizeof(frame)); } // FIXME: name hardcoded
    void reportVendorID()    { unsigned char frame[4] = { 0x87, 0x00, 0xF1, 0x0E }; TransmitFrame(0x00,frame,sizeof(frame)); } // report fake vendor ID
    // TODO: implement menu status query (0x8D) and report (0x8E,0x00)
    
    /*
    void handleKey(unsigned char key) {
      Serial.println(key);
      switch (key) {
        case 0x01: Keyboard.press(KEY_UP_ARROW); break;
        case 0x00: Keyboard.press(KEY_RETURN); break;
        case 0x02: Keyboard.press(KEY_DOWN_ARROW); break;
        case 0x03: Keyboard.press(KEY_LEFT_ARROW); break;
        case 0x04: Keyboard.press(KEY_RIGHT_ARROW); break;
        case 0x0D: Keyboard.press(KEY_ESC); break;
        case 0x4B: Keyboard.press(KEY_PAGE_DOWN); break;
        case 0x4C: Keyboard.press(KEY_PAGE_UP); break;
        case 0x53: Keyboard.press(KEY_HOME); break;
      }
    }
    */
    void powerButton(uint8_t action) {
      uint8_t powerStatus = digitalRead(DDC_LINE);
      DbgPrint("Current power status is %s\n", powerStatus == HIGH ? "on" : "off");

      while (action != powerStatus) {
        DbgPrint("Send power IR code\n");
        int khz=38;
        unsigned int Signal_0_0[] = {4550,4400,600,1650,550,1650,600,1650,550,550,600,500,600,550,550,550,600,500,600,1650,600,1600,600,1650,550,550,600,500,600,550,600,500,600,500,650,450,650,1600,600,500,650,450,650,500,600,500,600,500,600,550,600,1600,600,500,650,1600,650,1550,650,1600,650,1550,650,1600,650,1600,600}; //AnalysIR Batch Export - RAW
        irsend.sendRaw(Signal_0_0, sizeof(Signal_0_0)/sizeof(int), khz);
        delay(5000);
      }
    }
        
    void OnReceive(int source, int dest, unsigned char* buffer, int count) {
      if (count == 0) return;
      switch (buffer[0]) {

        Serial.println("000:" + buffer[0]);
        //Packet received at 5751: 04 -> 00: 40:36
        case 0x36:
          DbgPrint("Power off signal from CEC\n");
          powerButton(LOW);
          break;
        case 0x0D: 
          DbgPrint("Power on signal from CEC\n");
          powerButton(HIGH);
          break;
        default: CEC_Device::OnReceive(source,dest,buffer,count); break;
      /*
        case 0x83: report(PhysAddr); break;
        case 0x86: if (buffer[1] == phy1 && buffer[2] == phy2)
                   report(StreamState); break;
        
        case 0x8F: report(PowerState); break;
        case 0x9F: report(CECVersion); break;  
        
        case 0x46: report(OSDName);    break;
        case 0x8C: report(VendorID);   break;
        
        case 0x44: handleKey(buffer[1]); break;
        //case 0x45: Keyboard.releaseAll(); break;
        
        */
/*
Device ready
standby
Packet received at 7825: 04 -> 15: 4F:84:10:00:04 // сообщает свой адрес
Packet received at 7909: 04 -> 00: 40:0D          // просмотр изображения (так же, удаляет все меню)
Packet received at 8060: 04 -> 15: 4F:87:00:00:00 // Сообщает идентификатор поставщика устройства
Packet received at 8144: 04 -> 00: 40:0D
Packet received at 8224: 04 -> 00: 40:0D
Packet received at 8356: 04 -> 15: 4F:82:10:00    // активный источник
Packet received at 8440: 04 -> 00: 40:8F          // запрос текущего состояния клиента
Packet received at 9997: 04 -> 00: 40:8F
Packet received at 12002: 04 -> 00: 40:8F
Packet received at 14002: 04 -> 00: 40:8F
Packet received at 16003: 04 -> 00: 40:8F
Packet received at 18007: 04 -> 00: 40:8F
Packet received at 20016: 04 -> 00: 40:8F
Packet received at 22016: 04 -> 00: 40:8F
Packet received at 24017: 04 -> 00: 40:8F
Packet received at 26021: 04 -> 00: 40:8F
Packet received at 28025: 04 -> 00: 40:8F
Packet received at 30026: 04 -> 00: 40:91        // запрос для определения текущего языка
Packet received at 30110: 04 -> 00: 40:91
*/
      }
    }
};

// TODO: set physical address via serial (or even DDC?)

// Note: this does not need to correspond to the physical address (i.e. port number)
// where the Arduino is connected - in fact, it _should_ be a different port, namely
// the one where the PC to be controlled is connected. Basically, it is the address
// of the port where the CEC-less source device is plugged in.
MyCEC device(0x0000);

void setup()
{
  Serial.begin(115200);

  //irrecv.enableIRIn();

  pinMode(DDC_LINE, INPUT);
  
  device.MonitorMode = false;
  device.Promiscuous = true;
  device.Initialize(CEC_LogicalDevice::CDT_TV);
}

void loop()
{
  // FIXME: does it still work without serial connected?
  /*
  if (Serial.available())
  {
    unsigned char c = Serial.read();
    unsigned char buffer[2] = { c, 0 };
    Serial.print("Command: "); Serial.println((const char*)buffer);
    
    switch (c)
    {
      case 'v':
        // request vendor ID
        buffer[0] = 0x8C;
        device.TransmitFrame(0, buffer, 1);
        break;
      case 'p':
        // toggle promiscuous mode
        device.Promiscuous = !(device.Promiscuous);
        break;
    }
  }
  */

  /*
  if (irrecv.decode(&results)) {
    //Serial.print("0x");

    switch (results.value) {
      case 0xF4BA2988:
        Serial.println("Power on from IR");
        //Serial.println(results.value, HEX);
        break;
    }
    irrecv.resume();
  }
  */


  device.Run();
}

