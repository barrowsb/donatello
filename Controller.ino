#include <WiFi.h>

// wifi setup vars
const char *ssid = "Vishnu";
const char *password = "12345678";

// upd setup vars
WiFiUDP UDPTestServer;
IPAddress targetIP(192,168,4,113);
unsigned int UDPPort = 2808;
byte d_s[1];

// input setup vars
//    pins
int LPotPort = 33;
int RPotPort = 32;
int commSwitch = 21;

int Lswitch = 36;
int Rswitch = 34;

//    pot deadband
int LLowDeadband = 1600;
int LHighDeadband = 2020;
int RLowDeadband = 1730;
int RHighDeadband = 2280;
//    pot range
int Lmax = 4095;
int Rmax = 4095;
int Lmin = 0;
int Rmin = 0;

void setup()
{  
  // set pins as inputs
  pinMode(LPotPort,INPUT);
  pinMode(RPotPort,INPUT);
  pinMode(Lswitch,INPUT);
  pinMode(Rswitch,INPUT);
  pinMode(commSwitch,INPUT);

  // begin serial
  Serial.begin(115200);
  Serial.print("Start");

  // setup wifi
  Serial.println();
  Serial.print("Configuring access point...");
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // begin udp server
  UDPTestServer.begin(UDPPort);
  WiFi.setSleep(false);
}

void loop()
{
  // if comm switch set to ON
  if (digitalRead(commSwitch))
  {  
    // print pot values
    Serial.print("L: ");
    Serial.print(analogRead(LPotPort));
    Serial.print((char) 9);
    Serial.print("R: ");
    Serial.println(analogRead(RPotPort));
    
    // read, pack, and send udp
    send_UDPServer(LeftPotConvertToByte(analogRead(LPotPort)));
    delay(10);
    send_UDPServer(RightPotConvertToByte(analogRead(RPotPort)));
    delay(10);
    Serial.println();
    
  }
  
  // if comm switch set to OFF
  else
  {  
    // print switch state
    Serial.println("COMM OFF");
    
    // tell motores to stop
    send_UDPServer(1);
    send_UDPServer(128);
    Serial.println();
    
  }

  bool LS = digitalRead(Lswitch);
  bool RS = digitalRead(Rswitch);

  if(LS ^ RS){
    if(LS){
      Serial.println("Send Left Servo Command");
      send_UDPServer(63);
      
    }
    else{
      Serial.println("Send Right Servo Command");
      send_UDPServer(63+128);
      
    }
  }
  else{
    Serial.println("Send Servo Stop Command");
    send_UDPServer(63+64);
  }
  delay(10);
  
}

// trims pot value to [0,4095]
int applyCutoffs(int val, int minn, int lowDead, int highDead, int maxx)
{
  // if below deadband
  if(val<lowDead)
  {
    // if below min
    if(val<minn) 
    {
      // set to min
      val = minn;
    }
  }

  // if above deadband
  else if(val > highDead) 
  {
    // if above max
    if(val > maxx) 
    {
      // set to max
      val = maxx;
    }
  }

  // else (within deadband)
  else 
  {
    // set to -1
    val = -1;
  }
  
  return val;
}

// converts left pot value to byte for sending
byte LeftPotConvertToByte(int readVal) 
{
  // apply cutoffs and check deadband
  int val = applyCutoffs(readVal, Lmin, LLowDeadband, LHighDeadband, Lmax);

  // if within deadband, return stop val
  if(val < 0) 
  {
    //send 0 (+1)
    return 1;
  }

  // if below deadband, map to lower range and return
  if(val<LLowDeadband) 
  {
    //63 is reserved for left button press
    return map(val, Lmin, LLowDeadband,30 , 0);
  }

  // if above deadband, map to higher range and return
  if(val> LHighDeadband) 
  {
    //63+64 reserved for servo stop
    return map(val, LHighDeadband, Lmax, 0, 30) + (byte)64;
  }

  // else, return 0
  return 0;
}

// converts right pot value to byte for sending
byte RightPotConvertToByte(int readVal) {

  // apply cutoffs and check deadband
  int val = applyCutoffs(readVal, Rmin, RLowDeadband, RHighDeadband, Rmax);

  // if within deadband, return stop val
  if(val < 0) 
  {
    // send 128
    return 0 + (byte)128;
  }

  // if below deadband, map to lower range and return
  if(val<RLowDeadband) 
  {
    //63+128 is reserved for Right button press
    return map(val, Rmin, RLowDeadband,30 , 0) + (byte) 128;
  }

  // if above deadband, map to higher range and return
  if(val> RHighDeadband) 
  {
    //63+64+128 reserved for servo stop
    return map(val, RHighDeadband, Rmax, 0, 30) + (byte)64 + (byte)128;
  }

  // else, return 0
  return 0;
}

//function sends data to UDP port
void send_UDPServer(byte data_send) 
{
  // byte to send
  d_s[0] = data_send;

  // send byte and null terminate
  UDPTestServer.beginPacket(targetIP,UDPPort);
  UDPTestServer.printf("%s",d_s);
  UDPTestServer.endPacket();

  // print what was sent
  Serial.printf("UDP Sent: ");
  Serial.println(data_send);
  
}
