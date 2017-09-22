#include <UIPEthernet.h> // Used for Ethernet

// **** ETHERNET SETTING ****
byte mac[] = { 0x54, 0x34, 0x41, 0x30, 0x30, 0x32 };
IPAddress ip(192, 168, 1, 180);
EthernetServer server(80);
char tcpip_buffer[128];
char serial_buffer[128];

int tcpip_cnt = 0, serial_cnt = 0;
int timer = 0;

void setup() {
  Serial.begin(115200);

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();

  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  // listen for incoming clients
  EthernetClient client = server.available();

  if (client)
  {
    while (client.connected())
    {
      client.print("d");
      delay(30);
//      int _delay;
//      timer = millis();
// client.print("d");
//      while(!client.available())
//      {}
//       client.read();
//       _delay = millis() - timer;
//      
//      Serial.println(_delay);
      
//      if (client.available())
//      {
//        tcpip_buffer[tcpip_cnt++] = client.read();
//        if ((tcpip_cnt > 120) || (tcpip_buffer[tcpip_cnt - 1] == 0x04))
//        {
//          Serial.print(tcpip_buffer);
//          for(int i = 0; i <= tcpip_cnt + 1; i++)
//          { 
//            tcpip_buffer[i] = 0;
//          }
//          tcpip_cnt = 0;
//        }
//      }
//      if (Serial.available())
//      {
//        serial_buffer[serial_cnt++] = Serial.read();
//        if ((serial_cnt > 120) || (serial_buffer[serial_cnt - 1] == 0x04))
//        {
//          client.print(serial_buffer);
//          for(int i = 0; i <= serial_cnt + 1; i++)
//          {
//            serial_buffer[i] = 0;
//          }
//          serial_cnt = 0;
//        }
//      }
    }
  }
}

