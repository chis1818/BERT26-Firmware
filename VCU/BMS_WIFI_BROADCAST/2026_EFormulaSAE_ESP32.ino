#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
extern const char* base_html;
extern const char* basic_HTML;
// Wi-Fi Access Point Constants
const char* SSID="BERT.26";
const char* PASSWORD="BERT2026";
const char* domain="BERT26.local";
const byte DNS_PORT=53;
DNSServer dnsServer;
WebServer server(80);
IPAddress local_ip(192, 168, 1,1);
IPAddress gateway(192, 168, 1,1);
IPAddress subnet(255, 255, 255, 0);
void sendLargeHTML(const char* data) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  
  size_t length = strlen(data);
  size_t chunkSize = 16184; // Set once and forget
  
  for (size_t i = 0; i < length; i += chunkSize) {
    size_t size = (length - i < chunkSize) ? length - i : chunkSize;
    server.sendContent(data + i, size);
  }
  server.sendContent(""); // Finalize
}

void HTTP_handle_root(){
  
  sendLargeHTML(base_html);
}
enum{
  RPM,
  TEMP,
  VOLT,
  SOC
} signa;
const char * signal_names[]={
  "RPM",
  "Temp",
  "Voltage",
  "StateOfCharge"
};
const unsigned int signal_count=sizeof(signal_names)/sizeof(char*);
double signals[4];

void HTTP_handle_data(){
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent("{\"timeSent\":"+String(millis())+",");
  String toSend;
  for(int i=0;i<signal_count-1;i++){
    toSend+=String("\"")+String(signal_names[i])+"\":"+String(signals[i])+",";
  }
  toSend+=String("\"")+String(signal_names[signal_count-1])+"\":"+String(signals[signal_count-1])+"}";
  server.sendContent(toSend);

  server.sendContent("");
}

void HTTP_handle_basic(){
  sendLargeHTML(basic_HTML);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Setting up AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PASSWORD);
  delay(50);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  IPAddress IP=WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(IP);
  // Start the DNS server
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.on("/", HTTP_handle_root);
  server.on("/data", HTTP_handle_data);
  server.on("/basic", HTTP_handle_basic);
  server.begin();
  Serial.println("HTTP server started");
  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(analogRead(0));
  
}


void loop() {
  // put your main code here, to run repeatedly:
  /*digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  delay(90);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  delay(90);                      // wait for a second*/
    dnsServer.processNextRequest(); // Important: process DNS requests
  signals[RPM]=floor((double)(random(0,50000))/100.0)+4000.0;
  signals[TEMP]=50.0+((double)random(0,5000)/1000.0);
  signals[VOLT]=580.0+((double)random(0,4000)/1000.0);
  signals[SOC]=80+((double)random(0,4000)/1000.0);
  server.handleClient(); 
}
