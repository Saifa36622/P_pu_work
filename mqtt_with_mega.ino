#include <WiFiEspAT.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <Servo.h>


// Wi-Fi credentials
const char* ssid = "saifa";
const char* password = "Saifa36622";

// MQTT broker settings
const char* mqtt_broker = "mqtt-dashboard.com";
const int mqtt_port = 1883;  // Default MQTT port is 1883
const char* client_id = "clientId-WnhntobHce";  // Client ID for MQTT

WiFiClient espClient;        // Create a WiFi client for the PubSubClient
PubSubClient client(espClient);  // Initialize the PubSubClient with the WiFi client

bool isConnected = false;  // Flag to track Wi-Fi connection status
bool isMoving = false;     // Flag to track if the system is currently moving

enum state_motor {stop,moving};
enum state_mqtt {no_msg,new_msg};

state_motor state_x = stop;
state_motor state_z = stop;
state_mqtt state_mqtt = no_msg;
String lastMessage = "";
int stop_check = 0;

// -------------------------------------------------------------------------------------------------------
// from cho

AccelStepper stepperX(AccelStepper::DRIVER, 2, 5); // Step and Direction pins for X
AccelStepper steppery2(AccelStepper::DRIVER, 3, 6); // Step and Direction pins for Y
AccelStepper stepperz(AccelStepper::DRIVER, 4, 7); // Step and Direction pins for Y
// Servo myServo;

// Define limit switch pins
const int xLimitPin = 9; // X+ Endstop on CNC shield connected to pin 9
const int yLimitPin = 10; // Y+ Endstop on CNC shield connected to pin 10
const int zLimitPin = 11; // Y+ Endstop on CNC shield connected to pin 
// const int servoPin = 11;  // Servo control pin

volatile bool xLimitTriggered = false;
volatile bool yLimitTriggered = false;
unsigned long xLimitTriggerTime = 0;
unsigned long yLimitTriggerTime = 0;

volatile bool zLimitTriggered = false;
unsigned long zLimitTriggerTime = 0;

void xLimitISR() {
  xLimitTriggered = true;
  xLimitTriggerTime = millis(); // Record the time when the limit switch is triggered
}

void yLimitISR() {
  yLimitTriggered = true;
  yLimitTriggerTime = millis(); // Record the time when the limit switch is triggered
}

void zLimitISR() {
  zLimitTriggered = true;
  zLimitTriggerTime = millis(); // Record the time when the limit switch is triggered
}

// ---------------- Movement Function---------------------------------------------------------------------
void move_x(float moveDistance)
{
  // Serial.println("hi");
  stepperX.moveTo(moveDistance); // change the move distance by your desire
  while (stepperX.distanceToGo() != 0) {
      stepperX.run();
      delay(1);
  }

  // delay(5000);
  String finishMessage = "finish move to " + String(moveDistance);
  Serial.println(finishMessage);
  client.publish("teleprint/status/step_x", finishMessage.c_str());
  state_x = stop;
}

void move_y2(float moveDistance)
{
  // Serial.println("hi");
  steppery2.moveTo(moveDistance); // change the move distance by your desire
  while (steppery2.distanceToGo() != 0) {
      steppery2.run();
      delay(1);
  }

  String finishMessage = "finish move to " + String(moveDistance);
  Serial.println(finishMessage);
  client.publish("teleprint/status/step_y2", finishMessage.c_str());
  // state_z = stop;
}

void move_z(float moveDistance)
{
  // Serial.println("hi");
  stepperz.moveTo(moveDistance); // change the move distance by your desire
  while (stepperz.distanceToGo() != 0) {
      stepperz.run();
      delay(1);
  }

  String finishMessage = "finish move to " + String(moveDistance);
  Serial.println(finishMessage);
  client.publish("teleprint/status/step_z", finishMessage.c_str());
  // state_z = stop;
}


void stop_move(int motor_num)
{
  if (motor_num == 1)
  {
    stepperX.stop();

    String finishMessage = "finish stop the motorX";
    Serial.println(finishMessage);
    client.publish("teleprint/status/step_x", finishMessage.c_str());
  }
  else if (motor_num == 2)
  {
    steppery2.stop();

    String finishMessage = "finish stop the motory2";
    Serial.println(finishMessage);
    client.publish("teleprint/status/step_y2", finishMessage.c_str());
  }
  else if (motor_num == 3)
  {
    stepperX.stop();
    steppery2.stop();

    String finishMessage = "finish stop both motor";
    Serial.println(finishMessage);
    client.publish("teleprint/status/step_x", finishMessage.c_str());
    client.publish("teleprint/status/step_y2", finishMessage.c_str());
  }
}

void set_home()
{
  xLimitTriggered = false;
  yLimitTriggered = false;
  zLimitTriggered = false;

  stepperX.moveTo(-1000000);
  while (!xLimitTriggered) {
    stepperX.run();
    delay(1); // Small delay to avoid flooding the Serial Monitor

    // Check if limit switch is triggered during movement
    if (digitalRead(xLimitPin) == LOW) {
      stepperX.stop();
      xLimitTriggered = true;
      stepperX.setCurrentPosition(0); // Reset the position
      break;
    }
  }

  steppery2.moveTo(-1000000);
  while (!yLimitTriggered) {
    steppery2.run();
    delay(1); // Small delay to avoid flooding the Serial Monitor

    // Check if limit switch is triggered during movement
    if (digitalRead(yLimitPin) == LOW) {
      steppery2.stop();
      yLimitTriggered = true;
      steppery2.setCurrentPosition(0); // Reset the position
      break;
    }
  }
  stepperz.moveTo(-1000000);
  while (!zLimitTriggered) {
    stepperz.run();
    delay(1); // Small delay to avoid flooding the Serial Monitor

    // Check if limit switch is triggered during movement
    if (digitalRead(zLimitPin) == LOW) {
      stepperz.stop();
      zLimitTriggered = true;
      stepperz.setCurrentPosition(0); // Reset the position
      break;
    }
  }

}

// --------------------------------------------------------------------------------------------------------


// Callback function to handle incoming MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length)
 {

  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  
  // Print the message payload
  String message = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message += (char)payload[i];
  }
  Serial.println();

  // Ignore messages that start with "finish move"
  if (message.startsWith("finish move")) {
    // Serial.println("Ignoring finish move message.");
    return;  // Skip processing this message
  }
  if (message.startsWith("move in")) {
    // Serial.println("Ignoring move in message.");
    return;  // Skip processing this message
  }
  if (message.startsWith("finish stop")) {
    // Serial.println("Ignoring finish move message.");
    return;  // Skip processing this message
  }

  // Check if the topic is "teleprint/cmd/step_x" and the system is not already moving
  if (String(topic) == "teleprint/cmd/step_x" && state_x != moving) {
    // Convert message to float
    float moveDistance = message.toFloat();

    // Create a reply message
    String reply = "move in X-axis " + String(moveDistance) + " mm";
    Serial.println(reply);

    // Publish the reply to the same topic
    client.publish("teleprint/status/step_x", reply.c_str());
    state_x = moving;
    lastMessage = message;
    move_x(moveDistance);
  }

  if (String(topic) == "teleprint/cmd/step_y2") {
    // Convert message to float
    float moveDistance = message.toFloat();

    // Create a reply messagye
    String reply = "move in y2-axis " + String(moveDistance) + " mm";
    Serial.println(reply);

    // Publish the reply to the same topic
    client.publish("teleprint/status/step_y2", reply.c_str());
    state_z = moving;
    lastMessage = message;
    move_y2(moveDistance);
  }

  if (String(topic) == "teleprint/cmd/step_z") {
    // Convert message to float
    float moveDistance = message.toFloat();

    // Create a reply messagye
    String reply = "move in z-axis " + String(moveDistance) + " mm";
    Serial.println(reply);

    // Publish the reply to the same topic
    client.publish("teleprint/status/step_z", reply.c_str());
    state_z = moving;
    lastMessage = message;
    move_z(moveDistance);
  }

  if (String(topic) == "teleprint/cmd/stepend") {

    stop_check = 0;
    // Convert message to float
    String in_msg = message;
    // state_z = moving;
    lastMessage = message;
    if (in_msg == "StepXEnd")
    {
      stop_check = 1;
      String reply = "Stop the movement on motor X";
      Serial.println(reply);
      client.publish("teleprint/status/step_x", reply.c_str());
    }
    else if (in_msg == "StepZEnd")
    {
      stop_check = 2;
      String reply = "Stop the movement on motor y2";
      Serial.println(reply);
      client.publish("teleprint/status/step_y2", reply.c_str());
    }
    else if (in_msg == "StepXZEnd")
    {
      stop_check = 3;
      String reply = "Stop the movement on motor X and y2";
      Serial.println(reply);
      client.publish("teleprint/status/step_X", reply.c_str());
      client.publish("teleprint/status/step_y2", reply.c_str());
    }


    stop_move(stop_check);


  }



}

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  WiFi.init(Serial3);

  pinMode(xLimitPin, INPUT_PULLUP);
  pinMode(yLimitPin, INPUT_PULLUP);
  pinMode(zLimitPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(xLimitPin), xLimitISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(yLimitPin), yLimitISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(zLimitPin), yLimitISR, FALLING);


  // Define the servo motor

  stepperX.setMaxSpeed(1000);
  stepperX.setAcceleration(500);

  steppery2.setMaxSpeed(1000);
  steppery2.setAcceleration(500);

  stepperz.setMaxSpeed(2000);
  stepperz.setAcceleration(1000);

  // myServo.attach(servoPin);

    // Move servo to initial position
  // myServo.write(90);

  Serial.println("Setup complete.");




  // Check for the WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println();
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  // Print the MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  printMacAddress(mac);

  // Scan for existing networks
  Serial.println();
  Serial.println("Scanning available networks...");
  listNetworks();

  // Connect to the specified Wi-Fi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // Wait for the Wi-Fi connection to establish
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // If successfully connected to Wi-Fi
  Serial.println();
  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  isConnected = true;

  // Set up the MQTT client
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(mqttCallback);  // Set the callback function
  connectToMQTT();  // Connect to MQTT broker
}

void loop() {
  // Maintain MQTT connection
  if (isConnected) {
    if (!client.connected()) {
      connectToMQTT();
    }
    client.loop();  // Handle MQTT client tasks
  } else {
    delay(10000);
    Serial.println("Scanning available networks...");
    listNetworks();
  }
}

void connectToMQTT() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(mqtt_broker);
  
  // Attempt to connect to MQTT broker using only the client ID
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(client_id)) {
      Serial.println("connected");
      // Subscribe to the necessary topics
      client.subscribe("teleprint/status/init");

      client.subscribe("teleprint/cmd/step_x");  // Subscribe to the new topic

      client.subscribe("teleprint/cmd/step_y2");  // Subscribe to the new topic

      client.subscribe("teleprint/cmd/step_z");  // Subscribe to the new topic

     client.subscribe("teleprint/cmd/stepend");  // Subscribe to the new topic

      // client.subscribe("teleprint/cmd/step_z");  // Subscribe to the new topic
      // client.subscribe("teleprint/cmd/step_z");  // Subscribe to the new topic
      set_home();
      // Publish initial message
      client.publish("teleprint/status/init", "controller init");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void listNetworks() {
  Serial.println("* Scan Networks *");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a WiFi connection");
    while (true);
  }

  Serial.print("number of available networks: ");
  Serial.println(numSsid);

  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet + 1);
    Serial.print(") ");
    Serial.print("Signal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tChannel: ");
    Serial.print(WiFi.channel(thisNet));
    byte bssid[6];
    Serial.print("\t\tBSSID: ");
    printMacAddress(WiFi.BSSID(thisNet, bssid));
    Serial.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));
    Serial.print("\t\tSSID: ");
    Serial.println(WiFi.SSID(thisNet));
    Serial.flush();
  }
  Serial.println();
}

void printEncryptionType(int thisType) {
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.print("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.print("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.print("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.print("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.print("Auto");
      break;
    case ENC_TYPE_UNKNOWN:
    default:
      Serial.print("Unknown");
      break;
  }
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
