/* ESP32 HTTP IoT Server Example for Wokwi.com

  https://wokwi.com/projects/320964045035274834

  To test, you need the Wokwi IoT Gateway, as explained here:

  https://docs.wokwi.com/guides/esp32-wifi#the-private-gateway

  Then start the simulation, and open http://localhost:9080
  in another browser tab.

  Note that the IoT Gateway requires a Wokwi Club subscription.
  To purchase a Wokwi Club subscription, go to https://wokwi.com/club
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <uri/UriBraces.h>

// pins
#define GREEN_LED GPIO_NUM_5 // heartbeat LED
#define RED_LED   GPIO_NUM_4 // alert LED for sensor event
#define BLUE_LED  GPIO_NUM_15 // alert LED for button event
#define BUTTON_PIN GPIO_NUM_18
#define POT_PIN 34 // GPIO34
#define THRESHOLD 400
#define MAX_COUNT_SEM 3

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
// Defining the WiFi channel speeds up the connection:
#define WIFI_CHANNEL 6

SemaphoreHandle_t sensorSemaphore;   // counting semaphore
SemaphoreHandle_t buttonSemaphore;   // binary semaphore
SemaphoreHandle_t serialMutex;       // mutex for serial printing
QueueHandle_t sensorQueue;           // optional for extra credit

volatile int SEMCNT = 0; // counting semaphore counter

WebServer server(80);

const int LED1 = 4;
const int LED2 = 15;

bool led1State = false;
bool led2State = false;

void sendHtml() {
  String response = R"(
    <!DOCTYPE html><html>
      <head>
        <title>ESP32 Web Server Demo</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
          html { font-family: sans-serif; text-align: center; }
          body { display: inline-flex; flex-direction: column; }
          h1 { margin-bottom: 1.2em; } 
          h2 { margin: 0; }
          div { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: auto auto; grid-auto-flow: column; grid-gap: 1em; }
          .btn { background-color: #5B5; border: none; color: #fff; padding: 0.5em 1em;
                 font-size: 2em; text-decoration: none }
          .btn.OFF { background-color: #333; }
        </style>
      </head>
            
      <body>
        <h1>ESP32 Web Server</h1>

        <div>
          <h2>LED 1</h2>
          <a href="/toggle/1" class="btn LED1_TEXT">LED1_TEXT</a>
          <h2>LED 2</h2>
          <a href="/toggle/2" class="btn LED2_TEXT">LED2_TEXT</a>
        </div>
      </body>
    </html>
  )";
  response.replace("LED1_TEXT", led1State ? "ON" : "OFF");
  response.replace("LED2_TEXT", led2State ? "ON" : "OFF");
  server.send(200, "text/html", response);
}

// blinks green LED at 1 Hz
void HeartbeatTask(void *pvParameters) {
  Serial.println("Heartbeat Monitor System Starting...");
  while (1) {
    digitalWrite(GREEN_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(GREEN_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// monitors the button state and signals semaphore on press
void ButtonTask(void *pvParameters) {
  bool lastState = HIGH;
  bool currentState;

  while (1) {
    currentState = digitalRead(BUTTON_PIN);
    // detect falling edge
    if (currentState == LOW && lastState == HIGH) {
      xSemaphoreGive(buttonSemaphore);
      vTaskDelay(pdMS_TO_TICKS(50)); // debounce delay
    }
    lastState = currentState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// reads the analog sensor every 17 ms
void SensorTask(void *pvParameters) {
  while (1) {
    int val = analogRead(POT_PIN);

    // if value exceeds threshold, signal counting semaphore, and save value to queue
    if (val > THRESHOLD) {
      if(SEMCNT < MAX_COUNT_SEM+1) SEMCNT++;
      xSemaphoreGive(sensorSemaphore);
      xQueueSend(sensorQueue, &val, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(17));
  }
}

// handles events signaled by semaphores
void event_handler_task(void *pvParameters) {
    while (1) {
        // if a high heartbeat sensor event occurred, print alert and blink red LED
        if (xSemaphoreTake(sensorSemaphore, 0)) {
            SEMCNT--;

            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("Sensor event: High Heartbeat Set! Check Patient.");
            xSemaphoreGive(serialMutex);

            digitalWrite(RED_LED, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(RED_LED, LOW);
        }

        // if button event occurred, print alert and turn on blue LED 
        if (xSemaphoreTake(buttonSemaphore, 0)) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.println("Button event: System switched from normal to alert!");
            xSemaphoreGive(serialMutex);

            digitalWrite(BLUE_LED, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(BLUE_LED, LOW);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Idle delay to yield CPU
    }
}

void setup() {
  Serial.begin(115200); 

  // configure LED pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  // configure button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // configure ADC pin
  pinMode(POT_PIN, INPUT);

  // create queue for sensor values
  sensorQueue = xQueueCreate(10, sizeof(int));

  // semaphores and mutex initialization
  sensorSemaphore = xSemaphoreCreateCounting(MAX_COUNT_SEM, 0);
  buttonSemaphore = xSemaphoreCreateBinary();
  serialMutex = xSemaphoreCreateMutex();

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Connected!");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", sendHtml);

  server.on(UriBraces("/toggle/{}"), []() {
    String led = server.pathArg(0);
    Serial.print("Toggle LED #");
    Serial.println(led);

    switch (led.toInt()) {
      case 1:
        led1State = !led1State;
        digitalWrite(LED1, led1State);
        break;
      case 2:
        led2State = !led2State;
        digitalWrite(LED2, led2State);
        break;
    }

    sendHtml();
  });

  // Create your actual tasks with larger stack sizes
  xTaskCreate(HeartbeatTask, "Heartbeat", 2048, NULL, 1, NULL);
  xTaskCreate(ButtonTask, "Button", 2048, NULL, 3, NULL);
  xTaskCreate(SensorTask, "Sensor", 2048, NULL, 3, NULL);
  xTaskCreate(event_handler_task, "Event Handler", 2048, NULL, 2, NULL);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  delay(2);
}