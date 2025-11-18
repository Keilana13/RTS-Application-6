# Keilana Brooks
# EEE 4775: RTS
# Application 5

## Project Description
The goal of Application 5 is upgrade a basic ESP32 web-controlled LED system into a real-time, multitasking application using FreeRTOS. Starting from an Arduino-style WiFi web server, the system is expanded with tasks that monitor an analog sensor, detect button presses, handle alerts, and maintain a heartbeat LED. Using binary semaphores, counting semaphores, and a mutex, the tasks safely coordinate events and protect shared resources like Serial output. The ESP32 is simulated in Wokwi with WiFi enabled so the webpage can remotely toggle LEDs and interact with system states. Overall, the project demonstrates how a simple IoT sketch can be transformed into a responsive, reliable real-time embedded system.

### Q1: Migration Strategy & Refactor Map
Outline the three largest structural changes you had to make when porting the single-loop Arduino sketch to your chosen RTOS framework. Include one code snippet (before → after) that best illustrates the refactor.

The largest structural changes I had to make was taking the actions like the sensor reading and button handling and mkae them individual tasks and functions, including semaphore and mutexes instead of only using delays, and the setup of the webserver and how it communicates

Before: 
void loop() {
  int sensor = analogRead(A0);

  if (sensor > THRESHOLD) {
    Serial.println("ALERT!");
    digitalWrite(RED_LED, HIGH);
    delay(200);
    digitalWrite(RED_LED, LOW);
  }

  if (digitalRead(BUTTON) == HIGH) {
    mode = !mode;
    Serial.println("Mode toggled!");
    delay(150);
  }

  server.handleClient();  // web server runs
}

After: 
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

### Q2: Framework Trade-off Review
Compare ESP-IDF FreeRTOS vs Arduino + FreeRTOS for this project: list two development advantages and one limitation you encountered with the path you chose. If you had chosen the other path, which specific API or tooling difference do you think would have helped / hurt?

Two development advatages with working with the Arduino + FreeRTOS path was that the Arduino framework made it easy to reuse existing code for the LED, button, and sensor tasks and the task synchronization without rewriting everything. The second development advantage was the integration of the WiFi code and libraries. One limitation I encountered was the actual conversion and distinguishing between Arduino and ESP-IDF formating. 

### Q3: Queue Depth & Memory Footprint
How did you size your sensor-data queue (length & item size)? Show one experiment where the queue nearly overflowed and explain how you detected or mitigated it.

I initialized the queue to hold 10 integers. 

### Q4: Debug & Trace Toolkit
List the most valuable debug technique you used (e.g., esp_log_level, vTaskGetInfo,  print-timestamp). Show a short trace/log excerpt that helped you verify correct task sequencing or uncover a bug.

When my code wasn't initially working, I commented out all of the tasks in setup except the heartbeat task so I could see where the first error was orginating from. 

### Q5: Domain Reflection
Relate one design decision directly to your chosen theme’s stakes (e.g., “missing two heart-rate alerts could delay CPR by ≥1 s”). Briefly propose the next real feature you would add in an industrial version of this system.

If the sensor's value is set to high, indicating that the device's heartbeat rate is set too high for the patient, the system sends an alert to the terminal and the red LED turns on. Another feature I would add is if the patient's heartbeat falls below a certain threshold, the device would automatically bring the patient's heartbeat rate back up to a normal number.

### Q6: Web-Server Data Streaming: Benefits & Limitations
Describe two concrete advantages of pushing live sensor data to the on-board ESP32 web server (e.g., richer UI, off-board processing, remote monitoring, etc.). Identify two limitations or trade-offs this decision introduces for a real-time system (think: added latency, increased heap/stack usage, potential priority inversion, Wi-Fi congestion, attack surface, maintenance of HTTP context inside RTOS tasks, etc.).

Two adavtages of pushing live sensor data to the web server allows the data to be captured by other devices or services for long-term logging and better visualization and this also allows access to the system for multiple users simultaneously via the web server. Two limitations of these advatages would be the maintenance of HTTP context inside RTOS tasks and the increased usage of the heap/stack. Both of these limitations could also delay critical tasks if not prioritized correctly. If my system had to stream data reliably under heavy load, I would include additional semaphores and better prioritize tasks.