# Keilana Brooks
# EEE 4775: RTS
# Application 6

## Project Description
The goal of Application 6 is to make a system for the healthcare tech company, MedAid Inc., for their Wearable Heart Monitor. The system monitors the patient's heartbeat and when a high heartbeat is detected, an alert is displayed on the monitor and the red LED blinks. If a button press is detected, an alert is sent to the monitor and the blue LED blinks to notify the patient that the system's state has switched from normal to alert. Meanwhile, there is also a green LED that is always blinking to mimic the detected patient's heartbeat. Lastly, the red and blue LEDs can be controlled by a ESP32 webpage.  

In more technical terms, the system is a basic ESP32 web-controlled LED system into a real-time, multitasking application using FreeRTOS. Starting from an Arduino-style WiFi web server, the system is expanded with tasks that monitor an analog sensor, detect button presses using an ISR, handle alerts, and maintain a heartbeat LED. Using binary semaphores, counting semaphores, and a mutex, the tasks safely coordinate events and protect shared resources like Serial output. The ESP32 is simulated in Wokwi with WiFi enabled so the webpage can remotely toggle LEDs and interact with system states. 

### Q1: Scheduler Fit: How do your task priorities / RTOS settings guarantee every task’s deadline in Wokwi? Cite one timestamp pair that proves it.
The task priorities I have ensure that hard deadlines are met. For example, the sensor and button task have the highest priorities because they are hard deadline tasks. We can see that the deadlines were met for each task in the timestamps in the terminal: [8353 ms] Button event: System switched from normal to alert!
[10253 ms] Button event: System switched from normal to alert!
As soon as the button was pressed the event of the terminal print and LED blinking occured and the same occurred for the sensor task. The RTOS settings of the binary semaphores also ensured that tasks executed when a certain event occurred instead of constantly looping to check if a condition was met. 


### Q2:Race‑Proofing: Where could a race occur? Show the exact line(s) you protected and which primitive solved it.
A race could occur between the heartbeat task and the event_handler_task print to Serial. Since the green LED is constantly blinking, the timing of when the blue LED blinks could align. The race between these tasks is prevented through the use of a mutex for the Serial. The code below shows that a mutex is used for both prints, so only one task can print to the terminal at a time. 
// from heartbeat_task
// log heartbeat time
xSemaphoreTake(serialMutex, portMAX_DELAY);
logTime();
Serial.println("Heartbeat: Patient stable");
xSemaphoreGive(serialMutex);

// from event_sensor_task for the sensor event
xSemaphoreTake(serialMutex, portMAX_DELAY);
logTime();
Serial.println("Sensor event: High Heartbeat Set! Check Patient.");
xSemaphoreGive(serialMutex);

### Q3:Worst‑Case Spike: Describe the heaviest load you threw at the prototype (e.g., sensor spam, comm burst). What margin (of time) remained before a deadline would slip?
The heaviest load I threw at the prototype was a sensor spam from the SensorTask. When the SensorTask detects the sensor at a value higher than the threshold, it prints an alert every 10ms. So, there is a margin of 10ms which allows other tasks to print to the terminal if needed. After this 10ms margin, a deadline would most likely slip.

### Q4:Design Trade‑off: Name one feature you didn’t add (or simplified) to keep timing predictable. Why was that the right call for your chosen company?
A feature I kept simiplified to keep timing predictable were the features of the web server. This simplification was intentional to avoid unpredictable delays that could interfere with the heartbeat monitoring and alert tasks. For a wearable heart monitor, ensuring hard deadlines for sensor readings and alerts are critical for patient safety. So, I support my decision because it was the right trade‑off to prioritize timing predictability over advanced features.

## AI Tool Use:
### Inside ISR for debouncing problem:
// Copilot. Accessed 2025-12-04. Prompt: "when i press the button one time this line prints twice with 1000ms between each line: Button event: System switched from normal to alert!" Generated using Copilot.

  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  
  // Debouncing: ignore if interrupt occurred within 200ms
  if (interruptTime - lastInterruptTime > 200) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemISR, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  
  lastInterruptTime = interruptTime;

### Inside Button_Task: 
// ChatGPT. Accessed 2025-12-04. Prompt: "i want to keep the button task, but have the event handler task handle the event, but still use the ISR." Generated using ChatGPT.

  while (1) {
    // Wait for ISR to give semaphore to button task
    if (xSemaphoreTake(buttonSemISR, portMAX_DELAY)) {
      // give semaphore to event_handler_task to handle the button event
      xSemaphoreGive(buttonTaskSem);
    }
  }