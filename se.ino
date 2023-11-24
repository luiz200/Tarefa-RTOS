#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_random.h>


// Definindo as filas
QueueHandle_t queueIntReadings;
QueueHandle_t queueVoltage;
SemaphoreHandle_t voltageSemaphore;

// Pinos
const int analogPin = 25;
const int ledPin = 26;
const int ledPin2 = 2;

// Temporizadores
TimerHandle_t intReadingsTimer;

// Funções auxiliares
void intReadingsTimerCallback(TimerHandle_t xTimer);


void taskIntToVoltage(void *pvParameters) {
  //Serial.println("convertendo");
  while (1) {
    int sensorValue;
    if (xQueueReceive(queueIntReadings, &sensorValue, portMAX_DELAY)) {
      double voltage = sensorValue * (3.3 / 4096);
      xSemaphoreTake(voltageSemaphore, portMAX_DELAY);
      xQueueSend(queueVoltage, &voltage, portMAX_DELAY);
      xSemaphoreGive(voltageSemaphore);
    }
  }
}

void taskPlotVoltage(void *pvParameters) {
  //Serial.println("plotando led");
  while (1) {
    double voltage;
    if (xQueuePeek(queueVoltage, &voltage, portMAX_DELAY)) {
      //Serial.print("Enviando valor para o python: ");
      Serial.println(voltage);
    }
  }
}

void taskControlLed(void *pvParameters) {
  
  while (1) {
    double voltage;
    
    if (xQueueReceive(queueVoltage, &voltage, portMAX_DELAY)) {
      xSemaphoreTake(voltageSemaphore, portMAX_DELAY);
      
      if (voltage > 1.5) {
        digitalWrite(ledPin2, HIGH);
      } else {
        digitalWrite(ledPin2, LOW);
      }
      int sensor = (voltage * 4096) / 3.3;
      dacWrite(ledPin, map(sensor,0,255,0,4095));
      xSemaphoreGive(voltageSemaphore);
    }
  }
}

void intReadingsTimerCallback(TimerHandle_t xTimer) {
  
  int sensorValue = analogRead(analogPin);
  xQueueSend(queueIntReadings, &sensorValue, 0);
}

// Funções das tarefas

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  esp_random() % 100;

  // Inicializando as filas
  queueIntReadings = xQueueCreate(10, sizeof(int));
  queueVoltage = xQueueCreate(10, sizeof(double));

  // Inicializando o semáforo
  voltageSemaphore = xSemaphoreCreateMutex();

  // Criando as tarefas
  xTaskCreatePinnedToCore(taskIntToVoltage, "TaskIntToVoltage", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskPlotVoltage, "TaskPlotVoltage", 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskControlLed, "TaskControlLed", 1024, NULL, 1, NULL, 1);

  // Inicializando o temporizador
  intReadingsTimer = xTimerCreate("IntReadingsTimer", pdMS_TO_TICKS(300), pdTRUE, (void *)0, intReadingsTimerCallback);
  xTimerStart(intReadingsTimer, portMAX_DELAY);

}

void loop() {
  vTaskSuspend(NULL);
}