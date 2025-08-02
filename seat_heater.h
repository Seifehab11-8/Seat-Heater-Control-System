#ifndef SEAT_HEATER_H
#define SEAT_HEATER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

typedef enum {
    HEATER_OFF = 0,
    HEATER_LOW = 1,
    HEATER_MEDIUM = 2,
    HEATER_HIGH = 3
} heater_level_t;

typedef struct {
    float temperature;
    uint32_t timestamp;
} temp_reading_t;

typedef struct {
    heater_level_t level;
    bool power_on;
    float target_temp;
    bool safety_override;
} heater_control_t;

extern QueueHandle_t xTempQueue;
extern QueueHandle_t xControlQueue;
extern SemaphoreHandle_t xDisplayMutex;
extern TimerHandle_t xSafetyTimer;

extern volatile heater_control_t g_heater_control;
extern volatile float g_current_temp;
extern volatile bool g_system_fault;

void SystemInit(void);
float ReadTemperature(void);
void SetHeaterPWM(heater_level_t level);
void UpdateStatusLEDs(void);

void vTemperatureSensorTask(void *pvParameters);
void vHeaterControlTask(void *pvParameters);
void vUserInputTask(void *pvParameters);
void vDisplayTask(void *pvParameters);
void vSafetyMonitorTask(void *pvParameters);
void vSafetyTimerCallback(TimerHandle_t xTimer);

#endif
