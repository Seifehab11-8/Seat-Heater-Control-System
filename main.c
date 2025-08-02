#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#define TEMP_SENSOR_TASK_PRIORITY   (2)
#define HEATER_CONTROL_TASK_PRIORITY (3)
#define USER_INPUT_TASK_PRIORITY    (1)
#define DISPLAY_TASK_PRIORITY       (1)
#define SAFETY_TASK_PRIORITY        (4)

#define TEMP_QUEUE_SIZE             (5)
#define TEMP_READING_PERIOD_MS      (100)
#define HEATER_UPDATE_PERIOD_MS     (50)
#define DISPLAY_UPDATE_PERIOD_MS    (500)
#define SAFETY_CHECK_PERIOD_MS      (200)

#define MAX_TEMP_THRESHOLD          (45)
#define MIN_TEMP_THRESHOLD          (15)
#define TEMP_HYSTERESIS             (2)

#define HEATER_PWM_BASE             PWM0_BASE
#define HEATER_PWM_GEN              PWM_GEN_0
#define HEATER_PWM_OUT              PWM_OUT_0
#define HEATER_PWM_PIN              GPIO_PIN_0

#define TEMP_SENSOR_ADC_BASE        ADC0_BASE
#define TEMP_SENSOR_ADC_SEQ         ADC_SEQUENCER_0

#define USER_BUTTON_PORT            GPIO_PORTA_BASE
#define USER_BUTTON_PIN             GPIO_PIN_1
#define POWER_BUTTON_PIN            GPIO_PIN_2

#define LED_PORT                    GPIO_PORTB_BASE
#define POWER_LED_PIN               GPIO_PIN_0
#define HEATING_LED_PIN             GPIO_PIN_1
#define STATUS_LED_PIN              GPIO_PIN_2

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

QueueHandle_t xTempQueue;
QueueHandle_t xControlQueue;
SemaphoreHandle_t xDisplayMutex;
TimerHandle_t xSafetyTimer;

volatile heater_control_t g_heater_control = {HEATER_OFF, false, 25.0, false};
volatile float g_current_temp = 0.0;
volatile bool g_system_fault = false;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    while(1);
}

void vApplicationMallocFailedHook(void) {
    while(1);
}

void SystemInit(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    GPIOPinTypeGPIOInput(USER_BUTTON_PORT, USER_BUTTON_PIN | POWER_BUTTON_PIN);
    GPIOPadConfigSet(USER_BUTTON_PORT, USER_BUTTON_PIN | POWER_BUTTON_PIN, 
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    GPIOPinTypeGPIOOutput(LED_PORT, POWER_LED_PIN | HEATING_LED_PIN | STATUS_LED_PIN);
    
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    GPIOPinTypePWM(GPIO_PORTA_BASE, HEATER_PWM_PIN);
    PWMGenConfigure(HEATER_PWM_BASE, HEATER_PWM_GEN, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(HEATER_PWM_BASE, HEATER_PWM_GEN, SysCtlClockGet() / 8 / 1000);
    PWMPulseWidthSet(HEATER_PWM_BASE, HEATER_PWM_OUT, 0);
    PWMOutputState(HEATER_PWM_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(HEATER_PWM_BASE, HEATER_PWM_GEN);
    
    ADCSequenceConfigure(TEMP_SENSOR_ADC_BASE, TEMP_SENSOR_ADC_SEQ, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(TEMP_SENSOR_ADC_BASE, TEMP_SENSOR_ADC_SEQ, 0, 
                           ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(TEMP_SENSOR_ADC_BASE, TEMP_SENSOR_ADC_SEQ);
}

float ReadTemperature(void) {
    uint32_t adc_value[1];
    
    ADCProcessorTrigger(TEMP_SENSOR_ADC_BASE, TEMP_SENSOR_ADC_SEQ);
    while(!ADCIntStatus(TEMP_SENSOR_ADC_BASE, TEMP_SENSOR_ADC_SEQ, false));
    ADCSequenceDataGet(TEMP_SENSOR_ADC_BASE, TEMP_SENSOR_ADC_SEQ, adc_value);
    ADCIntClear(TEMP_SENSOR_ADC_BASE, TEMP_SENSOR_ADC_SEQ);
    
    return ((1475.0 - ((2475.0 * adc_value[0]) / 4096.0)) / 5.0);
}

void SetHeaterPWM(heater_level_t level) {
    uint32_t duty_cycle = 0;
    
    switch(level) {
        case HEATER_OFF:
            duty_cycle = 0;
            break;
        case HEATER_LOW:
            duty_cycle = 25;
            break;
        case HEATER_MEDIUM:
            duty_cycle = 50;
            break;
        case HEATER_HIGH:
            duty_cycle = 75;
            break;
    }
    
    uint32_t period = PWMGenPeriodGet(HEATER_PWM_BASE, HEATER_PWM_GEN);
    PWMPulseWidthSet(HEATER_PWM_BASE, HEATER_PWM_OUT, (period * duty_cycle) / 100);
}

void UpdateStatusLEDs(void) {
    if(g_heater_control.power_on) {
        GPIOPinWrite(LED_PORT, POWER_LED_PIN, POWER_LED_PIN);
    } else {
        GPIOPinWrite(LED_PORT, POWER_LED_PIN, 0);
    }
    
    if(g_heater_control.level > HEATER_OFF && g_heater_control.power_on) {
        GPIOPinWrite(LED_PORT, HEATING_LED_PIN, HEATING_LED_PIN);
    } else {
        GPIOPinWrite(LED_PORT, HEATING_LED_PIN, 0);
    }
    
    if(g_system_fault) {
        GPIOPinWrite(LED_PORT, STATUS_LED_PIN, STATUS_LED_PIN);
    } else {
        GPIOPinWrite(LED_PORT, STATUS_LED_PIN, 0);
    }
}

void vTemperatureSensorTask(void *pvParameters) {
    temp_reading_t temp_reading;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        temp_reading.temperature = ReadTemperature();
        temp_reading.timestamp = xTaskGetTickCount();
        g_current_temp = temp_reading.temperature;
        
        xQueueSend(xTempQueue, &temp_reading, portMAX_DELAY);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TEMP_READING_PERIOD_MS));
    }
}

void vHeaterControlTask(void *pvParameters) {
    temp_reading_t received_temp;
    heater_control_t local_control;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        if(xQueueReceive(xTempQueue, &received_temp, 0) == pdPASS) {
            local_control = g_heater_control;
            
            if(!local_control.power_on || local_control.safety_override) {
                local_control.level = HEATER_OFF;
            } else {
                float temp_diff = local_control.target_temp - received_temp.temperature;
                
                if(temp_diff > TEMP_HYSTERESIS + 5) {
                    local_control.level = HEATER_HIGH;
                } else if(temp_diff > TEMP_HYSTERESIS) {
                    local_control.level = HEATER_MEDIUM;
                } else if(temp_diff > 0) {
                    local_control.level = HEATER_LOW;
                } else {
                    local_control.level = HEATER_OFF;
                }
            }
            
            g_heater_control.level = local_control.level;
            SetHeaterPWM(local_control.level);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HEATER_UPDATE_PERIOD_MS));
    }
}

void vUserInputTask(void *pvParameters) {
    static uint8_t power_button_state = 0;
    static uint8_t level_button_state = 0;
    static TickType_t last_power_press = 0;
    static TickType_t last_level_press = 0;
    
    while(1) {
        uint8_t power_pin = GPIOPinRead(USER_BUTTON_PORT, POWER_BUTTON_PIN);
        uint8_t level_pin = GPIOPinRead(USER_BUTTON_PORT, USER_BUTTON_PIN);
        TickType_t current_time = xTaskGetTickCount();
        
        if(!power_pin && power_button_state && 
           (current_time - last_power_press) > pdMS_TO_TICKS(200)) {
            g_heater_control.power_on = !g_heater_control.power_on;
            if(!g_heater_control.power_on) {
                g_heater_control.level = HEATER_OFF;
            }
            last_power_press = current_time;
        }
        power_button_state = power_pin ? 1 : 0;
        
        if(!level_pin && level_button_state && g_heater_control.power_on &&
           (current_time - last_level_press) > pdMS_TO_TICKS(200)) {
            switch(g_heater_control.level) {
                case HEATER_OFF:
                    g_heater_control.target_temp = 25.0;
                    break;
                case HEATER_LOW:
                    g_heater_control.target_temp = 30.0;
                    break;
                case HEATER_MEDIUM:
                    g_heater_control.target_temp = 35.0;
                    break;
                case HEATER_HIGH:
                    g_heater_control.target_temp = 25.0;
                    break;
            }
            last_level_press = current_time;
        }
        level_button_state = level_pin ? 1 : 0;
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vDisplayTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        if(xSemaphoreTake(xDisplayMutex, portMAX_DELAY) == pdTRUE) {
            UpdateStatusLEDs();
            xSemaphoreGive(xDisplayMutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DISPLAY_UPDATE_PERIOD_MS));
    }
}

void vSafetyMonitorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        bool fault_detected = false;
        
        if(g_current_temp > MAX_TEMP_THRESHOLD) {
            fault_detected = true;
            g_heater_control.safety_override = true;
        } else if(g_current_temp < MIN_TEMP_THRESHOLD && g_heater_control.level > HEATER_OFF) {
            fault_detected = true;
        }
        
        if(fault_detected != g_system_fault) {
            g_system_fault = fault_detected;
            if(!fault_detected) {
                g_heater_control.safety_override = false;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAFETY_CHECK_PERIOD_MS));
    }
}

void vSafetyTimerCallback(TimerHandle_t xTimer) {
    if(g_current_temp > MAX_TEMP_THRESHOLD + 5) {
        g_heater_control.power_on = false;
        g_heater_control.level = HEATER_OFF;
        g_system_fault = true;
    }
}

int main(void) {
    SystemInit();
    
    xTempQueue = xQueueCreate(TEMP_QUEUE_SIZE, sizeof(temp_reading_t));
    xControlQueue = xQueueCreate(5, sizeof(heater_control_t));
    xDisplayMutex = xSemaphoreCreateMutex();
    xSafetyTimer = xTimerCreate("SafetyTimer", pdMS_TO_TICKS(1000), pdTRUE, 
                               (void *)0, vSafetyTimerCallback);
    
    if(xTempQueue == NULL || xControlQueue == NULL || 
       xDisplayMutex == NULL || xSafetyTimer == NULL) {
        while(1);
    }
    
    if(xTaskCreate(vTemperatureSensorTask, "TempSensor", 
                   configMINIMAL_STACK_SIZE, NULL, 
                   TEMP_SENSOR_TASK_PRIORITY, NULL) != pdPASS) {
        while(1);
    }
    
    if(xTaskCreate(vHeaterControlTask, "HeaterCtrl", 
                   configMINIMAL_STACK_SIZE, NULL, 
                   HEATER_CONTROL_TASK_PRIORITY, NULL) != pdPASS) {
        while(1);
    }
    
    if(xTaskCreate(vUserInputTask, "UserInput", 
                   configMINIMAL_STACK_SIZE, NULL, 
                   USER_INPUT_TASK_PRIORITY, NULL) != pdPASS) {
        while(1);
    }
    
    if(xTaskCreate(vDisplayTask, "Display", 
                   configMINIMAL_STACK_SIZE, NULL, 
                   DISPLAY_TASK_PRIORITY, NULL) != pdPASS) {
        while(1);
    }
    
    if(xTaskCreate(vSafetyMonitorTask, "Safety", 
                   configMINIMAL_STACK_SIZE, NULL, 
                   SAFETY_TASK_PRIORITY, NULL) != pdPASS) {
        while(1);
    }
    
    xTimerStart(xSafetyTimer, 0);
    
    vTaskStartScheduler();
    
    while(1);
}
