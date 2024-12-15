/* File:   PWM_3_1
 *
 * Author: Victor Adolfo Palacio Bastidas
 *
 * El código configura el PWM y modifica el Duty Cycle en tres canales (salida
 * por tres pines) diferentes, para esto se usa un timer para que cambie el Duty
 * Cycle cada vez que el timer se desborda, el código también muestra por
 * consola el valor del duty cycle que se está aplicando en cada pin.
 *
 * Created on 14 de Septiembre de 2024, 14:34
 *
 */

#include <stdio.h>
#include "driver/gpio.h"       //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\driver\gpio\include\driver
#include "freertos/FreeRTOS.h" //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\freertos\FreeRTOS-Kernel\include\freertos
#include "freertos/task.h"     //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\freertos\FreeRTOS-Kernel\include\freertos
#include "esp_log.h"           //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\log\include\esp_log.h
#include "freertos/timers.h"   //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\freertos\FreeRTOS-Kernel\include\freertos
#include "driver/ledc.h"       //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\driver\ledc\include\driver

uint8_t led_level = 0;
static const char *tag = "Main";              // Main es el módulo donde se está usando la librería, en este caso el main
TimerHandle_t xTimers;                        // Este tipo de variable existe dentro del timers.h
int interval = 240;                           // Intervalo de tiempo para el timer, o el evento
int timerId = 1;                              // Identificador del timer
int dutyRed = 0, dutyGreen = 0, dutyBlue = 0; // Variables para modificar el Duty Cycle de cada color

/*Funciones creadas*/
esp_err_t set_timer(void);    // Configura el Timer
esp_err_t set_pwm(void);      // Configura el PWM
esp_err_t set_pwm_duty(void); // Configura el duty cycle

/* vTimerCallback
* timers.h: línea 161: Each timer calls the same callback when it expires.
Define una función de devolución de llamada que será utilizada por varias
instancias del temporizador.
La función de devolución de llamada no hace nada más que contar la cantidad de
veces que el temporizador asociado expira y lo detiene una vez que ha expirado
10 veces, este control solo aplica cuando se usa la variable
xMaxExpiryCountBeforeStopping, en esta aplicación NO se está usando
*/
void vTimerCallback(TimerHandle_t pxTimer)
{

    if ((dutyRed <= 1023) && (dutyGreen <= 1023) && (dutyBlue <= 1023))
    {
        dutyRed += 50;
    }
    else if ((dutyRed > 1023) && (dutyGreen <= 1023) && (dutyBlue <= 1023))
    {
        dutyGreen += 50;
    }
    else if ((dutyGreen > 1023) && (dutyBlue <= 1023))
    {
        dutyBlue += 50;
        dutyRed = 0;
    }
    else if (dutyBlue > 1023)
    {
        dutyRed = 0;
        dutyGreen = 0;
        dutyBlue = 0;
    }

    ESP_LOGE(tag, "duty Cycle Red: %u", dutyRed);
    ESP_LOGI(tag, "duty Cycle Green: %u", dutyGreen);
    ESP_LOGW(tag, "duty Cycle Blue: %u", dutyBlue);

    set_pwm_duty();
}

/* app_main
 *Configura el PWM
 *Configura el timer
 */
void app_main(void)
{
    set_pwm();
    set_timer();
}

/* set_timer
/// @brief Configuura el timer así:
* Funcion xTimerCreate crea y configura el timer
* Timer: es el nombre, no es usado por el Kernel
* (pdMS_TO_TICKS(interval)): función que pasa de ms a ticks para definir
el PERIODO de duración del timer.
* vTimerCallback: función definida en la libreria "timers.h"
* pdTRUE: Este parametro afirma que se debe recargar el timer
/// @param No recibe nada
/// @return ok
*/
esp_err_t set_timer(void)
{
    ESP_LOGW(tag, "Timer init configuration");
    xTimers = xTimerCreate("Timer",                   // Solo es un nombre, no es usado por el kernel.
                           (pdMS_TO_TICKS(interval)), // Conversión de ms a Ticks.
                           pdTRUE,                    // El timer se autorecargará cuando expire. linea nueva 10082024
                           (void *)timerId,           // Asigna un unico ID para igual para indice del array
                           vTimerCallback);           // Cada timer llama la misma función callback cuando termina
    // Aquí sería muy interesante saber que valor recibe xTimers
    if (xTimers == NULL)
    {
        ESP_LOGE(tag, "The timer was not created.");
    }
    else
    {
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            ESP_LOGE(tag, "The timer could not be set into the active state");
        }
    }
    return ESP_OK;
}

/** set_pwm
 * @brief Se tiene que elegir un canal diferente por pin, según el tutorial se
 *  puede usar usar cualquier timer disponible, al parecer en el parametro duty
 *  es con el valor que arranca el PWM., los tres canales estan usando el mismo
 *  timer como fuente, por este motivo solo se configura el timer una vez, ya
 *  que es un solo timer como fuente.
 * @param el mismo timer multiplexado para cada canal.
 */
esp_err_t set_pwm(void)
{
    ledc_channel_config_t channelConfigRed = {0}; // En esta estructura channelConfigRed funciona como la variable de esta estructura
    channelConfigRed.gpio_num = 25;
    channelConfigRed.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigRed.channel = LEDC_CHANNEL_0;
    channelConfigRed.intr_type = LEDC_INTR_DISABLE;
    channelConfigRed.timer_sel = LEDC_TIMER_0;
    channelConfigRed.duty = 0;

    ledc_channel_config_t channelConfigBlue = {0}; // En esta estructura channelConfigBlue funciona como la variable de esta estructura
    channelConfigBlue.gpio_num = 27;
    channelConfigBlue.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigBlue.channel = LEDC_CHANNEL_1;
    channelConfigBlue.intr_type = LEDC_INTR_DISABLE;
    channelConfigBlue.timer_sel = LEDC_TIMER_0;
    channelConfigBlue.duty = 0;

    ledc_channel_config_t channelConfigGreen = {0}; // En esta estructura channelConfigBlue funciona como la variable de esta estructura
    channelConfigGreen.gpio_num = 26;
    channelConfigGreen.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigGreen.channel = LEDC_CHANNEL_2;
    channelConfigGreen.intr_type = LEDC_INTR_DISABLE;
    channelConfigGreen.timer_sel = LEDC_TIMER_0;
    channelConfigGreen.duty = 0;

    // No estoy seguro si esto se pueda llamar como funcion que apunta a una estructura
    ledc_channel_config(&channelConfigBlue);
    ledc_channel_config(&channelConfigGreen);
    ledc_channel_config(&channelConfigRed);

    ledc_timer_config_t timerConfig = {0};
    timerConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    timerConfig.duty_resolution = LEDC_TIMER_10_BIT;
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.freq_hz = 20000; // 20 Khz

    ledc_timer_config(&timerConfig);

    return ESP_OK;
}

esp_err_t set_pwm_duty(void)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, dutyRed);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, dutyGreen);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, dutyBlue);

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);

    return ESP_OK;
}