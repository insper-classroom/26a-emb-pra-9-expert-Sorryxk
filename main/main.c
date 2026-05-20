#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "mpu6050.h"

#include "Fusion.h"

/* ===== Lab9 metric instrumentation =====
 * - Toggle GPIOs around each task body to measure WCET/jitter on a scope.
 * - Disable STACK_MONITOR_ENABLED when measuring time (printf perturbs).
 */
#define INSTRUMENT_ENABLED      1
#define STACK_MONITOR_ENABLED   0   // <- liga para coletar high water mark; desliga para medir tempos

#define PIN_MPU_PROBE     16
#define PIN_FUSION_PROBE  17
#define PIN_UART_PROBE    18
#define PIN_PWM_PROBE     19

#define PIN_MPU_VCC       14   // GPIO alimentando o VCC do MPU (3.3 V)

#define SAMPLE_PERIOD (0.01f) // 100 Hz

#define UART_ID uart0
#define UART_BAUD 115200
#define UART_TX_GPIO 0
#define UART_RX_GPIO 1

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
const int LED_PIN = 15;        // LED único via PWM (antes era RGB)

QueueHandle_t xQueueMPU;
QueueHandle_t xQueuePos;
QueueHandle_t xQueueColor;
SemaphoreHandle_t xSemaphoreBtn;

typedef struct adc {
    int axis;
    int val;
} adc_t;

typedef struct data {
    FusionVector gyroscope;
    FusionVector accelerometer;
} data_t;

typedef struct led {
    int level;   // 0..255 PWM duty
} led_t;

#if INSTRUMENT_ENABLED
static inline void probe_init(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}
#define PROBE_HIGH(p) gpio_put((p), 1)
#define PROBE_LOW(p)  gpio_put((p), 0)
#else
#define probe_init(p) do {} while (0)
#define PROBE_HIGH(p) do {} while (0)
#define PROBE_LOW(p)  do {} while (0)
#endif

static int angle_to_255(float angle_deg) {
    if (angle_deg > 180.0f) angle_deg = 180.0f;
    else if (angle_deg < -180.0f) angle_deg = -180.0f;
    return (int)(angle_deg * (255.0f / 180.0f));
}

static int tilt_to_255(float tilt_deg) {
    if (tilt_deg < 0.0f) tilt_deg = 0.0f;
    else if (tilt_deg > 90.0f) tilt_deg = 90.0f;
    return (int)(tilt_deg * (255.0f / 90.0f));
}

static void mpu6050_init() {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    *temp = buffer[6] << 8 | buffer[7];
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[8 + i * 2] << 8 | buffer[8 + (i * 2) + 1]);
    }
}

void mpu6050_task(void *p) {
    probe_init(PIN_MPU_PROBE);
    mpu6050_init();

    while (true) {
        PROBE_HIGH(PIN_MPU_PROBE);
        int16_t acceleration[3], gyro[3], temp;
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        data_t sensor_data;
        sensor_data.gyroscope = gyroscope;
        sensor_data.accelerometer = accelerometer;
        xQueueSend(xQueueMPU, &sensor_data, pdMS_TO_TICKS(10));
        PROBE_LOW(PIN_MPU_PROBE);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void fusion_task(void *p) {
    probe_init(PIN_FUSION_PROBE);
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    data_t sensor_data;
    adc_t adc_x = {.axis = 0}, adc_y = {.axis = 1};
    led_t led_data;
    const int dead_zone = 15;
    const float mouse_speed = 0.25f;
    int send_counter = 0;

    while (true) {
        if (xQueueReceive(xQueueMPU, &sensor_data, pdMS_TO_TICKS(10))) {
            PROBE_HIGH(PIN_FUSION_PROBE);
            FusionVector gyroscope = sensor_data.gyroscope;
            FusionVector accelerometer = sensor_data.accelerometer;

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

            if (accelerometer.axis.y > 1.5f) {
                xSemaphoreGive(xSemaphoreBtn);
            }

            adc_x.val = (int)(-angle_to_255(euler.angle.yaw)  * mouse_speed);
            adc_y.val = (int)(-angle_to_255(euler.angle.roll) * mouse_speed);

            /* LED único: brilho proporcional ao tilt total (pitch+roll). */
            int brightness = tilt_to_255(fabsf(euler.angle.pitch))
                           + tilt_to_255(fabsf(euler.angle.roll));
            if (brightness > 255) brightness = 255;
            led_data.level = brightness;
            xQueueOverwrite(xQueueColor, &led_data);

            send_counter = (send_counter + 1) % 5;
            if (send_counter == 0) {
                if (adc_x.val > dead_zone || adc_x.val < -dead_zone) {
                    xQueueSend(xQueuePos, &adc_x, pdMS_TO_TICKS(10));
                }
                if (adc_y.val > dead_zone || adc_y.val < -dead_zone) {
                    xQueueSend(xQueuePos, &adc_y, pdMS_TO_TICKS(10));
                }
            }
            PROBE_LOW(PIN_FUSION_PROBE);
        }
    }
}

void uart_task(void *p) {
    probe_init(PIN_UART_PROBE);
    adc_t adc_data;
    while (true) {
        if (xQueueReceive(xQueuePos, &adc_data, pdMS_TO_TICKS(100))) {
            PROBE_HIGH(PIN_UART_PROBE);
            putchar_raw(0xFF);
            putchar_raw(adc_data.axis);
            putchar_raw(adc_data.val);
            putchar_raw(adc_data.val >> 8);
            stdio_flush();
            PROBE_LOW(PIN_UART_PROBE);
        }

        if (xSemaphoreTake(xSemaphoreBtn, 0) == pdTRUE) {
            PROBE_HIGH(PIN_UART_PROBE);
            putchar_raw(0xFF); putchar_raw(2); putchar_raw(1); putchar_raw(0);
            stdio_flush();
            vTaskDelay(pdMS_TO_TICKS(50));
            putchar_raw(0xFF); putchar_raw(2); putchar_raw(0); putchar_raw(0);
            stdio_flush();
            PROBE_LOW(PIN_UART_PROBE);
        }
    }
}

void pwm_task(void *p) {
    probe_init(PIN_PWM_PROBE);

    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    const uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
    const uint chan      = pwm_gpio_to_channel(LED_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_config_set_wrap(&config, 255);
    pwm_init(slice_num, &config, true);
    pwm_set_chan_level(slice_num, chan, 0);

    led_t led_data = {0};

    while (true) {
        if (xQueueReceive(xQueueColor, &led_data, pdMS_TO_TICKS(20))) {
            PROBE_HIGH(PIN_PWM_PROBE);
            pwm_set_chan_level(slice_num, chan, led_data.level);
            PROBE_LOW(PIN_PWM_PROBE);
        }
    }
}

#if STACK_MONITOR_ENABLED
void stack_monitor_task(void *p) {
    static TaskStatus_t tasks[16];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        UBaseType_t n = uxTaskGetSystemState(tasks, 16, NULL);
        printf("+------------------+-------+\n");
        printf("| %-16s | %5s |\n", "task", "free");
        printf("+------------------+-------+\n");
        for (UBaseType_t i = 0; i < n; i++) {
            printf("| %-16s | %5u |\n",
                   tasks[i].pcTaskName,
                   (unsigned)tasks[i].usStackHighWaterMark);
        }
        printf("+------------------+-------+\n");
        printf("| heap free min    | %5u |\n",
               (unsigned)xPortGetMinimumEverFreeHeapSize());
        printf("+------------------+-------+\n\n");
    }
}
#endif

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("LAB9-READY\r\n");

    /* GP14 alimenta o VCC do MPU (3.3 V). Ligar antes de qualquer I2C. */
    gpio_init(PIN_MPU_VCC);
    gpio_set_dir(PIN_MPU_VCC, GPIO_OUT);
    gpio_put(PIN_MPU_VCC, 1);
    sleep_ms(50); // tempo de boot do MPU

    xQueueMPU     = xQueueCreate(64, sizeof(data_t));
    xQueuePos     = xQueueCreate(64, sizeof(adc_t));
    xQueueColor   = xQueueCreate(1,  sizeof(led_t));
    xSemaphoreBtn = xSemaphoreCreateBinary();

    /* Stacks ajustadas pela regra dos 80%:
       Pior caso medido (HWM) -> alocação tal que uso <= ~80%.
       mpu: usado 94 -> 144 (65%)   [margem extra para I2C/ISR]
       fusion: usado 193 -> 256 (75%)
       uart: usado 84  -> 128 (66%)
       pwm: usado 40   -> 96  (42%) [headroom mínimo seguro de contexto+FPU] */
    TaskHandle_t hMpu = NULL, hFusion = NULL, hUart = NULL, hPwm = NULL;
    xTaskCreate(mpu6050_task, "mpu",    192, NULL, 2, &hMpu);
    xTaskCreate(fusion_task,  "fusion", 320, NULL, 2, &hFusion);
    xTaskCreate(uart_task,    "uart",   128, NULL, 1, &hUart);
    xTaskCreate(pwm_task,     "pwm",     96, NULL, 1, &hPwm);

#if STACK_MONITOR_ENABLED
    /* Durante a medida de stack, mantenha o IMU PARADO para a uart_task não
       enviar bytes binários que colidem com o printf do monitor no USB-CDC. */
    xTaskCreate(stack_monitor_task, "monitor", 1024, NULL, 1, NULL);
#endif

#if (configNUMBER_OF_CORES > 1) && (configUSE_CORE_AFFINITY == 1)
    #define CORE_0 (1 << 0)
    #define CORE_1 (1 << 1)
    /* Core 0: I/O (uart, pwm). Core 1: pesado (mpu I2C + fusion AHRS). */
    vTaskCoreAffinitySet(hMpu,    CORE_1);
    vTaskCoreAffinitySet(hFusion, CORE_1);
    if (hUart) vTaskCoreAffinitySet(hUart, CORE_0);
    vTaskCoreAffinitySet(hPwm,    CORE_0);
#endif

    vTaskStartScheduler();
    while (true);
}
