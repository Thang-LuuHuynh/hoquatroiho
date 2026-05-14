/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 + MPU6050 + Threshold Pitch/Roll/Yaw UART
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

#define MPU6050_ADDR                 (0x68 << 1)

#define MPU6050_SMPLRT_DIV           0x19
#define MPU6050_CONFIG               0x1A
#define MPU6050_GYRO_CONFIG          0x1B
#define MPU6050_ACCEL_CONFIG         0x1C
#define MPU6050_INT_PIN_CFG          0x37
#define MPU6050_INT_ENABLE           0x38
#define MPU6050_INT_STATUS           0x3A
#define MPU6050_ACCEL_XOUT_H         0x3B
#define MPU6050_PWR_MGMT_1           0x6B
#define MPU6050_WHO_AM_I             0x75

#define ACCEL_SCALE                  16384.0f
#define GYRO_SCALE                   131.0f
#define RAD_TO_DEG                   57.2957795f

/*
 * Chỉ gửi UART khi 1 trong 3 góc thay đổi vượt ngưỡng.
 * Pitch/Roll là giá trị chính.
 * Yaw là giá trị phụ cho GUI drone, có thể drift.
 */
#define ANGLE_SEND_THRESHOLD_DEG     0.50f
#define YAW_SEND_THRESHOLD_DEG       0.50f
#define FALLBACK_POLL_MS             5

#define CMD_NONE                     0
#define CMD_START                    1
#define CMD_STOP                     2
#define CMD_CAL                      3
#define CMD_PING                     4
#define CMD_READ                     5
#define CMD_DEBUG                    6
#define CMD_UNKNOWN                  7

/* USER CODE END PD */

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile uint8_t imu_data_ready = 0;
volatile uint32_t imu_irq_count = 0;

uint8_t rx_byte = 0;
char rx_line[32];
uint8_t rx_index = 0;

volatile uint8_t cmd_pending = CMD_NONE;

uint8_t streaming_enabled = 1;
uint8_t mpu_ok = 0;

float ax_offset = 0.0f;
float ay_offset = 0.0f;
float az_offset = 0.0f;

float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;

uint8_t filter_initialized = 0;

float last_sent_pitch = 0.0f;
float last_sent_roll = 0.0f;
float last_sent_yaw = 0.0f;

uint8_t first_send = 1;
uint8_t force_send = 0;

uint32_t last_update_ms = 0;
uint32_t last_no_int_msg_ms = 0;
uint32_t last_fallback_poll_ms = 0;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

void UART_Print(const char *s);
void Append_Float(char *buf, size_t buf_size, int *idx, float value, uint8_t decimals);

uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t data);
uint8_t MPU6050_Read_Reg(uint8_t reg, uint8_t *data);
uint8_t MPU6050_Init(void);

uint8_t MPU6050_Read_Raw(int16_t *ax_raw, int16_t *ay_raw, int16_t *az_raw,
                         int16_t *temp_raw,
                         int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw);

void MPU6050_Calibrate(uint16_t samples);
void MPU6050_Debug_Registers(void);

void IMU_Update_Filter_And_Send_If_Changed(void);
void IMU_Send_Frame(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float temp_c);

void Check_Button_PC15(void);
void Process_Command(void);
void Start_Stream_Clean(void);
float Abs_Float(float x);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

float Abs_Float(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}

void UART_Print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 100);
}

void Append_Float(char *buf, size_t buf_size, int *idx, float value, uint8_t decimals)
{
    int32_t scale = 1;

    for (uint8_t i = 0; i < decimals; i++)
    {
        scale *= 10;
    }

    int32_t scaled;

    if (value >= 0.0f)
    {
        scaled = (int32_t)(value * scale + 0.5f);
    }
    else
    {
        scaled = (int32_t)(value * scale - 0.5f);
    }

    int32_t abs_scaled = scaled;

    if (abs_scaled < 0)
    {
        abs_scaled = -abs_scaled;
    }

    int32_t whole = abs_scaled / scale;
    int32_t frac = abs_scaled % scale;

    if (scaled < 0)
    {
        *idx += snprintf(&buf[*idx], buf_size - *idx, "-%ld.", (long)whole);
    }
    else
    {
        *idx += snprintf(&buf[*idx], buf_size - *idx, "%ld.", (long)whole);
    }

    if (decimals == 1)
    {
        *idx += snprintf(&buf[*idx], buf_size - *idx, "%01ld", (long)frac);
    }
    else if (decimals == 2)
    {
        *idx += snprintf(&buf[*idx], buf_size - *idx, "%02ld", (long)frac);
    }
    else if (decimals == 3)
    {
        *idx += snprintf(&buf[*idx], buf_size - *idx, "%03ld", (long)frac);
    }
    else if (decimals == 4)
    {
        *idx += snprintf(&buf[*idx], buf_size - *idx, "%04ld", (long)frac);
    }
    else
    {
        *idx += snprintf(&buf[*idx], buf_size - *idx, "%ld", (long)frac);
    }
}

uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK)
    {
        return 1;
    }

    return 0;
}

uint8_t MPU6050_Read_Reg(uint8_t reg, uint8_t *data)
{
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg,
                         I2C_MEMADD_SIZE_8BIT, data, 1, 100) == HAL_OK)
    {
        return 1;
    }

    return 0;
}

uint8_t MPU6050_Init(void)
{
    uint8_t who = 0;
    char msg[100];

    HAL_Delay(100);

    if (!MPU6050_Read_Reg(MPU6050_WHO_AM_I, &who))
    {
        UART_Print("I2C ERROR: Cannot read WHO_AM_I\r\n");
        return 0;
    }

    snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X\r\n", who);
    UART_Print(msg);

    if (who != 0x68)
    {
        UART_Print("MPU6050 NOT FOUND\r\n");
        UART_Print("Check VCC, GND, PB6 SCL, PB7 SDA, AD0/address, pull-up\r\n");
        return 0;
    }

    /*
     * Wake up MPU6050.
     * 0x01: sleep = 0, clock = PLL with X gyro.
     */
    if (!MPU6050_Write_Reg(MPU6050_PWR_MGMT_1, 0x01)) return 0;
    HAL_Delay(100);

    /*
     * DLPF enabled => gyro output rate = 1 kHz.
     * Sample Rate = 1 kHz / (1 + SMPLRT_DIV).
     * 9 => 100 Hz DATA_READY.
     */
    if (!MPU6050_Write_Reg(MPU6050_SMPLRT_DIV, 9)) return 0;

    /*
     * DLPF_CFG = 3.
     */
    if (!MPU6050_Write_Reg(MPU6050_CONFIG, 0x03)) return 0;

    /*
     * Gyro +-250 deg/s.
     */
    if (!MPU6050_Write_Reg(MPU6050_GYRO_CONFIG, 0x00)) return 0;

    /*
     * Accel +-2g.
     */
    if (!MPU6050_Write_Reg(MPU6050_ACCEL_CONFIG, 0x00)) return 0;

    /*
     * INT_PIN_CFG = 0x00:
     * active high, push-pull, 50us pulse, clear by reading INT_STATUS.
     */
    if (!MPU6050_Write_Reg(MPU6050_INT_PIN_CFG, 0x00)) return 0;

    /*
     * Enable DATA_READY interrupt.
     */
    if (!MPU6050_Write_Reg(MPU6050_INT_ENABLE, 0x01)) return 0;

    uint8_t int_status = 0;
    MPU6050_Read_Reg(MPU6050_INT_STATUS, &int_status);

    UART_Print("MPU6050 INIT OK\r\n");

    return 1;
}

uint8_t MPU6050_Read_Raw(int16_t *ax_raw, int16_t *ay_raw, int16_t *az_raw,
                         int16_t *temp_raw,
                         int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw)
{
    uint8_t raw[14];

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, raw, 14, 100) != HAL_OK)
    {
        return 0;
    }

    *ax_raw   = (int16_t)((raw[0]  << 8) | raw[1]);
    *ay_raw   = (int16_t)((raw[2]  << 8) | raw[3]);
    *az_raw   = (int16_t)((raw[4]  << 8) | raw[5]);
    *temp_raw = (int16_t)((raw[6]  << 8) | raw[7]);
    *gx_raw   = (int16_t)((raw[8]  << 8) | raw[9]);
    *gy_raw   = (int16_t)((raw[10] << 8) | raw[11]);
    *gz_raw   = (int16_t)((raw[12] << 8) | raw[13]);

    return 1;
}

void MPU6050_Debug_Registers(void)
{
    uint8_t v = 0;
    char msg[100];

    UART_Print("\r\n----- MPU6050 REGISTER DEBUG -----\r\n");

    if (MPU6050_Read_Reg(MPU6050_INT_PIN_CFG, &v))
    {
        snprintf(msg, sizeof(msg), "DBG INT_PIN_CFG 0x37 = 0x%02X\r\n", v);
        UART_Print(msg);
    }

    if (MPU6050_Read_Reg(MPU6050_INT_ENABLE, &v))
    {
        snprintf(msg, sizeof(msg), "DBG INT_ENABLE  0x38 = 0x%02X\r\n", v);
        UART_Print(msg);
    }

    if (MPU6050_Read_Reg(MPU6050_INT_STATUS, &v))
    {
        snprintf(msg, sizeof(msg), "DBG INT_STATUS  0x3A = 0x%02X\r\n", v);
        UART_Print(msg);
    }

    if (MPU6050_Read_Reg(MPU6050_PWR_MGMT_1, &v))
    {
        snprintf(msg, sizeof(msg), "DBG PWR_MGMT_1  0x6B = 0x%02X\r\n", v);
        UART_Print(msg);
    }

    if (MPU6050_Read_Reg(MPU6050_SMPLRT_DIV, &v))
    {
        snprintf(msg, sizeof(msg), "DBG SMPLRT_DIV  0x19 = 0x%02X\r\n", v);
        UART_Print(msg);
    }

    if (MPU6050_Read_Reg(MPU6050_CONFIG, &v))
    {
        snprintf(msg, sizeof(msg), "DBG CONFIG      0x1A = 0x%02X\r\n", v);
        UART_Print(msg);
    }

    snprintf(msg, sizeof(msg),
             "DBG PB11 level = %d, irq_count=%lu\r\n",
             HAL_GPIO_ReadPin(MPU_INT_GPIO_Port, MPU_INT_Pin),
             imu_irq_count);
    UART_Print(msg);

    UART_Print("----------------------------------\r\n\r\n");
}

void MPU6050_Calibrate(uint16_t samples)
{
    int16_t ax_raw, ay_raw, az_raw;
    int16_t temp_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    float az_sum = 0.0f;

    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;

    uint16_t valid_samples = 0;

    UART_Print("CALIBRATING... Keep board flat and still\r\n");

    for (uint16_t i = 0; i < samples; i++)
    {
        if (MPU6050_Read_Raw(&ax_raw, &ay_raw, &az_raw,
                             &temp_raw,
                             &gx_raw, &gy_raw, &gz_raw))
        {
            ax_sum += ax_raw / ACCEL_SCALE;
            ay_sum += ay_raw / ACCEL_SCALE;
            az_sum += az_raw / ACCEL_SCALE;

            gx_sum += gx_raw / GYRO_SCALE;
            gy_sum += gy_raw / GYRO_SCALE;
            gz_sum += gz_raw / GYRO_SCALE;

            valid_samples++;
        }

        HAL_Delay(2);
    }

    if (valid_samples == 0)
    {
        UART_Print("CAL ERROR: no valid sample\r\n");
        return;
    }

    ax_offset = ax_sum / valid_samples;
    ay_offset = ay_sum / valid_samples;
    az_offset = (az_sum / valid_samples) - 1.0f;

    gx_offset = gx_sum / valid_samples;
    gy_offset = gy_sum / valid_samples;
    gz_offset = gz_sum / valid_samples;

    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    filter_initialized = 0;

    first_send = 1;
    force_send = 1;

    last_sent_pitch = 0.0f;
    last_sent_roll = 0.0f;
    last_sent_yaw = 0.0f;

    char msg[300];
    int idx = 0;

    idx += snprintf(&msg[idx], sizeof(msg) - idx, "CAL DONE: ax_off=");
    Append_Float(msg, sizeof(msg), &idx, ax_offset, 4);

    idx += snprintf(&msg[idx], sizeof(msg) - idx, " ay_off=");
    Append_Float(msg, sizeof(msg), &idx, ay_offset, 4);

    idx += snprintf(&msg[idx], sizeof(msg) - idx, " az_off=");
    Append_Float(msg, sizeof(msg), &idx, az_offset, 4);

    idx += snprintf(&msg[idx], sizeof(msg) - idx, " gx_off=");
    Append_Float(msg, sizeof(msg), &idx, gx_offset, 4);

    idx += snprintf(&msg[idx], sizeof(msg) - idx, " gy_off=");
    Append_Float(msg, sizeof(msg), &idx, gy_offset, 4);

    idx += snprintf(&msg[idx], sizeof(msg) - idx, " gz_off=");
    Append_Float(msg, sizeof(msg), &idx, gz_offset, 4);

    idx += snprintf(&msg[idx], sizeof(msg) - idx, "\r\n");

    UART_Print(msg);
}

void IMU_Send_Frame(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float temp_c)
{
    char buf[300];
    int idx = 0;

    idx += snprintf(&buf[idx], sizeof(buf) - idx, "$IMU,%lu,", HAL_GetTick());

    Append_Float(buf, sizeof(buf), &idx, ax, 3);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, ay, 3);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, az, 3);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, gx, 3);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, gy, 3);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, gz, 3);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, pitch, 2);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, roll, 2);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, yaw, 2);
    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",");

    Append_Float(buf, sizeof(buf), &idx, temp_c, 2);

    idx += snprintf(&buf[idx], sizeof(buf) - idx, ",%lu,OK\r\n", imu_irq_count);

    UART_Print(buf);
}

void IMU_Update_Filter_And_Send_If_Changed(void)
{
    int16_t ax_raw, ay_raw, az_raw;
    int16_t temp_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    if (!MPU6050_Read_Raw(&ax_raw, &ay_raw, &az_raw,
                          &temp_raw,
                          &gx_raw, &gy_raw, &gz_raw))
    {
        UART_Print("I2C READ ERROR\r\n");
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = (now - last_update_ms) / 1000.0f;
    last_update_ms = now;

    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }

    float ax = ax_raw / ACCEL_SCALE - ax_offset;
    float ay = ay_raw / ACCEL_SCALE - ay_offset;
    float az = az_raw / ACCEL_SCALE - az_offset;

    float gx = gx_raw / GYRO_SCALE - gx_offset;
    float gy = gy_raw / GYRO_SCALE - gy_offset;
    float gz = gz_raw / GYRO_SCALE - gz_offset;

    float temp_c = temp_raw / 340.0f + 36.53f;

    float roll_acc  = atan2f(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    const float alpha = 0.98f;

    if (!filter_initialized)
    {
        roll = roll_acc;
        pitch = pitch_acc;
        yaw = 0.0f;
        filter_initialized = 1;
    }
    else
    {
        /*
         * Pitch/Roll là giá trị chính.
         * Dùng complementary filter: gyro ngắn hạn + accel dài hạn.
         */
        roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * roll_acc;
        pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitch_acc;

        /*
         * Yaw là giá trị phụ cho GUI drone.
         * MPU6050 không có magnetometer nên yaw sẽ bị drift theo thời gian.
         */
        yaw += gz * dt;

        if (yaw > 180.0f)
        {
            yaw -= 360.0f;
        }
        else if (yaw < -180.0f)
        {
            yaw += 360.0f;
        }
    }

    float dp = Abs_Float(pitch - last_sent_pitch);
    float dr = Abs_Float(roll - last_sent_roll);
    float dy = Abs_Float(yaw - last_sent_yaw);

    if (dy > 180.0f)
    {
        dy = 360.0f - dy;
    }

    uint8_t should_send = 0;

    /*
     * Điều kiện gửi UART:
     * 1. Frame đầu tiên sau START/CAL.
     * 2. Lệnh READ ép gửi.
     * 3. Pitch thay đổi >= ANGLE_SEND_THRESHOLD_DEG.
     * 4. Roll thay đổi >= ANGLE_SEND_THRESHOLD_DEG.
     * 5. Yaw thay đổi >= YAW_SEND_THRESHOLD_DEG.
     */
    if (first_send)
    {
        should_send = 1;
        first_send = 0;
    }

    if (force_send)
    {
        should_send = 1;
        force_send = 0;
    }

    if (dp >= ANGLE_SEND_THRESHOLD_DEG)
    {
        should_send = 1;
    }

    if (dr >= ANGLE_SEND_THRESHOLD_DEG)
    {
        should_send = 1;
    }

    if (dy >= YAW_SEND_THRESHOLD_DEG)
    {
        should_send = 1;
    }

    if (!should_send)
    {
        return;
    }

    last_sent_pitch = pitch;
    last_sent_roll = roll;
    last_sent_yaw = yaw;

    IMU_Send_Frame(ax, ay, az, gx, gy, gz, temp_c);
}

void Start_Stream_Clean(void)
{
    uint8_t dummy = 0;

    streaming_enabled = 0;

    __disable_irq();
    imu_data_ready = 0;
    imu_irq_count = 0;
    __enable_irq();

    __HAL_GPIO_EXTI_CLEAR_IT(MPU_INT_Pin);
    MPU6050_Read_Reg(MPU6050_INT_STATUS, &dummy);

    last_update_ms = HAL_GetTick();
    last_no_int_msg_ms = HAL_GetTick();
    last_fallback_poll_ms = HAL_GetTick();

    first_send = 1;
    force_send = 1;

    streaming_enabled = 1;
}

void Check_Button_PC15(void)
{
    if (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(30);

        if (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET)
        {
            cmd_pending = CMD_CAL;

            while (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET)
            {
                HAL_Delay(10);
            }
        }
    }
}

void Process_Command(void)
{
    uint8_t cmd = cmd_pending;

    if (cmd == CMD_NONE)
    {
        return;
    }

    cmd_pending = CMD_NONE;

    if (cmd == CMD_START)
    {
        Start_Stream_Clean();
        UART_Print("CMD START OK\r\n");
    }
    else if (cmd == CMD_STOP)
    {
        streaming_enabled = 0;
        UART_Print("CMD STOP OK\r\n");
    }
    else if (cmd == CMD_CAL)
    {
        streaming_enabled = 0;
        UART_Print("CMD CAL OK\r\n");
        MPU6050_Calibrate(500);
        Start_Stream_Clean();
    }
    else if (cmd == CMD_PING)
    {
        UART_Print("PONG\r\n");
    }
    else if (cmd == CMD_READ)
    {
        force_send = 1;
        UART_Print("CMD READ OK\r\n");
    }
    else if (cmd == CMD_DEBUG)
    {
        UART_Print("CMD DEBUG OK\r\n");
        MPU6050_Debug_Registers();
    }
    else if (cmd == CMD_UNKNOWN)
    {
        UART_Print("UNKNOWN CMD\r\n");
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin)
    {
        imu_data_ready = 1;
        imu_irq_count++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_byte == '\n' || rx_byte == '\r')
        {
            if (rx_index > 0)
            {
                rx_line[rx_index] = '\0';

                if (strcmp(rx_line, "START") == 0)
                {
                    cmd_pending = CMD_START;
                }
                else if (strcmp(rx_line, "STOP") == 0)
                {
                    cmd_pending = CMD_STOP;
                }
                else if (strcmp(rx_line, "CAL") == 0)
                {
                    cmd_pending = CMD_CAL;
                }
                else if (strcmp(rx_line, "PING") == 0)
                {
                    cmd_pending = CMD_PING;
                }
                else if (strcmp(rx_line, "READ") == 0)
                {
                    cmd_pending = CMD_READ;
                }
                else if (strcmp(rx_line, "DEBUG") == 0)
                {
                    cmd_pending = CMD_DEBUG;
                }
                else
                {
                    cmd_pending = CMD_UNKNOWN;
                }

                rx_index = 0;
            }
        }
        else
        {
            if (rx_index < sizeof(rx_line) - 1)
            {
                rx_line[rx_index++] = rx_byte;
            }
            else
            {
                rx_index = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */

    UART_Print("\r\n============================\r\n");
    UART_Print("STM32F103 + MPU6050 THRESHOLD MODE\r\n");
    UART_Print("UART: 115200 8N1\r\n");
    UART_Print("Commands: START, STOP, CAL, PING, READ, DEBUG\r\n");
    UART_Print("Frame: $IMU,time,ax,ay,az,gx,gy,gz,pitch,roll,yaw,temp,irq,OK\r\n");
    UART_Print("Send: only when pitch/roll/yaw exceed threshold\r\n");
    UART_Print("Pitch/Roll: main. Yaw: demo only, may drift.\r\n");
    UART_Print("============================\r\n");

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    mpu_ok = MPU6050_Init();

    if (mpu_ok)
    {
        MPU6050_Debug_Registers();

        UART_Print("Place board flat. Auto calibrate in 1s...\r\n");
        HAL_Delay(1000);

        MPU6050_Calibrate(500);

        Start_Stream_Clean();

        UART_Print("SYSTEM READY\r\n");
    }
    else
    {
        UART_Print("SYSTEM STOPPED: MPU6050 INIT FAILED\r\n");
        streaming_enabled = 0;
    }

    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN WHILE */

        Check_Button_PC15();
        Process_Command();

        if (mpu_ok && streaming_enabled)
        {
            uint8_t int_status = 0;
            uint32_t now = HAL_GetTick();

            if (imu_data_ready)
            {
                imu_data_ready = 0;

                /*
                 * Clear DATA_RDY_INT status for this event.
                 */
                MPU6050_Read_Reg(MPU6050_INT_STATUS, &int_status);

                IMU_Update_Filter_And_Send_If_Changed();
            }
            else if ((now - last_fallback_poll_ms) >= FALLBACK_POLL_MS)
            {
                last_fallback_poll_ms = now;

                /*
                 * Fallback: if EXTI missed 50us pulse,
                 * poll DATA_RDY_INT bit.
                 */
                if (MPU6050_Read_Reg(MPU6050_INT_STATUS, &int_status) &&
                    (int_status & 0x01))
                {
                    IMU_Update_Filter_And_Send_If_Changed();
                }
                else
                {
                    if ((now - last_update_ms > 1000) &&
                        (now - last_no_int_msg_ms > 2000))
                    {
                        last_no_int_msg_ms = now;
                        UART_Print("WARN: No MPU6050 DATA_READY\r\n");
                    }
                }
            }
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * PC15 button:
     * Pull-up means not pressed = 1, pressed = 0.
     */
    GPIO_InitStruct.Pin = USER_BTN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

    /*
     * PB11 MPU6050 INT:
     * MPU6050 INT is active-high pulse.
     * STM32 catches rising edge.
     */
    GPIO_InitStruct.Pin = MPU_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
