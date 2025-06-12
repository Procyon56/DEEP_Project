/* =======================================================================
 *  platform_utils.c  —  Implémentation unique
 * ======================================================================= */

#include "outils.h"

#include "config.h"                          /* LED_GREEN_GPIO…            */
#include "stm32g4_sys.h"
#include "stm32g4_gpio.h"
#include "stm32g4_uart.h"
#include "stm32g4_utils.h"                   /* BSP_…                      */
#include "stm32g4_systick.h"

#include "MPU6050/stm32g4_mpu6050.h"         /* driver bas niveau          */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define HOLE_R      40

/*=======================================================================
 * SECTION 1 — BoardInit & Reset-button
 *=====================================================================*/
void Platform_BoardInit(void)
{
    HAL_Init();                             /* mandatory first call       */
    BSP_GPIO_enable();
    BSP_UART_init(UART2_ID, 115200);
    BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

    /* LED verte Nucleo pour heartbeat éventuel */
    BSP_GPIO_pin_config(LED_GREEN_GPIO, LED_GREEN_PIN,
                        GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                        GPIO_SPEED_FREQ_HIGH, GPIO_NO_AF);
}

void Platform_ResetButtonCheck(void)
{
    static uint32_t t0 = 0;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
        if (t0 == 0) t0 = HAL_GetTick();        /* début chrono          */
        if (HAL_GetTick() - t0 >= 3000)         /* >3 s → reset CPU      */
            NVIC_SystemReset();
    } else
        t0 = 0;
}

/*=======================================================================
 * SECTION 2 — Kalman
 *=====================================================================*/
void Kalman_Init(Kalman_t *k,
                 float q_angle, float q_bias, float r_measure)
{
    *k = (Kalman_t){0};
    k->P[0][0] = k->P[1][1] = 1.0f;
    /* si besoin stocker q_angle/q_bias/r_measure dans la struct */
}

float Kalman_GetAngle(Kalman_t *k,
                      float newAngle, float newRate, float dt)
{
    /* ---- prédiction ---- */
    k->rate  = newRate - k->bias;
    k->angle += dt * k->rate;

    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + 0.001f);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += 0.003f * dt;

    /* ---- correction ---- */
    float S  = k->P[0][0] + 0.03f;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;
    float y  = newAngle - k->angle;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00 = k->P[0][0], P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}

/*=======================================================================
 * SECTION 3 — MPU-6050 Wrapper
 *=====================================================================*/
static MPU6050_t mpuData;             /* persistant et privé */

bool MPU_Init_Wrapper(void)
{
    return MPU6050_Init(&mpuData, GPIOA, GPIO_PIN_0, MPU6050_Device_0,
                        MPU6050_Accelerometer_2G,
                        MPU6050_Gyroscope_250s) == MPU6050_Result_Ok;
}

bool MPU_ReadAccel(float *ax, float *ay, float *az)
{
    if (MPU6050_ReadAccelerometer(&mpuData) != MPU6050_Result_Ok)
        return false;
    *ax = mpuData.Accelerometer_X / 16384.0f;
    *ay = mpuData.Accelerometer_Y / 16384.0f;
    *az = mpuData.Accelerometer_Z / 16384.0f;
    return true;
}

bool MPU_ReadGyro(float *gx, float *gy, float *gz)
{
    if (MPU6050_ReadGyroscope(&mpuData) != MPU6050_Result_Ok)
        return false;
    *gx = mpuData.Gyroscope_X / 131.0f;
    *gy = mpuData.Gyroscope_Y / 131.0f;
    *gz = mpuData.Gyroscope_Z / 131.0f;
    return true;
}

bool MPU_ReadAll(float *ax, float *ay, float *az,
                 float *gx, float *gy, float *gz)
{
    if (MPU6050_ReadAll(&mpuData) != MPU6050_Result_Ok)
        return false;

    *ax = mpuData.Accelerometer_X / 16384.0f;
    *ay = mpuData.Accelerometer_Y / 16384.0f;
    *az = mpuData.Accelerometer_Z / 16384.0f;

    *gx = mpuData.Gyroscope_X / 131.0f;
    *gy = mpuData.Gyroscope_Y / 131.0f;
    *gz = mpuData.Gyroscope_Z / 131.0f;
    return true;
}

/*=======================================================================
 * SECTION 4 — Helpers graphiques (ILI9341)
 *=====================================================================*/
void GFX_DrawCursor(int x, int y, uint16_t col)
{
    ILI9341_DrawPixel(x, y, col);
    ILI9341_DrawPixel(x-1, y, col);  ILI9341_DrawPixel(x+1, y, col);
    ILI9341_DrawPixel(x, y-1, col);  ILI9341_DrawPixel(x, y+1, col);
}



/* -------- explosion étoile (couche rouge + jaune + centre) ---*/
void GFX_DrawExplosion(int cx, int cy, int r)
{
    /* extérieur rouge – 18 branches */
    for (int i = 0; i < 18; ++i) {
        int len  = r + ((i & 1) ? 8 : 0);
        float a  = i * (2.0f * M_PI / 18.0f);
        int x2   = cx + (int)(len * cosf(a));
        int y2   = cy + (int)(len * sinf(a));
        ILI9341_DrawLine(cx, cy, x2, y2, ILI9341_COLOR_RED);
    }
    /* intérieur jaune – 14 branches */
    for (int i = 0; i < 14; ++i) {
        int len  = r - 4 + ((i & 1) ? 6 : 0);
        float a  = i * (2.0f * M_PI / 14.0f);
        int x2   = cx + (int)(len * cosf(a));
        int y2   = cy + (int)(len * sinf(a));
        ILI9341_DrawLine(cx, cy, x2, y2, ILI9341_COLOR_YELLOW);
    }
    ILI9341_DrawFilledCircle(cx, cy, 6, ILI9341_COLOR_WHITE);
}

void GFX_AnimatePop(int cx, int cy, int r)
{
    GFX_DrawExplosion(cx, cy, r);
    HAL_Delay(120);
    ILI9341_DrawFilledCircle(cx, cy, r + 10, ILI9341_COLOR_WHITE);
}

/* -------- dessin/suppression taupe ---------------------------*/
void GFX_DrawTaupe(int cx, int cy, int draw)
{
    uint16_t body_col = draw ? COL_TAUPE_BODY : ILI9341_COLOR_WHITE;
    ILI9341_DrawFilledCircle(cx, cy, HOLE_R - 2, body_col);

    if (draw) {
        ILI9341_DrawFilledCircle(cx - 8, cy - 6, 4, COL_TAUPE_EYE);  /* œil G */
        ILI9341_DrawFilledCircle(cx + 8, cy - 6, 4, COL_TAUPE_EYE);  /* œil D */
        ILI9341_DrawFilledCircle(cx     , cy + 2, 3, COL_TAUPE_NOSE);/* nez   */
    }
}
