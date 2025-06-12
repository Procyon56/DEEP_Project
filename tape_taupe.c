/*
 *  Jeu Tap-Taupe
 *   Curseur commandé par linclinaison du MPU-6050
 *   Gâchette : bouton PA11 (niveau bas actif)
 *   Objectif : toucher 10 taupes le plus vite possible
 */
#include "tape_taupe.h"

#include "outils.h"

#include "tft_ili9341/stm32g4_ili9341.h"
#include "stm32g4_gpio.h"
#include "stm32g4_sys.h"

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

/*  Constantes du jeu */
#define HOLE_N      4
static const int holesX[HOLE_N] = {  80, 240,  80, 240};
static const int holesY[HOLE_N] = {  60,  60, 180, 180};
#define HOLE_R      40

#define CURSOR_SCALE    2.0f
#define MAX_SCORE       10
#define TAUPE_PERIOD_MS 1500

/*  Helpers internes */
static void  display_init(void);
static void  hud_draw_score(int score);
static void  hud_draw_time(uint32_t t0);


void Game_TapeTaupe(void)
{

    display_init();
    srand(HAL_GetTick());


    if (!MPU_Init_Wrapper()) {
        printf("MPU init error\r\n");
        while (1);
    }
    Kalman_t kalRoll = {0}, kalPitch = {0};


    uint32_t t_start = HAL_GetTick();
    uint32_t taupe_t = t_start;

    int  taupe_idx = rand() % HOLE_N;
    GFX_DrawTaupe(holesX[taupe_idx], holesY[taupe_idx], 1);

    int  score     = 0;


    const int cx = 160, cy = 120;
    int  curX = cx, curY = cy, oldX = cx, oldY = cy;
    float yaw_sum = 0.0f;

    uint32_t t_prev = HAL_GetTick();


    while (score < MAX_SCORE)
    {
        /*Lecture capteur*/
        float ax, ay, az, gx, gy, gz;
        if (MPU_ReadAll(&ax, &ay, &az, &gx, &gy, &gz))
        {
            uint32_t t_now = HAL_GetTick();
            float dt = (t_now - t_prev) / 1000.0f;
            t_prev   = t_now;

            /* Angles absolus via Kalman */
            float roll_acc  = atan2f( ay,  az) * 57.2958f;
            float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;

            float roll  = Kalman_GetAngle(&kalRoll , roll_acc , gx, dt);
            float pitch = Kalman_GetAngle(&kalPitch, pitch_acc, gy, dt);
            yaw_sum    += gz * dt;


            curX = cx - (int)(yaw_sum * CURSOR_SCALE);
            curY = cy + (int)(roll    * CURSOR_SCALE);

            /* limites écran 320×240 */
            if (curX < 0)   curX = 0;   if (curX > 319) curX = 319;
            if (curY < 0)   curY = 0;   if (curY > 239) curY = 239;


            GFX_DrawCursor(oldX, oldY, ILI9341_COLOR_WHITE);
            GFX_DrawCursor(curX, curY, ILI9341_COLOR_RED);
            oldX = curX; oldY = curY;

            hud_draw_score(score);
            hud_draw_time(t_start);


            if ((t_now - taupe_t) > TAUPE_PERIOD_MS) {
                GFX_DrawTaupe(holesX[taupe_idx], holesY[taupe_idx], 0);
                taupe_idx = rand() % HOLE_N;
                GFX_DrawTaupe(holesX[taupe_idx], holesY[taupe_idx], 1);
                taupe_t = t_now;
            }

            /*Détection bouton*/
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET)
            {
                int dx = curX - holesX[taupe_idx];
                int dy = curY - holesY[taupe_idx];
                if (dx*dx + dy*dy <= HOLE_R*HOLE_R) {
                    score++;
                    GFX_AnimatePop(holesX[taupe_idx], holesY[taupe_idx], HOLE_R);


                    for (int i = 0; i < HOLE_N; ++i)
                        ILI9341_DrawCircle(holesX[i], holesY[i],
                                           HOLE_R, ILI9341_COLOR_BLACK);


                    GFX_DrawTaupe(holesX[taupe_idx], holesY[taupe_idx], 0);
                    taupe_idx = rand() % HOLE_N;
                    GFX_DrawTaupe(holesX[taupe_idx], holesY[taupe_idx], 1);
                    taupe_t   = HAL_GetTick();
                }
                HAL_Delay(200);
            }
        }

        /*Reset*/
        Platform_ResetButtonCheck();
        HAL_Delay(10);
    }

    /*FIN DE PARTIE*/
    ILI9341_Fill(ILI9341_COLOR_WHITE);
    char buf[64];
    sprintf(buf, "Bravo ! Score:%d  Temps:%lds",
            score, (HAL_GetTick() - t_start) / 1000);
    ILI9341_Puts(40, 110, buf, &Font_11x18,
                 ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
    HAL_Delay(1500);
}

/*  Helpers  */
static void display_init(void)
{
    ILI9341_Init();
    ILI9341_Fill(ILI9341_COLOR_WHITE);
    ILI9341_Rotate(ILI9341_Orientation_Landscape_2);

    for (int i = 0; i < HOLE_N; ++i)
        ILI9341_DrawCircle(holesX[i], holesY[i],
                           HOLE_R, ILI9341_COLOR_BLACK);


    BSP_GPIO_pin_config(GPIOA, GPIO_PIN_11, GPIO_MODE_INPUT,
                        GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, GPIO_NO_AF);
}

static void hud_draw_score(int score)
{
    char txt[16];
    sprintf(txt, "Score:%2d", score);
    ILI9341_Puts(5, 5, txt, &Font_7x10,
                 ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
}

static void hud_draw_time(uint32_t t0)
{
    char txt[16];
    sprintf(txt, "Time:%lds", (HAL_GetTick() - t0) / 1000);
    ILI9341_Puts(250, 5, txt, &Font_7x10,
                 ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
}
