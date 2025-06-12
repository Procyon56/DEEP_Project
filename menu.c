/* ──────────────────────────────────────────────────────────────
 *  Menu principal — sélection de mini-jeux par inclinaison
 *  ------------------------------------------------------------
 *  • Gauche  : « Tap-Taupe »
 *  • Droite  : « Flappy Bird »
 *  • Validation : bouton PA11 (niveau bas)
 *  • Inclinaison mesurée par le MPU-6050 (pitch)
 * ──────────────────────────────────────────────────────────── */
#include "menu.h"

#include "stm32g4_gpio.h"
#include "outils.h"     /* Board, MPU, Kalman, couleurs, reset */
#include "tape_taupe.h"          /* Game_TapTaupe() */
#include "flappy_bird.h"        /* Game_FlappyBird() */

#include "tft_ili9341/stm32g4_ili9341.h"
#include <math.h>               /* atan2f, sqrtf */
#include <stdio.h>

/* ── Zones de surlignage ───────────────────────────────────── */
#define BOX_W       140
#define BOX_H       60
#define BOX_Y       80
#define BOX_X_LEFT   20
#define BOX_X_RIGHT 160

/* ── Seuils d’inclinaison (pitch) pour changer de sélection ── */
#define PITCH_LEFT_LIMIT   -20.0f
#define PITCH_RIGHT_LIMIT   20.0f

/* Prototypes internes */
static void draw_menu_boxes(int sel);

/* ───────────────────────────────────────────────────────────── */
void Menu_ShowAndRun(void)
{
    /* ----- Écran & bouton ----- */
    ILI9341_Init();
    ILI9341_Fill(ILI9341_COLOR_BLACK);
    ILI9341_Rotate(ILI9341_Orientation_Landscape_2);

    BSP_GPIO_pin_config(GPIOA, GPIO_PIN_11, GPIO_MODE_INPUT,
                        GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, GPIO_NO_AF);

    /* ----- Texte statique ----- */
    ILI9341_Puts(BOX_X_LEFT  + 14, BOX_Y + 22, "TAPE-TAUPE",
                 &Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ILI9341_Puts(BOX_X_RIGHT + 24, BOX_Y + 22, "FLAPPY",
                 &Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

    /* ----- Capteur ----- */
    if (!MPU_Init_Wrapper()) {
        printf("MPU init error\r\n");
        while (1);
    }
    Kalman_t kalPitch = {0};

    /* ----- Boucle menu ----- */
    int sel     = 0;      /* 0 = gauche, 1 = droite */
    int oldSel  = -1;     /* force premier dessin    */

    while (1)
    {
        /* ---------- Lecture MPU → choix ---------- */
        float ax, ay, az;
        if (MPU_ReadAccel(&ax, &ay, &az)) {
            /* Pitch à partir de l’accel (°) */
            float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;
            float pitch     = Kalman_GetAngle(&kalPitch, pitch_acc, 0.0f, 0.02f);

            if      (pitch < PITCH_LEFT_LIMIT)  sel = 0;
            else if (pitch > PITCH_RIGHT_LIMIT) sel = 1;
        }

        /* ---------- Mise à jour visuelle ---------- */
        if (sel != oldSel) {
            draw_menu_boxes(sel);
            oldSel = sel;
        }

        /* ---------- Validation bouton ---------- */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
            HAL_Delay(150);          /* anti-rebond */
            if (sel == 0)
                Game_TapeTaupe();
            else
                Game_FlappyBird();

            /* Quand le jeu se termine, on ré-affiche le menu. */
            ILI9341_Fill(ILI9341_COLOR_BLACK);
            ILI9341_Puts(BOX_X_LEFT  + 14, BOX_Y + 22, "TAP-TAUPE",
                         &Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
            ILI9341_Puts(BOX_X_RIGHT + 24, BOX_Y + 22, "FLAPPY",
                         &Font_11x18, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
            oldSel = -1;             /* force redraw des cadres */
        }

        Platform_ResetButtonCheck();
        HAL_Delay(20);
    }
}

/* ── Dessine ou efface les cadres de sélection ─────────────── */
static void draw_menu_boxes(int sel)
{
    uint16_t colL = (sel == 0) ? ILI9341_COLOR_YELLOW : ILI9341_COLOR_WHITE;
    uint16_t colR = (sel == 1) ? ILI9341_COLOR_YELLOW : ILI9341_COLOR_WHITE;

    ILI9341_DrawRectangle(BOX_X_LEFT , BOX_Y,
                          BOX_X_LEFT  + BOX_W, BOX_Y + BOX_H, colL);
    ILI9341_DrawRectangle(BOX_X_RIGHT, BOX_Y,
                          BOX_X_RIGHT + BOX_W, BOX_Y + BOX_H, colR);
}
