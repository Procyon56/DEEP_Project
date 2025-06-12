/* =======================================================================
 *  platform_utils.h  « boîte à outils » commune
 *  Regroupe :
 *     • Filtre de Kalman
 *     • Wrapper MPU-6050 (accel + gyro, valeurs physiques)
 *     • Couleurs ILI9341 fréquentes
 *     • Helpers graphiques de base
 *     • Gestion bouton reset (PA11)
 *     • Initialisation carte (HAL, GPIO, UART…)
 *  Tous les jeux/application incluent UNIQUEMENT ce header.
 * ======================================================================= */
#ifndef OUTILS_H
#define OUTILS_H

#include <stdbool.h>
#include <stdint.h>
#include "tft_ili9341/stm32g4_ili9341.h"   /* prototype des fonctions LCD */

/* -----------------------------------------------------------------------
 * 1) Board / reset
 * -------------------------------------------------------------------- */
void   Platform_BoardInit(void);
void   Platform_ResetButtonCheck(void);          /* reset si PA11 tenu > 3 s */

/* -----------------------------------------------------------------------
 * 2) Couleurs fréquemment utilisées
 * -------------------------------------------------------------------- */
#define COL_BACKGROUND   ILI9341_COLOR_BLUE
#define COL_FG_TEXT      ILI9341_COLOR_YELLOW
#define COL_TAUPE_BODY   ILI9341_COLOR_BROWN
#define COL_TAUPE_EYE    ILI9341_COLOR_BLACK
#define COL_TAUPE_NOSE   ILI9341_COLOR_MAGENTA

/* -----------------------------------------------------------------------
 * 3) Kalman
 * -------------------------------------------------------------------- */
typedef struct {
    float angle;
    float bias;
    float rate;
    float P[2][2];
} Kalman_t;

void  Kalman_Init(Kalman_t *k,
                  float q_angle, float q_bias, float r_measure);
float Kalman_GetAngle(Kalman_t *k,
                      float newAngle, float newRate, float dt);

/* -----------------------------------------------------------------------
 * 4) MPU-6050  (échelles : accel ±1 g, gyro ±250 °/s)
 * -------------------------------------------------------------------- */
bool  MPU_Init_Wrapper(void);
bool  MPU_ReadAccel(float *ax, float *ay, float *az);              /* g     */
bool  MPU_ReadGyro (float *gx, float *gy, float *gz);              /* °/s   */
bool  MPU_ReadAll  (float *ax, float *ay, float *az,
                    float *gx, float *gy, float *gz);

/* -----------------------------------------------------------------------
 * 5) Helpers graphiques génériques
 * -------------------------------------------------------------------- */
void  GFX_DrawCursor(int x, int y, uint16_t col);
void  GFX_DrawExplosion(int cx, int cy, int r);     /* étoile rouge/jaune  */
void  GFX_AnimatePop(int cx, int cy, int r);        /* explosion + efface  */
void  GFX_DrawTaupe(int cx, int cy, int draw);      /* corps + yeux + nez  */

#endif /* PLATFORM_UTILS_H */
