/*
 *  Flappy Bird (version MPU-6050)
 *   Pitch pour monter / descendre
 *   Tuyau unique qui défile de droite à gauche
 *   Score + accélération progressive
 *  */
#include "flappy_bird.h"

#include "outils.h"

#include "tft_ili9341/stm32g4_ili9341.h"
#include "stm32g4_gpio.h"
#include "stm32g4_sys.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

/*  Paramètres généraux  */
#define SCREEN_W       320
#define SCREEN_H       240

/* Oiseau */
#define BIRD_X         160
#define BIRD_R         8
#define PITCH_SCALE    2.0f     /* °  pixels */

/* Tuyau (barre) */
#define PIPE_W         5
#define PIPE_GAP       30       /* ouverture centrale */
#define PIPE_MIN_Y     0
#define PIPE_MAX_Y     (SCREEN_H - PIPE_GAP -  29)  /* 211 */

/* Vitesse */
#define PIPE_DX_START  3
#define PIPE_DX_INC    1        /* +1 px lorsque score++ */

/* Heads-up display */
#define HUD_COLOR_FG   ILI9341_COLOR_YELLOW
#define HUD_COLOR_BG   ILI9341_COLOR_BLUE

/*  Prototypes internes  */
static void display_init(void);
static void hud_draw_score(int score);
static int  pipes_new_y(void);


void Game_FlappyBird(void)
{
    /* ----- Initialisation écran + RNG ----- */
    display_init();
    srand(HAL_GetTick());

    /* ----- Capteur MPU ----- */
    if (!MPU_Init_Wrapper()) {
        printf("MPU init error\r\n");
        while (1);
    }
    Kalman_t kalPitch = {0};

    /* ----- Oiseau ----- */
    int birdY = SCREEN_H / 2;
    int oldBirdY = birdY;

    /* ----- Tuyau ----- */
    int pipeX  = 0;
    int pipeX1 = PIPE_W;
    int pipeY2 = pipes_new_y();          /* bord inférieur 1ère partie */
    int pipeY3 = pipeY2 + PIPE_GAP;      /* bord supérieur 2 partie   */
    int pipeDx = PIPE_DX_START;

    /* ----- Score & HUD ----- */
    int  score  = 0;
    bool scored = false;
    hud_draw_score(score);

    uint32_t tPrev = HAL_GetTick();

    /* ======================== BOUCLE JEU ======================== */
    while (1)
    {
        /* ---------- Remise à zéro (bouton 5 s) ---------- */
    	Platform_ResetButtonCheck();

        /* ---------- Oiseau : lecture MPU ---------- */
        float ax, ay, az, gx, gy, gz;
        if (MPU_ReadAll(&ax, &ay, &az, &gx, &gy, &gz))
        {
            float dt = (HAL_GetTick() - tPrev) / 1000.0f;
            tPrev    = HAL_GetTick();

            float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;
            float pitch = Kalman_GetAngle(&kalPitch, pitch_acc, gy, dt);

            oldBirdY = birdY;
            birdY = (SCREEN_H / 2) + (int)(pitch * PITCH_SCALE);

            /* bornes écran */
            if (birdY < BIRD_R)             birdY = BIRD_R;
            if (birdY > SCREEN_H - BIRD_R)  birdY = SCREEN_H - BIRD_R;
        }

        /* ---------- Effacement de la frame précédente ---------- */
        /* Efface tuyau */
        if (pipeX1 > PIPE_W)
            ILI9341_DrawFilledRectangle(pipeX, 0,
                                        pipeX1, SCREEN_H,
                                        HUD_COLOR_BG /* bleu fond */);
        /* Efface oiseau */
        ILI9341_DrawFilledCircle(BIRD_X, oldBirdY, BIRD_R, HUD_COLOR_BG);

        /* ---------- Mise à jour tuyau ---------- */
        pipeX  += pipeDx;
        pipeX1 += pipeDx;

        if (pipeX >= SCREEN_W)
        {

        	pipeX  = 0;
        }
        if (pipeX1 >= SCREEN_W)
        {
            pipeX1   = 0;
            pipeY2   = pipes_new_y();
            pipeY3   = pipeY2 + PIPE_GAP;
            scored   = false;          /* prêt pour le prochain point */
        }

        /* Dessine tuyau courant */
        if (pipeX1 > PIPE_W) {
            ILI9341_DrawFilledRectangle(pipeX, 0,
                                        pipeX1, pipeY2,
                                        ILI9341_COLOR_GREEN);
            ILI9341_DrawFilledRectangle(pipeX, pipeY3,
                                        pipeX1, SCREEN_H,
                                        ILI9341_COLOR_GREEN);

        }

        /* ---------- Dessine oiseau ---------- */
        ILI9341_DrawFilledCircle(BIRD_X, birdY, BIRD_R, HUD_COLOR_FG);

        /* ---------- Score lorsque l oiseau passe le tuyau ---------- */
        if (!scored && (pipeX > BIRD_X + BIRD_R))
        {
        	hud_draw_score(score);
            scored = true;
            score++;
            pipeDx += PIPE_DX_INC;             /* jeu plus rapide */



        }
        if(pipeX1 < 80 && pipeX1 > 70) hud_draw_score(score);

        /* ---------- Collision ---------- */
        bool hit_pipe  = (pipeX1 > BIRD_X - BIRD_R) &&
                         (pipeX  < BIRD_X + BIRD_R) &&
                         (birdY - BIRD_R < pipeY2 || birdY + BIRD_R > pipeY3);

        if (hit_pipe)
        {
            ILI9341_Puts(80, 110, "GAME  OVER",
                         &Font_11x18,
                         ILI9341_COLOR_WHITE, ILI9341_COLOR_RED);
            HAL_Delay(1000);
            return;           /* le menu reprend la main */
        }

        HAL_Delay(20);
    }
}

/*  Helpers  */
static void display_init(void)
{
    ILI9341_Init();
    ILI9341_Fill(HUD_COLOR_BG);
    ILI9341_Rotate(ILI9341_Orientation_Landscape_2);

    /* HUD fond bleu : rien dautre nécessaire */
}

static void hud_draw_score(int score)
{
    char txt[16];
    sprintf(txt, "Score:%2d", score);
    ILI9341_Puts(5, 5, txt, &Font_7x10,
                 HUD_COLOR_FG, HUD_COLOR_BG);
}

/* Position verticale aléatoire du tuyau */
static int pipes_new_y(void)
{
    return (rand() % (PIPE_MAX_Y - PIPE_MIN_Y + 1)) + PIPE_MIN_Y;
}
