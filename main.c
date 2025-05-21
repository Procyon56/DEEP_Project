/**
 *******************************************************************************
 * @file 	main.c
 * @author 	jjo
 * @date 	Mar 29, 2024
 * @brief	Fichier principal de votre projet sur carte NuclÃ©o STM32G431KB
 *******************************************************************************
 */

#include "config.h"
#include "stm32g4_sys.h"
#include <math.h>  // Nécessaire pour cos() et sin()
#include "tft_ili9341/stm32g4_ili9341.h"
#include <stdlib.h>   /* rand() */

#include "stm32g4_systick.h"
#include "stm32g4_gpio.h"
#include "stm32g4_uart.h"
#include "stm32g4_utils.h"
#include "MPU6050/stm32g4_mpu6050.h"

#include <stdio.h>

#define BLINK_DELAY		100	//ms



void write_LED(bool b)
{
	HAL_GPIO_WritePin(LED_GREEN_GPIO, LED_GREEN_PIN, b);
}

bool char_received(uart_id_t uart_id)
{
	if( BSP_UART_data_ready(uart_id) )	/* Si un caractÃ¨re est reÃ§u sur l'UART 2*/
	{
		/* On "utilise" le caractÃ¨re pour vider le buffer de rÃ©ception */
		BSP_UART_get_next_byte(uart_id);
		return true;
	}
	else
		return false;
}

void test_accelerometer(void) {
			MPU6050_t MPU6050_Data;

			if (MPU6050_Init(&MPU6050_Data, GPIOA, GPIO_PIN_0, MPU6050_Device_0,
						MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) != MPU6050_Result_Ok) {
				printf("Erreur d'initialisation du MPU6050\n");
				while (1);
			}

			while (1) {
				// Lis les données de l'accéléromètre
				if (MPU6050_ReadAccelerometer(&MPU6050_Data) == MPU6050_Result_Ok){

						//Affiche les valeurs de laccéléromètre
						printf("Accélération X: %d, Y: %d, Z: %d\n",
								MPU6050_Data.Accelerometer_X,
								MPU6050_Data.Accelerometer_Y,
								MPU6050_Data.Accelerometer_Z);
				} else {
					printf("Erreur de lecture de l'accélérometre\n");
				}
				HAL_Delay(1000);

			}
		}

void test_gyroscope(void) {
    MPU6050_t MPU6050_Data;

    if (MPU6050_Init(&MPU6050_Data, GPIOA, GPIO_PIN_0, MPU6050_Device_0,
                     MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) != MPU6050_Result_Ok) {
        printf("Erreur d'initialisation du MPU6050\n");
        while (1);
    }

    while (1) {
        if (MPU6050_ReadGyroscope(&MPU6050_Data) == MPU6050_Result_Ok) {
            printf("Gyro X: %d, Y: %d, Z: %d\n",
                   MPU6050_Data.Gyroscope_X,
                   MPU6050_Data.Gyroscope_Y,
                   MPU6050_Data.Gyroscope_Z);
        } else {
            printf("Erreur de lecture du gyroscope\n");
        }

        HAL_Delay(1000);
    }
}


void heartbeat(void)
{
	while(! char_received(UART2_ID) )
	{
		write_LED(true);
		HAL_Delay(50);
		write_LED(false);
		HAL_Delay(1500);
	}
}






void dvd_bounce(void) {
	    ILI9341_Init();
	    ILI9341_Fill(ILI9341_COLOR_WHITE);

	    int x = 100, y = 100;
	    int r = 40;
	    int r1 = 30;
	    int r2 = 20;
	    int r3 = 10;

	    int dx = 3, dy = 2;

	    while (1) {
	        // Efface l'ancienne position
	        ILI9341_DrawCircle(x, y, r, ILI9341_COLOR_WHITE);
	        ILI9341_DrawCircle(x, y, r1, ILI9341_COLOR_WHITE);
	        ILI9341_DrawCircle(x, y, r2, ILI9341_COLOR_WHITE);
	        ILI9341_DrawCircle(x, y, r3, ILI9341_COLOR_WHITE);

	        // Met à jour la position
	        x += dx;
	        y += dy;

	        // Rebonds sur les bords
	        if ((x - r <= 0) || (x + r >= 320)) dx = -dx;
	        if ((y - r <= 0) || (y + r >= 240)) dy = -dy;

	        // Dessine à la nouvelle position
	        ILI9341_DrawCircle(x, y, r, ILI9341_COLOR_BLACK);
	        ILI9341_DrawCircle(x, y, r1, ILI9341_COLOR_BLUE);
	        ILI9341_DrawCircle(x, y, r2, ILI9341_COLOR_RED);
	        ILI9341_DrawCircle(x, y, r3, ILI9341_COLOR_YELLOW);


	    }
	}

typedef struct {
    float angle;  // l'angle estimé
    float bias;   // biais du gyroscope
    float rate;   // vitesse angulaire non biaisée
    float P[2][2];
} Kalman_t;

Kalman_t kalmanX = {0}, kalmanY = {0};

float kalman_get_angle(Kalman_t* k, float newAngle, float newRate, float dt) {
    // Prédiction
    k->rate = newRate - k->bias;
    k->angle += dt * k->rate;

    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + 0.001f);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += 0.003f * dt;

    // Correction
    float S = k->P[0][0] + 0.03f;
    float K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    float y = newAngle - k->angle;
    k->angle += K[0] * y;
    k->bias += K[1] * y;

    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    return k->angle;
}

float yaw = 0;

void test_imu_kalman(void) {
	ILI9341_Init();
	ILI9341_Fill(ILI9341_COLOR_WHITE);
	ILI9341_Rotate(ILI9341_Orientation_Landscape_2);
	BSP_GPIO_pin_config(GPIOA, GPIO_PIN_11, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, GPIO_NO_AF);

	MPU6050_t mpu;
	float dt = 0.01f;
	uint32_t t_prev = HAL_GetTick();
	uint32_t taupe_timer = HAL_GetTick();

	Kalman_t kalmanX = {0}, kalmanY = {0};

	int centerX = 160;
	int centerY = 120;
	static int oldX = 160, oldY = 120;

	int pointX = 160, pointY = 120;

	// Définir les 4 trous fixes
	int trousX[4] = {80, 240, 80, 240};
	int trousY[4] = {60, 60, 180, 180};
	int taupe_index = rand() % 4;
	int radius = 15;

	// Affiche les trous
	for (int i = 0; i < 4; i++) {
		ILI9341_DrawCircle(trousX[i], trousY[i], radius, ILI9341_COLOR_BLACK);
	}
	// Affiche taupe active
	ILI9341_DrawFilledCircle(trousX[taupe_index], trousY[taupe_index], radius - 2, ILI9341_COLOR_BROWN);

	if (MPU6050_Init(&mpu, GPIOA, GPIO_PIN_0, MPU6050_Device_0,
	                 MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) != MPU6050_Result_Ok) {
		printf("Erreur init MPU6050\n");
		while (1);
	}

	while (1) {
		if (MPU6050_ReadAll(&mpu) == MPU6050_Result_Ok) {
			float accX = mpu.Accelerometer_X / 16384.0f;
			float accY = mpu.Accelerometer_Y / 16384.0f;
			float accZ = mpu.Accelerometer_Z / 16384.0f;

			float gyroX = mpu.Gyroscope_X / 131.0f;
			float gyroY = mpu.Gyroscope_Y / 131.0f;
			float gyroZ = mpu.Gyroscope_Z / 131.0f;

			uint32_t t_now = HAL_GetTick();
			dt = (t_now - t_prev) / 1000.0f;
			t_prev = t_now;

			float roll_acc = atan2f(accY, accZ) * 180.0f / M_PI;
			float pitch_acc = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 180.0f / M_PI;

			float roll = kalman_get_angle(&kalmanX, roll_acc, gyroX, dt);
			float pitch = kalman_get_angle(&kalmanY, pitch_acc, gyroY, dt);

			static float yaw_total = 0;
			yaw_total += gyroZ * dt;


			float scale = 4.0f;
			pointX = centerX + (int)(yaw_total * scale);
			pointY = centerY + (int)(roll * scale);

			if (pointX < 0) pointX = 0;
			if (pointX > 319) pointX = 319;
			if (pointY < 0) pointY = 0;
			if (pointY > 239) pointY = 239;

			// Efface ancienne croix
			ILI9341_DrawPixel(oldX, oldY, ILI9341_COLOR_WHITE);
			ILI9341_DrawPixel(oldX - 1, oldY, ILI9341_COLOR_WHITE);
			ILI9341_DrawPixel(oldX + 1, oldY, ILI9341_COLOR_WHITE);
			ILI9341_DrawPixel(oldX, oldY - 1, ILI9341_COLOR_WHITE);
			ILI9341_DrawPixel(oldX, oldY + 1, ILI9341_COLOR_WHITE);

			// Dessine nouvelle croix
			ILI9341_DrawPixel(pointX, pointY, ILI9341_COLOR_RED);
			ILI9341_DrawPixel(pointX - 1, pointY, ILI9341_COLOR_RED);
			ILI9341_DrawPixel(pointX + 1, pointY, ILI9341_COLOR_RED);
			ILI9341_DrawPixel(pointX, pointY - 1, ILI9341_COLOR_RED);
			ILI9341_DrawPixel(pointX, pointY + 1, ILI9341_COLOR_RED);

			oldX = pointX;
			oldY = pointY;

			// Changer taupe toutes les 1.5s
			if (HAL_GetTick() - taupe_timer > 1500) {
				ILI9341_DrawFilledCircle(trousX[taupe_index], trousY[taupe_index], radius - 2, ILI9341_COLOR_WHITE);
				taupe_index = rand() % 4;
				ILI9341_DrawFilledCircle(trousX[taupe_index], trousY[taupe_index], radius - 2, ILI9341_COLOR_BROWN);
				taupe_timer = HAL_GetTick();
			}

			// Tir
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
				int dx = pointX - trousX[taupe_index];
				int dy = pointY - trousY[taupe_index];
				if (dx * dx + dy * dy <= (radius * radius)) {
					printf("TAP !\n");
					ILI9341_DrawFilledCircle(trousX[taupe_index], trousY[taupe_index], radius - 2, ILI9341_COLOR_WHITE);
					taupe_index = rand() % 4;
					ILI9341_DrawFilledCircle(trousX[taupe_index], trousY[taupe_index], radius - 2, ILI9341_COLOR_BROWN);
					taupe_timer = HAL_GetTick();
				}
				HAL_Delay(200); // anti-rebond
			}

		} else {
			printf("Erreur lecture MPU6050\n");
		}

		HAL_Delay(10);
	}
}


//* ── Explosion étoile rouge / jaune ────────────────────────── */
    static void draw_explosion(int cx, int cy, int r)
    {
        /* couche extérieure rouge – 18 branches */
        for (int i = 0; i < 18; ++i) {
            int len  = r + ((i & 1) ? 8 : 0);          /* pointe longue / courte */
            float a  = i * (2.0f * M_PI / 18.0f);
            int x2   = cx + (int)(len * cosf(a));
            int y2   = cy + (int)(len * sinf(a));
            ILI9341_DrawLine(cx, cy, x2, y2, ILI9341_COLOR_RED);
        }
        /* couche intérieure jaune – 14 branches */
        for (int i = 0; i < 14; ++i) {
            int len  = r - 4 + ((i & 1) ? 6 : 0);
            float a  = i * (2.0f * M_PI / 14.0f);
            int x2   = cx + (int)(len * cosf(a));
            int y2   = cy + (int)(len * sinf(a));
            ILI9341_DrawLine(cx, cy, x2, y2, ILI9341_COLOR_YELLOW);
        }
        /* centre blanc */
        ILI9341_DrawFilledCircle(cx, cy, 6, ILI9341_COLOR_WHITE);
    }

    /* ── Animation : explosion puis effacement ─────────────────── */
    static void animate_pop(int cx, int cy, int r)
    {
        draw_explosion(cx, cy, r);
        HAL_Delay(120);                                   /* durée visible */
        /* efface zone (disque blanc un poil > explosion) */
        ILI9341_DrawFilledCircle(cx, cy, r + 10, ILI9341_COLOR_WHITE);

    }

void jeu_tap_taupe(void) {
    ILI9341_Init();
    ILI9341_Fill(ILI9341_COLOR_WHITE);
    ILI9341_Rotate(ILI9341_Orientation_Landscape_2);
    BSP_GPIO_pin_config(GPIOA, GPIO_PIN_11, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, GPIO_NO_AF);

    MPU6050_t mpu;
    Kalman_t kalmanX = {0}, kalmanY = {0};
    float dt = 0.01f;
    uint32_t t_prev = HAL_GetTick();
    uint32_t start_time = HAL_GetTick();
    uint32_t taupe_timer = HAL_GetTick();

    int centerX = 160, centerY = 120;
    int oldX = 160, oldY = 120;
    int pointX = 160, pointY = 120;
    float yaw_total = 0;
    float scale = 2.0f;

    int trousX[4] = {80, 240, 80, 240};
    int trousY[4] = {60, 60, 180, 180};
    int taupe_idx = rand() % 4;
    const int R = 40;

    int score = 0;

    if (MPU6050_Init(&mpu, GPIOA, GPIO_PIN_0, MPU6050_Device_0,
                     MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) != MPU6050_Result_Ok) {
        printf("Erreur init MPU6050\n");
        while (1);
    }

    // Affiche les trous
    for (int i = 0; i < 4; i++) {
        ILI9341_DrawCircle(trousX[i], trousY[i], R, ILI9341_COLOR_BLACK);
    }

    // ------- Taupe avec museau --------
    void draw_taupe(int cx, int cy, int draw) {
        uint16_t body_col = draw ? ILI9341_COLOR_BROWN : ILI9341_COLOR_WHITE;
        ILI9341_DrawFilledCircle(cx, cy, R - 2, body_col);
        if (draw) {
            ILI9341_DrawFilledCircle(cx - 8, cy - 6, 4, ILI9341_COLOR_BLACK);  // œil gauche
            ILI9341_DrawFilledCircle(cx + 8, cy - 6, 4, ILI9341_COLOR_BLACK);  // œil droit
            ILI9341_DrawFilledCircle(cx, cy + 2, 3, ILI9341_COLOR_MAGENTA);    // museau
        }
    }




    draw_taupe(trousX[taupe_idx], trousY[taupe_idx], 1);

    while (score < 10) {
        if (MPU6050_ReadAll(&mpu) == MPU6050_Result_Ok) {
            float accX = mpu.Accelerometer_X / 16384.0f;
            float accY = mpu.Accelerometer_Y / 16384.0f;
            float accZ = mpu.Accelerometer_Z / 16384.0f;

            float gyroX = mpu.Gyroscope_X / 131.0f;
            float gyroY = mpu.Gyroscope_Y / 131.0f;
            float gyroZ = mpu.Gyroscope_Z / 131.0f;

            uint32_t t_now = HAL_GetTick();
            dt = (t_now - t_prev) / 1000.0f;
            t_prev = t_now;

            float roll_acc = atan2f(accY, accZ) * 180.0f / M_PI;
            float pitch_acc = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 180.0f / M_PI;

            float roll = kalman_get_angle(&kalmanX, roll_acc, gyroX, dt);
            float pitch = kalman_get_angle(&kalmanY, pitch_acc, gyroY, dt);
            yaw_total += gyroZ * dt;

            pointX = centerX - (int)(yaw_total * scale);
            pointY = centerY + (int)(roll * scale);

            if (pointX < 0) pointX = 0;
            if (pointX > 319) pointX = 319;
            if (pointY < 0) pointY = 0;
            if (pointY > 239) pointY = 239;

            ILI9341_DrawPixel(oldX, oldY, ILI9341_COLOR_WHITE);
            ILI9341_DrawPixel(oldX - 1, oldY, ILI9341_COLOR_WHITE);
            ILI9341_DrawPixel(oldX + 1, oldY, ILI9341_COLOR_WHITE);
            ILI9341_DrawPixel(oldX, oldY - 1, ILI9341_COLOR_WHITE);
            ILI9341_DrawPixel(oldX, oldY + 1, ILI9341_COLOR_WHITE);

            ILI9341_DrawPixel(pointX, pointY, ILI9341_COLOR_RED);
            ILI9341_DrawPixel(pointX - 1, pointY, ILI9341_COLOR_RED);
            ILI9341_DrawPixel(pointX + 1, pointY, ILI9341_COLOR_RED);
            ILI9341_DrawPixel(pointX, pointY - 1, ILI9341_COLOR_RED);
            ILI9341_DrawPixel(pointX, pointY + 1, ILI9341_COLOR_RED);
            oldX = pointX;
            oldY = pointY;

            char txt[32];
            sprintf(txt, "Score: %d", score);
            ILI9341_Puts(5, 5, txt, &Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
            int elapsed = (HAL_GetTick() - start_time) / 1000;
            sprintf(txt, "Time: %ds", elapsed);
            ILI9341_Puts(250, 5, txt, &Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);

            if ((t_now - taupe_timer) > 1500) {
                draw_taupe(trousX[taupe_idx], trousY[taupe_idx], 0);
                taupe_idx = rand() % 4;
                draw_taupe(trousX[taupe_idx], trousY[taupe_idx], 1);
                taupe_timer = t_now;
            }

            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) {
                int dx = pointX - trousX[taupe_idx];
                int dy = pointY - trousY[taupe_idx];
                if (dx * dx + dy * dy <= R * R) {
                    score++;
                    animate_pop(trousX[taupe_idx], trousY[taupe_idx], R);
                    for (int i = 0; i < 4; i++) {
                            ILI9341_DrawCircle(trousX[i], trousY[i], R, ILI9341_COLOR_BLACK);
                        }

                    draw_taupe(trousX[taupe_idx], trousY[taupe_idx], 0);
                    taupe_idx = rand() % 4;
                    draw_taupe(trousX[taupe_idx], trousY[taupe_idx], 1);
                    taupe_timer = HAL_GetTick();
                }
                HAL_Delay(200);
            }
        }
        HAL_Delay(10);
    }

    ILI9341_Fill(ILI9341_COLOR_WHITE);
    int final_time = (HAL_GetTick() - start_time) / 1000;
    char endtxt[64];
    sprintf(endtxt, "Bravo ! Score: %d  Temps: %ds", score, final_time);
    ILI9341_Puts(40, 110, endtxt, &Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
}














/**
  * @brief  Point d'entrÃ©e de votre application
  */
int main(void)
{
	/* Cette ligne doit rester la premiÃ¨re de votre main !
	 * Elle permet d'initialiser toutes les couches basses des drivers (Hardware Abstraction Layer),
	 * condition prÃ©alable indispensable Ã  l'exÃ©cution des lignes suivantes.
	 */
	HAL_Init();

	/* Initialisation des pÃ©riphÃ©riques utilisÃ©s dans votre programme */
	BSP_GPIO_enable();
	BSP_UART_init(UART2_ID,115200);

	/* Indique que les printf sont dirigÃ©s vers l'UART2 */
	BSP_SYS_set_std_usart(UART2_ID, UART2_ID, UART2_ID);

	/* Initialisation du port de la led Verte (carte Nucleo) */
	BSP_GPIO_pin_config(LED_GREEN_GPIO, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_HIGH,GPIO_NO_AF);

	/* Hello student */
	printf("Hi <Student>, can you read me?\n");


	jeu_tap_taupe();




	/* TÃ¢che de fond, boucle infinie, Infinite loop,... quelque soit son nom vous n'en sortirez jamais */
	while (1)
	{

		if( char_received(UART2_ID) )
		{
			write_LED(true);		/* write_LED? Faites un ctrl+clic dessus pour voir... */
			HAL_Delay(BLINK_DELAY);	/* ... Ã§a fonctionne aussi avec les macros, les variables. C'est votre nouveau meilleur ami */
			write_LED(false);
		}

	}


}
