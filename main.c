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

#include "math.h"

#include "tft_ili9341/stm32g4_ili9341.h"

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

void test_imu_kalman(void) {
    MPU6050_t mpu;
    float dt = 0.01f;  // 10 ms
    uint32_t t_prev = HAL_GetTick();

    ILI9341_Init();
    ILI9341_Fill(ILI9341_COLOR_WHITE);

    // Initialisation du MPU6050
    if (MPU6050_Init(&mpu, GPIOA, GPIO_PIN_0, MPU6050_Device_0,
                     MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) != MPU6050_Result_Ok) {
        printf("Erreur d'initialisation du MPU6050\n");
        while (1);
    }

    while (1) {
        // Lire toutes les données du capteur (accéléromètre, gyroscope et température)
        if (MPU6050_ReadAll(&mpu) == MPU6050_Result_Ok) {

            // Récupérer les valeurs des capteurs
            float accX = mpu.Accelerometer_X / 16384.0f;
            float accY = mpu.Accelerometer_Y / 16384.0f;
            float accZ = mpu.Accelerometer_Z / 16384.0f;

            float gyroX = mpu.Gyroscope_X / 131.0f; // deg/s
            float gyroY = mpu.Gyroscope_Y / 131.0f;

            // Calcul des angles d'inclinaison à partir des données de l'accéléromètre
            float roll_acc = atan2f(accY, accZ) * 180.0f / 3.141592653589793;
            float pitch_acc = atan2f(-accX, sqrtf(accY*accY + accZ*accZ)) * 180.0f / 3.141592653589793;

            // Calcul de dt basé sur le temps écoulé
            uint32_t t_now = HAL_GetTick();
            dt = (t_now - t_prev) / 1000.0f;  // Convertir en secondes
            t_prev = t_now;

            // Appliquer le filtre de Kalman pour obtenir les angles filtrés
            float roll = kalman_get_angle(&kalmanX, roll_acc, gyroX, dt);
            float pitch = kalman_get_angle(&kalmanY, pitch_acc, gyroY, dt);

            // Afficher les résultats
            ILI9341_printf(100, 100, &Font_7x10, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE, "x = %.2f", roll_acc);
        } else {
            printf("Erreur de lecture du MPU6050\n");
        }

        HAL_Delay(10);  // Délai de 10 ms
    }
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


	dvd_bounce();




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

