#include "outils.h"
#include "menu.h"        /* ou vos jeux directement */

int main(void)
{
    Platform_BoardInit();     /* remplace l’ancien Board_Init() */
    Menu_ShowAndRun();        /* ou Game_TapTaupe(), etc.       */
    while (1) { __WFI(); }
}
