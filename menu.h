#ifndef MENU_H
#define MENU_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Affiche le menu principal et lance le jeu choisi.
 *
 * Boucle en permanence : ne revient jamais si tout se passe bien
 * (un jeu se termine → le menu est relancé).
 */
void Menu_ShowAndRun(void);

#ifdef __cplusplus
}


#endif

#endif /* MENU_H */
