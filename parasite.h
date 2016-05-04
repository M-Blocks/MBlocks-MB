/** \file parasite.h
 *  \brief Code to interface with parasite ESP8266 board.
 */

#ifndef PARASITE_H_
#define PARASITE_H_

#define PARASITE_ADDRESS              0x62

#define PARASITE_INIT_AI_REGISTER     0x11
#define PARASITE_LED_REGISTER         0x05

/** \fn bool parasite_turnon(void)
    \brief Turns on parasite board. Used on sleep over.
*/
bool parasite_turnon();

/** \fn bool parasite_turnoff(void)
    \brief Turns off parasite board. Used when going to sleep.
*/
bool parasite_turnoff();

/** \fn bool parasite_reset(void)
    \brief Reset parasite board.
*/
bool parasite_reset();

#endif
