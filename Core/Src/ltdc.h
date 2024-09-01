/*
 * ltdc.h
 *
 *  Created on: Apr 26, 2024
 *      Author: aledn
 */

#ifndef SRC_LTDC_H_
#define SRC_LTDC_H_

#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdlib.h>


#endif /* SRC_LTDC_H_ */
void FillScreenDayOrNight(uint32_t color);
void DrawLine(uint32_t color);
void DrawGameOver(uint32_t color);
void DrawMainCharacter(uint32_t color, uint32_t y);
void DrawObstacle(uint32_t color, uint32_t x);
void DrawLegsClear(uint32_t color);
void DrawLegsMainCharacter(uint32_t color, int flag);
