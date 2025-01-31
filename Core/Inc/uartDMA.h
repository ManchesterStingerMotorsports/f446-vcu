/*
 * uartDMA.h
 *
 *  Created on: Jan 25, 2025
 *      Author: amrlxyz
 */

#ifndef INC_UARTDMA_H_
#define INC_UARTDMA_H_

#define BUFF_SIZE 128


typedef struct
{
    char str[128];
    int len;
} printfString_t;



int printfDma(const char *format, ...);


#endif /* INC_UARTDMA_H_ */
