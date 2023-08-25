
/*********************************************************************************************/
/* STM32H7 SPI bare-metal driver                                                             */
/* Copyright (c) 2022-2023, Luigi Calligaris                                                 */
/* All rights reserved.                                                                      */
/*                                                                                           */
/* This software is distributed under the BSD (3-clause) license, which is reproduced below: */
/* ----------------------------------------------------------------------------------------- */
/*                                                                                           */
/* Redistribution and use in source and binary forms, with or without modification,          */
/* are permitted provided that the following conditions are met:                             */
/*                                                                                           */
/* * Redistributions of source code must retain the above copyright notice, this             */
/*   list of conditions and the following disclaimer.                                        */
/*                                                                                           */
/* * Redistributions in binary form must reproduce the above copyright notice, this          */
/*   list of conditions and the following disclaimer in the documentation and/or             */
/*   other materials provided with the distribution.                                         */
/*                                                                                           */
/* * Neither the name of SPRACE nor the one of UNESP nor the names of its                    */
/*   contributors may be used to endorse or promote products derived from                    */
/*   this software without specific prior written permission.                                */
/*                                                                                           */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND           */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED             */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                    */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR          */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES            */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;              */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON            */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                   */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS             */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                              */
/*                                                                                           */
/*********************************************************************************************/

#ifndef INC_H7SPI_CONFIG_H_
#define INC_H7SPI_CONFIG_H_

#include "main.h"


// Declare the following macros in main.h file to put the peripheral under the responsibility of this driver.
// You shall mind about collisions with the STM32Cube driver (if you use this driver,
// the peripheral should be unconfigured in the IOC file).


#ifndef H7SPI_PERIPH_ENABLE_SPI1
#define H7SPI_PERIPH_ENABLE_SPI1 0
#endif

#ifndef H7SPI_PERIPH_ENABLE_SPI2
#define H7SPI_PERIPH_ENABLE_SPI2 0
#endif

#ifndef H7SPI_PERIPH_ENABLE_SPI3
#define H7SPI_PERIPH_ENABLE_SPI3 0
#endif


#ifndef H7SPI_PERIPH_ENABLE_SPI4
#define H7SPI_PERIPH_ENABLE_SPI4 0
#endif


#ifndef H7SPI_PERIPH_ENABLE_SPI5
#define H7SPI_PERIPH_ENABLE_SPI5 0
#endif

#ifndef H7SPI_PERIPH_ENABLE_SPI6
#define H7SPI_PERIPH_ENABLE_SPI6 0
#endif


// Do you want to use the FreeRTOS-compatible function implementations?
#define H7SPI_USE_FREERTOS_IMPL 0

// Specify the IRQ priorities for the various SPI devices
#define H7I2C_IRQ_SPI1_PRI 7
#define H7I2C_IRQ_SPI2_PRI 7
#define H7I2C_IRQ_SPI3_PRI 7
#define H7I2C_IRQ_SPI4_PRI 7
#define H7I2C_IRQ_SPI5_PRI 7
#define H7I2C_IRQ_SPI6_PRI 7

#define H7I2C_IRQ_SPI1_SUBPRI 0
#define H7I2C_IRQ_SPI2_SUBPRI 1
#define H7I2C_IRQ_SPI3_SUBPRI 2
#define H7I2C_IRQ_SPI4_SUBPRI 3
#define H7I2C_IRQ_SPI5_SUBPRI 4
#define H7I2C_IRQ_SPI6_SUBPRI 5

// Do not edit this logic if you don't understand it
#if H7SPI_PERIPH_ENABLE_SPI1 == 1 || H7SPI_PERIPH_ENABLE_SPI2 == 1 || H7SPI_PERIPH_ENABLE_SPI3 == 1 || H7SPI_PERIPH_ENABLE_SPI4 == 1 || H7SPI_PERIPH_ENABLE_SPI5 == 1 || H7SPI_PERIPH_ENABLE_SPI6 == 1
#define H7SPI_PERIPH_ENABLE_ANY 1
#else
#define H7SPI_PERIPH_ENABLE_ANY 0
#endif


#endif // INC_H7SPI_CONFIG_H_
