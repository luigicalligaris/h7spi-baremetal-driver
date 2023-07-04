
/*********************************************************************************************/
/* STM32H7 I2C bare-metal driver                                                             */
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

#ifndef INC_H7SPI_BARE_H_
#define INC_H7SPI_BARE_H_

#include <stdint.h>

#include "h7spi_config.h"


typedef enum
{
  H7SPI_SPI1,
  H7SPI_SPI2,
  H7SPI_SPI3,
  H7SPI_SPI4,
  H7SPI_SPI5,
  H7SPI_SPI6
} h7spi_periph_t;

typedef enum
{
  H7SPI_FSM_STATE_UNINITIALIZED,
  H7SPI_FSM_STATE_IDLE,
  H7SPI_FSM_STATE_SETUP_TRANSFER,
  H7SPI_FSM_STATE_SHIFTING,
  H7SPI_FSM_STATE_DEINITIALIZING,

  H7SPI_FSM_STATE_ERROR_UDR,
  H7SPI_FSM_STATE_ERROR_OVR,
  H7SPI_FSM_STATE_ERROR_CRCE,
  H7SPI_FSM_STATE_ERROR_TIFRE,
  H7SPI_FSM_STATE_ERROR_MODF,

  H7SPI_FSM_STATE_UNMANAGED_BY_DRIVER
} h7spi_spi_fsm_state_t;


typedef enum
{
  H7SPI_RET_CODE_OK,
  H7SPI_RET_CODE_BUSY,
  H7SPI_RET_CODE_TIMEOUT,
  H7SPI_RET_CODE_PERIPH_IN_ERR_STATE,
  H7SPI_RET_CODE_INVALID_ARGS,
  H7SPI_RET_CODE_ERROR,
  H7SPI_RET_CODE_CLEARED_A_NONERROR_STATE,
  H7SPI_RET_CODE_UNMANAGED_BY_DRIVER
} h7spi_spi_ret_code_t;


typedef enum
{
  H7SPI_PIN_SPI1_SCK_PA5,
  H7SPI_PIN_SPI1_SCK_PB3,
  H7SPI_PIN_SPI1_SCK_PG11,

  H7SPI_PIN_SPI2_SCK_PA9,
  H7SPI_PIN_SPI2_SCK_PA12,
  H7SPI_PIN_SPI2_SCK_PB10,
  H7SPI_PIN_SPI2_SCK_PB13,
  H7SPI_PIN_SPI2_SCK_PD3,
  H7SPI_PIN_SPI2_SCK_PI1,

  H7SPI_PIN_SPI3_SCK_PB3,
  H7SPI_PIN_SPI3_SCK_PC10,

  H7SPI_PIN_SPI4_SCK_PE2,
  H7SPI_PIN_SPI4_SCK_PE12,

  H7SPI_PIN_SPI5_SCK_PF7,
  H7SPI_PIN_SPI5_SCK_PH6,
  H7SPI_PIN_SPI5_SCK_PK0,

  H7SPI_PIN_SPI6_SCK_PA5,
  H7SPI_PIN_SPI6_SCK_PB3,
  H7SPI_PIN_SPI6_SCK_PG13

} h7spi_pin_sck_t;

typedef enum
{
  H7SPI_PIN_SPI1_NSS_PA4,
  H7SPI_PIN_SPI1_NSS_PA15,
  H7SPI_PIN_SPI1_NSS_PG10,

  H7SPI_PIN_SPI2_NSS_PA11,
  H7SPI_PIN_SPI2_NSS_PB4,
  H7SPI_PIN_SPI2_NSS_PB9,
  H7SPI_PIN_SPI2_NSS_PB12,
  H7SPI_PIN_SPI2_NSS_PI0,

  H7SPI_PIN_SPI3_NSS_PA4,
  H7SPI_PIN_SPI3_NSS_PA15,

  H7SPI_PIN_SPI4_NSS_PE4,
  H7SPI_PIN_SPI4_NSS_PE11,

  H7SPI_PIN_SPI5_NSS_PF6,
  H7SPI_PIN_SPI5_NSS_PH5,
  H7SPI_PIN_SPI5_NSS_PK1,

  H7SPI_PIN_SPI6_NSS_PA4,
  H7SPI_PIN_SPI6_NSS_PA15,
  H7SPI_PIN_SPI6_NSS_PG8

} h7spi_pin_nss_t;

typedef enum
{
  H7SPI_PIN_SPI1_MISO_PA6,
  H7SPI_PIN_SPI1_MISO_PB4,
  H7SPI_PIN_SPI1_MISO_PG9,

  H7SPI_PIN_SPI2_MISO_PB14,
  H7SPI_PIN_SPI2_MISO_PC2,
  H7SPI_PIN_SPI2_MISO_PI2,

  H7SPI_PIN_SPI3_MISO_PB4,
  H7SPI_PIN_SPI3_MISO_PC11,

  H7SPI_PIN_SPI4_MISO_PE5,
  H7SPI_PIN_SPI4_MISO_PE13,

  H7SPI_PIN_SPI5_MISO_PF8,
  H7SPI_PIN_SPI5_MISO_PH7,
  H7SPI_PIN_SPI5_MISO_PJ11,

  H7SPI_PIN_SPI6_MISO_PA6,
  H7SPI_PIN_SPI6_MISO_PB4,
  H7SPI_PIN_SPI6_MISO_PG12

} h7spi_pin_miso_t;

typedef enum
{
  H7SPI_PIN_SPI1_MOSI_PA7,
  H7SPI_PIN_SPI1_MOSI_PB5,
  H7SPI_PIN_SPI1_MOSI_PD7,

  H7SPI_PIN_SPI2_MOSI_PB15,
  H7SPI_PIN_SPI2_MOSI_PC1,
  H7SPI_PIN_SPI2_MOSI_PC3,
  H7SPI_PIN_SPI2_MOSI_PI3,

  H7SPI_PIN_SPI3_MOSI_PB2,
  H7SPI_PIN_SPI3_MOSI_PB5,
  H7SPI_PIN_SPI3_MOSI_PC12,
  H7SPI_PIN_SPI3_MOSI_PD6,

  H7SPI_PIN_SPI4_MOSI_PE6,
  H7SPI_PIN_SPI4_MOSI_PE14,

  H7SPI_PIN_SPI5_MOSI_PF9,
  H7SPI_PIN_SPI5_MOSI_PF11,
  H7SPI_PIN_SPI5_MOSI_PJ10,

  H7SPI_PIN_SPI6_MOSI_PA7,
  H7SPI_PIN_SPI6_MOSI_PB5,
  H7SPI_PIN_SPI6_MOSI_PG14

} h7spi_pin_mosi_t;


typedef struct h7spi_periph_init_config_t
{
  uint8_t          irq_pri;
  uint8_t          irq_subpri;
  h7spi_pin_sck_t  pin_sck;
  h7spi_pin_nss_t  pin_nss;
  h7spi_pin_miso_t pin_miso;
  h7spi_pin_mosi_t pin_mosi;
  uint32_t         rcc_clksource;
  uint32_t         cfg1;
  uint32_t         cfg2;
  uint32_t         crcpoly;
  uint32_t         cr1cfg;
} h7spi_periph_init_config_t;



h7spi_spi_ret_code_t h7spi_spi_init(h7spi_periph_t peripheral);
h7spi_spi_ret_code_t h7spi_spi_init_by_config(h7spi_periph_t peripheral, h7spi_periph_init_config_t* init_config);
void h7spi_deinit(h7spi_periph_t peripheral);

h7spi_spi_ret_code_t h7spi_spi_reset_peripheral_full(h7spi_periph_t peripheral);
h7spi_spi_ret_code_t h7spi_spi_reset_peripheral_soft(h7spi_periph_t peripheral);
h7spi_spi_ret_code_t h7spi_spi_reset_driver(h7spi_periph_t peripheral);

int h7spi_spi_is_managed_by_this_driver(h7spi_periph_t peripheral);

int h7spi_is_ready(h7spi_periph_t peripheral);
h7spi_spi_ret_code_t h7spi_wait_until_ready(h7spi_periph_t peripheral, uint32_t timeout);

int h7spi_is_in_error(h7spi_periph_t peripheral);
h7spi_spi_fsm_state_t h7spi_get_state(h7spi_periph_t peripheral);
h7spi_spi_ret_code_t h7spi_clear_error_state(h7spi_periph_t peripheral);


h7spi_spi_ret_code_t h7spi_spi_master_shift_nonblocking(h7spi_periph_t peripheral, uint16_t shift_size,  uint8_t *mi_buf, uint8_t *mo_buf, uint32_t timeout);
h7spi_spi_ret_code_t h7spi_spi_master_shift_blocking(h7spi_periph_t peripheral, uint16_t shift_size,  uint8_t *mi_buf, uint8_t *mo_buf, uint32_t timeout);


#endif // INC_H7SPI_BARE_H_
