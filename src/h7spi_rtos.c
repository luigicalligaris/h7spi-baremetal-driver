
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

#include "main.h"
#include "h7spi_config.h"

#if H7SPI_USE_FREERTOS_IMPL == 1

#include "FreeRTOS.h"
#include "task.h"

#include "h7spi_rtos.h"
#include "h7spi_bare.h"
#include "h7spi_bare_priv.h"

h7spi_spi_ret_code_t h7spi_wait_until_ready(h7spi_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // Blocking loop, waiting for the I2C peripheral to become free, to get into error state or to timeout
  // This version yields control to scheduler if mutex is busy
  while (HAL_GetTick() - timestart < timeout)
  {
    if (h7spi_is_ready(peripheral))
      return H7SPI_RET_CODE_OK;
    else
      taskYIELD();
   }
   return H7SPI_RET_CODE_BUSY;
}

h7spi_spi_ret_code_t h7spi_spi_mutex_lock(h7spi_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // This infinite loop blocks the hosting task until the mutex is taken, an error takes place or time is out
  while (HAL_GetTick() - timestart < timeout)
  {
    switch ( h7spi_spi_mutex_lock_impl(peripheral) )
    {
      // We got the mutex
      case H7SPI_RET_CODE_OK:
        return H7SPI_RET_CODE_OK;

      // This device is not managed by the driver
      case H7SPI_RET_CODE_UNMANAGED_BY_DRIVER:
        return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;

      // We havent't got the mutex (yet)
      case H7SPI_RET_CODE_BUSY:
      default:
        taskYIELD();
    }
  }
  // We timed out
  return H7SPI_RET_CODE_BUSY;
}
#endif
