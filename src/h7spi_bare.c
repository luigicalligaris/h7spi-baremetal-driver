
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

#include <string.h>

#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h745xx.h"

#include "h7spi_config.h"

#include "h7spi_bare.h"
#include "h7spi_bare_priv.h"


// For an explanation of the CFG1 and CFG2 bitmaps, see RM0399

#if H7SPI_PERIPH_ENABLE_SPI1 == 1
uint8_t h7spi_mutex_spi1 = H7SPI_SPI_MUTEX_UNLOCKED;

h7spi_driver_instance_state_t h7spi_state_spi1 =
{
  .fsm_state     = H7SPI_FSM_STATE_UNINITIALIZED,
  .spi_base      = SPI1_BASE,
  .fifo_deph     = H7SPI_FIFO_DEPH_SPI2S1,
  .cr1_value     = 0UL,
  .cr2_value     = 0UL,
  .cfg1_value    = 0UL,
  .cfg2_value    = 0UL,
  .wr_todo       = 0UL,
  .wr_done       = 0UL,
  .wr_data       = NULL,
  .rd_todo       = 0UL,
  .rd_done       = 0UL,
  .rd_data       = NULL,
  .timestart     = 0UL,
  .timeout       = 0UL
};

h7spi_periph_init_config_t current_periph_init_config_spi1 = 
{
  .pin_sck       = H7SPI_PIN_SPI1_SCK_PA5,
  .pin_nss       = H7SPI_PIN_SPI1_NSS_PA4,
  .pin_miso      = H7SPI_PIN_SPI1_MISO_PA6,
  .pin_mosi      = H7SPI_PIN_SPI1_MOSI_PB5,
  .rcc_clksource = RCC_SPI1CLKSOURCE_PLL2,
};
#endif

#if H7SPI_PERIPH_ENABLE_SPI2 == 1
uint8_t h7spi_mutex_spi2 = H7SPI_SPI_MUTEX_UNLOCKED;

h7spi_driver_instance_state_t h7spi_state_spi2 =
{
  .fsm_state     = H7SPI_FSM_STATE_UNINITIALIZED,
  .spi_base      = SPI2_BASE,
  .fifo_deph     = H7SPI_FIFO_DEPH_SPI2S2,
  .cr1_value     = 0UL,
  .cr2_value     = 0UL,
  .cfg1_value    = 0UL,
  .cfg2_value    = 0UL,
  .wr_todo       = 0UL,
  .wr_done       = 0UL,
  .wr_data       = NULL,
  .rd_todo       = 0UL,
  .rd_done       = 0UL,
  .rd_data       = NULL,
  .timestart     = 0UL,
  .timeout       = 0UL
};

h7spi_periph_init_config_t current_periph_init_config_spi2 = 
{
  .pin_sck       = H7SPI_PIN_SPI2_SCK_PB10,
  .pin_nss       = H7SPI_PIN_SPI2_NSS_PB4,
  .pin_miso      = H7SPI_PIN_SPI2_MISO_PB14,
  .pin_mosi      = H7SPI_PIN_SPI3_MOSI_PB2,
  .rcc_clksource = RCC_SPI2CLKSOURCE_PLL2,
};
#endif

#if H7SPI_PERIPH_ENABLE_SPI3 == 1
uint8_t h7spi_mutex_spi3 = H7SPI_SPI_MUTEX_UNLOCKED;

h7spi_driver_instance_state_t h7spi_state_spi3 =
{
  .fsm_state     = H7SPI_FSM_STATE_UNINITIALIZED,
  .spi_base      = SPI3_BASE,
  .fifo_deph     = H7SPI_FIFO_DEPH_SPI2S3,
  .cr1_value     = 0UL,
  .cr2_value     = 0UL,
  .cfg1_value    = 0UL,
  .cfg2_value    = 0UL,
  .ier_value     = 0UL,
  .shift_size    = 0UL,
  .shift_tx_cont = 0UL,
  .shift_rx_cont = 0UL,
  .wr_data       = NULL,
  .rd_data       = NULL,
  .timestart     = 0UL,
  .timeout       = 0UL
};

h7spi_periph_init_config_t current_periph_init_config_spi3 = 
{
  .pin_sck       = H7SPI_PIN_SPI3_SCK_PB3, //H7SPI_PIN_SPI3_SCK_PC10,
  .pin_nss       = H7SPI_PIN_SPI3_NSS_PA4, //H7SPI_PIN_SPI3_NSS_PA15,
  .pin_miso      = H7SPI_PIN_SPI3_MISO_PB4, //H7SPI_PIN_SPI3_MISO_PC11,
  .pin_mosi      = H7SPI_PIN_SPI3_MOSI_PB5,  //H7SPI_PIN_SPI3_MOSI_PC12,
  .rcc_clksource = RCC_SPI3CLKSOURCE_PLL2,
};
#endif

#if H7SPI_PERIPH_ENABLE_SPI4 == 1
uint8_t h7spi_mutex_spi4 = H7SPI_SPI_MUTEX_UNLOCKED;

h7spi_driver_instance_state_t h7spi_state_spi4 =
{
  .fsm_state     = H7SPI_FSM_STATE_UNINITIALIZED,
  .spi_base      = SPI4_BASE,
  .fifo_deph     = H7SPI_FIFO_DEPH_SPI4,
  .cr1_value     = 0UL,
  .cr2_value     = 0UL,
  .cfg1_value    = 0UL,
  .cfg2_value    = 0UL,
  .wr_todo       = 0UL,
  .wr_done       = 0UL,
  .wr_data       = NULL,
  .rd_todo       = 0UL,
  .rd_done       = 0UL,
  .rd_data       = NULL,
  .timestart     = 0UL,
  .timeout       = 0UL
};

h7spi_periph_init_config_t current_periph_init_config_spi4 = 
{
  .pin_sck       = H7SPI_PIN_SPI4_SCK_PE2,
  .pin_nss       = H7SPI_PIN_SPI4_NSS_PE4,
  .pin_miso      = H7SPI_PIN_SPI4_MISO_PE5,
  .pin_mosi      = H7SPI_PIN_SPI4_MOSI_PE6,
  .rcc_clksource = RCC_SPI45CLKSOURCE_PLL2,
};
#endif

#if H7SPI_PERIPH_ENABLE_SPI5 == 1
uint8_t h7spi_mutex_spi5 = H7SPI_SPI_MUTEX_UNLOCKED;

h7spi_driver_instance_state_t h7spi_state_spi5 =
{
  .fsm_state     = H7SPI_FSM_STATE_UNINITIALIZED,
  .spi_base      = SPI5_BASE,
  .fifo_deph     = H7SPI_FIFO_DEPH_SPI5,
  .rcc_clksource = RCC_SPI45CLKSOURCE_PLL2,
  .cr1_value     = 0UL,
  .cr2_value     = 0UL,
  .cfg1_value    = 0UL,
  .cfg2_value    = 0UL,
  .wr_todo       = 0UL,
  .wr_done       = 0UL,
  .wr_data       = NULL,
  .rd_todo       = 0UL,
  .rd_done       = 0UL,
  .rd_data       = NULL,
  .timestart     = 0UL,
  .timeout       = 0UL
};

h7spi_periph_init_config_t current_periph_init_config_spi5 = 
{
  .pin_sck       = H7SPI_PIN_SPI5_SCK_PF7,
  .pin_nss       = H7SPI_PIN_SPI5_NSS_PF6,
  .pin_miso      = H7SPI_PIN_SPI5_MISO_PF8,
  .pin_mosi      = H7SPI_PIN_SPI5_MOSI_PF9,
  .rcc_clksource = RCC_SPI45CLKSOURCE_PLL2,
};
#endif

#if H7SPI_PERIPH_ENABLE_SPI6 == 1
uint8_t h7spi_mutex_spi6 = H7SPI_SPI_MUTEX_UNLOCKED;

h7spi_driver_instance_state_t h7spi_state_spi6 =
{
  .fsm_state     = H7SPI_FSM_STATE_UNINITIALIZED,
  .spi_base      = SPI6_BASE,
  .fifo_deph     = H7SPI_FIFO_DEPH_SPI6,
  .rcc_clksource = RCC_SPI6CLKSOURCE_PLL2,
  .cr1_value     = 0UL,
  .cr2_value     = 0UL,
  .cfg1_value    = 0UL,
  .cfg2_value    = 0UL,
  .wr_todo       = 0UL,
  .wr_done       = 0UL,
  .wr_data       = NULL,
  .rd_todo       = 0UL,
  .rd_done       = 0UL,
  .rd_data       = NULL,
  .timestart     = 0UL,
  .timeout       = 0UL
};

h7spi_periph_init_config_t current_periph_init_config_spi6 = 
{
  .pin_sck       = H7SPI_PIN_SPI6_SCK_PA5,
  .pin_nss       = H7SPI_PIN_SPI6_NSS_PA4,
  .pin_miso      = H7SPI_PIN_SPI6_MISO_PA6,
  .pin_mosi      = H7SPI_PIN_SPI6_MOSI_PB5,
  .rcc_clksource = RCC_SPI6CLKSOURCE_PLL2,
};
#endif


#if H7SPI_PERIPH_ENABLE_SPI1 == 1
void SPI1_IRQHandler(void)
{
  H7SPI_IRQHandler_Impl(H7SPI_SPI1);
}
#endif

#if H7SPI_PERIPH_ENABLE_SPI2 == 1
void SPI2_IRQHandler(void)
{
  H7SPI_IRQHandler_Impl(H7SPI_SPI2);
}
#endif

#if H7SPI_PERIPH_ENABLE_SPI3 == 1
void SPI3_IRQHandler(void)
{
  H7SPI_IRQHandler_Impl(H7SPI_SPI3);
}
#endif

#if H7SPI_PERIPH_ENABLE_SPI4 == 1
void SPI4_IRQHandler(void)
{
  H7SPI_IRQHandler_Impl(H7SPI_SPI4);
}
#endif

#if H7SPI_PERIPH_ENABLE_SPI5 == 1
void SPI5_IRQHandler(void)
{
  H7SPI_IRQHandler_Impl(H7SPI_SPI5);
}
#endif

#if H7SPI_PERIPH_ENABLE_SPI6 == 1
void SPI6_IRQHandler(void)
{
  H7SPI_IRQHandler_Impl(H7SPI_SPI6);
}
#endif

inline static h7spi_driver_instance_state_t* h7spi_get_driver_instance(h7spi_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      return &h7spi_state_spi1;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      return &h7spi_state_spi2;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      return &h7spi_state_spi3;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      return &h7spi_state_spi4;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      return &h7spi_state_spi5;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      return &h7spi_state_spi6;
#endif
    default:
      return NULL;
  };
}

inline int h7spi_is_peripheral_managed_by_this_driver(h7spi_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      return 1;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      return 1;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      return 1;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      return 1;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      return 1;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      return 1;
#endif
    default:
      return 0;
  };
}

h7spi_spi_fsm_state_t h7spi_get_state(h7spi_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      return h7spi_state_spi1.fsm_state;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      return h7spi_state_spi2.fsm_state;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      return h7spi_state_spi3.fsm_state;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      return h7spi_state_spi4.fsm_state;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      return h7spi_state_spi5.fsm_state;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      return h7spi_state_spi6.fsm_state;
#endif
    default:
      break;
  };
  return H7SPI_FSM_STATE_UNMANAGED_BY_DRIVER;
}

int h7spi_is_in_error(h7spi_periph_t peripheral)
{
  switch(h7spi_get_state(peripheral))
  {
    case H7SPI_FSM_STATE_ERROR_UDR:
    case H7SPI_FSM_STATE_ERROR_OVR:
    case H7SPI_FSM_STATE_ERROR_CRCE:
    case H7SPI_FSM_STATE_ERROR_TIFRE:
    case H7SPI_FSM_STATE_ERROR_MODF:
      return 1;
    default:
      break;
  }
  return 0;
}

int h7spi_is_ready(h7spi_periph_t peripheral)
{
  switch(h7spi_get_state(peripheral))
  {
    case H7SPI_FSM_STATE_UNINITIALIZED:
    case H7SPI_FSM_STATE_IDLE:
      return 1;
    default:
      break;
  }
  return 0;
}

__weak h7spi_spi_ret_code_t h7spi_wait_until_ready(h7spi_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // Blocking loop, waiting for the SPI peripheral to become free, to get into error state or to timeout
  // This version loops until timeout if device is busy
  while (HAL_GetTick() - timestart < timeout)
  {
    if (h7spi_is_ready(peripheral))
      return H7SPI_RET_CODE_OK;
  }
  return H7SPI_RET_CODE_BUSY;
}


h7spi_spi_ret_code_t h7spi_spi_mutex_lock_impl(h7spi_periph_t peripheral)
{
  uint8_t* p_mutex = NULL;

  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      p_mutex = &h7spi_mutex_spi1;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      p_mutex = &h7spi_mutex_spi2;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      p_mutex = &h7spi_mutex_spi3;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      p_mutex = &h7spi_mutex_spi4;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      p_mutex = &h7spi_mutex_spi5;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      p_mutex = &h7spi_mutex_spi6;
      break;
#endif
    default:
      return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  // Cortex-M exclusive monitor read: sets the exclusive monitor flag on address p_mutex
  if (H7SPI_SPI_MUTEX_UNLOCKED == __LDREXB(p_mutex))
  {
    // Cortex-M exclusive monitor write: only writes and returns 0 if exclusive 
    // monitor flag is still set, otherwise return nonzero and write nothing
    if (0 == __STREXB(H7SPI_SPI_MUTEX_LOCKED, p_mutex))
    {
      __DMB();// Data Memory Barrier
      return H7SPI_RET_CODE_OK;
    }
  }
  return H7SPI_RET_CODE_BUSY;
}


__weak h7spi_spi_ret_code_t h7spi_spi_mutex_lock(h7spi_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // This is a simple infinite loop. As this is bare metal code it's supposed not to be an issue, 
  // but in general it is inefficient. The RTOS implementation solves this with a yield call.
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
        continue;
    }
  }

  // We timed out
  return H7SPI_RET_CODE_BUSY;
}


h7spi_spi_ret_code_t h7spi_spi_mutex_release(h7spi_periph_t peripheral)
{
  uint8_t* p_mutex = NULL;

  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      p_mutex = &h7spi_mutex_spi1;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      p_mutex = &h7spi_mutex_spi2;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      p_mutex = &h7spi_mutex_spi3;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      p_mutex = &h7spi_mutex_spi4;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      p_mutex = &h7spi_mutex_spi5;
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      p_mutex = &h7spi_mutex_spi6;
      break;
#endif
    default:
      return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  *p_mutex = H7SPI_SPI_MUTEX_UNLOCKED;

  return H7SPI_RET_CODE_OK;
}


h7spi_spi_ret_code_t h7spi_spi_mutex_release_fromISR(h7spi_periph_t peripheral)
{
  return h7spi_spi_mutex_release(peripheral);
}


h7spi_spi_ret_code_t h7spi_spi_reset_peripheral_full(h7spi_periph_t peripheral)
{
  h7spi_driver_instance_state_t* instance = h7spi_get_driver_instance(peripheral);

  if (!instance)
    return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;

  SPI_TypeDef* const spix = (SPI_TypeDef*) instance->spi_base;

  // Disable interrupts
  MODIFY_REG(spix->IER,
      SPI_IER_RXPIE     | SPI_IER_TXPIE     | SPI_IER_DXPIE     | SPI_IER_EOTIE
    | SPI_IER_TXTFIE    | SPI_IER_UDRIE     | SPI_IER_OVRIE     | SPI_IER_CRCEIE
    | SPI_IER_TIFREIE   | SPI_IER_MODFIE    | SPI_IER_TSERFIE,
    0x00000000);

  // Clear the Control Register 1
  MODIFY_REG(spix->CR1,
      SPI_CR1_SPE       | SPI_CR1_MASRX     | SPI_CR1_CSTART    | SPI_CR1_CSUSP
    | SPI_CR1_HDDIR     | SPI_CR1_SSI       | SPI_CR1_CRC33_17  | SPI_CR1_RCRCINI
    | SPI_CR1_TCRCINI   | SPI_CR1_IOLOCK,
    0x00000000);

  // Clear the Control Register 2
  MODIFY_REG(spix->CR2, SPI_CR2_TSER | SPI_CR2_TSIZE, 0x00000000);

  // Clear the Config Register 1
  MODIFY_REG(spix->CFG1,
      SPI_CFG1_DSIZE    | SPI_CFG1_FTHLV    | SPI_CFG1_UDRCFG   | SPI_CFG1_UDRDET
    | SPI_CFG1_RXDMAEN  | SPI_CFG1_TXDMAEN  | SPI_CFG1_CRCSIZE  | SPI_CFG1_CRCEN
    | SPI_CFG1_MBR,
    0x00000000);

  // Clear the Config Register 2
  MODIFY_REG(spix->CFG2,
      SPI_CFG2_MSSI     | SPI_CFG2_MIDI     | SPI_CFG2_IOSWP    | SPI_CFG2_COMM
    | SPI_CFG2_SP       | SPI_CFG2_MASTER   | SPI_CFG2_LSBFRST  | SPI_CFG2_CPHA
    | SPI_CFG2_CPOL     | SPI_CFG2_SSM      | SPI_CFG2_SSIOP    | SPI_CFG2_SSOM
    | SPI_CFG2_AFCNTR,
    0x00000000);

  // Clear interrupts
  MODIFY_REG(spix->IFCR,
      SPI_IFCR_EOTC     | SPI_IFCR_TXTFC    | SPI_IFCR_UDRC     | SPI_IFCR_OVRC
    | SPI_IFCR_CRCEC    | SPI_IFCR_TIFREC   | SPI_IFCR_MODFC    | SPI_IFCR_TSERFC
    | SPI_IFCR_SUSPC,
    0x00000000);

  return H7SPI_RET_CODE_OK;
}


h7spi_spi_ret_code_t h7spi_spi_reset_peripheral_soft(h7spi_periph_t peripheral)
{
  h7spi_driver_instance_state_t* instance = h7spi_get_driver_instance(peripheral);

  if (!instance)
    return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;

  SPI_TypeDef* spix = (SPI_TypeDef*) instance->spi_base;

  CLEAR_BIT(spix->CR1, SPI_CR1_SPE);
  ((void) READ_BIT(spix->CR1, SPI_CR1_SPE)); // the cast to void is to suppress the "value computed is not used [-Wunused-value]" warning
  SET_BIT(spix->CR1, SPI_CR1_SPE);

  return H7SPI_RET_CODE_OK;
}


h7spi_spi_ret_code_t h7spi_spi_reset_driver(h7spi_periph_t peripheral)
{
  h7spi_driver_instance_state_t* instance = h7spi_get_driver_instance(peripheral);

  if (!instance)
    return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;

  instance->fsm_state = H7SPI_FSM_STATE_IDLE;
  h7spi_spi_mutex_release(peripheral);

  return H7SPI_RET_CODE_OK;
}


h7spi_spi_ret_code_t h7spi_spi_init(h7spi_periph_t peripheral)
{
  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      h7spi_spi_init_by_config(peripheral, &current_periph_init_config_spi1);
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      h7spi_spi_init_by_config(peripheral, &current_periph_init_config_spi2);
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      h7spi_spi_init_by_config(peripheral, &current_periph_init_config_spi3);
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      h7spi_spi_init_by_config(peripheral, &current_periph_init_config_spi4);
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      h7spi_spi_init_by_config(peripheral, &current_periph_init_config_spi5);
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      h7spi_spi_init_by_config(peripheral, &current_periph_init_config_spi6);
      break;
#endif
    default:
      break;
  };
  return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;
}


h7spi_spi_ret_code_t h7spi_spi_init_by_config(h7spi_periph_t peripheral, h7spi_periph_init_config_t* init_config)
{
  if (!init_config)
    return H7SPI_RET_CODE_INVALID_ARGS;

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
      PeriphClkInitStruct.Spi123ClockSelection = init_config->rcc_clksource;
      memcpy(&current_periph_init_config_spi1, init_config, sizeof(h7spi_periph_init_config_t));
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
      PeriphClkInitStruct.Spi123ClockSelection = init_config->rcc_clksource;
      memcpy(&current_periph_init_config_spi2, init_config, sizeof(h7spi_periph_init_config_t));
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
      PeriphClkInitStruct.Spi123ClockSelection = init_config->rcc_clksource;
      memcpy(&current_periph_init_config_spi3, init_config, sizeof(h7spi_periph_init_config_t));
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
      PeriphClkInitStruct.Spi45ClockSelection  = init_config->rcc_clksource;
      memcpy(&current_periph_init_config_spi4, init_config, sizeof(h7spi_periph_init_config_t));
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI5;
      PeriphClkInitStruct.Spi45ClockSelection  = init_config->rcc_clksource;
      memcpy(&current_periph_init_config_spi5, init_config, sizeof(h7spi_periph_init_config_t));
      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
      PeriphClkInitStruct.Spi6ClockSelection   = init_config->rcc_clksource;
      memcpy(&current_periph_init_config_spi6, init_config, sizeof(h7spi_periph_init_config_t));
      break;
#endif
    default:
      return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  switch(peripheral)
  {
#if H7SPI_PERIPH_ENABLE_SPI1 == 1
    case H7SPI_SPI1:
      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_sck)
      {
        case H7SPI_PIN_SPI1_SCK_PA5:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PA5  (SPI1_SCK)
          MODIFY_REG(GPIOA->AFR[0], 0b1111 << 20, 0b0101 << 20);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA5
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA5
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA5
          MODIFY_REG(GPIOA->OTYPER, 0b1 <<  5, 0b1 <<  5);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA5
          MODIFY_REG(GPIOA->MODER, 0b11 << 10, 0b10 << 10);
          break;

        case H7SPI_PIN_SPI1_SCK_PB3:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PB3  (SPI1_SCK)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 12, 0b0101 << 12);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB3
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 <<  6, 0b11 <<  6);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB3
          MODIFY_REG(GPIOB->PUPDR, 0b11 <<  6, 0b00 <<  6);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB3
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  3, 0b1 <<  3);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB3
          MODIFY_REG(GPIOB->MODER, 0b11 <<  6, 0b10 <<  6);
          break;

        case H7SPI_PIN_SPI1_SCK_PG11:
          __HAL_RCC_GPIOG_CLK_ENABLE();

          // GPIOG AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PG11 (SPI1_SCK)
          MODIFY_REG(GPIOG->AFR[1], 0b1111 << 12, 0b0101 << 12);
          // GPIOG OSPEEDR: Set very high speed = 0b11 to pin PB8
          MODIFY_REG(GPIOG->OSPEEDR, 0b11 << 22, 0b11 << 22);
          // GPIOG PUPDR: Set no pull-up = 0b00 to pin PB8
          MODIFY_REG(GPIOG->PUPDR, 0b11 << 22, 0b00 << 22);
          // GPIOG OTYPEDR: Set open drain = 0b1 to pin PB8
          MODIFY_REG(GPIOG->OTYPER, 0b1 << 11, 0b1 << 11);
          // GPIOG MODER: Set alternate mode = 0b10 to pins PB8
          MODIFY_REG(GPIOG->MODER, 0b11 << 22, 0b10 << 22);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_nss)
      {
        case H7SPI_PIN_SPI1_NSS_PA4:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PA4  (SPI1_NSS)
          MODIFY_REG(GPIOA->AFR[0], 0b1111 << 16, 0b0101 << 16);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA4
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA4
          MODIFY_REG(GPIOA->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA4
          MODIFY_REG(GPIOA->OTYPER, 0b1 <<  4, 0b1 <<  4);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA4
          MODIFY_REG(GPIOA->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI1_NSS_PA15:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PA15 (SPI1_NSS)
          MODIFY_REG(GPIOA->AFR[1], 0b1111 << 28, 0b0101 << 28);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PB9
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 30, 0b11 << 30);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PB9
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 30, 0b00 << 30);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PB9
          MODIFY_REG(GPIOA->OTYPER, 0b1 << 15, 0b1 << 15);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PB9
          MODIFY_REG(GPIOA->MODER, 0b11 << 30, 0b10 << 30);
          break;

        case H7SPI_PIN_SPI1_NSS_PG10:
          __HAL_RCC_GPIOG_CLK_ENABLE();

          // GPIOG AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PG10 (SPI1_NSS)
          MODIFY_REG(GPIOG->AFR[1], 0b1111 <<  8, 0b0101 <<  8);
          // GPIOG OSPEEDR: Set very high speed = 0b11 to pin PB9
          MODIFY_REG(GPIOG->OSPEEDR, 0b11 << 20, 0b11 << 20);
          // GPIOG PUPDR: Set no pull-up = 0b00 to pin PB9
          MODIFY_REG(GPIOG->PUPDR, 0b11 << 20, 0b00 << 20);
          // GPIOG OTYPEDR: Set open drain = 0b1 to pin PB9
          MODIFY_REG(GPIOG->OTYPER, 0b1 << 10, 0b1 << 10);
          // GPIOG MODER: Set alternate mode = 0b10 to pins PB9
          MODIFY_REG(GPIOG->MODER, 0b11 << 20, 0b10 << 20);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_miso)
      {
        case H7SPI_PIN_SPI1_MISO_PA6:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PA6  (SPI1_NSS)
          MODIFY_REG(GPIOA->AFR[0], 0b1111 << 24, 0b0101 << 24);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA6
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA6
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 12, 0b00 << 12);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA6
          MODIFY_REG(GPIOA->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA6
          MODIFY_REG(GPIOA->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7SPI_PIN_SPI1_MISO_PB4:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PB4  (SPI1_NSS)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 16, 0b0101 << 16);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB4
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB4
          MODIFY_REG(GPIOB->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB4
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  4, 0b1 <<  4);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB4
          MODIFY_REG(GPIOB->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI1_MISO_PG9:
          __HAL_RCC_GPIOG_CLK_ENABLE();

          // GPIOG AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PG9  (SPI1_NSS)
          MODIFY_REG(GPIOG->AFR[1], 0b1111 <<  4, 0b0101 <<  4);
          // GPIOG OSPEEDR: Set very high speed = 0b11 to pin PB9
          MODIFY_REG(GPIOG->OSPEEDR, 0b11 << 19, 0b11 << 19);
          // GPIOG PUPDR: Set no pull-up = 0b00 to pin PB9
          MODIFY_REG(GPIOG->PUPDR, 0b11 << 19, 0b00 << 19);
          // GPIOG OTYPEDR: Set open drain = 0b1 to pin PB9
          MODIFY_REG(GPIOG->OTYPER, 0b1 <<  9, 0b1 <<  9);
          // GPIOG MODER: Set alternate mode = 0b10 to pins PB9
          MODIFY_REG(GPIOG->MODER, 0b11 << 19, 0b10 << 19);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_mosi)
      {
        case H7SPI_PIN_SPI1_MOSI_PA7:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PA7  (SPI1_MOSI)
          MODIFY_REG(GPIOA->AFR[0], 0b1111 << 28, 0b0101 << 28);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA7
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA7
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 14, 0b00 << 14);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA7
          MODIFY_REG(GPIOA->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA7
          MODIFY_REG(GPIOA->MODER, 0b11 << 14, 0b10 << 14);
          break;

        case H7SPI_PIN_SPI1_MOSI_PB5:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PB5  (SPI1_MOSI)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 20, 0b0101 << 20);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB5
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB5
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB5
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  5, 0b1 <<  5);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB5
          MODIFY_REG(GPIOB->MODER, 0b11 << 10, 0b10 <<  10);
          break;

        case H7SPI_PIN_SPI1_MOSI_PD7:
          __HAL_RCC_GPIOD_CLK_ENABLE();

          // GPIOD AFRL: Set alternate function SPI1 = 5 = 0b0101 (see datasheet chapt 5) to pin PD7  (SPI1_MOSI)
          MODIFY_REG(GPIOD->AFR[0], 0b1111 << 28, 0b0101 << 28);
          // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PB9
          MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOD PUPDR: Set no pull-up = 0b00 to pin PB9
          MODIFY_REG(GPIOD->PUPDR, 0b11 << 14, 0b00 << 14);
          // GPIOD OTYPEDR: Set open drain = 0b1 to pin PB9
          MODIFY_REG(GPIOD->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOD MODER: Set alternate mode = 0b10 to pins PB9
          MODIFY_REG(GPIOD->MODER, 0b11 << 14, 0b10 << 14);
          break;

        default:
          Error_Handler();
      }

      //
      // *** SPI SETUP ***
      //

      __HAL_RCC_SPI1_CLK_ENABLE();

      h7spi_spi_reset_peripheral_full(H7SPI_SPI1);

      // Set the Config Register 1 as in configuration struct
      MODIFY_REG(SPI1->CFG1,
          SPI_CFG1_DSIZE    | SPI_CFG1_FTHLV    | SPI_CFG1_UDRCFG   | SPI_CFG1_UDRDET
        | SPI_CFG1_RXDMAEN  | SPI_CFG1_TXDMAEN  | SPI_CFG1_CRCSIZE  | SPI_CFG1_CRCEN
        | SPI_CFG1_MBR,
        init_config->cfg1);

      // Set the Config Register 2 as in configuration struct
      MODIFY_REG(SPI1->CFG2,
          SPI_CFG2_MSSI     | SPI_CFG2_MIDI     | SPI_CFG2_IOSWP    | SPI_CFG2_COMM
        | SPI_CFG2_SP       | SPI_CFG2_MASTER   | SPI_CFG2_LSBFRST  | SPI_CFG2_CPHA
        | SPI_CFG2_CPOL     | SPI_CFG2_SSM      | SPI_CFG2_SSIOP    | SPI_CFG2_SSOM
        | SPI_CFG2_AFCNTR,
        init_config->cfg2);

      // Set the value for the CRC polynomial
      MODIFY_REG(SPI1->CRCPOLY, SPI_CRCPOLY_CRCPOLY, init_config->crcpoly);

      // Set TSER and TSIZE to zero in Control Register 2, as we are not yet transferring right now
      MODIFY_REG(SPI1->CR2, SPI_CR2_TSER | SPI_CR2_TSIZE, 0x00000000);

      // Set the non-control configuration bits in the Control Register 1
      MODIFY_REG(SPI1->CR1, SPI_CR1_MASRX | SPI_CR1_CRC33_17 | SPI_CR1_RCRCINI | SPI_CR1_TCRCINI | SPI_CR1_IOLOCK,
        init_config->cr1cfg
        );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(SPI1_IRQn, H7I2C_IRQ_SPI1_PRI, H7I2C_IRQ_SPI1_SUBPRI);
      HAL_NVIC_EnableIRQ(SPI1_IRQn);

      h7spi_state_spi1.fsm_state = H7SPI_FSM_STATE_IDLE;

      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI2 == 1
    case H7SPI_SPI2:
      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_sck)
      {
        case H7SPI_PIN_SPI2_SCK_PA9:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PA9  (SPI2_SCK)
          MODIFY_REG(GPIOA->AFR[1], 0b1111 <<  8, 0b0101 <<  8);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA9
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 18, 0b11 << 18);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA9
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 18, 0b00 << 18);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA9
          MODIFY_REG(GPIOA->OTYPER, 0b1 <<  9, 0b1 <<  9);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA9
          MODIFY_REG(GPIOA->MODER, 0b11 << 18, 0b10 << 18);
          break;

        case H7SPI_PIN_SPI2_SCK_PA12:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PA12 (SPI2_SCK)
          MODIFY_REG(GPIOA->AFR[1], 0b1111 << 16, 0b0101 << 16);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA12
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA12
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 24, 0b00 << 24);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA12
          MODIFY_REG(GPIOA->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA12
          MODIFY_REG(GPIOA->MODER, 0b11 << 24, 0b10 << 24);
          break;

        case H7SPI_PIN_SPI2_SCK_PB10:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PB10 (SPI2_SCK)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 << 12, 0b0101 << 12);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB10
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 20, 0b11 << 20);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB10
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 20, 0b00 << 20);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB10
          MODIFY_REG(GPIOB->OTYPER, 0b1 << 10, 0b1 << 10);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB10
          MODIFY_REG(GPIOB->MODER, 0b11 << 20, 0b10 << 20);
          break;

        case H7SPI_PIN_SPI2_SCK_PB13:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PB13 (SPI2_SCK)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 << 20, 0b0101 << 20);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB13
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 26, 0b11 << 26);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB13
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 26, 0b00 << 26);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB13
          MODIFY_REG(GPIOB->OTYPER, 0b1 << 13, 0b1 << 13);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB13
          MODIFY_REG(GPIOB->MODER, 0b11 << 26, 0b10 << 26);
          break;

        case H7SPI_PIN_SPI2_SCK_PD3:
          __HAL_RCC_GPIOD_CLK_ENABLE();

          // GPIOD AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PD3  (SPI2_SCK)
          MODIFY_REG(GPIOD->AFR[0], 0b1111 << 12, 0b0101 << 12);
          // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PF1
          MODIFY_REG(GPIOD->OSPEEDR, 0b11 <<  6, 0b11 <<  6);
          // GPIOD PUPDR: Set no pull-up = 0b00 to pin PF1
          MODIFY_REG(GPIOD->PUPDR, 0b11 <<  6, 0b00 <<  6);
          // GPIOD OTYPEDR: Set open drain = 0b1 to pin PF1
          MODIFY_REG(GPIOD->OTYPER, 0b1 <<  3, 0b1 <<  3);
          // GPIOD MODER: Set alternate mode = 0b10 to pins PF1
          MODIFY_REG(GPIOD->MODER, 0b11 <<  6, 0b10 <<  6);
          break;

        case H7SPI_PIN_SPI2_SCK_PI1:
          __HAL_RCC_GPIOI_CLK_ENABLE();

          // GPIOI AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PI1  (SPI2_SCK)
          MODIFY_REG(GPIOI->AFR[0], 0b1111 <<  4, 0b0101 <<  4);
          // GPIOI OSPEEDR: Set very high speed = 0b11 to pin PI1
          MODIFY_REG(GPIOI->OSPEEDR, 0b11 <<  2, 0b11 <<  2);
          // GPIOI PUPDR: Set no pull-up = 0b00 to pin PI1
          MODIFY_REG(GPIOI->PUPDR, 0b11 <<  2, 0b00 <<  2);
          // GPIOI OTYPEDR: Set open drain = 0b1 to pin PI1
          MODIFY_REG(GPIOI->OTYPER, 0b1 <<  1, 0b1 <<  1);
          // GPIOI MODER: Set alternate mode = 0b10 to pins PI1
          MODIFY_REG(GPIOI->MODER, 0b11 <<  2, 0b10 <<  2);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_nss)
      {
        case H7SPI_PIN_SPI2_NSS_PA11:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PA11 (SPI2_NSS)
          MODIFY_REG(GPIOA->AFR[1], 0b1111 << 12, 0b0101 << 12);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA4
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 22, 0b11 << 22);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA4
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 22, 0b00 << 22);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA4
          MODIFY_REG(GPIOA->OTYPER, 0b1 << 11, 0b1 << 11);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA4
          MODIFY_REG(GPIOA->MODER, 0b11 << 22, 0b10 << 22);
          break;

        case H7SPI_PIN_SPI2_NSS_PB4:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI2 = 7 = 0b0111 (see datasheet chapt 5) to pin PB4  (SPI2_NSS)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 16, 0b0111 << 16);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB4
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB4
          MODIFY_REG(GPIOB->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB4
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  4, 0b1 <<  4);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB4
          MODIFY_REG(GPIOB->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI2_NSS_PB9:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PB9  (SPI2_NSS)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 <<  4, 0b0101 <<  4);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB9
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 18, 0b11 << 18);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB9
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 18, 0b00 << 18);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB9
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  9, 0b1 <<  9);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB9
          MODIFY_REG(GPIOB->MODER, 0b11 << 18, 0b10 << 18);
          break;

        case H7SPI_PIN_SPI2_NSS_PB12:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PB12  (SPI2_NSS)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 << 16, 0b0101 << 16);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB12
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB12
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 24, 0b00 << 24);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB12
          MODIFY_REG(GPIOB->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB12
          MODIFY_REG(GPIOB->MODER, 0b11 << 24, 0b10 << 24);
          break;

        case H7SPI_PIN_SPI2_NSS_PI0:
          __HAL_RCC_GPIOI_CLK_ENABLE();

          // GPIOI AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PI0  (SPI2_SCL)
          MODIFY_REG(GPIOI->AFR[0], 0b1111 <<  0, 0b0101 <<  0);
          // GPIOI OSPEEDR: Set very high speed = 0b11 to pin PI0
          MODIFY_REG(GPIOI->OSPEEDR, 0b11 <<  0, 0b11 <<  0);
          // GPIOI PUPDR: Set no pull-up = 0b00 to pin PI0
          MODIFY_REG(GPIOI->PUPDR, 0b11 <<  0, 0b00 <<  0);
          // GPIOI OTYPEDR: Set open drain = 0b1 to pin PI0
          MODIFY_REG(GPIOI->OTYPER, 0b1 <<  0, 0b1 <<  0);
          // GPIOI MODER: Set alternate mode = 0b10 to pins PI0
          MODIFY_REG(GPIOI->MODER, 0b11 <<  0, 0b10 <<  0);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_miso)
      {
        case H7SPI_PIN_SPI2_MISO_PB14:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PB14 (SPI2_MISO)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 << 24, 0b0101 << 24);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB14
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB14
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 12, 0b00 << 12);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB14
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB14
          MODIFY_REG(GPIOB->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7SPI_PIN_SPI2_MISO_PC2:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PC2  (SPI2_MISO)
          MODIFY_REG(GPIOC->AFR[0], 0b1111 <<  8, 0b0101 <<  8);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC2
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 <<  4, 0b11 <<  4);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PC2
          MODIFY_REG(GPIOC->PUPDR, 0b11 <<  4, 0b00 <<  4);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PC2
          MODIFY_REG(GPIOC->OTYPER, 0b1 <<  2, 0b1 <<  2);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PC2
          MODIFY_REG(GPIOC->MODER, 0b11 <<  4, 0b10 <<  4);
          break;

        case H7SPI_PIN_SPI2_MISO_PI2:
          __HAL_RCC_GPIOI_CLK_ENABLE();

          // GPIOI AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PI2  (SPI2_MISO)
          MODIFY_REG(GPIOI->AFR[0], 0b1111 <<  8, 0b0101 <<  8);
          // GPIOI OSPEEDR: Set very high speed = 0b11 to pin PI2
          MODIFY_REG(GPIOI->OSPEEDR, 0b11 <<  4, 0b11 <<  4);
          // GPIOI PUPDR: Set no pull-up = 0b00 to pin PI2
          MODIFY_REG(GPIOI->PUPDR, 0b11 <<  4, 0b00 <<  4);
          // GPIOI OTYPEDR: Set open drain = 0b1 to pin PI2
          MODIFY_REG(GPIOI->OTYPER, 0b1 <<  2, 0b1 <<  2);
          // GPIOI MODER: Set alternate mode = 0b10 to pins PI2
          MODIFY_REG(GPIOI->MODER, 0b11 <<  4, 0b10 <<  4);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_mosi)
      {
        case H7SPI_PIN_SPI2_MOSI_PB15:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PB15 (SPI2_MISO)
          MODIFY_REG(GPIOB->AFR[1], 0b1111 << 28, 0b0101 << 28);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB14
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 30, 0b11 << 30);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB14
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 30, 0b00 << 30);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB14
          MODIFY_REG(GPIOB->OTYPER, 0b1 << 15, 0b1 << 15);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB14
          MODIFY_REG(GPIOB->MODER, 0b11 << 30, 0b10 << 30);
          break;

        case H7SPI_PIN_SPI2_MOSI_PC1:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PC1  (SPI2_MISO)
          MODIFY_REG(GPIOC->AFR[0], 0b1111 <<  4, 0b0101 <<  4);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC1
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 <<  2, 0b11 <<  2);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PC1
          MODIFY_REG(GPIOC->PUPDR, 0b11 <<  2, 0b00 <<  2);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PC1
          MODIFY_REG(GPIOC->OTYPER, 0b1 <<  1, 0b1 <<  1);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PC1
          MODIFY_REG(GPIOC->MODER, 0b11 <<  2, 0b10 <<  2);
          break;

        case H7SPI_PIN_SPI2_MOSI_PC3:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PC3  (SPI2_MISO)
          MODIFY_REG(GPIOC->AFR[0], 0b1111 << 12, 0b0101 << 12);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC3
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 <<  6, 0b11 <<  6);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PC3
          MODIFY_REG(GPIOC->PUPDR, 0b11 <<  6, 0b00 <<  6);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PC3
          MODIFY_REG(GPIOC->OTYPER, 0b1 <<  3, 0b1 <<  3);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PC3
          MODIFY_REG(GPIOC->MODER, 0b11 <<  6, 0b10 <<  6);
          break;

        case H7SPI_PIN_SPI2_MOSI_PI3:
          __HAL_RCC_GPIOI_CLK_ENABLE();

          // GPIOI AFRL: Set alternate function SPI2 = 5 = 0b0101 (see datasheet chapt 5) to pin PI3  (SPI2_MISO)
          MODIFY_REG(GPIOI->AFR[0], 0b1111 << 12, 0b0101 << 12);
          // GPIOI OSPEEDR: Set very high speed = 0b11 to pin PI3
          MODIFY_REG(GPIOI->OSPEEDR, 0b11 <<  6, 0b11 <<  6);
          // GPIOI PUPDR: Set no pull-up = 0b00 to pin PI3
          MODIFY_REG(GPIOI->PUPDR, 0b11 <<  6, 0b00 <<  6);
          // GPIOI OTYPEDR: Set open drain = 0b1 to pin PI3
          MODIFY_REG(GPIOI->OTYPER, 0b1 <<  3, 0b1 <<  3);
          // GPIOI MODER: Set alternate mode = 0b10 to pins PI3
          MODIFY_REG(GPIOI->MODER, 0b11 <<  6, 0b10 <<  6);
          break;

        default:
          Error_Handler();
      }

      //
      // *** SPI SETUP ***
      //

      __HAL_RCC_SPI2_CLK_ENABLE();

      h7spi_spi_reset_peripheral_full(H7SPI_SPI2);

      // Set the Config Register 1 as in configuration struct
      MODIFY_REG(SPI2->CFG1,
          SPI_CFG1_DSIZE    | SPI_CFG1_FTHLV    | SPI_CFG1_UDRCFG   | SPI_CFG1_UDRDET
        | SPI_CFG1_RXDMAEN  | SPI_CFG1_TXDMAEN  | SPI_CFG1_CRCSIZE  | SPI_CFG1_CRCEN
        | SPI_CFG1_MBR,
        init_config->cfg1);

      // Set the Config Register 2 as in configuration struct
      MODIFY_REG(SPI2->CFG2,
          SPI_CFG2_MSSI     | SPI_CFG2_MIDI     | SPI_CFG2_IOSWP    | SPI_CFG2_COMM
        | SPI_CFG2_SP       | SPI_CFG2_MASTER   | SPI_CFG2_LSBFRST  | SPI_CFG2_CPHA
        | SPI_CFG2_CPOL     | SPI_CFG2_SSM      | SPI_CFG2_SSIOP    | SPI_CFG2_SSOM
        | SPI_CFG2_AFCNTR,
        init_config->cfg2);

      // Set the value for the CRC polynomial
      MODIFY_REG(SPI2->CRCPOLY, SPI_CRCPOLY_CRCPOLY, init_config->crcpoly);

      // Set TSER and TSIZE to zero in Control Register 2, as we are not yet transferring right now
      MODIFY_REG(SPI2->CR2, SPI_CR2_TSER | SPI_CR2_TSIZE, 0x00000000);

      // Set the non-control configuration bits in the Control Register 1
      MODIFY_REG(SPI2->CR1, SPI_CR1_MASRX | SPI_CR1_CRC33_17 | SPI_CR1_RCRCINI | SPI_CR1_TCRCINI | SPI_CR1_IOLOCK,
        init_config->cr1cfg
        );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(SPI2_IRQn, H7I2C_IRQ_SPI2_PRI, H7I2C_IRQ_SPI2_SUBPRI);
      HAL_NVIC_EnableIRQ(SPI2_IRQn);

      h7spi_state_spi2.fsm_state = H7SPI_FSM_STATE_IDLE;

      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI3 == 1
    case H7SPI_SPI3:
      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_sck)
      {
        case H7SPI_PIN_SPI3_SCK_PB3:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI3 = 6 = 0b0110 (see datasheet chapt 5) to pin PB3  (SPI3_SCK)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 12, 0b0110 << 12);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB3
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 <<  6, 0b11 <<  6);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB3
          MODIFY_REG(GPIOB->PUPDR, 0b11 <<  6, 0b00 <<  6);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB3
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  3, 0b0 <<  3);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB3
          MODIFY_REG(GPIOB->MODER, 0b11 <<  6, 0b10 <<  6);
          break;

        case H7SPI_PIN_SPI3_SCK_PC10:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI3 = 6 = 0b0110 (see datasheet chapt 5) to pin PC10 (SPI3_SCK)
          MODIFY_REG(GPIOC->AFR[1], 0b1111 <<  8, 0b0110 <<  8);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC10
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 20, 0b11 << 20);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PC10
          MODIFY_REG(GPIOC->PUPDR, 0b11 << 20, 0b01 << 20);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PC10
          MODIFY_REG(GPIOC->OTYPER, 0b1 << 10, 0b1 << 10);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PC10
          MODIFY_REG(GPIOC->MODER, 0b11 << 20, 0b10 << 20);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_nss)
      {
        case H7SPI_PIN_SPI3_NSS_PA4:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI3 = 6 = 0b0110 (see datasheet chapt 5) to pin PA4  (SPI3_NSS)
          MODIFY_REG(GPIOA->AFR[0], 0b1111 << 16, 0b0110 << 16);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA4
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA4
          MODIFY_REG(GPIOA->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA4
          MODIFY_REG(GPIOA->OTYPER, 0b1 <<  4, 0b0 <<  4);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA4
          MODIFY_REG(GPIOA->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI3_NSS_PA15:
          __HAL_RCC_GPIOA_CLK_ENABLE();

          // GPIOA AFRL: Set alternate function SPI3 = 6 = 0b0110 (see datasheet chapt 5) to pin PA15  (SPI3_NSS)
          MODIFY_REG(GPIOA->AFR[1], 0b1111 << 28, 0b0110 << 28);
          // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA15
          MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 30, 0b11 << 30);
          // GPIOA PUPDR: Set no pull-up = 0b00 to pin PA15
          MODIFY_REG(GPIOA->PUPDR, 0b11 << 30, 0b01 << 30);
          // GPIOA OTYPEDR: Set open drain = 0b1 to pin PA15
          MODIFY_REG(GPIOA->OTYPER, 0b1 << 15, 0b1 << 15);
          // GPIOA MODER: Set alternate mode = 0b10 to pins PA15
          MODIFY_REG(GPIOA->MODER, 0b11 << 30, 0b10 << 30);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_miso)
      {
        case H7SPI_PIN_SPI3_MISO_PB4:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI3 = 6 = 0b0110 (see datasheet chapt 5) to pin PB4  (SPI3_MISO)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 16, 0b0110 << 16);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB4
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB4
          MODIFY_REG(GPIOB->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB4
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  4, 0b0 <<  4);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB4
          MODIFY_REG(GPIOB->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI3_MISO_PC11:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI3 = 6 = 0b0110 (see datasheet chapt 5) to pin PC11 (SPI3_MISO)
          MODIFY_REG(GPIOC->AFR[1], 0b1111 << 12, 0b0110 << 12);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC11
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 22, 0b11 << 22);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PC11
          MODIFY_REG(GPIOC->PUPDR, 0b11 << 22, 0b01 << 22);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PC11
          MODIFY_REG(GPIOC->OTYPER, 0b1 << 11, 0b1 << 11);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PC11
          MODIFY_REG(GPIOC->MODER, 0b11 << 22, 0b10 << 22);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_mosi)
      {
        case H7SPI_PIN_SPI3_MOSI_PB2:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOB AFRL: Set alternate function SPI3 = 7 = 0b0111 (see datasheet chapt 5) to pin PB2 (SPI3_MOSI)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 <<  8, 0b0111 <<  8);
          // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB2
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 <<  4, 0b11 <<  4);
          // GPIOB PUPDR: Set no pull-up = 0b00 to pin PB2
          MODIFY_REG(GPIOB->PUPDR, 0b11 <<  4, 0b01 <<  4);
          // GPIOB OTYPEDR: Set open drain = 0b1 to pin PB2
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  2, 0b1 <<  2);
          // GPIOB MODER: Set alternate mode = 0b10 to pins PB2
          MODIFY_REG(GPIOB->MODER, 0b11 <<  4, 0b10 <<  4);
          break;

        case H7SPI_PIN_SPI3_MOSI_PB5:
          __HAL_RCC_GPIOB_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI3 = 7 = 0b0111 (see datasheet chapt 5) to pin PB5  (SPI3_MOSI)
          MODIFY_REG(GPIOB->AFR[0], 0b1111 << 20, 0b0111 << 20);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PB5
          MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PB5
          MODIFY_REG(GPIOB->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PB5
          MODIFY_REG(GPIOB->OTYPER, 0b1 <<  5, 0b0 <<  5);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PB5
          MODIFY_REG(GPIOB->MODER, 0b11 << 10, 0b10 << 10);
          break;

        case H7SPI_PIN_SPI3_MOSI_PC12:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI3 = 6 = 0b0110 (see datasheet chapt 5) to pin PC12  (SPI3_MOSI)
          MODIFY_REG(GPIOC->AFR[1], 0b1111 << 16, 0b0110 << 16);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC12
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PC12
          MODIFY_REG(GPIOC->PUPDR, 0b11 << 24, 0b01 << 24);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PC12
          MODIFY_REG(GPIOC->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PC12
          MODIFY_REG(GPIOC->MODER, 0b11 << 24, 0b10 << 24);
          break;

        case H7SPI_PIN_SPI3_MOSI_PD6:
          __HAL_RCC_GPIOI_CLK_ENABLE();

          // GPIOI AFRL: Set alternate function SPI3 = 5 = 0b0101 (see datasheet chapt 5) to pin PD6  (SPI3_MOSI)
          MODIFY_REG(GPIOI->AFR[0], 0b1111 << 24, 0b0101 << 24);
          // GPIOI OSPEEDR: Set very high speed = 0b11 to pin PD6
          MODIFY_REG(GPIOI->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOI PUPDR: Set no pull-up = 0b00 to pin PD6
          MODIFY_REG(GPIOI->PUPDR, 0b11 << 12, 0b01 << 12);
          // GPIOI OTYPEDR: Set open drain = 0b1 to pin PD6
          MODIFY_REG(GPIOI->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOI MODER: Set alternate mode = 0b10 to pins PD6
          MODIFY_REG(GPIOI->MODER, 0b11 << 12, 0b10 << 12);
          break;

        default:
          Error_Handler();
      }

      //
      // *** SPI SETUP ***
      //

      __HAL_RCC_SPI3_CLK_ENABLE();

      h7spi_spi_reset_peripheral_full(H7SPI_SPI3);

      // Set the Config Register 1 as in configuration struct
      MODIFY_REG(SPI3->CFG1,
          SPI_CFG1_DSIZE    | SPI_CFG1_FTHLV    | SPI_CFG1_UDRCFG   | SPI_CFG1_UDRDET
        | SPI_CFG1_RXDMAEN  | SPI_CFG1_TXDMAEN  | SPI_CFG1_CRCSIZE  | SPI_CFG1_CRCEN
        | SPI_CFG1_MBR,
        init_config->cfg1);

      // Set the Config Register 2 as in configuration struct
      MODIFY_REG(SPI3->CFG2,
          SPI_CFG2_MSSI     | SPI_CFG2_MIDI     | SPI_CFG2_IOSWP    | SPI_CFG2_COMM
        | SPI_CFG2_SP       | SPI_CFG2_MASTER   | SPI_CFG2_LSBFRST  | SPI_CFG2_CPHA
        | SPI_CFG2_CPOL     | SPI_CFG2_SSM      | SPI_CFG2_SSIOP    | SPI_CFG2_SSOM
        | SPI_CFG2_AFCNTR,
        init_config->cfg2);

      // Set the value for the CRC polynomial
      //MODIFY_REG(SPI3->CRCPOLY, SPI_CRCPOLY_CRCPOLY, init_config->crcpoly);

      // Set TSER and TSIZE to zero in Control Register 2, as we are not yet transferring right now
      MODIFY_REG(SPI3->CR2, SPI_CR2_TSER | SPI_CR2_TSIZE, 0x00000000);

      // Set the non-control configuration bits in the Control Register 1
      MODIFY_REG(SPI3->CR1, SPI_CR1_MASRX | SPI_CR1_CRC33_17 | SPI_CR1_RCRCINI | SPI_CR1_TCRCINI | SPI_CR1_IOLOCK,
        init_config->cr1cfg
        );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(SPI3_IRQn, H7I2C_IRQ_SPI3_PRI, H7I2C_IRQ_SPI3_SUBPRI);
      HAL_NVIC_EnableIRQ(SPI3_IRQn);

      h7spi_state_spi3.fsm_state = H7SPI_FSM_STATE_IDLE;

      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI4 == 1
    case H7SPI_SPI4:
      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_sck)
      {
        case H7SPI_PIN_SPI4_SCK_PE2:
          __HAL_RCC_GPIOE_CLK_ENABLE();

          // GPIOE AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE2  (SPI4_SCL)
          MODIFY_REG(GPIOE->AFR[0], 0b1111 <<  8, 0b0101 <<  8);
          // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE2
          MODIFY_REG(GPIOE->OSPEEDR, 0b11 <<  4, 0b11 <<  4);
          // GPIOE PUPDR: Set no pull-up = 0b00 to pin PE2
          MODIFY_REG(GPIOE->PUPDR, 0b11 <<  4, 0b00 <<  4);
          // GPIOE OTYPEDR: Set push-pull output type = 0b0 to pin PE2
          MODIFY_REG(GPIOE->OTYPER, 0b1 <<  2, 0b0 <<  2);
          // GPIOE MODER: Set alternate mode = 0b10 to pins PE2
          MODIFY_REG(GPIOE->MODER, 0b11 <<  4, 0b10 <<  4);
          break;

        case H7SPI_PIN_SPI4_SCK_PE12:
          __HAL_RCC_GPIOE_CLK_ENABLE();

          // GPIOE AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE12  (SPI4_SCL)
          MODIFY_REG(GPIOE->AFR[1], 0b1111 << 16, 0b0101 << 16);
          // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE12
          MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOE PUPDR: Set no pull-up = 0b00 to pin PE12
          MODIFY_REG(GPIOE->PUPDR, 0b11 << 24, 0b00 << 24);
          // GPIOE OTYPEDR: Set open drain = 0b1 to pin PE12
          MODIFY_REG(GPIOE->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOE MODER: Set alternate mode = 0b10 to pins PE12
          MODIFY_REG(GPIOE->MODER, 0b11 << 24, 0b10 << 24);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_nss)
      {
        case H7SPI_PIN_SPI4_NSS_PE4:
          __HAL_RCC_GPIOE_CLK_ENABLE();

          // GPIOE AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE4  (SPI4_NSS)
          MODIFY_REG(GPIOE->AFR[0], 0b1111 << 16, 0b0101 << 16);
          // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE4
          MODIFY_REG(GPIOE->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOE PUPDR: Set no pull-up = 0b00 to pin PE4
          MODIFY_REG(GPIOE->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOE OTYPEDR: Set push-pull output = 0b0 to pin PE4
          MODIFY_REG(GPIOE->OTYPER, 0b1 <<  4, 0b0 <<  4);
          // GPIOE MODER: Set alternate mode = 0b10 to pins PE4
          MODIFY_REG(GPIOE->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI4_NSS_PE11:
          __HAL_RCC_GPIOE_CLK_ENABLE();

          // GPIOE AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE11  (SPI4_NSS)
          MODIFY_REG(GPIOE->AFR[1], 0b1111 << 12, 0b0101 << 12);
          // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE11
          MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 22, 0b11 << 22);
          // GPIOE PUPDR: Set no pull-up = 0b00 to pin PE11
          MODIFY_REG(GPIOE->PUPDR, 0b11 << 22, 0b00 << 22);
          // GPIOE OTYPEDR: Set open drain = 0b1 to pin PE11
          MODIFY_REG(GPIOE->OTYPER, 0b1 << 11, 0b1 << 11);
          // GPIOE MODER: Set alternate mode = 0b10 to pins PE11
          MODIFY_REG(GPIOE->MODER, 0b11 << 22, 0b10 << 22);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_miso)
      {
        case H7SPI_PIN_SPI4_MISO_PE5
          __HAL_RCC_GPIOE_CLK_ENABLE();

          // GPIOE AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE5  (SPI3_MISO)
          MODIFY_REG(GPIOE->AFR[0], 0b1111 << 20, 0b0110 << 20);
          // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE5
          MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOE PUPDR: Set no pull-up = 0b00 to pin PE5
          MODIFY_REG(GPIOE->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOE OTYPEDR: Set push-pull output = 0b0 to pin PE5
          MODIFY_REG(GPIOE->OTYPER, 0b1 <<  5, 0b0 <<  5);
          // GPIOE MODER: Set alternate mode = 0b10 to pins PE5
          MODIFY_REG(GPIOE->MODER, 0b11 << 10, 0b10 << 10);
          break;

        case H7SPI_PIN_SPI4_MISO_PE13
          __HAL_RCC_GPIOE_CLK_ENABLE();

          // GPIOE AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE13 (SPI3_MISO)
          MODIFY_REG(GPIOE->AFR[1], 0b1111 << 20, 0b0110 << 20);
          // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE13
          MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 26, 0b11 << 26);
          // GPIOE PUPDR: Set no pull-up = 0b00 to pin PE13
          MODIFY_REG(GPIOE->PUPDR, 0b11 << 26, 0b00 << 26);
          // GPIOE OTYPEDR: Set open drain = 0b1 to pin PE13
          MODIFY_REG(GPIOE->OTYPER, 0b1 << 13, 0b1 << 13);
          // GPIOE MODER: Set alternate mode = 0b10 to pins PE13
          MODIFY_REG(GPIOE->MODER, 0b11 << 26, 0b10 << 26);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_mosi)
      {
        case H7SPI_PIN_SPI4_MOSI_PE6:
          __HAL_RCC_GPIOI_CLK_ENABLE();

          // GPIOI AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE6  (SPI3_MOSI)
          MODIFY_REG(GPIOI->AFR[0], 0b1111 << 24, 0b0101 << 24);
          // GPIOI OSPEEDR: Set very high speed = 0b11 to pin PE6
          MODIFY_REG(GPIOI->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOI PUPDR: Set no pull-up = 0b00 to pin PE6
          MODIFY_REG(GPIOI->PUPDR, 0b11 << 12, 0b00 << 12);
          // GPIOI OTYPEDR: Set push-pull output = 0b0 to pin PE6
          MODIFY_REG(GPIOI->OTYPER, 0b1 <<  6, 0b0 <<  6);
          // GPIOI MODER: Set alternate mode = 0b10 to pins PE6
          MODIFY_REG(GPIOI->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7SPI_PIN_SPI4_MOSI_PE14:
          __HAL_RCC_GPIOC_CLK_ENABLE();

          // GPIOC AFRL: Set alternate function SPI4 = 5 = 0b0101 (see datasheet chapt 5) to pin PE14  (SPI3_MOSI)
          MODIFY_REG(GPIOC->AFR[1], 0b1111 << 16, 0b0110 << 16);
          // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PE14
          MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOC PUPDR: Set no pull-up = 0b00 to pin PE14
          MODIFY_REG(GPIOC->PUPDR, 0b11 << 24, 0b00 << 24);
          // GPIOC OTYPEDR: Set open drain = 0b1 to pin PE14
          MODIFY_REG(GPIOC->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOC MODER: Set alternate mode = 0b10 to pins PE14
          MODIFY_REG(GPIOC->MODER, 0b11 << 24, 0b10 << 24);
          break;

        default:
          Error_Handler();
      }

      //
      // *** SPI SETUP ***
      //

      __HAL_RCC_SPI4_CLK_ENABLE();

      h7spi_spi_reset_peripheral_full(H7SPI_SPI4);

      // Set the Config Register 1 as in configuration struct
      MODIFY_REG(SPI4->CFG1,
          SPI_CFG1_DSIZE    | SPI_CFG1_FTHLV    | SPI_CFG1_UDRCFG   | SPI_CFG1_UDRDET
        | SPI_CFG1_RXDMAEN  | SPI_CFG1_TXDMAEN  | SPI_CFG1_CRCSIZE  | SPI_CFG1_CRCEN
        | SPI_CFG1_MBR,
        init_config->cfg1);

      // Set the Config Register 2 as in configuration struct
      MODIFY_REG(SPI4->CFG2,
          SPI_CFG2_MSSI     | SPI_CFG2_MIDI     | SPI_CFG2_IOSWP    | SPI_CFG2_COMM
        | SPI_CFG2_SP       | SPI_CFG2_MASTER   | SPI_CFG2_LSBFRST  | SPI_CFG2_CPHA
        | SPI_CFG2_CPOL     | SPI_CFG2_SSM      | SPI_CFG2_SSIOP    | SPI_CFG2_SSOM
        | SPI_CFG2_AFCNTR,
        init_config->cfg2);

      // Set the value for the CRC polynomial
      MODIFY_REG(SPI4->CRCPOLY, SPI_CRCPOLY_CRCPOLY, init_config->crcpoly);

      // Set TSER and TSIZE to zero in Control Register 2, as we are not yet transferring right now
      MODIFY_REG(SPI4->CR2, SPI_CR2_TSER | SPI_CR2_TSIZE, 0x00000000);

      // Set the non-control configuration bits in the Control Register 1
      MODIFY_REG(SPI4->CR1, SPI_CR1_MASRX | SPI_CR1_CRC33_17 | SPI_CR1_RCRCINI | SPI_CR1_TCRCINI | SPI_CR1_IOLOCK,
        init_config->cr1cfg
        );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(SPI4_IRQn, H7I2C_IRQ_SPI4_PRI, H7I2C_IRQ_SPI4_SUBPRI);
      HAL_NVIC_EnableIRQ(SPI4_IRQn);

      h7spi_state_spi4.fsm_state = H7SPI_FSM_STATE_IDLE;

      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI5 == 1
    case H7SPI_SPI5:
      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_sck)
      {
        case H7SPI_PIN_SPI5_SCK_PF7:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PF7  (SPI4_SCL)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 28, 0b0101 << 28);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF7
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PF7
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 14, 0b00 << 14);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF7
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF7
          MODIFY_REG(GPIOF->MODER, 0b11 << 14, 0b10 << 14);
          break;

        case H7SPI_PIN_SPI5_SCK_PH6:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PH6  (SPI4_SCL)
          MODIFY_REG(GPIOH->AFR[0], 0b1111 << 24, 0b0101 << 24);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH6
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOH PUPDR: Set no pull-up = 0b00 to pin PH6
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 12, 0b00 << 12);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH6
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH6
          MODIFY_REG(GPIOH->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7SPI_PIN_SPI5_SCK_PK0:
          __HAL_RCC_GPIOK_CLK_ENABLE();

          // GPIOK AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PK0  (SPI4_SCL)
          MODIFY_REG(GPIOK->AFR[1], 0b1111 <<  0, 0b0101 <<  0);
          // GPIOK OSPEEDR: Set very high speed = 0b11 to pin PK0
          MODIFY_REG(GPIOK->OSPEEDR, 0b11 <<  0, 0b11 << 24);
          // GPIOK PUPDR: Set no pull-up = 0b00 to pin PK0
          MODIFY_REG(GPIOK->PUPDR, 0b11 <<  0, 0b00 <<  0);
          // GPIOK OTYPEDR: Set open drain = 0b1 to pin PK0
          MODIFY_REG(GPIOK->OTYPER, 0b1 <<  0, 0b1 <<  0);
          // GPIOK MODER: Set alternate mode = 0b10 to pins PK0
          MODIFY_REG(GPIOK->MODER, 0b11 <<  0, 0b10 <<  0);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_nss)
      {
        case H7SPI_PIN_SPI5_NSS_PF6:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PF6  (SPI4_NSS)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 24, 0b0101 << 24);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF6
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PF6
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 12, 0b00 << 12);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF6
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF6
          MODIFY_REG(GPIOF->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7SPI_PIN_SPI5_NSS_PH5:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PH5  (SPI4_NSS)
          MODIFY_REG(GPIOH->AFR[0], 0b1111 << 20, 0b0101 << 20);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH5
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOH PUPDR: Set no pull-up = 0b00 to pin PH5
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH5
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  5, 0b1 <<  5);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH5
          MODIFY_REG(GPIOH->MODER, 0b11 << 10, 0b10 << 10);
          break;

        case H7SPI_PIN_SPI5_NSS_PK1:
          __HAL_RCC_GPIOK_CLK_ENABLE();

          // GPIOK AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PK1  (SPI4_NSS)
          MODIFY_REG(GPIOK->AFR[0], 0b1111 <<  4, 0b0101 <<  4);
          // GPIOK OSPEEDR: Set very high speed = 0b11 to pin PK1
          MODIFY_REG(GPIOK->OSPEEDR, 0b11 <<  2, 0b11 <<  2);
          // GPIOK PUPDR: Set no pull-up = 0b00 to pin PK1
          MODIFY_REG(GPIOK->PUPDR, 0b11 <<  2, 0b00 <<  2);
          // GPIOK OTYPEDR: Set open drain = 0b1 to pin PK1
          MODIFY_REG(GPIOK->OTYPER, 0b1 <<  1, 0b1 <<  1);
          // GPIOK MODER: Set alternate mode = 0b10 to pins PK1
          MODIFY_REG(GPIOK->MODER, 0b11 <<  2, 0b10 <<  2);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_miso)
      {
        case H7SPI_PIN_SPI5_MISO_PF8
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PF8  (SPI3_MISO)
          MODIFY_REG(GPIOF->AFR[1], 0b1111 <<  0, 0b0101 <<  0);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF8
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 16, 0b11 << 16);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PF8
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 16, 0b00 << 16);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF8
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  8, 0b1 <<  8);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF8
          MODIFY_REG(GPIOF->MODER, 0b11 << 16, 0b10 << 16);
          break;

        case H7SPI_PIN_SPI5_MISO_PH7
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PH7 (SPI3_MISO)
          MODIFY_REG(GPIOH->AFR[1], 0b1111 << 28, 0b0101 << 28);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH7
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOH PUPDR: Set no pull-up = 0b00 to pin PH7
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 14, 0b00 << 14);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PH7
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PH7
          MODIFY_REG(GPIOH->MODER, 0b11 << 14, 0b10 << 14);
          break;

        case H7SPI_PIN_SPI5_MISO_PJ11
          __HAL_RCC_GPIOJ_CLK_ENABLE();

          // GPIOJ AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PJ11  (SPI3_MISO)
          MODIFY_REG(GPIOJ->AFR[0], 0b1111 << 12, 0b0101 << 12);
          // GPIOJ OSPEEDR: Set very high speed = 0b11 to pin PJ11
          MODIFY_REG(GPIOJ->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOJ PUPDR: Set no pull-up = 0b00 to pin PJ11
          MODIFY_REG(GPIOJ->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOJ OTYPEDR: Set open drain = 0b1 to pin PJ11
          MODIFY_REG(GPIOJ->OTYPER, 0b1 <<  5, 0b1 <<  5);
          // GPIOJ MODER: Set alternate mode = 0b10 to pins PJ11
          MODIFY_REG(GPIOJ->MODER, 0b11 << 10, 0b10 << 10);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_mosi)
      {
        case H7SPI_PIN_SPI5_MOSI_PF9:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PF9  (SPI3_MOSI)
          MODIFY_REG(GPIOF->AFR[1], 0b1111 <<  4, 0b0101 <<  4);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF9
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 18, 0b11 << 18);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PF9
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 18, 0b00 << 18);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF9
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  9, 0b1 <<  9);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF9
          MODIFY_REG(GPIOF->MODER, 0b11 << 18, 0b10 << 18);
          break;

        case H7SPI_PIN_SPI5_MOSI_PF11:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PF11  (SPI3_MOSI)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 12, 0b0101 << 12);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF11
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 22, 0b11 << 22);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PF11
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 22, 0b00 << 22);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PF11
          MODIFY_REG(GPIOF->OTYPER, 0b1 << 11, 0b1 << 11);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PF11
          MODIFY_REG(GPIOF->MODER, 0b11 << 22, 0b10 << 22);
          break;

        case H7SPI_PIN_SPI5_MOSI_PJ10:
          __HAL_RCC_GPIOJ_CLK_ENABLE();

          // GPIOJ AFRL: Set alternate function SPI5 = 5 = 0b0101 (see datasheet chapt 5) to pin PJ10  (SPI3_MOSI)
          MODIFY_REG(GPIOJ->AFR[1], 0b1111 <<  8, 0b0101 <<  8);
          // GPIOJ OSPEEDR: Set very high speed = 0b11 to pin PJ10
          MODIFY_REG(GPIOJ->OSPEEDR, 0b11 << 20, 0b11 << 20);
          // GPIOJ PUPDR: Set no pull-up = 0b00 to pin PJ10
          MODIFY_REG(GPIOJ->PUPDR, 0b11 << 20, 0b00 << 20);
          // GPIOJ OTYPEDR: Set open drain = 0b1 to pin PJ10
          MODIFY_REG(GPIOJ->OTYPER, 0b1 << 10, 0b1 << 10);
          // GPIOJ MODER: Set alternate mode = 0b10 to pins PJ10
          MODIFY_REG(GPIOJ->MODER, 0b11 << 20, 0b10 << 20);
          break;

        default:
          Error_Handler();
      }

      //
      // *** SPI SETUP ***
      //

      __HAL_RCC_SPI5_CLK_ENABLE();

      h7spi_spi_reset_peripheral_full(H7SPI_SPI5);

      // Set the Config Register 1 as in configuration struct
      MODIFY_REG(SPI5->CFG1,
          SPI_CFG1_DSIZE    | SPI_CFG1_FTHLV    | SPI_CFG1_UDRCFG   | SPI_CFG1_UDRDET
        | SPI_CFG1_RXDMAEN  | SPI_CFG1_TXDMAEN  | SPI_CFG1_CRCSIZE  | SPI_CFG1_CRCEN
        | SPI_CFG1_MBR,
        init_config->cfg1);

      // Set the Config Register 2 as in configuration struct
      MODIFY_REG(SPI5->CFG2,
          SPI_CFG2_MSSI     | SPI_CFG2_MIDI     | SPI_CFG2_IOSWP    | SPI_CFG2_COMM
        | SPI_CFG2_SP       | SPI_CFG2_MASTER   | SPI_CFG2_LSBFRST  | SPI_CFG2_CPHA
        | SPI_CFG2_CPOL     | SPI_CFG2_SSM      | SPI_CFG2_SSIOP    | SPI_CFG2_SSOM
        | SPI_CFG2_AFCNTR,
        init_config->cfg2);

      // Set the value for the CRC polynomial
      MODIFY_REG(SPI5->CRCPOLY, SPI_CRCPOLY_CRCPOLY, init_config->crcpoly);

      // Set TSER and TSIZE to zero in Control Register 2, as we are not yet transferring right now
      MODIFY_REG(SPI5->CR2, SPI_CR2_TSER | SPI_CR2_TSIZE, 0x00000000);

      // Set the non-control configuration bits in the Control Register 1
      MODIFY_REG(SPI5->CR1, SPI_CR1_MASRX | SPI_CR1_CRC33_17 | SPI_CR1_RCRCINI | SPI_CR1_TCRCINI | SPI_CR1_IOLOCK,
        init_config->cr1cfg
        );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(SPI5_IRQn, H7I2C_IRQ_SPI5_PRI, H7I2C_IRQ_SPI5_SUBPRI);
      HAL_NVIC_EnableIRQ(SPI5_IRQn);

      h7spi_state_spi5.fsm_state = H7SPI_FSM_STATE_IDLE;

      break;
#endif
#if H7SPI_PERIPH_ENABLE_SPI6 == 1
    case H7SPI_SPI6:
      //
      // *** GPIO PIN SETUP ***
      //

      switch(init_config->pin_sck)
      {
        case H7SPI_PIN_SPI6_SCK_PA5:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI6 = 8 = 0b1000 (see datasheet chapt 5) to pin PA5  (SPI6_SCK)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 20, 0b1000 << 20);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PA5
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PA5
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PA5
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  5, 0b1 <<  5);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PA5
          MODIFY_REG(GPIOF->MODER, 0b11 << 10, 0b10 << 10);
          break;

        case H7SPI_PIN_SPI6_SCK_PB3:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function SPI6 = 5 = 0b0101 (see datasheet chapt 5) to pin PB3  (SPI6_SCK)
          MODIFY_REG(GPIOH->AFR[0], 0b1111 << 12, 0b0101 << 12);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PB3
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 <<  6, 0b11 <<  6);
          // GPIOH PUPDR: Set no pull-up = 0b00 to pin PB3
          MODIFY_REG(GPIOH->PUPDR, 0b11 <<  6, 0b00 <<  6);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PB3
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  3, 0b1 <<  3);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PB3
          MODIFY_REG(GPIOH->MODER, 0b11 <<  6, 0b10 <<  6);
          break;

        case H7SPI_PIN_SPI6_SCK_PG13:
          __HAL_RCC_GPIOK_CLK_ENABLE();

          // GPIOK AFRL: Set alternate function SPI6 = 5 = 0b0101 (see datasheet chapt 5) to pin PG13 (SPI6_SCK)
          MODIFY_REG(GPIOK->AFR[1], 0b1111 << 20, 0b0101 << 20);
          // GPIOK OSPEEDR: Set very high speed = 0b11 to pin PG13
          MODIFY_REG(GPIOK->OSPEEDR, 0b11 << 26, 0b11 << 26);
          // GPIOK PUPDR: Set no pull-up = 0b00 to pin PG13
          MODIFY_REG(GPIOK->PUPDR, 0b11 << 26, 0b00 << 26);
          // GPIOK OTYPEDR: Set open drain = 0b1 to pin PG13
          MODIFY_REG(GPIOK->OTYPER, 0b1 << 13, 0b1 << 13);
          // GPIOK MODER: Set alternate mode = 0b10 to pins PG13
          MODIFY_REG(GPIOK->MODER, 0b11 << 26, 0b10 << 26);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_nss)
      {
        case H7SPI_PIN_SPI6_NSS_PA4:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI6 = 8 = 0b1000 (see datasheet chapt 5) to pin PA4  (SPI6_NSS)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 16, 0b1000 << 16);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PA4
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PA4
          MODIFY_REG(GPIOF->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PA4
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  4, 0b1 <<  4);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PA4
          MODIFY_REG(GPIOF->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI6_NSS_PA15:
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function SPI6 = 7 = 0b0111 (see datasheet chapt 5) to pin PA15 (SPI6_NSS)
          MODIFY_REG(GPIOH->AFR[1], 0b1111 << 28, 0b0111 << 28);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PA15
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 30, 0b11 << 30);
          // GPIOH PUPDR: Set no pull-up = 0b00 to pin PA15
          MODIFY_REG(GPIOH->PUPDR, 0b11 << 30, 0b00 << 30);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PA15
          MODIFY_REG(GPIOH->OTYPER, 0b1 << 15, 0b1 << 15);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PA15
          MODIFY_REG(GPIOH->MODER, 0b11 << 30, 0b10 << 30);
          break;

        case H7SPI_PIN_SPI6_NSS_PG8:
          __HAL_RCC_GPIOK_CLK_ENABLE();

          // GPIOK AFRL: Set alternate function SPI6 = 5 = 0b0101 (see datasheet chapt 5) to pin PG8  (SPI6_NSS)
          MODIFY_REG(GPIOK->AFR[1], 0b1111 <<  0, 0b0101 <<  0);
          // GPIOK OSPEEDR: Set very high speed = 0b11 to pin PG8
          MODIFY_REG(GPIOK->OSPEEDR, 0b11 << 16, 0b11 << 16);
          // GPIOK PUPDR: Set no pull-up = 0b00 to pin PG8
          MODIFY_REG(GPIOK->PUPDR, 0b11 << 16, 0b00 << 16);
          // GPIOK OTYPEDR: Set open drain = 0b1 to pin PG8
          MODIFY_REG(GPIOK->OTYPER, 0b1 <<  8, 0b1 <<  8);
          // GPIOK MODER: Set alternate mode = 0b10 to pins PG8
          MODIFY_REG(GPIOK->MODER, 0b11 << 16, 0b10 << 16);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_miso)
      {
        case H7SPI_PIN_SPI6_MISO_PA6
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI6 = 8 = 0b1000 (see datasheet chapt 5) to pin PA6  (SPI6_MISO)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 24, 0b1000 << 24);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PA6
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 12, 0b11 << 12);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PA6
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 12, 0b00 << 12);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PA6
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  6, 0b1 <<  6);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PA6
          MODIFY_REG(GPIOF->MODER, 0b11 << 12, 0b10 << 12);
          break;

        case H7SPI_PIN_SPI6_MISO_PB4
          __HAL_RCC_GPIOH_CLK_ENABLE();

          // GPIOH AFRL: Set alternate function SPI6 = 8 = 0b1000 (see datasheet chapt 5) to pin PB4 (SPI6_MISO)
          MODIFY_REG(GPIOH->AFR[0], 0b1111 << 16, 0b1000 << 16);
          // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PB4
          MODIFY_REG(GPIOH->OSPEEDR, 0b11 <<  8, 0b11 <<  8);
          // GPIOH PUPDR: Set no pull-up = 0b00 to pin PB4
          MODIFY_REG(GPIOH->PUPDR, 0b11 <<  8, 0b00 <<  8);
          // GPIOH OTYPEDR: Set open drain = 0b1 to pin PB4
          MODIFY_REG(GPIOH->OTYPER, 0b1 <<  4, 0b1 <<  4);
          // GPIOH MODER: Set alternate mode = 0b10 to pins PB4
          MODIFY_REG(GPIOH->MODER, 0b11 <<  8, 0b10 <<  8);
          break;

        case H7SPI_PIN_SPI6_MISO_PG12
          __HAL_RCC_GPIOJ_CLK_ENABLE();

          // GPIOJ AFRL: Set alternate function SPI6 = 5 = 0b0101 (see datasheet chapt 5) to pin PG12  (SPI6_MISO)
          MODIFY_REG(GPIOJ->AFR[1], 0b1111 << 16, 0b0101 << 16);
          // GPIOJ OSPEEDR: Set very high speed = 0b11 to pin PG12
          MODIFY_REG(GPIOJ->OSPEEDR, 0b11 << 24, 0b11 << 24);
          // GPIOJ PUPDR: Set no pull-up = 0b00 to pin PG12
          MODIFY_REG(GPIOJ->PUPDR, 0b11 << 24, 0b00 << 24);
          // GPIOJ OTYPEDR: Set open drain = 0b1 to pin PG12
          MODIFY_REG(GPIOJ->OTYPER, 0b1 << 12, 0b1 << 12);
          // GPIOJ MODER: Set alternate mode = 0b10 to pins PG12
          MODIFY_REG(GPIOJ->MODER, 0b11 << 24, 0b10 << 24);
          break;

        default:
          Error_Handler();
      }

      switch(init_config->pin_mosi)
      {
        case H7SPI_PIN_SPI6_MOSI_PA7:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI6 = 8 = 0b1000 (see datasheet chapt 5) to pin PA7  (SPI6_MOSI)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 28, 0b1000 << 28);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PA7
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 14, 0b11 << 14);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PA7
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 14, 0b00 << 14);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PA7
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  7, 0b1 <<  7);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PA7
          MODIFY_REG(GPIOF->MODER, 0b11 << 14, 0b10 << 14);
          break;

        case H7SPI_PIN_SPI6_MOSI_PB5:
          __HAL_RCC_GPIOF_CLK_ENABLE();

          // GPIOF AFRL: Set alternate function SPI6 = 8 = 0b1000 (see datasheet chapt 5) to pin PB5  (SPI6_MOSI)
          MODIFY_REG(GPIOF->AFR[0], 0b1111 << 20, 0b0101 << 20);
          // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PB5
          MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 10, 0b11 << 10);
          // GPIOF PUPDR: Set no pull-up = 0b00 to pin PB5
          MODIFY_REG(GPIOF->PUPDR, 0b11 << 10, 0b00 << 10);
          // GPIOF OTYPEDR: Set open drain = 0b1 to pin PB5
          MODIFY_REG(GPIOF->OTYPER, 0b1 <<  5, 0b1 <<  5);
          // GPIOF MODER: Set alternate mode = 0b10 to pins PB5
          MODIFY_REG(GPIOF->MODER, 0b11 << 10, 0b10 << 10);
          break;

        case H7SPI_PIN_SPI6_MOSI_PG14:
          __HAL_RCC_GPIOJ_CLK_ENABLE();

          // GPIOJ AFRL: Set alternate function SPI6 = 5 = 0b0101 (see datasheet chapt 5) to pin PG14 (SPI6_MOSI)
          MODIFY_REG(GPIOJ->AFR[1], 0b1111 << 24, 0b0101 << 24);
          // GPIOJ OSPEEDR: Set very high speed = 0b11 to pin PG14
          MODIFY_REG(GPIOJ->OSPEEDR, 0b11 << 28, 0b11 << 28);
          // GPIOJ PUPDR: Set no pull-up = 0b00 to pin PG14
          MODIFY_REG(GPIOJ->PUPDR, 0b11 << 28, 0b00 << 28);
          // GPIOJ OTYPEDR: Set open drain = 0b1 to pin PG14
          MODIFY_REG(GPIOJ->OTYPER, 0b1 << 14, 0b1 << 14);
          // GPIOJ MODER: Set alternate mode = 0b10 to pins PG14
          MODIFY_REG(GPIOJ->MODER, 0b11 << 28, 0b10 << 28);
          break;

        default:
          Error_Handler();
      }

      //
      // *** SPI SETUP ***
      //

      __HAL_RCC_SPI6_CLK_ENABLE();

      h7spi_spi_reset_peripheral_full(H7SPI_SPI6);

      // Set the Config Register 1 as in configuration struct
      MODIFY_REG(SPI6->CFG1,
          SPI_CFG1_DSIZE    | SPI_CFG1_FTHLV    | SPI_CFG1_UDRCFG   | SPI_CFG1_UDRDET
        | SPI_CFG1_RXDMAEN  | SPI_CFG1_TXDMAEN  | SPI_CFG1_CRCSIZE  | SPI_CFG1_CRCEN
        | SPI_CFG1_MBR,
        init_config->cfg1);

      // Set the Config Register 2 as in configuration struct
      MODIFY_REG(SPI6->CFG2,
          SPI_CFG2_MSSI     | SPI_CFG2_MIDI     | SPI_CFG2_IOSWP    | SPI_CFG2_COMM
        | SPI_CFG2_SP       | SPI_CFG2_MASTER   | SPI_CFG2_LSBFRST  | SPI_CFG2_CPHA
        | SPI_CFG2_CPOL     | SPI_CFG2_SSM      | SPI_CFG2_SSIOP    | SPI_CFG2_SSOM
        | SPI_CFG2_AFCNTR,
        init_config->cfg2);

      // Set the value for the CRC polynomial
      MODIFY_REG(SPI6->CRCPOLY, SPI_CRCPOLY_CRCPOLY, init_config->crcpoly);

      // Set TSER and TSIZE to zero in Control Register 2, as we are not yet transferring right now
      MODIFY_REG(SPI6->CR2, SPI_CR2_TSER | SPI_CR2_TSIZE, 0x00000000);

      // Set the non-control configuration bits in the Control Register 1
      MODIFY_REG(SPI6->CR1, SPI_CR1_MASRX | SPI_CR1_CRC33_17 | SPI_CR1_RCRCINI | SPI_CR1_TCRCINI | SPI_CR1_IOLOCK,
        init_config->cr1cfg
        );

      //
      // *** NVIC SETUP ***
      //

      HAL_NVIC_SetPriority(SPI6_IRQn, H7I2C_IRQ_SPI6_PRI, H7I2C_IRQ_SPI6_SUBPRI);
      HAL_NVIC_EnableIRQ(SPI6_IRQn);

      h7spi_state_spi6.fsm_state = H7SPI_FSM_STATE_IDLE;

      break;
#endif
    default:
      return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  return H7SPI_RET_CODE_OK;
}


h7spi_spi_ret_code_t h7spi_clear_error_state(h7spi_periph_t peripheral)
{
  uint32_t const timeout = 100;

  h7spi_driver_instance_state_t* instance = h7spi_get_driver_instance(peripheral);

  if (!instance)
    return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;

  if (h7spi_spi_mutex_lock(peripheral, timeout) == H7SPI_RET_CODE_OK)
  {
    switch(instance->fsm_state)
    {
      case H7SPI_FSM_STATE_ERROR_UDR:
      case H7SPI_FSM_STATE_ERROR_OVR:
      case H7SPI_FSM_STATE_ERROR_CRCE:
      case H7SPI_FSM_STATE_ERROR_TIFRE:
      case H7SPI_FSM_STATE_ERROR_MODF:
        instance->fsm_state = H7SPI_FSM_STATE_IDLE;
        h7spi_spi_mutex_release(peripheral);
        return H7SPI_RET_CODE_OK;
      default:
        h7spi_spi_mutex_release(peripheral);
        return H7SPI_RET_CODE_CLEARED_A_NONERROR_STATE;
    }
  }
  return H7SPI_RET_CODE_BUSY;
}


void H7SPI_IRQHandler_Impl(h7spi_periph_t peripheral)
{
  h7spi_driver_instance_state_t* instance = h7spi_get_driver_instance(peripheral);
  SPI_TypeDef* hardware = (SPI_TypeDef *) instance->spi_base;

  uint32_t const sr = hardware->SR;

  instance->cr1_value = hardware->CR1;
  instance->cr2_value = hardware->CR2;

  uint8_t* ptxdr = (uint8_t*) &hardware->TXDR;

  uint32_t ifcr = 0UL;

  // SUSP: Master mode suspended
  // In Master mode, SUSP is set by hardware when a CSUSP request is done.
  if ( READ_BIT(sr, SPI_SR_SUSP ) != 0 )
  {
    ifcr |= SPI_IFCR_SUSPC;
  }

  // CRCE: CRC Error
  if ( READ_BIT(sr, SPI_SR_CRCE) != 0 )
  {
    instance->fsm_state = H7SPI_FSM_STATE_ERROR_CRCE;
    SET_BIT(instance->cr1_value, SPI_CR1_CSUSP );
    ifcr |= SPI_IFCR_CRCEC;
  }

  // TIFRE: TI Frame Format Error
  if ( READ_BIT(sr, SPI_SR_TIFRE) != 0 )
  {
    instance->fsm_state = H7SPI_FSM_STATE_ERROR_TIFRE;
    SET_BIT(instance->cr1_value, SPI_CR1_CSUSP );
    ifcr |= SPI_IFCR_TIFREC;
  }

  // MODF: Mode Fault Error
  if ( READ_BIT(sr, SPI_SR_MODF) != 0 )
  {
    instance->fsm_state = H7SPI_FSM_STATE_ERROR_MODF;
    SET_BIT(instance->cr1_value, SPI_CR1_CSUSP );
    ifcr |= SPI_IFCR_MODFC;
  }

  // TSERF: TSER value transferred to TSIZE (new value may be loaded to TSER)
  if ( READ_BIT(sr, SPI_SR_TSERF) != 0 )
  {
    ifcr |= SPI_IFCR_TSERFC;
  }

  // Data received in RxFIFO (one data packet available - FIFO threshold)
  // RXP flag is changed by hardware. It monitors number of overall data currently available at
  // RxFIFO if SPI is enabled. It has to be checked once a data packet is completely read out from RxFIFO.
  if ( READ_BIT(sr, SPI_SR_RXP) != 0 )
  {
    if ( instance->fsm_state == H7SPI_FSM_STATE_SHIFTING )
    {
      while ( READ_BIT(hardware->SR, SPI_SR_RXP) != 0 )
      {
        if ( instance->shift_rx_cont < instance->shift_size)
        {
          instance->rd_data[instance->shift_rx_cont] = (uint8_t) (0x000000FF & READ_REG(hardware->RXDR));
          instance->shift_rx_cont++;
        }
        else
          break;
      }
    }
  }

  // TxFIFO ready to be loaded (space available for the application to push at least one complete data packet)
  // This flag is changed by hardware. It has to be checked once a complete data packet is stored at TxFIFO
  if ( READ_BIT(sr, SPI_SR_TXP) != 0 )
  {
    if ( instance->fsm_state == H7SPI_FSM_STATE_SHIFTING )
    {
      while ( READ_BIT(hardware->SR, SPI_SR_TXP) != 0 )
      {
        if ( instance->shift_tx_cont < instance->shift_size)
        {
          *ptxdr = instance->wr_data[instance->shift_tx_cont];

          //hardware->TXDR = instance->wr_data[instance->shift_tx_cont];
          instance->shift_tx_cont++;
        }
        else
          break;
      }
    }
  }

  // DXP: Both TXP and RXP are active
  if ( READ_BIT(sr, SPI_SR_DXP) != 0 )
  {
    // TODO
  }

  // TXTF: Transmission Transfer Filled
  if ( READ_BIT(sr, SPI_SR_TXTF) != 0 )
  {
    ifcr |= SPI_IFCR_TXTFC;
  }

  // OVR: overrun condition.
  if ( READ_BIT(sr, SPI_SR_UDR) != 0 )
  {
    instance->fsm_state = H7SPI_FSM_STATE_ERROR_UDR;
    SET_BIT(instance->cr1_value, SPI_CR1_CSUSP );
    ifcr |= SPI_IFCR_UDRC;
  }

  // UDR: underrun condition.
  if ( READ_BIT(sr, SPI_SR_OVR) != 0 )
  {
    instance->fsm_state = H7SPI_FSM_STATE_ERROR_OVR;
    SET_BIT(instance->cr1_value, SPI_CR1_CSUSP );
    ifcr |= SPI_IFCR_OVRC;
  }

  // Commit to the hardware (CR1, ICR registers) the state changer.
  MODIFY_REG(hardware->CR1, SPI_CR1_CSUSP, instance->cr1_value);
  READ_REG(hardware->CR1);

  // EOT: End of Transfer is set by hardware as soon as a full transfer is completed,
  //      that is when TSIZE number of data have been transmitted and/or received on the SPI.
  if ( READ_BIT(sr, SPI_SR_EOT) != 0 )
  {
    instance->fsm_state = H7SPI_FSM_STATE_IDLE;
    h7spi_spi_mutex_release_fromISR(peripheral);
    // Read the last data from RxFIFO
    ifcr |= SPI_IFCR_EOTC;
    // Disable SPI interrupt
    hardware->IER = 0;
    READ_REG(hardware->IER);
  }

  hardware->IFCR = ifcr;
  READ_REG(hardware->IFCR);
}


static int h7spi_spi_pre_transaction_check(h7spi_periph_t peripheral, uint32_t timeout)
{
  h7spi_driver_instance_state_t* instance = h7spi_get_driver_instance(peripheral);

  if (!instance)
    return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;

  // Do not run if the driver is in error state
  switch (instance->fsm_state)
  {
    case H7SPI_FSM_STATE_ERROR_UDR:
    case H7SPI_FSM_STATE_ERROR_OVR:
    case H7SPI_FSM_STATE_ERROR_CRCE:
    case H7SPI_FSM_STATE_ERROR_TIFRE:
    case H7SPI_FSM_STATE_ERROR_MODF:
      return H7SPI_RET_CODE_PERIPH_IN_ERR_STATE;
    default:
      break;
  }

  // Lazy initialization. This branch usually should happen just once
  if (instance->fsm_state == H7SPI_FSM_STATE_UNINITIALIZED)
    h7spi_spi_init(peripheral);

  instance->fsm_state = H7SPI_FSM_STATE_SETUP_TRANSFER;

  return H7SPI_RET_CODE_OK;
}

static int h7spi_spi_master_shift_transaction(h7spi_periph_t peripheral, uint16_t shift_size, uint8_t *mi_buf, uint8_t *mo_buf, uint32_t timeout)
{
  if ( (shift_size == 0UL) || (mi_buf == NULL) || (mo_buf == NULL))
    return H7SPI_RET_CODE_INVALID_ARGS;

  h7spi_driver_instance_state_t* instance = h7spi_get_driver_instance(peripheral);

  if (!instance)
    return H7SPI_RET_CODE_UNMANAGED_BY_DRIVER;

  if(h7spi_spi_mutex_lock(peripheral, timeout) != H7SPI_RET_CODE_OK)
    return H7SPI_RET_CODE_BUSY;

  int const check_ret_val = h7spi_spi_pre_transaction_check(peripheral,timeout);

  if ( check_ret_val != H7SPI_RET_CODE_OK)
  {
    h7spi_spi_mutex_release(peripheral);
    return check_ret_val;
  }

  instance->timestart     = HAL_GetTick();
  instance->timeout       = timeout;

  instance->shift_size    = shift_size;
  instance->shift_tx_cont = 0;
  instance->shift_rx_cont = 0;
  instance->rd_data       = mi_buf;
  instance->wr_data       = mo_buf;


  // Update FSM state
  instance->fsm_state = H7SPI_FSM_STATE_SETUP_TRANSFER;

  // Default value for CFG1
  instance->cfg1_value =
    ( (7UL << SPI_CFG1_MBR_Pos    ) & SPI_CFG1_MBR      ) | // MBR       = SPI master clock/4 (default)
    ( (0UL << SPI_CFG1_CRCEN_Pos  ) & SPI_CFG1_CRCEN    ) | // CRCEN     = CRC calculation disabled
    ( (7UL << SPI_CFG1_CRCSIZE_Pos) & SPI_CFG1_CRCSIZE  ) | // CRCSIZE   = length of CRC frame to be transacted and compared
    ( (0UL << SPI_CFG1_TXDMAEN_Pos) & SPI_CFG1_TXDMAEN  ) | // TXDMAEN   = Tx DMA disable
    ( (0UL << SPI_CFG1_RXDMAEN_Pos) & SPI_CFG1_RXDMAEN  ) | // RXDMAEN   = Rx DMA disable
    ( (0UL << SPI_CFG1_UDRDET_Pos ) & SPI_CFG1_UDRDET   ) | // UDRDET    = underrun is detected at end of the last data frame
    ( (0UL << SPI_CFG1_UDRCFG_Pos ) & SPI_CFG1_UDRCFG   ) | // UDRCFG    = behavior of salve transmitter at underrun condition. It send a constant pattern defined by the user
    ( (0UL << SPI_CFG1_FTHLV_Pos  ) & SPI_CFG1_FTHLV    ) | // FTHLV     = number of data frames at single data packet
    ( (0UL << SPI_CFG1_DSIZE_Pos  ) & SPI_CFG1_DSIZE    ) ; // DSIZE     = bits length in a single SPI data frame

  // Default value for CFG2
  instance->cfg2_value =
    ( (1UL << SPI_CFG2_AFCNTR_Pos ) & SPI_CFG2_AFCNTR   ) | // AFCNTR    = the peripheral keeps always control of all associated GPIO
    ( (0UL << SPI_CFG2_SSOM_Pos   ) & SPI_CFG2_SSOM     ) | // SSOM      = SS is kept at active level till data transfer is completed, it becomes inactive with EOT flag
    ( (1UL << SPI_CFG2_SSOE_Pos   ) & SPI_CFG2_SSOE     ) | // SSOE      = SS output is enabled. The SPI cannot work in a multi-master enviroment
    ( (0UL << SPI_CFG2_SSIOP_Pos  ) & SPI_CFG2_SSIOP    ) | // SSIOP     = low level is active for SS signal
    ( (0UL << SPI_CFG2_SSM_Pos    ) & SPI_CFG2_SSM      ) | // SSM       = SS input value is determined by the PAD
    ( (0UL << SPI_CFG2_CPOL_Pos   ) & SPI_CFG2_CPOL     ) | // CPOL      = SCK signal is at 0 when idle
    ( (0UL << SPI_CFG2_CPHA_Pos   ) & SPI_CFG2_CPHA     ) | // CPHA      = the first clock transition is the first data capture edge
    ( (0UL << SPI_CFG2_LSBFRST_Pos) & SPI_CFG2_LSBFRST  ) | // LSBFRST   = MSB transmitted first
    ( (1UL << SPI_CFG2_MASTER_Pos ) & SPI_CFG2_MASTER   ) | // MASTER    = SPI Master
    ( (0UL << SPI_CFG2_SP_Pos     ) & SPI_CFG2_SP       ) | // SP        = SPI Motorola
    ( (0UL << SPI_CFG2_COMM_Pos   ) & SPI_CFG2_COMM     ) | // COMM      = full-duplex
    ( (0UL << SPI_CFG2_MIDI_Pos   ) & SPI_CFG2_MIDI     ) | // MIDI      = no delay between two consecutive data frames in master mode
    ( (0UL << SPI_CFG2_MSSI_Pos   ) & SPI_CFG2_MSSI     ) ; // MSSI      = no delay inserted between active edge of SS and first transaction in master mode when SSOE is enable

  // Default value for IER
  instance->ier_value =
    ( (1UL << SPI_IER_TSERFIE_Pos ) & SPI_IER_TSERFIE  ) | // TSERFIE   = interrupt enable for additional number of transaction reaload interrupt enable
    ( (1UL << SPI_IER_MODFIE_Pos  ) & SPI_IER_MODFIE   ) | // MODFIE    = interrupt enable for mode fault occurs when the master device has its internal SS signal pulled down.
    ( (0UL << SPI_IER_TIFREIE_Pos ) & SPI_IER_TIFREIE  ) | // TIFREIE   = interrupt disable for TI frame format error
    ( (0UL << SPI_IER_CRCEIE_Pos  ) & SPI_IER_CRCEIE   ) | // CRCEIE    = interrupt disable for CRC error
    ( (0UL << SPI_IER_OVRIE_Pos   ) & SPI_IER_OVRIE    ) | // OVRIE     = interrupt disable for overrrun
    ( (0UL << SPI_IER_UDRIE_Pos   ) & SPI_IER_UDRIE    ) | // UDRIE     = interrupt disable for underrun
    ( (1UL << SPI_IER_TXTFIE_Pos  ) & SPI_IER_TXTFIE   ) | // TXTFIE    = interrupt enable for transmission transfer filled
    ( (1UL << SPI_IER_EOTIE_Pos   ) & SPI_IER_EOTIE    ) | // EOTIE     = interrupt enable for End of Transfer, master mode suspended or TxFIFO empty
    ( (1UL << SPI_IER_DXPIE_Pos   ) & SPI_IER_DXPIE    ) | // DXPIE     = interrupt enable for both TXP and RXP active
    ( (1UL << SPI_IER_TXPIE_Pos   ) & SPI_IER_TXPIE    ) | // TXPIE     = interrupt enable for TxFIFO ready to be loaded (space available for one data packet)
    ( (1UL << SPI_IER_RXPIE_Pos   ) & SPI_IER_RXPIE    ) ; // RXPIE     = interrupt enable for data received in RxFIFO (one data packet available )

  // CR2 value according to custom transfer.
  instance->cr2_value =
    ( (0UL << SPI_CR2_TSER_Pos  ) & SPI_CR2_TSER      ) | // TSER      = no extensions to be reload into TSIZE
    ( (0UL << SPI_CR2_TSIZE_Pos ) & SPI_CR2_TSIZE     ) ; // TSIZE     = number of data transfer

  // Default value for CR1
  instance->cr1_value =
    ( (0UL << SPI_CR1_SPE_Pos      ) & SPI_CR1_SPE       ) | // SPE       = peripheral enable
    ( (0UL << SPI_CR1_MASRX_Pos    ) & SPI_CR1_MASRX     ) | // MASRX     = SPI flow is suspended temporary on RxFIFO full condition, before reaching overrrun
    ( (0UL << SPI_CR1_CSTART_Pos   ) & SPI_CR1_CSTART    ) | // CSTART    = master transfer is at idle. This bit is set by software and cleared by hardware
    ( (0UL << SPI_CR1_CSUSP_Pos    ) & SPI_CR1_CSUSP     ) | // CSUSP     = in master mode, when this bit is set by software, the SPI communication is suspended
    ( (0UL << SPI_CR1_HDDIR_Pos    ) & SPI_CR1_HDDIR     ) | // HDDIR     = transmitter. This bit is ignored in Full-Duplex or any Simplex configuration
    ( (0UL << SPI_CR1_SSI_Pos      ) & SPI_CR1_SSI       ) | // SSI       = this bit has an effect only when the SSM bit is set
    ( (0UL << SPI_CR1_CRC33_17_Pos ) & SPI_CR1_CRC33_17  ) | // CRC33_17  = full size (33-bit or 17-bit) CRC polynominal is not used
    ( (0UL << SPI_CR1_RCRCINI_Pos  ) & SPI_CR1_RCRCINI   ) | // RCRCINI   = all zero pattern is applied for CRC calculation initialization pattern control for receiver
    ( (0UL << SPI_CR1_TCRCINI_Pos  ) & SPI_CR1_TCRCINI   ) | // TCRCINI   = all zero pattern is applied for CRC calculation initialization pattern control for transmitter
    ( (0UL << SPI_CR1_IOLOCK_Pos   ) & SPI_CR1_IOLOCK    ) ; // IOLOCK    = AF configuration is not locked

    MODIFY_REG(instance->cr2_value, SPI_CR2_TSIZE, (instance->shift_size << SPI_CR2_TSIZE_Pos));

    // Compute the packet size to define FIFO threshold vale
    // In this driver the data frame will be 8 bits.
    uint8_t fifo_deph = instance->fifo_deph;

    uint32_t tSIZE; // keeps the amount of data packets to be shifted. The data packet is defined by the FIFO's Threshold.
                    // The data packets are composed by data frames.
    uint32_t tSER;  // in the actual shift transaction, defines the amount of data frames which is not enough to complete a packet and reach FIFO's Threshold.

    uint8_t fTHLV;  // keeps FIFO' Threshold value. The size of the packet must not exceed 1/2 of FIFO space.
    uint8_t max_fifo_th = ( fifo_deph >> 1 ); // Defines maximum number of data frames at single data packet.

    if ( shift_size > max_fifo_th )
    {
      // The amount of data frames to shift is greater than FIFO Threshold.
      tSIZE = (uint16_t) ( shift_size / max_fifo_th );
      tSER  = (uint16_t) ( shift_size % max_fifo_th );
      fTHLV = max_fifo_th-1;
    }
    else
    {
      // The amount of data frames to shift is lower than maximum FIFO's Threshold value.
      tSIZE = (uint32_t) shift_size;
      tSER  = (uint32_t) 0;
      fTHLV = (uint8_t)  (shift_size-1);
    }

    // Fixed shift parameter for bits length in a single SPI data frame.
    uint8_t dSIZE = 7;
    MODIFY_REG(instance->cfg1_value,SPI_CFG1_DSIZE, dSIZE << SPI_CFG1_DSIZE_Pos);
    MODIFY_REG(instance->cfg1_value,SPI_CFG1_FTHLV, fTHLV << SPI_CFG1_FTHLV_Pos);

    // Write shift-specific parameters on instance register
    MODIFY_REG(instance->cr2_value, SPI_CR2_TSIZE, tSIZE << SPI_CR2_TSIZE_Pos);
    MODIFY_REG(instance->cr2_value,SPI_CR2_TSER, tSER << SPI_CR2_TSER_Pos);


    // Commit the new register values to hardware
    SPI_TypeDef* hardware = (SPI_TypeDef*) instance->spi_base;

    hardware->CFG1 = instance->cfg1_value;
    READ_REG(hardware->CFG1);

    hardware->CFG2 = instance->cfg2_value;
    READ_REG(hardware->CFG2);

    hardware->CR2  = instance->cr2_value;
    READ_REG(hardware->CR2);

    hardware->CR1  = instance->cr1_value;
    READ_REG(hardware->CR1);

    hardware->IFCR = 0UL;


    SET_BIT(hardware->CR1, SPI_CR1_SPE);
    READ_REG(hardware->CR1);

    SET_BIT(hardware->CR1,SPI_CR1_CSTART);
    READ_REG(hardware->CR1);

    instance->fsm_state = H7SPI_FSM_STATE_SHIFTING;

    hardware->IER  = instance->ier_value;
        READ_REG(hardware->IER);

    return H7SPI_RET_CODE_OK;
}



////////////////////////////
//                        //
// NON-BLOCKING FUNCTIONS //
//                        //
////////////////////////////
h7spi_spi_ret_code_t  h7spi_spi_master_shift_nonblocking(h7spi_periph_t peripheral, uint16_t shift_size,  uint8_t* mi_buf, uint8_t* mo_buf, uint32_t timeout)
{
  return h7spi_spi_master_shift_transaction(peripheral,shift_size,mi_buf,mo_buf,timeout);
}


////////////////////////
//                    //
// BLOCKING FUNCTIONS //
//                    //
////////////////////////
h7spi_spi_ret_code_t h7spi_spi_master_shift_blocking(h7spi_periph_t peripheral, uint16_t shift_size, uint8_t *mi_buf, uint8_t *mo_buf, uint32_t timeout)
{
  h7spi_spi_ret_code_t ret = h7spi_spi_master_shift_nonblocking(peripheral, shift_size,mi_buf,mo_buf,timeout);

  if (ret != H7SPI_RET_CODE_OK)
  {
    h7spi_wait_until_ready(peripheral, timeout);
    return ret;
  }
  else
  {
    return  h7spi_wait_until_ready(peripheral,timeout);
  }
}





