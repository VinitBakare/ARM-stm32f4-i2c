#include<stdint.h>
#include "stm32f4xx.h"
#include "hal_driver.h"
#include "led.h"
#include "stm32f4xx_hal_i2c.h"

static void I2C_MasterTransmit_TXE(i2c_handle_t *hi2c)
{
  /* Write data to DR */
  hi2c->Instance->DR = (*hi2c->pBuffPtr++);
  hi2c->XferCount--;

  if(hi2c->XferCount == 0)
  {
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
  }

}
 void HAL_I2C_MemTxCpltCallback(i2c_handle_t *hi2c)
{
  UNUSED(hi2c);
}

  void HAL_I2C_MasterTxCpltCallback(i2c_handle_t *hi2c)
{
  UNUSED(hi2c);
}

 void HAL_I2C_MemRxCpltCallback(i2c_handle_t *hi2c)
{
  UNUSED(hi2c);
}

 void HAL_I2C_SlaveRxCpltCallback(i2c_handle_t *hi2c)
{
  UNUSED(hi2c);
}

  void HAL_I2C_SlaveTxCpltCallback(i2c_handle_t *hi2c)
{
  UNUSED(hi2c);
}

 void HAL_I2C_MasterRxCpltCallback(i2c_handle_t *hi2c)
{
  UNUSED(hi2c);
}
  

static void I2C_MasterTransmit_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    hi2c->Instance->DR = (*hi2c->pBuffPtr++);
    hi2c->XferCount--;
  }
  else
  {
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

    hi2c->Instance->CR1 |= I2C_REG_CR1_STOP_GEN;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_TX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemTxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterTxCpltCallback(hi2c);
    }
  }

}

static void I2C_MasterReceive_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount == 3)
  {
    hi2c->Instance->CR1 &= ~I2C_REG_CR1_ACK;

    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  else if(hi2c->XferCount == 2)
  {
    hi2c->Instance->CR1 |= I2C_REG_CR1_STOP_GEN;

    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;

    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;

			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_RX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemRxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterRxCpltCallback(hi2c);
    }
  }
  else
  {
    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
 
}

static void I2C_Slave_ADDR(i2c_handle_t *hi2c)
{
	uint32_t tmpreg;
    tmpreg = hi2c->Instance->SR1;  //read SR1     
    tmpreg = hi2c->Instance->SR2;  //read SR2
}

void hal_clear_stop_flag(i2c_handle_t *hi2c)
{
	 uint32_t tmpreg;
	 tmpreg = hi2c->Instance->SR1;      //reading from SR1
   hi2c->Instance->CR1 |= I2C_REG_CR1_ENABLE_I2C;  //writing to SR1
}

static void I2C_Slave_STOPF(i2c_handle_t *hi2c)
{
	
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

  hal_clear_stop_flag(hi2c);
	
	 

  hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

  hi2c->State = HAL_I2C_STATE_READY;

  HAL_I2C_SlaveRxCpltCallback(hi2c);

  
}

static void I2C_Slave_AF(i2c_handle_t *hi2c)
{
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

  hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_AF_FAILURE_FLAG);

  hi2c->Instance->CR1 &= ~I2C_REG_CR1_ACK;

  hi2c->State = HAL_I2C_STATE_READY;

  HAL_I2C_SlaveTxCpltCallback(hi2c);

 
}


static void I2C_SlaveTransmit_TXE(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    hi2c->Instance->DR = (*hi2c->pBuffPtr++);
    hi2c->XferCount--;
  }
  
}

static void I2C_SlaveTransmit_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    hi2c->Instance->DR = (*hi2c->pBuffPtr++);
    hi2c->XferCount--;
  }
  
}


static void I2C_MasterReceive_RXNE(i2c_handle_t *hi2c)
{
  uint32_t tmp = 0;

  tmp = hi2c->XferCount;
  if(tmp > 3)
  {
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  else if((tmp == 2) || (tmp == 3))
  {
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
  }
  else
  {
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_RX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemRxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterRxCpltCallback(hi2c);
    }
  }
  
}

static void I2C_SlaveReceive_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  
}

static void I2C_SlaveReceive_RXNE(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  
}


void HAL_I2C_EV_IRQHandler(i2c_handle_t *hi2c)
{
  uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0, tmp4 = 0;
  if(( hi2c->Instance->SR2 & I2C_REG_SR2_MSL_FLAG ))
  {
    /* I2C in mode Transmitter */
    if(( hi2c->Instance->SR2 & I2C_REG_SR2_TRA_FLAG))
    {
     tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_TXE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* TXE set and BTF reset */
      if(tmp1 && tmp2 && (! tmp3))
      {
        I2C_MasterTransmit_TXE(hi2c);
      }
      /* BTF set */
      else if((tmp3 && tmp4 ))
      {
        I2C_MasterTransmit_BTF(hi2c);
      }
    }
    /* I2C in mode Receiver */
    else
    {
      
			tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_RXNE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* RXNE set and BTF reset */
      if((tmp1) && (tmp2) && (! tmp3))
      {
        I2C_MasterReceive_RXNE(hi2c);
      }
      /* BTF set */
      else if((tmp3) && (tmp4))
      {
        I2C_MasterReceive_BTF(hi2c);
      }
    }
  }
  else
  {
    
		tmp1 = ( hi2c->Instance->SR1 & I2C_REG_SR1_ADDR_FLAG);
    tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
		tmp3 = ( hi2c->Instance->SR1 & I2C_REG_SR1_STOP_DETECTION_FLAG);
    tmp4 = ( hi2c->Instance->SR2 & I2C_REG_SR2_TRA_FLAG);
    /* ADDR set */
    if((tmp1 ) && (tmp2))
    {
			led_turn_on(GPIOD,LED_GREEN);
      I2C_Slave_ADDR(hi2c);
    }
    /* STOPF set */
    else if((tmp3) && (tmp2))
    {
      I2C_Slave_STOPF(hi2c);
    }
    /* I2C in mode Transmitter */
    else if(tmp4)
    {
			tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_TXE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* TXE set and BTF reset */
      if((tmp1) && (tmp2) && (! tmp3))
      {
        I2C_SlaveTransmit_TXE(hi2c);
      }
      /* BTF set */
      else if((tmp3) && (tmp4))
      {
        I2C_SlaveTransmit_BTF(hi2c);
      }
    }
    /* I2C in mode Receiver */
    else
    {
      tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_RXNE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* RXNE set and BTF reset */
      if((tmp1 ) && (tmp2 ) && (! tmp3))
      {
        I2C_SlaveReceive_RXNE(hi2c);
      }
      /* BTF set */
      else if((tmp3) && (tmp4))
      {
        I2C_SlaveReceive_BTF(hi2c);
      }
    }
  }
}

void HAL_I2C_ErrorCallback(i2c_handle_t *I2cHandle)
{
  while(1)
  {
		led_toggle(GPIOD,LED_RED);
  }
}

void HAL_I2C_ER_IRQHandler(i2c_handle_t *hi2c)
{
  uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0;
	tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_BUS_ERROR_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  if((tmp1) && (tmp2 ))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;

		hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_BUS_ERROR_FLAG);
  }

  tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_ARLO_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  if((tmp1 ) && (tmp2))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_ARLO;

    /* Clear ARLO flag */
    hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_ARLO_FLAG);
  }

  tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_AF_FAILURE_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  if((tmp1 ) && (tmp2 ))
  {
    tmp1 = ( hi2c->Instance->SR2 & I2C_REG_SR2_MSL_FLAG );
    tmp2 = hi2c->XferCount;
    tmp3 = hi2c->State;
    if(( ! tmp1 ) && (tmp2 == 0) && (tmp3 == HAL_I2C_STATE_BUSY_TX))
    {
      I2C_Slave_AF(hi2c);
    }
    else
    {
      hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
      /* Clear AF flag  */
       hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_AF_FAILURE_FLAG);
    }
  }

 tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_OVR_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  if((tmp1) && (tmp2))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_OVR;
    /* Clear OVR flag */
    hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_OVR_FLAG);
  }

  if(hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
  {
    hi2c->State = HAL_I2C_STATE_READY;
    
    hi2c->Instance->CR1 &= ~I2C_REG_CR1_POS;
    
    HAL_I2C_ErrorCallback(hi2c);
  }
}
