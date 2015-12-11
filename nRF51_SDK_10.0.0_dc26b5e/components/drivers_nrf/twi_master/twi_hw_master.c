/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "global.h"
#include "twi_master.h"
#include "twi_master_config.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "app_error.h"

/* Max cycles approximately to wait on RXDREADY and TXDREADY event, 
 * This is optimized way instead of using timers, this is not power aware. */
#define MAX_TIMEOUT_LOOPS             (20000UL)        /**< MAX while loops to wait for RXD/TXD event */

static bool initialized = false;

static bool twi_master_write(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
    uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for EVENTS_TXDSENT event*/

    if (data_length == 0)
    {
        /* Return false for requesting data of size 0 */
        return false;
    }

    NRF_TWI1->TXD           = *data++;
    NRF_TWI1->TASKS_STARTTX = 1;

    /** @snippet [TWI HW master write] */            
    while (true)
    {
        while(NRF_TWI1->EVENTS_TXDSENT == 0 && NRF_TWI1->EVENTS_ERROR == 0 && (--timeout))
        {
            // Do nothing.
        }

        if (timeout == 0)
        {
          // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at 
          // Product Anomaly Notification document found at 
          // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
          NRF_TWI1->EVENTS_ERROR = 0;
          NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos; 
          NRF_TWI1->POWER        = 0; 
          nrf_delay_us(5); 
          NRF_TWI1->POWER        = 1; 
          NRF_TWI1->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

          (void)twi_master_init();          

          return false;
        }

        /* We sent a byte, so decrement the counter */
        data_length--;

        /* Clear the TXDSENT event */
        NRF_TWI1->EVENTS_TXDSENT = 0;

        /* If the slave NACK'd, we will not send any additional bytes */
        if (NRF_TWI1->EVENTS_ERROR != 0) {
        	/* Clear the address and data NACK flags */
        	NRF_TWI1->ERRORSRC = TWI_ERRORSRC_ANACK_Clear << TWI_ERRORSRC_ANACK_Pos;
        	NRF_TWI1->ERRORSRC = TWI_ERRORSRC_DNACK_Clear << TWI_ERRORSRC_DNACK_Pos;
        	/* Clear the error event flag */
        	NRF_TWI1->EVENTS_ERROR = 0;
        	/* Break, instead of return, so that we'll still send a stop
        	 * condition. */
        	break;
        }

        /* Check whether we've sent all bytes... */
        if (data_length == 0) {
            break;
        }

        /* If not, send the next */
        NRF_TWI1->TXD = *data++;
    }
    /** @snippet [TWI HW master write] */            
    
    if (issue_stop_condition) {
        NRF_TWI1->EVENTS_STOPPED = 0; 
        NRF_TWI1->TASKS_STOP     = 1; 
        /* Wait until stop sequence is sent */ 
        timeout = MAX_TIMEOUT_LOOPS;
        while((NRF_TWI1->EVENTS_STOPPED == 0) && (--timeout));
        /* Now that the stop condition has been sent, clear the STOPPED
         * event flag. */
        NRF_TWI1->EVENTS_STOPPED = 0;
    }

    /* We only return success to the caller if the number bytes sent before we
     * received a NACK matches the number bytes the caller instructed us to
     * transmit. */
    if (data_length == 0) {
    	return true;
    }

    return false;
}


/** @brief Function for read by twi_master. 
 */
static bool twi_master_read(uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
	uint32_t err_code;
	uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for RXDREADY event*/
	uint8_t softdevice_enabled;

	err_code = sd_softdevice_is_enabled(&softdevice_enabled);
	APP_ERROR_CHECK(err_code);

    if (data_length == 0)
    {
        /* Return false for requesting data of size 0 */
        return false;
    }
    else if ((data_length == 1) && issue_stop_condition)
    {
    	if (softdevice_enabled) {
    		err_code = sd_ppi_channel_assign(TWI_MASTER_PPI_CHANNEL,
    				&(NRF_TWI1->EVENTS_BB), &(NRF_TWI1->TASKS_STOP));
    		APP_ERROR_CHECK(err_code);
    	} else {
    		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TWI1->EVENTS_BB);
    		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].TEP = (uint32_t)&(NRF_TWI1->TASKS_STOP);
    	}

    }
    else
    {
    	if (softdevice_enabled) {
    		err_code = sd_ppi_channel_assign(TWI_MASTER_PPI_CHANNEL,
    				&(NRF_TWI1->EVENTS_BB), &(NRF_TWI1->TASKS_SUSPEND));
    		APP_ERROR_CHECK(err_code);
    	} else {
    		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TWI1->EVENTS_BB);
    		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].TEP = (uint32_t)&(NRF_TWI1->TASKS_SUSPEND);
    	}
    }
    
    if (softdevice_enabled) {
    	err_code = sd_ppi_channel_enable_set(TWI_MASTER_PPI_CHEN_MASK);
    	APP_ERROR_CHECK(err_code);
    } else {
    	NRF_PPI->CHENSET = TWI_MASTER_PPI_CHEN_MASK;
    }

    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->TASKS_STARTRX   = 1;
    
    /** @snippet [TWI HW master read] */                
    while (true)
    {
        while(NRF_TWI1->EVENTS_RXDREADY == 0 && NRF_TWI1->EVENTS_ERROR == 0 && (--timeout))
        {    
            /* Do nothing while waiting for the RXDREADY event (meaning that a
        	 * new byte is available). */
        }
        NRF_TWI1->EVENTS_RXDREADY = 0;

        //if (timeout == 0 || NRF_TWI1->EVENTS_ERROR != 0)
        if (timeout == 0) {
			// Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at
			// Product Anomaly Notification document found at
			// https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
			NRF_TWI1 ->EVENTS_ERROR = 0;
			NRF_TWI1 ->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
			NRF_TWI1 ->POWER = 0;
			nrf_delay_us(5);
			NRF_TWI1 ->POWER = 1;
			NRF_TWI1 ->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

			(void)twi_master_init();
          
			return false;
        } else if (NRF_TWI1->EVENTS_ERROR != 0) {
        	/* If we detected an error (which as far as the NRF is concerned,
        	 * means that the master received a NACK in place of an ACK, we
        	 * abort the transaction.  Since we (the master) are currently
        	 * reading from the slave, the only possible NACK is the NACK the
        	 * slaves would send (tacitly) in response to non-matching
        	 * addresses.  If no slave ACKs the address we sent, we will not
        	 * attempt to read additional bytes. */

        	/* First, clear the address and data NACK flags */
        	NRF_TWI1->ERRORSRC = (TWI_ERRORSRC_ANACK_Clear << TWI_ERRORSRC_ANACK_Pos);
        	NRF_TWI1->ERRORSRC = (TWI_ERRORSRC_DNACK_Clear << TWI_ERRORSRC_DNACK_Pos);
        	/* Clear the error event flag */
        	NRF_TWI1->EVENTS_ERROR = 0;

        	/* Issue a stop condition if requested by the caller */
            if (issue_stop_condition) {
                NRF_TWI1->EVENTS_STOPPED = 0;
                NRF_TWI1->TASKS_STOP     = 1;
                /* Wait until stop sequence is sent */
                timeout = MAX_TIMEOUT_LOOPS;
                while((NRF_TWI1->EVENTS_STOPPED == 0) && (--timeout));
                /* Clear the STOPPED event flag */
                NRF_TWI1->EVENTS_STOPPED = 0;
            }

            /* Disable the PPI channel used to trigger the suspend or stop
             * task at every byte boundary. */
            if (softdevice_enabled) {
            	err_code = sd_ppi_channel_enable_clr(TWI_MASTER_PPI_CHEN_MASK);
            	APP_ERROR_CHECK(err_code);
            } else {
            	NRF_PPI->CHENCLR = TWI_MASTER_PPI_CHEN_MASK;
            }

            return false;
        }

        *data++ = NRF_TWI1->RXD;

        /* Configure PPI to stop TWI master before we get last BB event */
        if ((--data_length == 1) && issue_stop_condition)
        {
        	if (softdevice_enabled) {
        		err_code = sd_ppi_channel_assign(TWI_MASTER_PPI_CHANNEL,
        				&(NRF_TWI1->EVENTS_BB), &(NRF_TWI1->TASKS_STOP));
        		APP_ERROR_CHECK(err_code);
        	} else {
        		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TWI1->EVENTS_BB);
        		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].TEP = (uint32_t)&(NRF_TWI1->TASKS_STOP);
        	}
        }

        if (data_length == 0)
        {
            break;
        }

        // Recover the peripheral as indicated by PAN 56: "TWI: TWI module lock-up." found at 
        // Product Anomaly Notification document found at 
        // https://www.nordicsemi.com/eng/Products/Bluetooth-R-low-energy/nRF51822/#Downloads
        nrf_delay_us(20);      
        NRF_TWI1->TASKS_RESUME = 1;
    }
    /** @snippet [TWI HW master read] */                    

    /* Wait until stop sequence is sent */
    while((NRF_TWI1->EVENTS_STOPPED == 0) && issue_stop_condition)
    {
        // Do nothing.
    }
    NRF_TWI1->EVENTS_STOPPED = 0;

    if (softdevice_enabled) {
    	err_code = sd_ppi_channel_enable_clr(TWI_MASTER_PPI_CHEN_MASK);
    	APP_ERROR_CHECK(err_code);
    } else {
    	NRF_PPI->CHENCLR = TWI_MASTER_PPI_CHEN_MASK;
    }

    return true;
}


/**
 * @brief Function for detecting stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
static bool twi_master_clear_bus(void)
{
    uint32_t twi_state;
    bool bus_clear;
    uint32_t clk_pin_config;
    uint32_t data_pin_config;
        
    // Save and disable TWI hardware so software can take control over the pins.
    twi_state        = NRF_TWI1->ENABLE;
    NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    
    clk_pin_config                                        =  \
            NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER];    
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] =   \
            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
          | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
          | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
          | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
          | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    

    data_pin_config                                      = \
        NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER];
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] = \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    
      
    TWI_SDA_HIGH();
    TWI_SCL_HIGH();
    TWI_DELAY();

    if ((TWI_SDA_READ() == 1) && (TWI_SCL_READ() == 1))
    {
        bus_clear = true;
    }
    else
    {
        uint_fast8_t i;
        bus_clear = false;

        // Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9 
        // for slave to respond) to SCL line and wait for SDA come high.
        for (i=18; i--;)
        {
            TWI_SCL_LOW();
            TWI_DELAY();
            TWI_SCL_HIGH();
            TWI_DELAY();

            if (TWI_SDA_READ() == 1)
            {
                bus_clear = true;
                break;
            }
        }
    }
    
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] = clk_pin_config;
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER]  = data_pin_config;

    NRF_TWI1->ENABLE = twi_state;

    return bus_clear;
}


/** @brief Function for initializing the twi_master.
 */
bool twi_master_init(void)
{
	uint32_t err_code;
	uint8_t softdevice_enabled;

    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is 
       disabled, these pins must be configured in the GPIO peripheral.
    */
    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] =     \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);   

    NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] =      \
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) \
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->EVENTS_TXDSENT  = 0;
    NRF_TWI1->PSELSCL         = TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER;
    NRF_TWI1->PSELSDA         = TWI_MASTER_CONFIG_DATA_PIN_NUMBER;
    NRF_TWI1->FREQUENCY       = TWI_FREQUENCY_FREQUENCY_K100 << TWI_FREQUENCY_FREQUENCY_Pos;


	/* Whether or not the softdevice is enabled determines how we configure the
	 * PPI channels.  Either way, the final configuration is the same. */
    err_code = sd_softdevice_is_enabled(&softdevice_enabled);
    APP_ERROR_CHECK(err_code);

    if (softdevice_enabled) {
    	err_code = sd_ppi_channel_assign(TWI_MASTER_PPI_CHANNEL,
    			&(NRF_TWI1->EVENTS_BB), &(NRF_TWI1->TASKS_SUSPEND));
    	APP_ERROR_CHECK(err_code);

    	err_code = sd_ppi_channel_enable_clr(TWI_MASTER_PPI_CHEN_MASK);
    	APP_ERROR_CHECK(err_code);
    } else {
		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].EEP = (uint32_t)&(NRF_TWI1->EVENTS_BB);
		NRF_PPI->CH[TWI_MASTER_PPI_CHANNEL].TEP = (uint32_t)&(NRF_TWI1->TASKS_SUSPEND);
		NRF_PPI->CHENCLR = TWI_MASTER_PPI_CHENCLR_MASK;
    }

    NRF_TWI1->ENABLE          = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

    initialized = twi_master_clear_bus();

    return initialized;
}


void twi_master_deinit() {
	uint32_t err_code;
	uint8_t softdevice_enabled;

	/* Whether or not the softdevice is enabled determines how we disable the
	 * PPI channels. */
	err_code = sd_softdevice_is_enabled(&softdevice_enabled);
	APP_ERROR_CHECK(err_code);

	/* Disable the TWI's PPI channels */
	if (softdevice_enabled) {
    	err_code = sd_ppi_channel_enable_clr(TWI_MASTER_PPI_CHEN_MASK);
    	APP_ERROR_CHECK(err_code);
	} else {
		NRF_PPI->CHENCLR = TWI_MASTER_PPI_CHENCLR_MASK;
	}

	/* Disable the TWI interface, primarily so that it does not consume power
	 * or keep the 16MHz clock active when the nRF is in idle mode. */
	NRF_TWI1->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);

	/* Ensure that the SCL and SDA pins are inputs.  We assume that they do not
	 * need to be pulled-up as there should be external pull-up resistors on
	 * the I2C bus. */
	nrf_gpio_cfg_input(TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(TWI_MASTER_CONFIG_DATA_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

	initialized = false;
}

bool twi_master_get_init() {
	return initialized;
}

/** @brief  Function for transfer by twi_master.
 */ 
bool twi_master_transfer(uint8_t   address, 
                         uint8_t * data, 
                         uint8_t   data_length, 
                         bool      issue_stop_condition)
{
    bool transfer_succeeded = false;
    if (data_length > 0 && twi_master_clear_bus())
    {
        NRF_TWI1->ADDRESS = (address >> 1);

        if ((address & TWI_READ_BIT))
        {
            transfer_succeeded = twi_master_read(data, data_length, issue_stop_condition);
        }
        else
        {
            transfer_succeeded = twi_master_write(data, data_length, issue_stop_condition);
        }
    }
    return transfer_succeeded;
}

/*lint --flb "Leave library region" */
