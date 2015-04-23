/*
 * global.h
 *
 *  Created on: Dec 3, 2013
 *      Author: kwgilpin
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>
#include <stdbool.h>

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         	/**< The advertising timeout (in units of seconds). */

#define SECOND_1_25_MS_UNITS            800                                         /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                         /**< Definition of 1 second, when 1 unit is 10 ms. */

#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 100)                /**< Minimum acceptable connection interval (10 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 10)                 /**< Maximum acceptable connection interval (100 ms), Connection interval uses 1.25 ms units. */

#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                    /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_TIMER_PRESCALER             9                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            8                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         1/*5*/                                      /**< Size of timer operation queues. */

#define USEC_PER_APP_TIMER_TICK			((uint32_t)ROUNDED_DIV((APP_TIMER_PRESCALER + 1) * (uint64_t)1000000, (uint64_t)APP_TIMER_CLOCK_FREQ))

#define MPU6050_I2C_ADDR_CENTRAL		(0x68)
#define MPU6050_I2C_ADDR_FACE			(0x69)

#define PWM_CH0_RESET_PPI_CHANNEL		(1)
#define PWM_CH0_RESET_PPI_CHENSET_MASK	(PPI_CHENSET_CH1_Msk)
#define PWM_CH0_RESET_PPI_CHENCLR_MASK	(PPI_CHENCLR_CH1_Msk)
#define PWM_CH0_RESET_PPI_CHEN_MASK		(PPI_CHEN_CH1_Msk)

#define PWM_CH0_MATCH_PPI_CHANNEL		(2)
#define PWM_CH0_MATCH_PPI_CHENSET_MASK	(PPI_CHENSET_CH2_Msk)
#define PWM_CH0_MATCH_PPI_CHENCLR_MASK	(PPI_CHENCLR_CH2_Msk)
#define PWM_CH0_MATCH_PPI_CHEN_MASK		(PPI_CHEN_CH2_Msk)

#define PWM_CH1_RESET_PPI_CHANNEL		(3)
#define PWM_CH1_RESET_PPI_CHENSET_MASK	(PPI_CHENSET_CH3_Msk)
#define PWM_CH1_RESET_PPI_CHENCLR_MASK	(PPI_CHENCLR_CH3_Msk)
#define PWM_CH1_RESET_PPI_CHEN_MASK		(PPI_CHEN_CH3_Msk)

#define PWM_CH1_MATCH_PPI_CHANNEL		(4)
#define PWM_CH1_MATCH_PPI_CHENSET_MASK	(PPI_CHENSET_CH4_Msk)
#define PWM_CH1_MATCH_PPI_CHENCLR_MASK	(PPI_CHENCLR_CH4_Msk)
#define PWM_CH1_MATCH_PPI_CHEN_MASK		(PPI_CHEN_CH4_Msk)

#define	TWI_MASTER_PPI_CHANNEL			(5)
#define TWI_MASTER_PPI_CHENSET_MASK		PPI_CHENSET_CH5_Msk
#define TWI_MASTER_PPI_CHENCLR_MASK		PPI_CHENCLR_CH5_Msk
#define TWI_MASTER_PPI_CHEN_MASK		PPI_CHEN_CH5_Msk

#define	FREQCNTR_PPI_CHANNEL			(7)
#define FREQCNTR_PPI_CHENSET_MASK		PPI_CHENSET_CH7_Msk
#define FREQCNTR_PPI_CHENCLR_MASK		PPI_CHENCLR_CH7_Msk
#define FREQCNTR_PPI_CHEN_MASK			PPI_CHEN_CH7_Msk

#define PWM_CH0_GPIOTE_CHANNEL			(0)
#define PWM_CH1_GPIOTE_CHANNEL			(1)
#define FREQCNTR_GPIOTE_CHANNEL			(3)

#endif /* GLOBAL_H_ */

