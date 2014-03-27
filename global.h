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

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            5                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                           /**< Size of timer operation queues. */

#define ADC_PPI_CHANNEL 				(5)
#define ADC_PPI_CHENSET_MASK			PPI_CHENSET_CH5_Msk
#define ADC_PPI_CHENCLR_MASK			PPI_CHENCLR_CH5_Msk
#define ADC_PPI_CHEN_MASK				PPI_CHEN_CH5_Msk

#define	TWI_MASTER_PPI_CHANNEL			(6)
#define TWI_MASTER_PPI_CHENSET_MASK		PPI_CHENSET_CH6_Msk
#define TWI_MASTER_PPI_CHENCLR_MASK		PPI_CHENCLR_CH6_Msk
#define TWI_MASTER_PPI_CHEN_MASK		PPI_CHEN_CH6_Msk

#define	FREQCNTR_PPI_CHANNEL			(7)
#define FREQCNTR_PPI_CHENSET_MASK		PPI_CHENSET_CH7_Msk
#define FREQCNTR_PPI_CHENCLR_MASK		PPI_CHENCLR_CH7_Msk
#define FREQCNTR_PPI_CHEN_MASK			PPI_CHEN_CH7_Msk

#endif /* GLOBAL_H_ */
