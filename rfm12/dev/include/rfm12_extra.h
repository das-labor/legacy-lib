/**** RFM 12 library for Atmel AVR Microcontrollers *******
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA.
 *
 * @author Peter Fuhrmann, Hans-Gert Dahmen, Soeren Heisrath
 */
 
/** \file rfm12_extra.h
 * \brief rfm12 library extra features header
 * \author Hans-Gert Dahmen
 * \author Peter Fuhrmann
 * \author Soeren Heisrath
 * \version 0.9.0
 * \date 08.09.09
 *
 * This header declares all stuff related to extra features.
 *
 * \note It is usually not required to explicitly include this header,
 * as this is already done by rfm12.h.
 */
 
 /******************************************************
 *                                                    *
 *    NO  C O N F I G U R A T I O N  IN THIS FILE     *
 *                                                    *
 *      ( thou shalt not change lines below )         *
 *                                                    *
 ******************************************************/
 
#ifndef _RFM12_EXTRA_H
#define _RFM12_EXTRA_H

/************************
 * amplitude modulation receive mode
*/

#if RFM12_RECEIVE_CW
	/** \name States and buffer size for the amplitude modulated receive feature.
	* \anchor cw_defines
	* \note You need to define RFM12_RECEIVE_CW as 1 to enable this.
	* \see rfrxbuf_t and ISR(ADC_vect, ISR_NOBLOCK)
	* @{
	*/
	//! The CW receive buffer size.
	#define RFM12_CW_RFRXBUF_SIZE 55
	//! The CW receive buffer is empty.
	#define RFM12_CW_STATE_EMPTY 0
	//! The CW receive buffer is active.
	#define RFM12_CW_STATE_RECEIVING 1
	//! The CW receive buffer is full.
	#define RFM12_CW_STATE_FULL 2
	//@}

	//! The receive buffer structure for the amplitude modulated receive feature.
	/** \note You need to define RFM12_RECEIVE_CW as 1 to enable this.
	* \see cw_rxbuf (for further usage instructions) and ISR(ADC_vect, ISR_NOBLOCK)
	* \headerfile rfm12.h
	*/
	typedef struct
	{
		//! A pointer into the buffer, used while receiving.
		volatile	uint8_t p;
		
		//! The buffer's state.
		/** \see See \ref cw_defines for a list of possible states */
		volatile	uint8_t state;
		
		//! The data buffer
		uint8_t 	buf[RFM12_CW_RFRXBUF_SIZE];
	} rfm12_rfrxbuf_t;

	//see rfm12_extra.c for more documentation
	extern rfrxbuf_t cw_rxbuf;
	
	//see rfm12_extra.c for more documentation	
	void adc_init();
#endif /* RFM12_RECEIVE_CW */


/************************
 * amplitude modulated raw tx mode
*/
 
#if RFM12_RAW_TX
	//see rfm12_extra.c for more documentation
	void rfm12_rawmode(uint8_t setting);
	
	
	//! Enable the transmitter immediately (Raw transmission mode).
	/** This will send out the current buffer contents.
	* This function is used to emulate amplitude modulated signals.
	*
	* \note You need to define RFM12_RAW_TX as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_off() and rfm12_rawmode()
	*/
	static inline void rfm12_tx_on (void)
	{
		/* set enable transmission bit now. */
		rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ET);
	}


	//! Set default power mode (usually transmitter off, receiver on).
	/** This will usually stop a transmission.
	* This function is used to emulate amplitude modulated signals.
	*
	* \note You need to define RFM12_RAW_TX as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_on() and rfm12_rawmode()
	*/
	static inline void rfm12_tx_off (void)
	{
		/* turn off everything. */
		rfm12_data(RFM12_CMD_PWRMGT);
	}
#endif /* RFM12_RAW_TX  */


/************************
 * rfm12 wakeup timer mode
*/

#if RFM12_USE_WAKEUP_TIMER
	//this function sets the wakeup timer register
	//(see datasheet for values)
	//see rfm12_extra.c for more documentation
	void rfm12_set_wakeup_timer(uint16_t val);
#endif /* RFM12_USE_WAKEUP_TIMER */


/************************
 * rfm12 low battery detector mode
*/

#if RFM12_LOW_BATT_DETECTOR
	/**\name States for the low battery detection feature .
	* \anchor batt_states
	* \see rfm12_set_batt_detector() and rfm12_get_batt_status()
	* @{
	*/
	//! Battery voltage is okay.
	#define RFM12_BATT_OKAY 0
	//! Low battery voltage detected.
	#define RFM12_BATT_LOW 1
	//@}

	//this function sets the low battery detector and microcontroller clock divider register
	//(see datasheet for values)
	//see rfm12_extra.c for more documentation
	void rfm12_set_batt_detector(uint16_t val);
	
	//return the battery status
	//see rfm12_extra.c for more documentation
	uint8_t rfm12_get_batt_status();
#endif /* RFM12_LOW_BATT_DETECTOR */

#endif /* _RFM12_EXTRA_H */
