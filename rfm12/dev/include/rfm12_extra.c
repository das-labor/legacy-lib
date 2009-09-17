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
 
/** \file rfm12_extra.c
 * \brief rfm12 library extra features source
 * \author Hans-Gert Dahmen
 * \author Peter Fuhrmann
 * \author Soeren Heisrath
 * \version 0.9.0
 * \date 08.09.09
 *
 * This file implements all stuff related to extra features.
 *
 * \note This file is included directly by rfm12.c, for performance reasons.
 */
 
/******************************************************
 *    THIS FILE IS BEING INCLUDED DIRECTLY		*
 *		(for performance reasons)				*
 ******************************************************/

 
/************************
 * amplitude modulation receive mode
*/

#if RFM12_RECEIVE_CW
	//! The CW-mode receive buffer structure.
	/** You will need to poll the state field of this structure to determine
	* if data is available, see \ref cw_defines. \n
	* Received data can be read from the buf field.
	* It is necessary to reset the state field  to RFM12_CW_STATE_EMPTY after reading.
	*
	* \note You need to define RFM12_RECEIVE_CW as 1 to enable this.
	*/
	rfm12_rfrxbuf_t cw_rxbuf;

	
	//! CW-mode ADC interrupt.
	/** This interrupt function directly measures the receive signal strength
	* on an analog output pin of the rf12 ic.
	*
	* You will need to solder something onto your rf12 module to make this to work.
	*
	* \note You need to define RFM12_RECEIVE_CW as 1 to enable this.
	* \see adc_init() and rfm12_rfrxbuf_t
	*/
	ISR(ADC_vect, ISR_NOBLOCK)
	{
		static uint16_t adc_average;
		static uint8_t pulse_timer;	
		uint8_t    value;
		static uint8_t oldvalue;
		static uint8_t ignore;
		uint16_t adc;
		

		
		ADCSRA = (1<<ADEN) | (1<<ADFR) | (0<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz

		
		adc = ADC;
		
		adc_average -= adc_average/64;
		adc_average +=adc;
		
		value = (ADC > ((adc_average/64)+50) )?1:0;
		
		if(value)
		{
			PORTD |= (1<<PD7);
		}else
		{
			PORTD &= ~(1<<PD7);
		}


		if(TCNT0 > 0xE0){
			ignore = 0;
		}
		
		if(cw_rxbuf.state == RFM12_CW_STATE_EMPTY)
		{
			if(value && (!ignore) )
			{
				//pulse_timer = 0;
				TCNT0 = 0;
				cw_rxbuf.p   = 0;
				cw_rxbuf.state = RFM12_CW_STATE_RECEIVING;
			}
		}else if(cw_rxbuf.state == RFM12_CW_STATE_FULL)
		{
			if(value)
			{
				TCNT0 = 0;
				ignore = 1;
			}
			
		}else if(cw_rxbuf.state == RFM12_CW_STATE_RECEIVING)
		{
			if(value != oldvalue)
			{

				cw_rxbuf.buf[cw_rxbuf.p] = TCNT0;
				TCNT0 = 0;
				//pulse_timer = 0;
				if(cw_rxbuf.p != (RFM12_CW_RFRXBUF_SIZE-1) )
				{
					cw_rxbuf.p++;
				}
			}else if(TCNT0 > 0xe0)
			{
				//if( !value ){
				//PORTD |= (1<<PD6);
					cw_rxbuf.state = RFM12_CW_STATE_FULL;
				//}else{
				//	cw_rxbuf.state = STATE_EMPTY;
				//}
			}
		}

		oldvalue = value;

		ADCSRA = (1<<ADEN) | (1<<ADFR) | (1<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz


	}

	
	//! CW-mode ADC interrupt setup.
	/** This will setup the ADC interrupt to receive amplitude modulated signals.
	*
	* \note You need to define RFM12_RECEIVE_CW as 1 to enable this.
	* \see ISR(ADC_vect, ISR_NOBLOCK) and rfm12_rfrxbuf_t
	*/
	void adc_init()
	{
		ADMUX  = (1<<REFS0) | (1<<REFS1); //Internal 2.56V Reference, MUX0
		
		ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADFR) | (1<<ADIE) //start free running mode
				| (1<<ADPS2) | (1<<ADPS1);  //preescaler to clk/64
											//samplerate = 16MHz/(64*13) = 19231 Hz
		
	}
#endif /* RFM12_RECEIVE_CW */


/************************
 * amplitude modulated raw tx mode
*/

#if RFM12_RAW_TX
	//! En- or disable raw transmissions.
	/** When enabling raw mode, this function puts the internal state machine
	*into transmit mode and disables the interrupt.
	* Otherwise it will restore normale operation.
	*
	* \param [setting] Pass 1 to enable the raw mode, 0 to disable it.
	* \note You need to define RFM12_RAW_TX as 1 to enable this.
	* \warning This will interfere with the wakeup timer feature.
	* \todo Use power management shadow register if the wakeup timer feature is enabled.
	* \see rfm12_tx_on() and rfm12_tx_off()
	*/
	void rfm12_rawmode(uint8_t setting)
	{
		if (setting)
		{
			ctrl.rfm12_state = STATE_TX;
			RFM12_INT_OFF();
		} else
		{
			/* re-enable the receiver */
			rfm12_data(RFM12_CMD_PWRMGT | PWRMGT_DEFAULT | RFM12_PWRMGT_ER);
			RFM12_INT_ON();
			ctrl.rfm12_state = STATE_RX_IDLE;
		}
	}
#endif /* RFM12_RAW_TX */


/************************
 * rfm12 wakeup timer mode
*/

#if RFM12_USE_WAKEUP_TIMER
	//! This function sets the wakeup timer register.
	/** \param [val]  The wakeup timer period value to be passed to the rf12. \n
	* See the rf12 datasheet for valid values.
	*/
	void rfm12_set_wakeup_timer(uint16_t val)
	{	
		//set wakeup timer
		rfm12_data (RFM12_CMD_WAKEUP | (val & 0x1FFF));
	
		//reset wakeup timer
		rfm12_data(RFM12_CMD_PWRMGT | (PWRMGT_DEFAULT & ~RFM12_PWRMGT_EW));
		rfm12_data(RFM12_CMD_PWRMGT |  PWRMGT_DEFAULT);		
	}
#endif /* RFM12_USE_WAKEUP_TIMER */


/************************
 * rfm12 low battery detector mode
*/

#if RFM12_LOW_BATT_DETECTOR
	//! This function sets the low battery detector and microcontroller clock divider register.
	/** \param [val]  The register value to be passed to the rf12. \n
	* See the rf12 datasheet for valid values.
	* \see rfm12_get_batt_status()
	*/
	void rfm12_set_batt_detector(uint16_t val)
	{	
		//set the low battery detector and microcontroller clock divider register
		rfm12_data (RFM12_CMD_LBDMCD | (val & 0x01FF));
	}
	
	//! Return the current low battery detector status.
	/** \returns One of thse defines: \ref batt_states
	* \see rfm12_set_batt_detector() and the \ref batt_states defines
	*/
	uint8_t rfm12_get_batt_status()
	{
		return ctrl.low_batt;
	}
#endif /* RFM12_LOW_BATT_DETECTOR */
