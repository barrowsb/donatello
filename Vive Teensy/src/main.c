/* Name: main.c
 * Author: Vishnu Prem
 * Description: Teensy (vive+uart)
 */

#include "teensy_general.h"  
#include "t_usb.h" // t_usb.h should be in the 'inc' folder
#include <avr/pgmspace.h>
#include "uart.h"

int X_MAX = 6000;
int X_MIN = 2000;
int Y_MAX = 6000;
int Y_MIN = 2000;

//GLOBAL CALC VALS
int X_VAL_3;
int Y_VAL_3;
int X_VAL_1;
int Y_VAL_1;




int map_value(int data, int axis)
{
	//maps value to min-max range and encodes axis val
	
	int res_data = 0;
	
	if(axis == 0)//x axis
	{
		res_data = ((data - X_MIN)/((X_MAX - X_MIN)*1.0)) * 127; 
		
		if(res_data > 127)
			res_data = 127;
		else if(res_data < 0)
			res_data = 0;
		
		m_usb_tx_string("\t mapped x: ");				
		m_usb_tx_int(res_data);
	}
	else if(axis == 1)//y axis
	{
		res_data = ((data - Y_MIN)/((Y_MAX - Y_MIN)*1.0)) * 127;
		res_data = res_data + 127;
		
		if(res_data < 127)
			res_data = 127;
		else if(res_data > 255)
			res_data = 255;
		
		m_usb_tx_string("\t mapped y: ");				
		m_usb_tx_int(res_data);
	}
	return res_data;
}

void transmit_uart_int(int data, char termChar)
{
	unsigned char uart_byte_low;
	unsigned char uart_byte_high;
	
	
  	uart_byte_low = data & 0xFF;	//first byte of int	
	uart_byte_high = data>>8;
	
	uart_putchar(uart_byte_low);
	uart_putchar(uart_byte_high);
	uart_putchar(termChar);
	
	m_usb_tx_char(uart_byte_low);
	m_usb_tx_char(uart_byte_high);
	m_usb_tx_char(termChar);
	m_usb_tx_char('\n');
	
	
	
	

}

int main(void)
{

	teensy_clockdivide(0);			//resets system clock frequency
	
	uart_init(115200);
  
	
    m_usb_init();					 // setup usb connection
    //while (!m_usb_isconnected());    // wait for usb connection to be established
	m_usb_tx_string("Connected");
	clear(DDRD,4);
	clear(DDRC,7);					//PC7 pin as input
	clear(TCCR3B, ICES3);			//store timer3 at falling edge
	clear(TCCR1B, ICES1);
	
	
	set(TCCR3B, CS31);				//start timer with prescale /8
	set(TCCR1B, CS11);
    
	
	//Timer 3 vars
	int t_curr_3, t_prev_3 = 0;		//stores the timer value for button press
	int time_3;						//time elapsed between presseS				
	int rollover_count_3 = 0;			//counts the number of time overflow occurs between 2 presses
	
	
	int first_3 = 1;						//to store the first t_prev
	
	int sync_3 = 0;				//variables for identifying pattern of pulses
	int x_3 = 0;						//...
	int y_3 = 0;						//...
	char READY_FLAG_3 = 0;
	
	//Timer 1 vars
	int t_curr_1, t_prev_1 = 0;		//stores the timer value for button press
	int time_1;						//time elapsed between presseS				
	int rollover_count_1 = 0;			//counts the number of time overflow occurs between 2 presses
	
	
	int first_1 = 1;						//to store the first t_prev
	
	int sync_1 = 0;				//variables for identifying pattern of pulses
	int x_1 = 0;						//...
	int y_1 = 0;						//...
	char READY_FLAG_1 = 0;
	

	while(1)						//infinite loop
	{	
		//uart_putchar('^');
		//Timer 3 loop	
		if(bit_is_set(TIFR3, ICF3))	//if input capture//if faling edge input captured at C7
		{		
			
			if(first_3 == 1 )						//on first pulse
			{	t_prev_3 = ICR3;
				first_3 = 0;
			}
			else
			{
				time_3 = (ICR3 - t_prev_3)*0.5;	
				if( time_3 > 500 ){
					t_curr_3 = ICR3;
				
							//convert ticks to nanoseconds
				
					
				
					if( time_3 > 8000)							//if sync pulse
					{	
						if(sync_3 < 3)
						{
							sync_3 += 1;
							x_3 = 1;
							y_3 = 0;
						}
					}		
					else if(sync_3)
					{	
						
						if(x_3 == 1 && y_3 ==0)							//if x sweep
						{	m_usb_tx_string("\nx3: ");				
							m_usb_tx_long(time_3);					//prints elapsed time
							//m_usb_tx_string(" nanoseconds");
							X_VAL_3 = time_3; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!need to change
							y_3 = 1;
								
						}
						else if(x_3 ==1 && y_3 ==1)						//if sync pulse after the x pulse
						{
							x_3 = 0;
						}
						else if(x_3 ==0 && y_3 ==1)						//if y sweep
						{
							m_usb_tx_string("\t\t y3: ");				
							m_usb_tx_long(time_3);					//prints elapsed time
							//m_usb_tx_string(" nanoseconds");
							Y_VAL_3 = time_3; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!need to change
							READY_FLAG_3 = 1;
							y_3 = 0;
							sync_3 = 0;
						}
							
						
					}	
					t_prev_3 = t_curr_3;
				}
				
			}
			
			rollover_count_3 = 0;
			
			
			set(TIFR3,ICF3); 							//resets input capture flag
		}
		else if(bit_is_set(TIFR3, TOV3))				//if timer overflows
		{
			rollover_count_3++;							//track number of overflows
			set(TIFR3,TOV3);							//resets overflow flag
		}
		
		
		//Timer 1 loop
		if(bit_is_set(TIFR1, ICF1))	//if input capture//if faling edge input captured at C7
		{		
			
			if(first_1 == 1 )						//on first pulse
			{	t_prev_1 = ICR1;
				first_1 = 0;
			}
			else
			{
				time_1 = (ICR1 - t_prev_1)*0.5;
				if( time_1 > 500 ){
					
					t_curr_1 = ICR1;
					
									//convert ticks to nanoseconds
					
					
					if( time_1 > 8000)							//if sync pulse
					{	
						if(sync_1 < 3)
						{
							sync_1 += 1;
							x_1 = 1;
							y_1 = 0;
						}
					}		
					else if(sync_1)
					{	
						
						if(x_1 == 1 && y_1 ==0)							//if x sweep
						{	m_usb_tx_string("\nx1: ");				
							m_usb_tx_long(time_1);					//prints elapsed time
							//m_usb_tx_string(" nanoseconds");
							X_VAL_1 = time_1; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!need to change
							y_1 = 1;
								
						}
						else if(x_1 ==1 && y_1 ==1)						//if sync pulse after the x pulse
						{
							x_1 = 0;
						}
						else if(x_1 ==0 && y_1 ==1)						//if y sweep
						{
							m_usb_tx_string("\t\t y1: ");				
							m_usb_tx_long(time_1);					//prints elapsed time
							//m_usb_tx_string(" nanoseconds");
							Y_VAL_1 = time_1; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!need to change
							READY_FLAG_1 = 1;
							y_1 = 0;
							sync_1 = 0;
						}
							
						
					}	
					t_prev_1 = t_curr_1;
				}
				
			}
			
			rollover_count_1 = 0;
			
			
			set(TIFR1,ICF1); 							//resets input capture flag
		}
		else if(bit_is_set(TIFR1, TOV1))				//if timer overflows
		{
			rollover_count_1++;							//track number of overflows
			set(TIFR1,TOV1);							//resets overflow flag
		}
	
		
		if(READY_FLAG_1 && READY_FLAG_3){
			//calc and send uart
			
			
			READY_FLAG_1 = 0;
			READY_FLAG_3 = 0;
			send_package();
		}
		
		
	
	
	}
	
    return 0;   					//never executed
}

int send_package(){
	
	transmit_uart_int(X_VAL_1, '!');
	transmit_uart_int(Y_VAL_1, '@');
	transmit_uart_int(X_VAL_3, '#');
	transmit_uart_int(Y_VAL_3, '$');
	uart_putchar('~');
	
	m_usb_tx_string("send package");

}

