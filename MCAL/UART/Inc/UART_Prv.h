#ifndef UART_PRV_H
#define UART_PRV_H



/*==============================================================================================================================================
 *@fn    UART_HANDLE_IT
 *@brief  This function is used to Handle Interrupts
 *@paramter[in] UARTNumber : UART Number Needed To Be Handled
 *@retval void
 *@note  This Function is Private
 *==============================================================================================================================================*/
static void UART_HANDLE_IT( UART_ID_t UARTNumber );


#define UARTNUMBER 6
#define Clock_Freq 16000000UL
#define FlagsNumber 10
#define OneBitMasking 0b1
#define TwoBitMasking 0b11
#define FourBitMasking 0b1111
#define FiveBitMasking 0b11111
#define ElevenBitMasking 0xFFF


















#endif