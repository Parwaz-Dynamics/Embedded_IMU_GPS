/*
 * UartRingbuffer.c
 *
 *  Created on: 10-Jul-2019
 *      Author: Controllerstech
 *
 *  Modified on: 11-April-2020
 */

#include <string.h>
#include <UARTRingBuffer.h>

/**** define the UART you are using  ****/

extern UART_HandleTypeDef huart1;

#define uart &huart1

#define TIMEOUT_DEF 500  // 500ms timeout
uint16_t timeout;

/* put the following in the ISR 

extern void UART_ISR (UART_HandleTypeDef *huart);
extern uint16_t timeout;

*/

/****************=======================>>>>>>>>>>> NO CHANGES AFTER THIS =======================>>>>>>>>>>>**********************/


ring_buffer rx_buffer = { { 0 }, 0, 0};
ring_buffer tx_buffer = { { 0 }, 0, 0};

ring_buffer *_rx_buffer;
ring_buffer *_tx_buffer;

void store_char(unsigned char c, ring_buffer *buffer);


void RingBuf_init(void)
{
  _rx_buffer = &rx_buffer;
  _tx_buffer = &tx_buffer;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(uart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(uart, UART_IT_RXNE);
}

void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if(i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}

/* checks, if the entered string is present in the giver buffer ?
 */
static int check_for (char *str, char *buffertolookinto)
{
	int stringlength = strlen (str);
	int bufferlength = strlen (buffertolookinto);
	int so_far = 0;
	int indx = 0;
repeat:
	while (str[so_far] != buffertolookinto[indx])
		{
			indx++;
			if (indx>stringlength) return 0;
		}
	if (str[so_far] == buffertolookinto[indx])
	{
		while (str[so_far] == buffertolookinto[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == stringlength);
	else
	{
		so_far =0;
		if (indx >= bufferlength) return -1;
		goto repeat;
	}

	if (so_far == stringlength) return 1;
	else return -1;
}

int UART_Read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if(_rx_buffer->head == _rx_buffer->tail)
  {
    return -1;
  }
  else
  {
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
    return c;
  }
}

/* writes a single character to the uart and increments head
 */
void UART_Write(int c)
{
	if (c>=0)
	{
		int i = (_tx_buffer->head + 1) % UART_BUFFER_SIZE;
		while (i == _tx_buffer->tail);

		_tx_buffer->buffer[_tx_buffer->head] = (uint8_t)c;
		_tx_buffer->head = i;

		__HAL_UART_ENABLE_IT(uart, UART_IT_TXE); // Enable UART transmission interrupt
	}
}

/* Print a number with any base (2, 8, 10, 16, etc.) */
void UART_PrintBase(long n, uint8_t base)
{
    char buffer[32];  // Buffer for string conversion
    char *ptr = buffer + 31;  // Start from end of buffer
    *ptr = '\0';  // Null terminate

    // Handle negative numbers for base 10 only
    if (base == 10 && n < 0) {
        UART_Write('-');
        n = -n;
    }

    // Convert number to string in specified base
    do {
        int digit = n % base;
        *(--ptr) = (digit < 10) ? (digit + '0') : (digit - 10 + 'A');
        n /= base;
    } while (n > 0);

    // Send the string
    while (*ptr) {
        UART_Write(*ptr++);
    }
}

/* checks if the new data is available in the incoming buffer
 */
int IsDataAvailable(void)
{
  return (uint16_t)(UART_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % UART_BUFFER_SIZE;
}

/* sends the string to the uart
 */
void Uart_sendstring (const char *s)
{
	while(*s) UART_Write(*s++);
}

void GetDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto)
{
	int startStringLength = strlen (startString);
	int endStringLength   = strlen (endString);
	int so_far = 0;
	int indx = 0;
	int startposition = 0;
	int endposition = 0;

repeat1:
	while (startString[so_far] != buffertocopyfrom[indx]) indx++;
	if (startString[so_far] == buffertocopyfrom[indx])
	{
		while (startString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == startStringLength) startposition = indx;
	else
	{
		so_far =0;
		goto repeat1;
	}

	so_far = 0;

repeat2:
	while (endString[so_far] != buffertocopyfrom[indx]) indx++;
	if (endString[so_far] == buffertocopyfrom[indx])
	{
		while (endString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == endStringLength) endposition = indx-endStringLength;
	else
	{
		so_far =0;
		goto repeat2;
	}

	so_far = 0;
	indx=0;

	for (int i=startposition; i<endposition; i++)
	{
		buffertocopyinto[indx] = buffertocopyfrom[i];
		indx++;
	}
}

void Uart_flush (void)
{
	memset(_rx_buffer->buffer,'\0', UART_BUFFER_SIZE);
	_rx_buffer->head = 0;
	_rx_buffer->tail = 0;
}

int Uart_peek()
{
  if(_rx_buffer->head == _rx_buffer->tail)
  {
    return -1;
  }
  else
  {
    return _rx_buffer->buffer[_rx_buffer->tail];
  }
}

/* copies the data from the incoming buffer into our buffer
 * Must be used if you are sure that the data is being received
 * it will copy irrespective of, if the end string is there or not
 * if the end string gets copied, it returns 1 or else 0
 * Use it either after (IsDataAvailable) or after (Wait_for) functions
 */
int Copy_upto (char *string, char *buffertocopyinto)
{
	int so_far =0;
	int len = strlen (string);
	int indx = 0;

again:
	while (Uart_peek() != string[so_far])
		{
			buffertocopyinto[indx] = _rx_buffer->buffer[_rx_buffer->tail];
			_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
			indx++;
			while (!IsDataAvailable());

		}
	while (Uart_peek() == string [so_far])
	{
		so_far++;
		buffertocopyinto[indx++] = UART_Read();
		if (so_far == len) return 1;
		timeout = TIMEOUT_DEF;
		while ((!IsDataAvailable())&&timeout);
		if (timeout == 0) return 0;
	}

	if (so_far != len)
	{
		so_far = 0;
		goto again;
	}

	if (so_far == len) return 1;
	else return 0;
}

/* must be used after wait_for function
 * get the entered number of characters after the entered string
 */
int Get_after (char *string, uint8_t numberofchars, char *buffertosave)
{
	for (int indx=0; indx<numberofchars; indx++)
	{
		timeout = TIMEOUT_DEF;
		while ((!IsDataAvailable())&&timeout);  // wait until some data is available
		if (timeout == 0) return 0;  // if data isn't available within time, then return 0
		buffertosave[indx] = UART_Read();  // save the data into the buffer... increments the tail
	}
	return 1;
}

/* Waits for a particular string to arrive in the incoming buffer... It also increments the tail
 * returns 1, if the string is detected
 */
// added timeout feature so the function won't block the processing of the other functions
int Wait_for(char *string)
{
    int so_far = 0;
    int len = strlen(string);
    int position = 0;
    unsigned int start_tail;
    int chars_consumed = 0;

    if (len == 0) return -1;

again:
    timeout = TIMEOUT_DEF;
    while ((!IsDataAvailable()) && timeout) {
        HAL_Delay(1);
        timeout--;
    }
    if (timeout == 0) return -1;

    start_tail = _rx_buffer->tail;
    chars_consumed = 0;

    // Find first character
    while (Uart_peek() != string[so_far]) {
        if (_rx_buffer->tail != _rx_buffer->head) {
            _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
            chars_consumed++;
        } else {
            return -1;
        }
        timeout = TIMEOUT_DEF;
        while ((!IsDataAvailable()) && timeout) {
            HAL_Delay(1);
            timeout--;
        }
        if (timeout == 0) return -1;
    }

    // Check remaining characters
    while (Uart_peek() == string[so_far]) {
        so_far++;
        _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
        chars_consumed++;

        if (so_far == len) {
            // Return position of last character (0-based index)
            return chars_consumed - 1;  // Subtract 1 for 0-based index
        }

        timeout = TIMEOUT_DEF;
        while ((!IsDataAvailable()) && timeout) {
            HAL_Delay(1);
            timeout--;
        }
        if (timeout == 0) return -1;
    }

    if (so_far != len) {
        so_far = 0;
        _rx_buffer->tail = (start_tail + 1) % UART_BUFFER_SIZE;
        goto again;
    }

    return -1;
}

void UART_ISR (UART_HandleTypeDef *huart)
{
    uint32_t isrflags = READ_REG(huart->Instance->SR);   // SR on F4, not ISR
    uint32_t cr1its   = READ_REG(huart->Instance->CR1);

    /* RX: Data Register Not Empty */
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
        huart->Instance->SR;                         /* Read SR to clear flags */
        unsigned char c = huart->Instance->DR;       /* DR on F4, not RDR */
        store_char(c, _rx_buffer);
        return;
    }

    /* TX: Transmit Data Register Empty */
    if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
    {
        if (tx_buffer.head == tx_buffer.tail)
        {
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
        }
        else
        {
            unsigned char c = tx_buffer.buffer[tx_buffer.tail];
            tx_buffer.tail = (tx_buffer.tail + 1) % UART_BUFFER_SIZE;

            huart->Instance->SR;              /* Read SR */
            huart->Instance->DR = c;          /* DR on F4, not TDR */
        }
        return;
    }
}


/*** Deprecated For now. This is not needed, try using other functions to meet the requirement ***/
/*
uint16_t Get_position (char *string)
{
  static uint8_t so_far;
  uint16_t counter;
  int len = strlen (string);
  if (_rx_buffer->tail>_rx_buffer->head)
  {
	  if (Uart_read() == string[so_far])
	  		{
	  		  counter=UART_BUFFER_SIZE-1;
	  		  so_far++;
	  		}
	  else so_far=0;
  }
  unsigned int start = _rx_buffer->tail;
  unsigned int end = _rx_buffer->head;
  for (unsigned int i=start; i<end; i++)
  {
	  if (Uart_read() == string[so_far])
		{
		  counter=i;
		  so_far++;
		}
	  else so_far =0;
  }

  if (so_far == len)
	{
	  so_far =0;
	  return counter;
	}
  else return -1;
}


void Get_string (char *buffer)
{
	int index=0;

	while (_rx_buffer->tail>_rx_buffer->head)
	{
		if ((_rx_buffer->buffer[_rx_buffer->head-1] == '\n')||((_rx_buffer->head == 0) && (_rx_buffer->buffer[UART_BUFFER_SIZE-1] == '\n')))
			{
				buffer[index] = Uart_read();
				index++;
			}
	}
	unsigned int start = _rx_buffer->tail;
	unsigned int end = (_rx_buffer->head);
	if ((_rx_buffer->buffer[end-1] == '\n'))
	{

		for (unsigned int i=start; i<end; i++)
		{
			buffer[index] = Uart_read();
			index++;
		}
	}
}
*/
