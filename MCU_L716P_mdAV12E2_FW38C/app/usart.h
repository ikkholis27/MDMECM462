#ifndef _GLOBAL_H_
#define _GLOBAL_H_

//#define UART0_BAUDRATE  115200 	// for DEBUGMODE 1 (DebugMode)
//#define UART0_BAUDRATE  9600 		// For DEBUGMODE 0 and METERDLMS 0 (Meter Atlas/MK10E or Genius/MK6N)
//#define UART0_BAUDRATE  19200			// For DEBUGMODE 0 and METERDLMS 1 (Meter DLMS/MK10MI)

/*
#ifdef DEBUGMODE
#define UART0_BAUDRATE  115200
#else
#define UART0_BAUDRATE  9600
#endif
*/

//#define UART1_BAUDRATE  115200
#define UART1_BAUDRATE  9600

void uart0_init(long baud0);
void uart_init(int mode);
void uart_init_live(long mode);
int uart1_send_string(char *str);
void uart0_handle(void);
void uart1_handle(void);

#endif /* _GLOBAL_H_ */
