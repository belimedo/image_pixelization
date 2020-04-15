#ifndef _ALTERA_ADDRES_BASE_H_
#define _ALTERA_ADDRES_BASE_H_

/*
 * This file was automatically generated by the swinfo2header utility.
 * 
 * Created from SOPC Builder system 'impix_system' in
 * file './impix_system.sopcinfo'.
 */

/*
 * This file contains macros for module 'hps_0' and devices
 * connected to the following master:
 *   h2f_axi_master
 * 
 * Do not include this header file and another header file created for a
 * different module or master group at the same time.
 * Doing so may result in duplicate macro names.
 * Instead, use the system header file which has macros with unique names.
 */

/*
 * Macros for device 'pio_0', class 'altera_avalon_pio'
 * The macros are prefixed with 'PIO_0_'.
 * The prefix is the slave descriptor.
 */
#define PIO_0_COMPONENT_TYPE altera_avalon_pio
#define PIO_0_COMPONENT_NAME pio_0
#define PIO_0_BASE 0x0
#define PIO_0_SPAN 16
#define PIO_0_END 0xf
#define PIO_0_BIT_CLEARING_EDGE_REGISTER 1
#define PIO_0_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PIO_0_CAPTURE 0
#define PIO_0_DATA_WIDTH 4
#define PIO_0_DO_TEST_BENCH_WIRING 0
#define PIO_0_DRIVEN_SIM_VALUE 0
#define PIO_0_EDGE_TYPE NONE
#define PIO_0_FREQ 50000000
#define PIO_0_HAS_IN 0
#define PIO_0_HAS_OUT 1
#define PIO_0_HAS_TRI 0
#define PIO_0_IRQ_TYPE NONE
#define PIO_0_RESET_VALUE 0

/*
 * Macros for device 'sysid_qsys_0', class 'altera_avalon_sysid_qsys'
 * The macros are prefixed with 'SYSID_QSYS_0_'.
 * The prefix is the slave descriptor.
 */
#define SYSID_QSYS_0_COMPONENT_TYPE altera_avalon_sysid_qsys
#define SYSID_QSYS_0_COMPONENT_NAME sysid_qsys_0
#define SYSID_QSYS_0_BASE 0x10
#define SYSID_QSYS_0_SPAN 8
#define SYSID_QSYS_0_END 0x17
#define SYSID_QSYS_0_ID 0
#define SYSID_QSYS_0_TIMESTAMP 1586947588

/*
 * Macros for device 'jtag_uart_0', class 'altera_avalon_jtag_uart'
 * The macros are prefixed with 'JTAG_UART_0_'.
 * The prefix is the slave descriptor.
 */
#define JTAG_UART_0_COMPONENT_TYPE altera_avalon_jtag_uart
#define JTAG_UART_0_COMPONENT_NAME jtag_uart_0
#define JTAG_UART_0_BASE 0x18
#define JTAG_UART_0_SPAN 8
#define JTAG_UART_0_END 0x1f
#define JTAG_UART_0_IRQ 0
#define JTAG_UART_0_READ_DEPTH 64
#define JTAG_UART_0_READ_THRESHOLD 8
#define JTAG_UART_0_WRITE_DEPTH 64
#define JTAG_UART_0_WRITE_THRESHOLD 8


#endif /* _ALTERA_ADDRES_BASE_H_ */
