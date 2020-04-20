#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"

//#include "soc_cv_av/socal/socal.h"
//#include "soc_cv_av/socal/hps.h"
//#include "soc_cv_av/socal/alt_gpio.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"


#include "addres_base.h"

// Ovo treba za lw axi
// #define HW_REGS_BASE ( ALT_STM_OFST )
// #define HW_REGS_SPAN ( 0x04000000 )
// #define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define ALT_AXI_FPGASLVS_OFST (0xC0000000) // axi_master
#define HW_FPGA_AXI_SPAN (0x40000000) // Bridge span 1GB
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )

#define OFST 10

int main(int argc, char* argv[])
{
	void*	virtual_base;
	void* 	axi_virtual_base;
	
	int 	fd;
	void*	addr_lwh2f_led;
	void* 	addr_h2f_led;
	
	(void)argc;
	(void)argv;

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
	if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
		printf("ERROR: could not open \"/dev/mem\"...\n");
		
		return 1;
	}

	//virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
	axi_virtual_base = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, ALT_AXI_FPGASLVS_OFST );
	//virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
	
	
	if (axi_virtual_base == MAP_FAILED) {
		printf("ERROR: mmap() failed...\n");
		close(fd);

		return 1;
	}

	// if (virtual_base == MAP_FAILED) {
	// 	printf("ERROR: mmap() failed...\n");
	// 	close(fd);

	// 	return 1;
	// }
	
	addr_h2f_led = axi_virtual_base + ((unsigned long)(PIO_0_BASE) & (unsigned long)(HW_FPGA_AXI_MASK));
	//addr_lwh2f_led = virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + LEDLWH2F_BASE) & (unsigned long)(HW_REGS_MASK));
	
	int i = 0;
    uint8_t test = 1;
	// while(i < 5) {
	// 	printf("Unesi test:");
	// 	scanf(" %c",&test);
	// 	*((uint8_t*)addr_h2f_led) = test;
	// 	i++;	
	// 	printf("%08x\n",*((uint8_t*)axi_virtual_base));
	// 	printf("%08x%08x%08x%08x\n",*((uint32_t*)(axi_virtual_base + (unsigned long)(0x03))));
	// }
	uint8_t old_value = 0x00;
	uint32_t new_value[4] = {0};
	uint8_t stop = 0;
	printf(" %08x\n",0x10);
	while(stop < 30) {
		//printf("Ista?!");
		old_value = *((uint8_t*)axi_virtual_base);
		new_value[0] = *((uint32_t*)axi_virtual_base);
		new_value[1] = *((uint32_t*)(axi_virtual_base + 0x01));
		new_value[2] = *((uint32_t*)(axi_virtual_base + 0x02));
		new_value[3] = *((uint32_t*)(axi_virtual_base + 0x03));
		printf("Vrijednosti su: %08x - old_value, %032x - new_value[0], %032x - new_value[1], %032x - new_value[2], %032x - new_value[3]\n",old_value,new_value[0],new_value[1],new_value[2],new_value[3]);
		//*((uint8_t*)addr_h2f_led) = new_value;
		stop++;
		sleep(1);
		//usleep(10000);
	}
    
	//alt_write_byte(addr_h2f_led, test);
	
	// while (1) {
	// 	int a;
	// 	scanf("%d", &a);
	// 	alt_write_word(addr_h2f_led + 0, a);
	// 	alt_write_word(addr_lwh2f_led + 0, a);
	// 	//*(uint32_t *)addr_h2f_led = a;
	// 	//*(uint32_t *)addr_lwh2f_led = a;
		
	// }

	// Clean up our memory mapping and exit
	if (munmap(axi_virtual_base, HW_FPGA_AXI_SPAN) != 0) {
		printf("ERROR: munmap() failed...\n");
		close(fd);

		return 1;
	}

	// if (munmap(virtual_base, HW_REGS_SPAN) != 0) {
	// 	printf("ERROR: munmap() failed...\n");
	// 	close(fd);

	// 	return 1;
	// }
	
	close(fd);

	return 0;
}
