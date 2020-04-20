#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"

#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"

#include "addres_base.h"

#define ALT_AXI_FPGASLVS_OFST (0xC0000000) 			// axi_master
#define HW_FPGA_AXI_SPAN (0x40000000) 				// Bridge span 1GB
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )

#define OFST 10

/*Size of the block for pixelation*/
static int N = 4;

int XSIZE, YSIZE;
unsigned char *rgb_pic;
unsigned char *y_pic;
unsigned char *y_pic2;
unsigned char *u_pic;
unsigned char *v_pic;

unsigned char rgb_buff[16][48];
unsigned char y_buff[16][16];
unsigned char y_buff11x11[11][11];
unsigned char y_buff8x8[8][8];
unsigned char u_buff[8][8];
unsigned char v_buff[8][8];
short dct_coefs[8][8];
short dct_coefs2[8][8];

void* addr_h2f_led;		// Virtuela adresa Led dioda!
void* addr_h2f_sw;		// Virtuelna adresa prekidaca!
	
/* Funkcije neophodne za kontrolu prekidaca + on/off dioda/7s! */
int pronalazak_aktivnog_prekidaca(uint8_t podatak);
void* funkcija_thread (void *arg);
void oslobadjanje_resursa();

/* Funkcije neophodne za izvrsavanje pikselizacije! */
int read_bmp(FILE *f, unsigned char **image);
void write_bmp(FILE *f, unsigned char *image);
void rgb_to_yuv420(unsigned char rgb[16][48], unsigned char y[16][16], unsigned char u[8][8], unsigned char v[8][8]);
void yuv420_to_rgb(unsigned char y[16][16], unsigned char u[8][8], unsigned char v[8][8], unsigned char rgb[16][48]);

void convert_rgb_to_yuv420();
void convert_yuv420_to_rgb();
void extend_borders();
void perform_filtering();

int main(int argc, char* argv[])
{
	int 	fd;
	void* 	axi_virtual_base;	// AXI vir. adresa!
	pthread_t upravljanje_hw;
	FILE *f;

	// Osnovni uslov, kao argumente, prosljedjujemo naziv slike nad kojom vrsimo pikselizaciju, kao i novi naziv!
  	if (argc < 3){  
    	printf("Neophodno je da specificirate ulazni/izlazi fajl!\n");
    	return -1;
  	}

	f = fopen(argv[1], "rb");
	if (read_bmp(f, &rgb_pic) == 0){
    	printf("GRESKA: onemoguceno ucitavanje ulaznog fajla...\n");
    	return -1;
  	}
  	fclose(f);

  	printf("XSIZE: %d, YSIZE: %d\n",XSIZE,YSIZE);
    //TODO: hoce li ovo ovako raditi (nije radilo sa | , radice sa ||) ili da se radi mod? %
  	if ((XSIZE & 0xf) || (YSIZE & 0xf)){
    	printf("Velicina ulazne slike, mora da bude mnozilac 16x16\n");
    	return -1;
  	}

	/* Mapiranje adresnog prostora LED dioda, u korisnicki prostor, tako da mozemo ostvariti pristup. */
	if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
		printf("GRESKA: onemoguceno otvaranje \"/dev/mem\"...\n");
		return -1;
	}

	axi_virtual_base = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, ALT_AXI_FPGASLVS_OFST );
	
	if (axi_virtual_base == MAP_FAILED) {
		printf("ERROR: mmap() failed...\n");
		close(fd);

		return -1;
	}

	/* Odredjivanje virtuelne adrese led dioda. */
	addr_h2f_led = axi_virtual_base + ((unsigned long)(PIO_0_BASE) & (unsigned long)(HW_FPGA_AXI_MASK));

	/* Odredjivanje virtuelne adrese prekidaca. */
	addr_h2f_sw = axi_virtual_base + ((unsigned long)(SWITCHES_INPUT_BASE) & (unsigned long)(HW_FPGA_AXI_MASK));

	/* Neophodna alociranja za smjestanje slike u YUV formatu! */
	y_pic = (unsigned char *)malloc((XSIZE+16)*(YSIZE+16));
 	y_pic2 = (unsigned char *)malloc((XSIZE+16)*(YSIZE+16));
	u_pic = (unsigned char *)malloc((XSIZE+16)*(YSIZE+16)/4);
    v_pic = (unsigned char *)malloc((XSIZE+16)*(YSIZE+16)/4);

    // Resetujemo ledice i displeje za svaki slucaj
    *((uint8_t*)addr_h2f_led) = 0x00;

	uint8_t stanje_prekidaca, low;
	uint8_t kraj = 0;
	uint8_t prekidac;

	while(kraj == 0) {

		stanje_prekidaca = *((uint8_t*)addr_h2f_sw); // Citanje stanja prekidaca!
		stanje_prekidaca = stanje_prekidaca & 0x0F;  // Uzimamo niza 4 bita, koji reprezentuju 4 prekidaca!

		if(stanje_prekidaca) {
			// Posto imamo 1 na jednom od prekidaca, prioritetno provjeravamo o kojem se radi!
			// Najveci prioritet, ima posljednji prekidac, dok najmanji prioritet ima prvi!
			prekidac = 0;
			
	        while(prekidac < 4){
		        if((stanje_prekidaca << (4+prekidac)) & 0x80) {	//Ako imamo jedinicu, to je to!
			        break;
		        }
		        prekidac++;	
	        }
	        //*((uint8_t*)addr_h2f_led) = stanje_prekidaca;  //Ukljucivanje led diode! TODO

			//Podesavanje parametara pikselizacije, te obavljanje pikselizacije!
			switch (prekidac) {
				case 0: 	                            // 4 prekidac! 32x32
            N = 32; 
            *((uint8_t*)addr_h2f_led) = 0x08;
            break;				
				case 1:                 				// 3 prekidac! 16x16
            N = 16;
            *((uint8_t*)addr_h2f_led) = 0x04;
            break;
        case 2: 	            				// 2 prekidac! 08x08
            N = 8;
            *((uint8_t*)addr_h2f_led) = 0x02;
            break;
				case 3: 	            				// 1 prekidac! 04x04
            N = 4;
            *((uint8_t*)addr_h2f_led) = 0x01;
            break;               
				default: 	
            N = 4;
			}

  			convert_rgb_to_yuv420();
  			extend_borders();
  			perform_filtering();
  			convert_yuv420_to_rgb();

			kraj = 1;
		}
    usleep(100); // Spavaj 0.1 ms
  }

	/* Otvaranje izlaznog fajla, te smjestanje pikselizovane slike! */
  	f = fopen(argv[2], "wb");
  	write_bmp(f, rgb_pic);
  	fclose(f);

	// Clean up our memory mapping and exit
	if (munmap(axi_virtual_base, HW_FPGA_AXI_SPAN) != 0) {
		printf("ERROR: munmap() failed...\n");
		close(fd);
		oslobadjanje_resursa();
		return -1;
	}

	close(fd);
	oslobadjanje_resursa();

	return 0;
}

/* Funkcija kojom vrsimo oslobadjanje alociranih resursa programa! */
void oslobadjanje_resursa(){
    free(rgb_pic);
    free(y_pic);
    free(y_pic2);
    free(u_pic);
    free(v_pic);
}


/* Konverzija iz RGB u YUV format! */
void convert_rgb_to_yuv420()
{
  int x, y, i, j;

  for (y=0; y<YSIZE/16; y++)
  {
    for (x=0; x<XSIZE/16; x++)
    {
      for (j=0; j<16; j++)
      {
        for (i=0; i<48; i++)
        {
          rgb_buff[j][i] = rgb_pic[(16*y+j)*3*XSIZE+48*x+i];
        }
      }

      rgb_to_yuv420(rgb_buff, y_buff, u_buff, v_buff);

      for (j=0; j<8; j++)
      {
        for (i=0; i<8; i++)
        {
          y_pic[(16*y+2*j+8)*(XSIZE+16)+16*x+2*i+8] = y_buff[2*j][2*i];
          y_pic[(16*y+2*j+8)*(XSIZE+16)+16*x+2*i+9] = y_buff[2*j][2*i+1];
          y_pic[(16*y+2*j+9)*(XSIZE+16)+16*x+2*i+8] = y_buff[2*j+1][2*i];
          y_pic[(16*y+2*j+9)*(XSIZE+16)+16*x+2*i+9] = y_buff[2*j+1][2*i+1];
          u_pic[(8*y+j+4)*(XSIZE/2+8)+8*x+i+4] = u_buff[j][i];
          v_pic[(8*y+j+4)*(XSIZE/2+8)+8*x+i+4] = v_buff[j][i];
        }
      }
    }
  }
}

void convert_yuv420_to_rgb()
{
  int x, y, i, j;

  for (y=0; y<YSIZE/16; y++)
  {
    for (x=0; x<XSIZE/16; x++)
    {
      for (j=0; j<8; j++)
      {
        for (i=0; i<8; i++)
        {
          y_buff[2*j  ][2*i  ] = y_pic[(16*y+2*j+8)*(XSIZE+16)+16*x+2*i+8];
          y_buff[2*j  ][2*i+1] = y_pic[(16*y+2*j+8)*(XSIZE+16)+16*x+2*i+9];
          y_buff[2*j+1][2*i  ] = y_pic[(16*y+2*j+9)*(XSIZE+16)+16*x+2*i+8];
          y_buff[2*j+1][2*i+1] = y_pic[(16*y+2*j+9)*(XSIZE+16)+16*x+2*i+9];
          u_buff[j][i] = u_pic[(8*y+j+4)*(XSIZE/2+8)+8*x+i+4];
          v_buff[j][i] = v_pic[(8*y+j+4)*(XSIZE/2+8)+8*x+i+4];
        }
      }

      yuv420_to_rgb(y_buff, u_buff, v_buff, rgb_buff);

      for (j=0; j<16; j++)
      {
        for (i=0; i<48; i++)
        {
          rgb_pic[(16*y+j)*3*XSIZE+48*x+i] = rgb_buff[j][i];
        }
      }
    }
  }
}

void extend_borders()
{
  int i;

  for (i=0; i<8; i++)
  {
    memcpy(&y_pic[i*(XSIZE+16)+8], &y_pic[8*(XSIZE+16)+8], XSIZE);
    memcpy(&y_pic[(YSIZE+8+i)*(XSIZE+16)+8], &y_pic[(YSIZE+7)*(XSIZE+16)+8], XSIZE);
  }

  for (i=0; i<YSIZE+16; i++)
  {
    memset(&y_pic[i*(XSIZE+16)], y_pic[i*(XSIZE+16)+8], 8);
    memset(&y_pic[i*(XSIZE+16)+XSIZE+8], y_pic[i*(XSIZE+16)+XSIZE+7], 8);
  }

  for (i=0; i<4; i++)
  {
    memcpy(&u_pic[i*(XSIZE/2+8)+4], &u_pic[4*(XSIZE/2+8)+4], XSIZE/2);
    memcpy(&u_pic[(YSIZE/2+4+i)*(XSIZE/2+8)+4], &u_pic[(YSIZE/2+3)*(XSIZE/2+8)+4], XSIZE/2);
    memcpy(&v_pic[i*(XSIZE/2+8)+4], &v_pic[4*(XSIZE/2+8)+4], XSIZE/2);
    memcpy(&v_pic[(YSIZE/2+4+i)*(XSIZE/2+8)+4], &v_pic[(YSIZE/2+3)*(XSIZE/2+8)+4], XSIZE/2);
  }

  for (i=0; i<YSIZE/2+8; i++)
  {
    memset(&u_pic[i*(XSIZE/2+8)], u_pic[i*(XSIZE/2+8)+4], 4);
    memset(&u_pic[i*(XSIZE/2+8)+XSIZE/2+4], u_pic[i*(XSIZE/2+8)+XSIZE/2+3], 4);
    memset(&v_pic[i*(XSIZE/2+8)], v_pic[i*(XSIZE/2+8)+4], 4);
    memset(&v_pic[i*(XSIZE/2+8)+XSIZE/2+4], v_pic[i*(XSIZE/2+8)+XSIZE/2+3], 4);
  }
}

/*The function that performs pixelation*/
void perform_filtering()
{
		int i, j, ioff, joff;

	//Pikselizacija slike u YUV420 formatu: (Samo y komponenta!) 		

        for (ioff = 0; ioff < YSIZE; ioff+=N) {
            for (joff = 0; joff < XSIZE ; joff+=N) {
				int srednja_vrijednost = 0;
                for (i = 0; i < N; i++) {
                    for (j = 0; j < N; j++) {
						srednja_vrijednost+=y_pic[(XSIZE)*(i+ioff)+(j+joff)];
                    }

                }
				// Korekcija vrijednosti unutar matrice:
				for (i = 0; i < N; i++)
                    for (j = 0; j < N; j++) 
                        y_pic[(XSIZE)*(i+ioff)+(j+joff)] = srednja_vrijednost/(N*N);           
			}
        }
}

#pragma pack(2)
typedef struct
{
   short  bfType;  /* always 'BM' */
   int    bfSize;  /* size of bitmap file in bytes */
   short  bfReserved1; /* always 0 */
   short  bfReserved2; /* always 0 */
   int    bfOffBits;   /* offset to data for bitmap */
   int    biSize; 
   int    biWidth; 
   int    biHeight; 
   short  biPlanes; 
   short  biBitCount;
   int    biCompression;
   int    biSizeImage; 
   int    biXPelsPerMeter; 
   int    biYPelsPerMeter; 
   int    biClrUsed; 
   int    biClrImportant; 
} BITMAPFILEHEADER; 
#pragma pack(4)

int read_bmp(FILE *f, unsigned char **image)
{
  BITMAPFILEHEADER bmhdr;

  if (f == NULL)
    return 0;

  fread(&bmhdr, sizeof(bmhdr), 1, f);

  if (bmhdr.bfType != 0x4d42)
    return 0;

  XSIZE = bmhdr.biWidth;
  YSIZE = bmhdr.biHeight;
  if (YSIZE < 0)
    YSIZE = -YSIZE;

  *image = (unsigned char *)malloc(XSIZE*YSIZE*3);
  fread(*image, 1, XSIZE*YSIZE*3, f);

  return 1;
}

void write_bmp(FILE *f, unsigned char *image)
{
  BITMAPFILEHEADER bmhdr = { 0x4d42, 3*XSIZE*YSIZE+54, 0, 0, 54, 40, XSIZE, YSIZE, 1, 24, 0, 0, 100, 100, 0, 0};

	if (f)
	{
		fwrite(&bmhdr, 54, 1, f);
		fwrite(image, 1, 3*XSIZE*YSIZE, f);
	}
}

void rgb_to_yuv420(unsigned char rgb[16][48], unsigned char y[16][16], unsigned char u[8][8], unsigned char v[8][8])
{
  int i, j;
  int Y, U, V;

  for (j=0; j<8; j++)
  {
    for (i=0; i<8; i++)
    {
       Y =   66 * rgb[2*j][6*i+2] + 129 * rgb[2*j][6*i+1] +  25 * rgb[2*j][6*i];
       U = - 38 * rgb[2*j][6*i+2] -  74 * rgb[2*j][6*i+1] + 112 * rgb[2*j][6*i];
       V =  112 * rgb[2*j][6*i+2] -  94 * rgb[2*j][6*i+1] -  18 * rgb[2*j][6*i];

       Y = (Y + 128) >> 8;
       y[2*j][2*i] = Y + 16;

       Y  =   66 * rgb[2*j][6*i+5] + 129 * rgb[2*j][6*i+4] +  25 * rgb[2*j][6*i+3];
       U += - 38 * rgb[2*j][6*i+5] -  74 * rgb[2*j][6*i+4] + 112 * rgb[2*j][6*i+3];
       V +=  112 * rgb[2*j][6*i+5] -  94 * rgb[2*j][6*i+4] -  18 * rgb[2*j][6*i+3];

       Y = (Y + 128) >> 8;
       y[2*j][2*i+1] = Y + 16;

       Y  =   66 * rgb[2*j+1][6*i+2] + 129 * rgb[2*j+1][6*i+1] +  25 * rgb[2*j+1][6*i];
       U += - 38 * rgb[2*j+1][6*i+2] -  74 * rgb[2*j+1][6*i+1] + 112 * rgb[2*j+1][6*i];
       V +=  112 * rgb[2*j+1][6*i+2] -  94 * rgb[2*j+1][6*i+1] -  18 * rgb[2*j+1][6*i];

       Y = (Y + 128) >> 8;
       y[2*j+1][2*i] = Y + 16;

       Y  =   66 * rgb[2*j+1][6*i+5] + 129 * rgb[2*j+1][6*i+4] +  25 * rgb[2*j+1][6*i+3];
       U += - 38 * rgb[2*j+1][6*i+5] -  74 * rgb[2*j+1][6*i+4] + 112 * rgb[2*j+1][6*i+3];
       V +=  112 * rgb[2*j+1][6*i+5] -  94 * rgb[2*j+1][6*i+4] -  18 * rgb[2*j+1][6*i+3];

       Y = (Y + 128) >> 8;
       y[2*j+1][2*i+1] = Y + 16;

       U = (U + 512) >> 10;
       u[j][i] = U + 128;

       V = (V + 512) >> 10;
       v[j][i] = V + 128;
    }
  }
}

void yuv420_to_rgb(unsigned char y[16][16], unsigned char u[8][8], unsigned char v[8][8], unsigned char rgb[16][48])
{
  int i, j, Y, U, V, UV, r, g, b;
    
  for (j=0; j<8; j++)
  {
    for (i=0; i<8; i++)
    {
      U = u[j][i] - 128;
      V = v[j][i] - 128;

      UV = -6657 * U - 13424 * V;
      U = 33311 * U;
      V = 26355 * V;

      Y = 19077 * (y[2*j][2*i] - 16);

      r = (Y + V) >> 14;
      if (r < 0)
        r = 0;
      else if (r > 255)
        r = 255;
      g = (Y + UV) >> 14;
      if (g < 0)
        g = 0;
      else if (g > 255)
        g = 255;
      b = (Y + U) >> 14;
      if (b < 0)
        b = 0;
      else if (b > 255)
        b = 255;

      rgb[2*j][6*i  ] = b;
      rgb[2*j][6*i+1] = g;
      rgb[2*j][6*i+2] = r;

      Y = 19077 * (y[2*j][2*i+1] - 16);

      r = (Y + V) >> 14;
      if (r < 0)
        r = 0;
      else if (r > 255)
        r = 255;
      g = (Y + UV) >> 14;
      if (g < 0)
        g = 0;
      else if (g > 255)
        g = 255;
      b = (Y + U) >> 14;
      if (b < 0)
        b = 0;
      else if (b > 255)
        b = 255;

      rgb[2*j][6*i+3] = b;
      rgb[2*j][6*i+4] = g;
      rgb[2*j][6*i+5] = r;

      Y = 19077 * (y[2*j+1][2*i] - 16);

      r = (Y + V) >> 14;
      if (r < 0)
        r = 0;
      else if (r > 255)
        r = 255;
      g = (Y + UV) >> 14;
      if (g < 0)
        g = 0;
      else if (g > 255)
        g = 255;
      b = (Y + U) >> 14;
      if (b < 0)
        b = 0;
      else if (b > 255)
        b = 255;

      rgb[2*j+1][6*i  ] = b;
      rgb[2*j+1][6*i+1] = g;
      rgb[2*j+1][6*i+2] = r;

      Y = 19077 * (y[2*j+1][2*i+1] - 16);

      r = (Y + V) >> 14;
      if (r < 0)
        r = 0;
      else if (r > 255)
        r = 255;
      g = (Y + UV) >> 14;
      if (g < 0)
        g = 0;
      else if (g > 255)
        g = 255;
      b = (Y + U) >> 14;
      if (b < 0)
        b = 0;
      else if (b > 255)
        b = 255;

      rgb[2*j+1][6*i+3] = b;
      rgb[2*j+1][6*i+4] = g;
      rgb[2*j+1][6*i+5] = r;
    }
  }
}
