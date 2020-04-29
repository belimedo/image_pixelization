/******************************************************************************
 * Description
 * ************* 
 * Realizacija C programa u okviru predmeta Ugradjeni Racunarski Sistemi, IV godina Racunarsko Inzenjerstvo Elektrotehnicki fakultet u Banjoj Luci. 
 * Program se zasniva na izvrsavanju pikselizacije nad odredjenom slikom, te se tako obradjena slika, cuva na istoj lokaciji, pod drugim nazivom.
 * Pikselizacija se izvrsava u YUV420 kolor prostoru, koji se dobija transformacijom ucitane slike iz RGB kolor prostora (slika se ucitava kao bit mapa).
 * Osnovni zadatak pikselizacije, je "zamucivanje" slike, sto se dobija usrednjavanjem vrijednosti intenziteta svih piksela unutar bloka definisane velicine,
 * te se takvo usrednjavanje vrsi nad citavom slikom. U zavisnosti od toga koliki blok posmatramo, imamo veci/manji stepen zamucenja.
 * Prethodno pomenuti parametri neophodni za pikselizaciju, unose se prilikom pokretanja izvrsavanja programa, kao argumenti komandne linije, na nacin:
 *		argv[0] = Naziv programa koji se izvrsava.
 *		argv[1] = Naziv slike nad kojom se vrsi pikselizacija.
 *		argv[2] = Naziv fajla u koji se smjesta pikselizovana slika.
 *
 * Izvrsavanje pikselizacije, je uslovljeno ukljucivanjem jednog od 4 koriscena prekidaca Cyclone V FPGA ploce. U skladu sa tim,
 * unutar main funkcije, vrsi se provjera statusnog registra interrupt linije dovedene sa prekidaca. Nakon sto se detektuje promjena stanja u statusnom registru rezultovana
 * pomjeranjem switch-a, procitaju se vrijednosti switch-eva sa memorijski mapirane adrese "addr_h2f_sw". U zavisnosti od procitane vrijednosti,vrsi se pikselizacija
 * slike sa predefinisanom velicinom bloka. Osim toga, procitani podatak se obradjuje i preko PIO-a prosljedjuje na 7s displej i odgovarajuce LE diode koji ispisuju 
 * velicinu bloka sa kojim se vrsi pikselizacija. na sljedeci nacin:
 *   SW0  => LED0 ukljucena, ostale iskljucene. Velicina bloka iznosi 4x4, sto se ispisuje adekvatno na 7s displej.
 *   SW1  => LED1 ukljucena, ostale iskljucene. Velicina bloka iznosi 8x8, sto se ispisuje adekvatno na 7s displej.
 *   SW2  => LED2 ukljucena, ostale iskljucene. Velicina bloka iznosi 16x16, sto se ispisuje adekvatno na 7s displej.
 *   SW3  => LED3 ukljucena, ostale iskljucene. Velicina bloka iznosi 32x32, sto se ispisuje adekvatno na 7s displej.
 *
 * Takodje, bitno je napomenuti da se citanje stanja prekidaca vrsi prioritetno, tako da najveci prioritet ima SW3, a najmanji SW0.
 * 
 * NOTE:  Jedno pokretanje programa, rezultuje jednom odradjenom pikselizacijom na propisan nacin.
 *        Potrebno je da prije pokretanja programa, izvrsite adekvatno podesavanje HW, kao i da resetujete sve prekidace.(Pozeljno!!)
 * 
 * Peripherals Exercised by SW
 * *****************************
 * LEDs
 * Seven Segment Display
 * Buttons (SW0-SW3)

 * Software Files
 * ****************
 * main.c 			==>  Glavni program koji izvrsava prethodno opisano.
 * adres_base.h 	==>  Header fajl, koji sadrzi korisne bazne adrese.
 * 
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <pthread.h>
#include "hwlib.h"

#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "addres_base.h"

#define ALT_AXI_FPGASLVS_OFST (0xC0000000) // axi_master
#define HW_FPGA_AXI_SPAN (0x40000000) // Bridge span 1GB
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )

// Veličina bloka za pikselizaciju
int32_t N = 4;

// Pomocne promjenljive i bufferi
int32_t XSIZE, YSIZE;
uint8_t *rgb_pic;
uint8_t *rgb_pic_out;
uint8_t *y_pic;
uint8_t *u_pic;
uint8_t *v_pic;

uint8_t rgb_buff[16][48];
uint8_t y_buff[16][16];
uint8_t u_buff[8][8];
uint8_t v_buff[8][8];

// Funkcije za citanje i upis slike, konverziju izmedju RGB i YUV kolor prostora, te izvrsavanje pikselizacije
int32_t read_bmp(FILE *f, uint8_t **image);
void write_bmp(FILE *f, uint8_t *image);
void rgb_to_yuv420(uint8_t rgb[16][48], uint8_t y[16][16], uint8_t u[8][8], uint8_t v[8][8]);
void yuv420_to_rgb(uint8_t y[16][16], uint8_t u[8][8], uint8_t v[8][8], uint8_t rgb[16][48]);

void convert_rgb_to_yuv420();
void convert_yuv420_to_rgb();
void extend_borders();
void perform_filtering();

// Funkcija za ispis informacija o ulaznoj i izlaznoj slici
void print_info(char* input_pic,char* output_pic);

// Pomocna promjenljiva za izlaz iz programa i funkcija koju izvrsava thread u tu svrhu
bool program_exit;
void* qt_func(void* pParam);

int main(int argc, char* argv[])
{
	FILE* f_in;
	FILE* f_out;
	int fd;
	
	void* axi_virtual_base;
	void* addr_h2f_sw;
	void* addr_h2f_ind;
	
	// Thread pomocu kojeg iniciramo kraj programa unosom karaktera 'q' na stdin
	pthread_t q_thread;
	
	pthread_create(&q_thread, NULL, qt_func, NULL);
	
	// Osnovni uslov, kao argumente, prosljedjujemo naziv slike nad kojom vrsimo pikselizaciju, kao i novi naziv!
  	if (argc < 3){  
    	printf("Neophodno je da specificirate ulazni/izlazi fajl!\n");
    	return 1;
  	}
	
	// Ucitavanje ulazne slike
	f_in = fopen(argv[1], "rb");
	if (read_bmp(f_in, &rgb_pic) == 0 || read_bmp(f_in, &rgb_pic_out) == 0){
    	printf("GRESKA: onemoguceno ucitavanje ulaznog fajla...\n");
    	return 1;
  	}
	fclose(f_in);
  	if ((XSIZE & 0xf) || (YSIZE & 0xf)){
    	printf("Velicina ulazne slike, mora da bude mnozilac 16x16\n");
    	return 1;
  	}
	
	printf("Da biste izasli iz programa, unesite karakter 'q'.\n");
	
	// Ispis informacija o slikama
	print_info(argv[1],argv[2]);
	
	// Pristup memorijskom uredjaju
	if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
		printf("ERROR: could not open \"/dev/mem\"...\n");
		return 1;
	}
	
	// Mapiranje HPS fizickog memorijskog prostora u virtuelni
	axi_virtual_base = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, ALT_AXI_FPGASLVS_OFST );
	if (axi_virtual_base == MAP_FAILED) {
		printf("ERROR: mmap() failed...\n");
		close(fd);
		return 1;
	}
	
	// Odredjivanje virtuelne adrese PIO registara za prekidace i indikatore (crvene ledice i 7seg)
	addr_h2f_sw = axi_virtual_base + ((unsigned long)(PIO_SW_BASE) & (unsigned long)(HW_FPGA_AXI_MASK));
	addr_h2f_ind = axi_virtual_base + ((unsigned long)(PIO_IND_BASE) & (unsigned long)(HW_FPGA_AXI_MASK));
	
	// Neophodna alociranja za smjestanje slike u YUV formatu
	y_pic = (uint8_t*)malloc((XSIZE+16)*(YSIZE+16));
 	u_pic = (uint8_t*)malloc((XSIZE+16)*(YSIZE+16)/4);
    v_pic = (uint8_t*)malloc((XSIZE+16)*(YSIZE+16)/4);
	
	// Vrijeme pauze izmedju registrovanja interapta do reseta edgecapture registra (switch debouncing mehanizam)
	struct timespec st;
	st.tv_sec = 0;
	st.tv_nsec = 100000000; // desetinka
	
	// Vrijednosti stanja prekidaca
    uint32_t sw_val;
	
	while(1) {
		// Ako je iniciran kraj programa (unesen karakter 'q')
		if(program_exit) break;
		
		// Ako je detektovan interapt na prekidacu (citanje edgecapture registra)
		if(*((uint32_t*)(addr_h2f_sw+12)) != 0)
		{
			// Resetovanje edgecapture registra da bi se mogao detektovati interapt u sljedecoj iteraciji
			nanosleep(&st, NULL);
			*((uint32_t*)(addr_h2f_sw+12)) = 1;
			
			// Citanje stanja prekidaca
			sw_val = *((uint32_t*)addr_h2f_sw);
			
			// Podesavanje parametara na osnovu stanja prekidaca
			switch(sw_val){
				case 0:
					*((uint32_t*)addr_h2f_ind) = 0;
					break;
				case 1:
					*((uint32_t*)addr_h2f_ind) = 1;
					N = 4;
					break;
				case 2 ... 3:
					*((uint32_t*)addr_h2f_ind) = 2;
					N = 8;
					break;
				case 4 ... 7:
					*((uint32_t*)addr_h2f_ind) = 4;
					N = 16;
					break;
				case 8 ... 15:
					*((uint32_t*)addr_h2f_ind) = 8;
					N = 32;
					break;
			}
			
			// Postupak pikselizacije
			if(sw_val){
				printf("Velicina bloka: %dx%d\n", N, N);
				
				convert_rgb_to_yuv420();
				extend_borders();
				perform_filtering();
				convert_yuv420_to_rgb();
				
				// Otvaranje izlaznog fajla, te smjestanje pikselizovane slike
				f_out = fopen(argv[2], "wb");
				write_bmp(f_out, rgb_pic_out);
				fclose(f_out);
				
				printf("Slika je uspjesno pikselizovana.\n\n");
			}
		}
	}
	
	pthread_join(q_thread, NULL);
	
	// Gasenje indikatora
	*((uint32_t*)addr_h2f_ind) = 0;
	
	// Oslobadjanje resursa
	if (munmap(axi_virtual_base, HW_FPGA_AXI_SPAN) != 0) {
		printf("ERROR: munmap() failed...\n");
		close(fd);
		return 1;
	}
	
	close(fd);
	
	free(rgb_pic);
	free(rgb_pic_out);
	free(y_pic);
	free(u_pic);
	free(v_pic);
	
	return 0;
}

void* qt_func(void* pParam){
	char c = '0';
	
	while(c != 'q')
		c = getchar();
	
	program_exit = 1;
	
	return 0;
}

void print_info(char* input_pic,char* output_pic){
	printf("------------------------------------------------------------------------\n");
	printf("\t\t\tPIKSELIZACIJA SLIKE!!\n");
	printf("------------------------------------------------------------------------\n");
	printf("Naziv ulazne slike: %s\n", input_pic);
  	printf("Velicina: (%d,%d)\n", XSIZE, YSIZE);
	printf("Naziv izlazne slike: %s\n", output_pic);
	printf("-------------------------------------------------------------------------\n");
}

// Struktura BMP zaglavlja
#pragma pack(2)
typedef struct
{
   int16_t  bfType;  /* always 'BM' */
   int32_t    bfSize;  /* size of bitmap file in bytes */
   int16_t  bfReserved1; /* always 0 */
   int16_t  bfReserved2; /* always 0 */
   int32_t    bfOffBits;   /* offset to data for bitmap */
   int32_t    biSize; 
   int32_t    biWidth; 
   int32_t    biHeight; 
   int16_t  biPlanes; 
   int16_t  biBitCount;
   int32_t    biCompression;
   int32_t    biSizeImage; 
   int32_t    biXPelsPerMeter; 
   int32_t    biYPelsPerMeter; 
   int32_t    biClrUsed; 
   int32_t    biClrImportant; 
} BITMAPFILEHEADER; 
#pragma pack(4)

int32_t read_bmp(FILE *f, uint8_t **image)
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

  *image = (uint8_t *)malloc(XSIZE*YSIZE*3);
  fread(*image, 1, XSIZE*YSIZE*3, f);

  return 1;
}

void write_bmp(FILE *f, uint8_t *image)
{
  BITMAPFILEHEADER bmhdr = { 0x4d42, 3*XSIZE*YSIZE+54, 0, 0, 54, 40, XSIZE, YSIZE, 1, 24, 0, 0, 100, 100, 0, 0};

	if (f)
	{
		fwrite(&bmhdr, 54, 1, f);
		fwrite(image, 1, 3*XSIZE*YSIZE, f);
	}
}

void rgb_to_yuv420(uint8_t rgb[16][48], uint8_t y[16][16], uint8_t u[8][8], uint8_t v[8][8])
{
  int32_t i, j;
  int32_t Y, U, V;

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

void yuv420_to_rgb(uint8_t y[16][16], uint8_t u[8][8], uint8_t v[8][8], uint8_t rgb[16][48])
{
  int32_t i, j, Y, U, V, UV, r, g, b;
    
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

void convert_rgb_to_yuv420()
{
  int32_t x, y, i, j;

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
  int32_t x, y, i, j;

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
          rgb_pic_out[(16*y+j)*3*XSIZE+48*x+i] = rgb_buff[j][i];
        }
      }
    }
  }
}

void extend_borders()
{
  int32_t i;

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

void perform_filtering()
{
  int32_t i, j;
  int32_t average = 0;
  int32_t vertical_offset,horizontal_offset;
  int32_t overhead; // Za rubne piksele gdje mozda nije 32x32 blok
  for (vertical_offset = 0; vertical_offset < (YSIZE+16);vertical_offset+=N ) {
    for (horizontal_offset = 0;horizontal_offset < (XSIZE+16); horizontal_offset+=N ) {
      average   = 0;
      overhead  = 0;
      for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
          if ((XSIZE+16)*(YSIZE+16)>(XSIZE + 16)*(i + vertical_offset)+ j+horizontal_offset)
  				  average +=y_pic[(XSIZE + 16)*(i + vertical_offset)+ j+horizontal_offset];
          else 
            overhead++;
        }
      }
      for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
          if ((XSIZE+16)*(YSIZE+16)>(XSIZE + 16)*(i + vertical_offset)+ j+horizontal_offset) {
            if (overhead)
              y_pic[(XSIZE + 16)*(i + vertical_offset)+ j+horizontal_offset] = average/overhead;
            else 
              y_pic[(XSIZE + 16)*(i + vertical_offset)+ j+horizontal_offset] = average/(N*N);
          }
        }
      }
    }
  }    
}
