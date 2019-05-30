#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <alsa/asoundlib.h>
#include <alsa/pcm_external.h>
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include "uwachannel.h"
#include "xuwachannel_accelerator.h"

static XUwachannel_accelerator_uio_info uio_info;

typedef struct snd_pcm_uwa {
	snd_pcm_extplug_t ext;
	int framesent;
	int init_done;	
	int time_variant;
	int verbose;
	// DMA proxy 
	uwa_dma_t *tx_proxy_interface_p;
	uwa_dma_t *rx_proxy_interface_p;
	int tx_proxy_fd;
	int rx_proxy_fd;
	snd_pcm_channel_area_t *uwa_input;
	snd_pcm_channel_area_t *uwa_output;
	// UWA-CA ctrl
	XUwachannel_accelerator uwa_ca_dev;
	char *InstanceName; 
	// CIR FD
	FILE* ch1_cir_r_fd;
	FILE* ch1_cir_i_fd;
	FILE* ch2_cir_r_fd;
	FILE* ch2_cir_i_fd;
	// CIR Buffer Pointers
	int *cir_r_1_buff_0;
	int *cir_r_1_buff_1;
	int *cir_r_1_buff_2;
	int *cir_r_1_buff_3;
	int *cir_i_1_buff_0;
	int *cir_i_1_buff_1;	
	int *cir_i_1_buff_2;
	int *cir_i_1_buff_3;
	int *cir_r_2_buff_0;
	int *cir_r_2_buff_1;
	int *cir_r_2_buff_2;
	int *cir_r_2_buff_3;
	int *cir_i_2_buff_0;
	int *cir_i_2_buff_1;
	int *cir_i_2_buff_2;
	int *cir_i_2_buff_3;
	//CIR Status
	int cir_pos; 
	int ncir;
	int start_flag;
	int throw_unstable_flag;
	int ncoef;
	long cir_update_rate_us;
	snd_pcm_uframes_t cir_update_rate_frames;
	// Propagation Delay (tau0) for two communication link
	snd_pcm_channel_area_t *tau0[2];
	int *buf_tau0[2];
	unsigned int curpos[2];
	long tau0_us[2];
	snd_pcm_uframes_t tau0_frames[2];
} snd_pcm_uwa_t;


static void *threadTX (void* ext) 
{
	int dummy;	
	snd_pcm_uwa_t *uwa = (snd_pcm_uwa_t *) ext; 
	
	ioctl(uwa->tx_proxy_fd, 0, &dummy);
	if (uwa->tx_proxy_interface_p->status != PROXY_NO_ERROR)
		SNDERR("DMA Proxy TX transfer error\n");
	return NULL;
}

static void *threadRX (void* ext) 
{
	int dummy;	
	snd_pcm_uwa_t *uwa = (snd_pcm_uwa_t *) ext; 
	
	ioctl(uwa->rx_proxy_fd, 0, &dummy);
	if (uwa->rx_proxy_interface_p->status != PROXY_NO_ERROR)
		SNDERR("DMA Proxy RX transfer error\n");
	return NULL;
}

static inline void *area_addr(const snd_pcm_channel_area_t *area, snd_pcm_uframes_t offset)
{
	unsigned int bitofs = area->first + area->step * offset;
	return (char *) area->addr + bitofs / 8;
}

static inline unsigned int area_step(const snd_pcm_channel_area_t *area)
{
	return area->step / 8;
}

static int line_from_file(char* filename, char* linebuf) {
	char* s;
	int i;
	FILE* fp = fopen(filename, "r");
	if (!fp) return -1;
	s = fgets(linebuf, MAX_UIO_NAME_SIZE, fp);
	fclose(fp);
	if (!s) return -2;
	for (i=0; (*s)&&(i<MAX_UIO_NAME_SIZE); i++) {
	if (*s == '\n') *s = 0;
	s++;
	}
	return 0;
}

static int uio_info_read_name(XUwachannel_accelerator_uio_info* info) {
	char file[ MAX_UIO_PATH_SIZE ];
	sprintf(file, "/sys/class/uio/uio%d/name", info->uio_num);
	return line_from_file(file, info->name);
}

static int uio_info_read_version(XUwachannel_accelerator_uio_info* info) {
	char file[ MAX_UIO_PATH_SIZE ];
	sprintf(file, "/sys/class/uio/uio%d/version", info->uio_num);
	return line_from_file(file, info->version);
}

static int uio_info_read_map_addr(XUwachannel_accelerator_uio_info* info, int n) {
	int ret;
	char file[ MAX_UIO_PATH_SIZE ];
	info->maps[n].addr = UIO_INVALID_ADDR;
	sprintf(file, "/sys/class/uio/uio%d/maps/map%d/addr", info->uio_num, n);
	FILE* fp = fopen(file, "r");
	if (!fp) return -1;
	ret = fscanf(fp, "0x%x", &info->maps[n].addr);
	fclose(fp);
	if (ret < 0) return -2;
	return 0;
}

static int uio_info_read_map_size(XUwachannel_accelerator_uio_info* info, int n) {
	int ret;
	char file[ MAX_UIO_PATH_SIZE ];
	sprintf(file, "/sys/class/uio/uio%d/maps/map%d/size", info->uio_num, n);
	FILE* fp = fopen(file, "r");
	if (!fp) return -1;
	ret = fscanf(fp, "0x%x", &info->maps[n].size);
	fclose(fp);
	if (ret < 0) return -2;
	return 0;
}

int XUwachannel_accelerator_Initialize(XUwachannel_accelerator *InstancePtr, const char* InstanceName) {
	XUwachannel_accelerator_uio_info *InfoPtr = &uio_info;
	struct dirent **namelist;
	int i, n;
	char* s;
	char file[ MAX_UIO_PATH_SIZE ];
	char name[ MAX_UIO_NAME_SIZE ];
	int flag = 0;

	assert(InstancePtr != NULL);

	n = scandir("/sys/class/uio", &namelist, 0, alphasort);
	if (n < 0)  return XST_DEVICE_NOT_FOUND;
	for (i = 0;  i < n; i++) {
	strcpy(file, "/sys/class/uio/");
	strcat(file, namelist[i]->d_name);
	strcat(file, "/name");
	if ((line_from_file(file, name) == 0) && (strcmp(name, InstanceName) == 0)) {
	    flag = 1;
	    s = namelist[i]->d_name;
	s += 3; // "uio"
	InfoPtr->uio_num = atoi(s);
	break;
	}
	}
	if (flag == 0)  return XST_DEVICE_NOT_FOUND;

	uio_info_read_name(InfoPtr);
	uio_info_read_version(InfoPtr);
	for (n = 0; n < MAX_UIO_MAPS; ++n) {
	uio_info_read_map_addr(InfoPtr, n);
	uio_info_read_map_size(InfoPtr, n);
	}

	sprintf(file, "/dev/uio%d", InfoPtr->uio_num);
	if ((InfoPtr->uio_fd = open(file, O_RDWR)) < 0) {
	return XST_OPEN_DEVICE_FAILED;
	}

	// NOTE: slave interface 'Uwachannel_ctrl' should be mapped to uioX/map0
	InstancePtr->Uwachannel_ctrl_BaseAddress = (u32)mmap(NULL, InfoPtr->maps[0].size, PROT_READ|PROT_WRITE, MAP_SHARED, InfoPtr->uio_fd, 0 * getpagesize());
	assert(InstancePtr->Uwachannel_ctrl_BaseAddress);

	InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

	return XST_SUCCESS;
}

int XUwachannel_accelerator_Release(XUwachannel_accelerator *InstancePtr) {
	XUwachannel_accelerator_uio_info *InfoPtr = &uio_info;

	assert(InstancePtr != NULL);
	assert(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	munmap((void*)InstancePtr->Uwachannel_ctrl_BaseAddress, InfoPtr->maps[0].size);

	close(InfoPtr->uio_fd);

	return XST_SUCCESS;
}

static void UWA_CA_start(void *uwa_handle)
{
	snd_pcm_uwa_t *pAccelerator = (snd_pcm_uwa_t *) uwa_handle;
	XUwachannel_accelerator_Start(&pAccelerator->uwa_ca_dev);
}

static int float2fixed (float x){
	int y;

	y = (int)(fabsf(x)*((1<<31)-1));

	if (x < 0)
		y = ~y;

	return y;
};

static int float2fixed24bit (float x){   
	int y;
	int mask = 0xFFFFFF;

	y = (int)(fabsf(x)*((1<<23)-1) + 0.5);

	if (x < 0)
		y = (y ^ mask);

	return y << 8; //padding to the left
}

static float fixed24bittofloat (int x){
	float y;
	int signmask = 0x7FFFFF;
	int flipmask = 0xFFFFFF;
	int x_shift;

	x_shift = x >> 8 & 0X00FFFFFF;

	if ((x_shift|signmask)==signmask){
		y = (float)x_shift/((1<<23)-1);
	} else{
		x_shift = x_shift^flipmask;
		y = -1 * (float)x_shift/(1<<23);
	};

	return y;
}

static void reload_cir (snd_pcm_uwa_t* uwa, int index, int length){

	
	XUwachannel_accelerator_Write_c_re_1_0_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_1_buff_0 + index), length);
	XUwachannel_accelerator_Write_c_re_1_1_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_1_buff_1 + index), length);
	XUwachannel_accelerator_Write_c_re_1_2_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_1_buff_2 + index), length);
	XUwachannel_accelerator_Write_c_re_1_3_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_1_buff_3 + index), length);
	
	XUwachannel_accelerator_Write_c_im_1_0_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_1_buff_0 + index), length);
	XUwachannel_accelerator_Write_c_im_1_1_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_1_buff_1 + index), length);
	XUwachannel_accelerator_Write_c_im_1_2_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_1_buff_2 + index), length);
	XUwachannel_accelerator_Write_c_im_1_3_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_1_buff_3 + index), length);
	
	XUwachannel_accelerator_Write_c_re_2_0_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_2_buff_0 + index), length);
	XUwachannel_accelerator_Write_c_re_2_1_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_2_buff_1 + index), length);
	XUwachannel_accelerator_Write_c_re_2_2_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_2_buff_2 + index), length);
	XUwachannel_accelerator_Write_c_re_2_3_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_r_2_buff_3 + index), length);

	XUwachannel_accelerator_Write_c_im_2_0_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_2_buff_0 + index), length);
	XUwachannel_accelerator_Write_c_im_2_1_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_2_buff_1 + index), length);
	XUwachannel_accelerator_Write_c_im_2_2_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_2_buff_2 + index), length);
	XUwachannel_accelerator_Write_c_im_2_3_V_Words(&uwa->uwa_ca_dev, 0, (uwa->cir_i_2_buff_3 + index), length);
};


static int get_number_of_lines (FILE *file) {

	char buffer[1024 + 1]; 
	char lastchar = '\n';
	size_t bytes;
	int lines = 0;

	while ((bytes = fread(buffer,1,sizeof(buffer)-1, file))){
		lastchar = buffer[bytes - 1];
		for (char *c = buffer; (c = memchr(c,'\n', bytes - (c - buffer))); c++){
			lines ++;
		}
	}

	if (lastchar != '\n') {
		lines ++;
	}

	return lines; 
};


static void cir_alloc_memory (int **buff_0, int **buff_1,
							  int **buff_2, int **buff_3,  int size){

	*buff_0 = (int *) calloc (1,size*sizeof(int));
	if (*buff_0 == NULL){
		SNDERR("Malloc failed! \n");
		exit(EXIT_FAILURE);
	};

	*buff_1 = (int *) calloc (1,size*sizeof(int));
	if (*buff_1 == NULL){
		SNDERR("Malloc failed! \n");
		exit(EXIT_FAILURE);
	};

	*buff_2 = (int *) calloc (1,size*sizeof(int));
	if (*buff_1 == NULL){
		SNDERR("Malloc failed! \n");
		exit(EXIT_FAILURE);
	};

	*buff_3 = (int *) calloc (1,size*sizeof(int));
	if (*buff_1 == NULL){
		SNDERR("Malloc failed! \n");
		exit(EXIT_FAILURE);
	};

};


static void cir_init_memory (FILE *cir_fd, int *buff_0, int *buff_1, 
							 int *buff_2, int *buff_3,  int size){
	float cir;
	int cir_int;
	
	for (int i=0 ; i < size ; i++){
		for (int s = 0; s < 4 ;s++){
			fscanf(cir_fd, "%f", &cir);
			cir_int = float2fixed(cir);
			if (s == 0){
				buff_0[i] = cir_int;
			}else if (s == 1){
				buff_1[i] = cir_int;
			}else if (s == 2){
				buff_2[i] = cir_int;
			}else if (s == 3){
				buff_3[i] = cir_int;
			}	 
		}
	}
};


static inline snd_pcm_uframes_t time_to_frames(unsigned int rate,
					       unsigned long long time)
{
	return (time * rate) / 1000000ULL;
};


static void delayed_copy(snd_pcm_uwa_t *uwa,
			 const snd_pcm_channel_area_t *dst_areas,
			 snd_pcm_uframes_t dst_offset,
			 const snd_pcm_channel_area_t *src_areas,
			 snd_pcm_uframes_t src_offset,
			 unsigned int size)
{
	unsigned int i, p, curpos, dst_step, src_step;
	int *dst, *src;

	for (i=0; i<2; i++){
		if (!uwa->tau0_us[i]) {
			snd_pcm_areas_copy(dst_areas + i, dst_offset, src_areas + i, src_offset,
					1, size, SND_PCM_FORMAT_S32);
			return;
		}

		int delay = uwa->tau0_frames[i];

		if (delay > size)
			delay = size;
		
		dst = (int *)area_addr(dst_areas + i, dst_offset);
		dst_step = area_step(dst_areas + i) / 4;
		curpos = uwa->curpos[i];
		
		for (p = 0; p < delay; p++) {
				*dst = uwa->buf_tau0[i][curpos];
				dst += dst_step;
				curpos = (curpos + 1) % uwa->tau0_frames[i];
		}

		snd_pcm_area_copy(dst_areas + i, dst_offset + delay,
					src_areas + i, src_offset,
					size - delay, SND_PCM_FORMAT_S32);

		src = (int *)area_addr(src_areas + i, src_offset + size - delay);
		src_step = area_step(src_areas + i) / 4;
		curpos = uwa->curpos[i];
		for (p = 0; p < delay; p++) {
			uwa->buf_tau0[i][curpos] = *src;
			src += src_step;
			curpos = (curpos + 1) % uwa->tau0_frames[i];
		}
		uwa->curpos[i] = curpos;	
	}
};


static snd_pcm_sframes_t uwa_transfer(snd_pcm_extplug_t *ext,
	       			      const snd_pcm_channel_area_t *dst_areas,
	       			      snd_pcm_uframes_t dst_offset,
	       			      const snd_pcm_channel_area_t *src_areas,
	       			      snd_pcm_uframes_t src_offset,
	       			      snd_pcm_uframes_t size) {
	

	int width;
	int i, s, sts;
	pthread_t tid1, tid2;
	float max_val = 0 ;
	float val = 0;
	int temp_val;
	int *src = (int *) area_addr(src_areas,src_offset);

	//Passing Pointer from ALSA framework	
	snd_pcm_uwa_t *uwa = (snd_pcm_uwa_t *) ext; 

	//ALSA memory map parameters
	width = snd_pcm_format_physical_width(SND_PCM_FORMAT_S32);

	//UWA Channel Accelerator (UWA-CA) Parameters
	int nframes = 96;			
	int nchannels = 2;
	int packetsize = nframes * nchannels ;
	float threshold = 0.2;
	
	//DMA transfer always UWA-CA packet size
	uwa->tx_proxy_interface_p->length = packetsize * width / 8;  //convert frame to bytes
	uwa->rx_proxy_interface_p->length = packetsize * width / 8;

	if (!uwa->start_flag) {
		//Throw first 0.5 s because unstable value;
		if (!uwa->throw_unstable_flag) {
			if (uwa->framesent >= 24000){
				uwa->throw_unstable_flag = 1;
				uwa->framesent = 0;
				// Update first array of CIR
				reload_cir(uwa, uwa->cir_pos * uwa->ncoef/4, uwa->ncoef/4);
				printf ("CIR Updated, Position: %i \n", uwa->cir_pos);
				uwa->cir_pos += 1;
				printf("Emulator Ready. \n");
			} else {
				snd_pcm_areas_copy(dst_areas, dst_offset, src_areas , src_offset,
				 	 2, size, SND_PCM_FORMAT_S32);
				uwa->framesent+=size;
			};
		} else {
			//Find Max Value in Each Frame
			for (i = 0 ; i < size ; i++){
				temp_val = *(src + i);
				val = fixed24bittofloat(temp_val);
				if (max_val < val) {
					max_val = val;		
				};
			};
			if (max_val >= threshold) {
				uwa->start_flag = 1;
				uwa->framesent = 0;
				printf("Start Emulator! \n");	
	
				// Activate Hardware Accelerator
				UWA_CA_start(uwa);
				sts = XUwachannel_accelerator_IsReady (&uwa->uwa_ca_dev);

				// Transfer
				__retry:
				if (!sts){

				delayed_copy(uwa, uwa->uwa_input, 0, src_areas, src_offset, size);
				/*snd_pcm_areas_copy(uwa->uwa_input, 0, src_areas , src_offset,
					  2, size, SND_PCM_FORMAT_S32);*/

				s = pthread_create (&tid1, NULL, threadTX, uwa);
				if (s!=0)
					SNDERR("pthread_create TX failed");	

				s = pthread_create (&tid2, NULL, threadRX, uwa);
				if (s!=0)
					SNDERR("pthread_create RX failed");

				s = pthread_join(tid1, NULL);
				if (s!=0)
					SNDERR("pthread_join TX failed");

				s = pthread_join(tid2, NULL);
				if (s!=0)
					SNDERR("pthread_join RX failed");

				//curent assumption is fixed 96*2 frame size; 
				snd_pcm_areas_copy(dst_areas, dst_offset , uwa->uwa_output, 0,
						           2, size, SND_PCM_FORMAT_S32);	
	
				} else goto __retry; 

				uwa->framesent += size;

			} else {
				// Activate Hardware Accelerator
				UWA_CA_start(uwa);
				sts = XUwachannel_accelerator_IsReady (&uwa->uwa_ca_dev);

				// Transfer
				__retry2:
				if (!sts){

				delayed_copy(uwa, uwa->uwa_input, 0, src_areas, src_offset, size);
				/*snd_pcm_areas_copy(uwa->uwa_input, 0, src_areas , src_offset,
					  2, size, SND_PCM_FORMAT_S32);*/

				s = pthread_create (&tid1, NULL, threadTX, uwa);
				if (s!=0)
					SNDERR("pthread_create TX failed");	

				s = pthread_create (&tid2, NULL, threadRX, uwa);
				if (s!=0)
					SNDERR("pthread_create RX failed");

				s = pthread_join(tid1, NULL);
				if (s!=0)
					SNDERR("pthread_join TX failed");

				s = pthread_join(tid2, NULL);
				if (s!=0)
					SNDERR("pthread_join RX failed");

				//curent assumption is fixed 96*2 frame size; 
				snd_pcm_areas_copy(dst_areas, dst_offset , uwa->uwa_output, 0,
						           2, size, SND_PCM_FORMAT_S32);	
	
				} else goto __retry2; 

				uwa->framesent += size;
				//snd_pcm_areas_copy(dst_areas, dst_offset, src_areas , src_offset,
				 // 	               2, size, SND_PCM_FORMAT_S32);
			};

		};
		

	} else {

		//Update CIR
		if (uwa->time_variant){
			if (uwa->framesent == uwa->cir_update_rate_frames) {
				reload_cir(uwa, uwa->cir_pos * uwa->ncoef/4, uwa->ncoef/4);
				if (uwa->verbose > 0)
				printf ("CIR Updated, Position: %i \n", uwa->cir_pos);
				uwa->cir_pos += 1;
				uwa->framesent = 0;
				if (uwa->cir_pos == uwa->ncir){
					uwa->cir_pos = 0;
					printf ("NCIR: %i CIR Rewind!\n",uwa->ncir);
				}
			}	
		}
		// Activate Hardware Accelerator
		UWA_CA_start(uwa);
		sts = XUwachannel_accelerator_IsReady (&uwa->uwa_ca_dev);

		// Transfer
		_retry:
		if (!sts){
				delayed_copy(uwa, uwa->uwa_input, 0, src_areas, src_offset, size);
				/*snd_pcm_areas_copy(uwa->uwa_input, 0, src_areas , src_offset,
					  2, size, SND_PCM_FORMAT_S32);*/
				s = pthread_create (&tid1, NULL, threadTX, uwa);
				if (s!=0)
					SNDERR("pthread_create TX failed");	

				s = pthread_create (&tid2, NULL, threadRX, uwa);
				if (s!=0)
					SNDERR("pthread_create RX failed");

				s = pthread_join(tid1, NULL);
				if (s!=0)
					SNDERR("pthread_join TX failed");

				s = pthread_join(tid2, NULL);
				if (s!=0)
					SNDERR("pthread_join RX failed");

				snd_pcm_areas_copy(dst_areas, dst_offset , uwa->uwa_output, 0,
						           2, size, SND_PCM_FORMAT_S32);	
	
		} else goto _retry; 

		uwa->framesent += size;

	};

	return size; 
};

static int uwa_init (snd_pcm_extplug_t *ext) {
	
	int chn, status;
	int channels = 2; //this time fixed channel 
	int lines, l, i; 
 
    snd_pcm_uwa_t *uwa = (snd_pcm_uwa_t *) ext; 

	if (!uwa->init_done) {

		uwa->tx_proxy_fd = open("/dev/uwa_channel_tx", O_RDWR);

		if (uwa->tx_proxy_fd < 1) {
			SNDERR("Unable to open TX DMA proxy device");
			return -1;
		}

		uwa->rx_proxy_fd = open("/dev/uwa_channel_rx", O_RDWR);
		
		if (uwa->tx_proxy_fd < 1) {
			SNDERR("Unable to open RX DMA proxy device");
			return -1;
		}

		uwa->tx_proxy_interface_p = (uwa_dma_t *)
					mmap(NULL, sizeof(uwa_dma_t),
					PROT_READ | PROT_WRITE, MAP_SHARED, uwa->tx_proxy_fd, 0);

		uwa->rx_proxy_interface_p = (uwa_dma_t *)
					mmap(NULL, sizeof(uwa_dma_t),
					PROT_READ | PROT_WRITE, MAP_SHARED, uwa->rx_proxy_fd, 0);

			if ((uwa->rx_proxy_interface_p == MAP_FAILED) || 
			(uwa->tx_proxy_interface_p == MAP_FAILED)) {
				SNDERR("Failed to mmap\n");
				return 1;
			}
		
		uwa->uwa_input = calloc(2, sizeof(snd_pcm_channel_area_t));
			if (uwa->uwa_input == NULL) {
					printf("No enough memory\n");
					exit(EXIT_FAILURE);
			}
		
		uwa->uwa_output = calloc(2, sizeof(snd_pcm_channel_area_t));
			if (uwa->uwa_output == NULL) {
					printf("No enough memory\n");
					exit(EXIT_FAILURE);
			}

		for (chn = 0; chn < channels; chn++)  {
			uwa->uwa_input[chn].addr = &uwa->tx_proxy_interface_p->buffer ;
			uwa->uwa_input[chn].first = chn * snd_pcm_format_physical_width(SND_PCM_FORMAT_S32);
			uwa->uwa_input[chn].step = channels * snd_pcm_format_physical_width(SND_PCM_FORMAT_S32);
			
			uwa->uwa_output[chn].addr = &uwa->rx_proxy_interface_p->buffer ;
			uwa->uwa_output[chn].first = chn * snd_pcm_format_physical_width(SND_PCM_FORMAT_S32);
			uwa->uwa_output[chn].step = channels * snd_pcm_format_physical_width(SND_PCM_FORMAT_S32);
		}

		// Propagation Delay Init
		for (i=0; i < 2; i++) {
			uwa->tau0_frames[i] = time_to_frames (ext->rate, uwa->tau0_us[i]);
			printf("tau0_%i: %lu us, %i frames.\n",i ,uwa->tau0_us[i], (int)uwa->tau0_frames[i]);

			uwa->tau0[i] = calloc(1, sizeof(snd_pcm_channel_area_t));
				if (uwa->tau0[i] == NULL)
					return -ENOMEM;
			
			uwa->buf_tau0[i] = (int*)calloc(uwa->tau0_frames[i] , 
				(snd_pcm_format_physical_width(SND_PCM_FORMAT_S32)/8));
				if (uwa->buf_tau0[i] == NULL)
					return -ENOMEM;

			uwa->tau0[i]->addr = uwa->buf_tau0[i];
			uwa->tau0[i]->first = 0;
			uwa->tau0[i]->step = snd_pcm_format_physical_width(SND_PCM_FORMAT_S32);

			uwa->curpos[i] = 0;
		};

		
		// UWA-CA Init
		uwa->InstanceName = "uwachannel_accelerator";

		//Init Accelerator
		status = XUwachannel_accelerator_Initialize (&uwa->uwa_ca_dev,uwa->InstanceName);

		if (status != XST_SUCCESS)
		{
			puts("UWA Channel Accelerator Initialization Failed");
			exit (-1);
		}	
	
		// Init CIR file
		uwa->cir_pos = 0;
		uwa->start_flag = 0;

		uwa->ch1_cir_r_fd = fopen("cir_r_TX.dat","r");

		if (uwa->ch1_cir_r_fd == NULL) {
			SNDERR("Failed to open cir_r_TX.dat\n");
			exit (-1);
		};

		uwa->ch1_cir_i_fd = fopen("cir_i_TX.dat","r");

		if (uwa->ch1_cir_i_fd == NULL) {
			SNDERR("Failed to open cir_i_TX.dat\n");
			exit (-1);
		};

		uwa->ch2_cir_r_fd = fopen("cir_r_RX.dat","r");

		if (uwa->ch2_cir_r_fd == NULL) {
			SNDERR("Failed to open cir_r_RX.dat\n");
			exit (-1);
		};

		uwa->ch2_cir_i_fd = fopen("cir_i_RX.dat","r");

		if (uwa->ch2_cir_i_fd == NULL) {
			SNDERR("Failed to open cir_i_RX.dat\n");
			exit (-1);
		};

		lines = get_number_of_lines(uwa->ch1_cir_r_fd);
			
		l = get_number_of_lines(uwa->ch1_cir_i_fd);
		if (l != lines){
			SNDERR("CIR Number Inequal.");	
		}
		
		l = get_number_of_lines(uwa->ch2_cir_r_fd);
		if (l != lines){
			SNDERR("CIR Number Inequal.");	
		}
		
		l = get_number_of_lines(uwa->ch2_cir_i_fd);
		if (l != lines){
			SNDERR("CIR Number Inequal.");	
		}

		uwa->ncir=l/uwa->ncoef;

		if (uwa->ncir == 1){
			uwa->time_variant = 0;
			printf("NCIR Sets: %i , Time Invariant Channel. \n", uwa->ncir);
		} else {
			uwa->time_variant = 1;
			printf("NCIR Sets: %i , Time Variant Channel. \n", uwa->ncir);
			// CIR_update_rate init
			uwa->cir_update_rate_frames = time_to_frames (ext->rate, uwa->cir_update_rate_us);
			printf("CIR Update Rate: %lu us, %i frames.\n",uwa->cir_update_rate_us, (int)uwa->cir_update_rate_frames);
		}
		
		
		rewind (uwa->ch1_cir_r_fd);
		rewind (uwa->ch1_cir_i_fd);
		rewind (uwa->ch2_cir_r_fd);
		rewind (uwa->ch2_cir_i_fd);

		printf("Loading CIR...\n");
		cir_alloc_memory(&uwa->cir_r_1_buff_0, &uwa->cir_r_1_buff_1, 
						 &uwa->cir_r_1_buff_2, &uwa->cir_r_1_buff_3, lines/4);
		cir_alloc_memory(&uwa->cir_i_1_buff_0, &uwa->cir_i_1_buff_1, 
						 &uwa->cir_i_1_buff_2, &uwa->cir_i_1_buff_3, lines/4);
		cir_alloc_memory(&uwa->cir_r_2_buff_0, &uwa->cir_r_2_buff_1, 
						 &uwa->cir_r_2_buff_2, &uwa->cir_r_2_buff_3, lines/4);
		cir_alloc_memory(&uwa->cir_i_2_buff_0, &uwa->cir_i_2_buff_1, 
						 &uwa->cir_i_2_buff_2, &uwa->cir_i_2_buff_3, lines/4);

		//printf("Load CIR to buffer \n");
		cir_init_memory(uwa->ch1_cir_r_fd, uwa->cir_r_1_buff_0, uwa->cir_r_1_buff_1,
					    				   uwa->cir_r_1_buff_2, uwa->cir_r_1_buff_3, lines/4);
		cir_init_memory(uwa->ch1_cir_i_fd, uwa->cir_i_1_buff_0, uwa->cir_i_1_buff_1,
					    				   uwa->cir_i_1_buff_2, uwa->cir_i_1_buff_3, lines/4);
		cir_init_memory(uwa->ch2_cir_r_fd, uwa->cir_r_2_buff_0, uwa->cir_r_2_buff_1,
										   uwa->cir_r_2_buff_2, uwa->cir_r_2_buff_3, lines/4);
		cir_init_memory(uwa->ch2_cir_i_fd, uwa->cir_i_2_buff_0, uwa->cir_i_2_buff_1, 
										   uwa->cir_i_2_buff_2, uwa->cir_i_2_buff_3, lines/4);
		/*
		printf("cir_r_1_buff_0 address = %x\n",uwa->cir_r_1_buff_0);
		printf("cir_r_1_buff_0 address = %x\n",uwa->cir_r_1_buff_0);
		printf("cir_i_1_buff_0 address = %x\n",uwa->cir_i_1_buff_0);
		printf("cir_i_1_buff_1 address = %x\n",uwa->cir_i_1_buff_1);
		printf("cir_r_2_buff_0 address = %x\n",uwa->cir_r_2_buff_0);
		printf("cir_r_2_buff_1 address = %x\n",uwa->cir_r_2_buff_1);
		printf("cir_i_2_buff_0 address = %x\n",uwa->cir_i_2_buff_0);
		printf("cir_i_2_buff_1 address = %x\n",uwa->cir_i_2_buff_1);
		*/
		printf("CIR Load OK! \n");

		fclose (uwa->ch1_cir_r_fd);
		fclose (uwa->ch1_cir_i_fd);
		fclose (uwa->ch2_cir_r_fd);
		fclose (uwa->ch2_cir_i_fd);
		uwa->init_done = 1;

	}

	return 0;
}

static int uwa_close (snd_pcm_extplug_t *ext) {
	snd_pcm_uwa_t *uwa = (snd_pcm_uwa_t *) ext; 
	XUwachannel_accelerator_Release(&uwa->uwa_ca_dev);
	munmap(uwa->tx_proxy_interface_p, sizeof(uwa_dma_t));
	munmap(uwa->rx_proxy_interface_p, sizeof(uwa_dma_t));
	close(uwa->tx_proxy_fd);
	close(uwa->rx_proxy_fd);
	free (uwa->uwa_input);
	free (uwa->uwa_output);
	free (uwa->cir_r_1_buff_0);
	free (uwa->cir_i_1_buff_0);
	free (uwa->cir_r_2_buff_0);
	free (uwa->cir_i_2_buff_0);
	free (uwa->cir_r_1_buff_1);
	free (uwa->cir_i_1_buff_1);
	free (uwa->cir_r_2_buff_1);
	free (uwa->cir_i_2_buff_1);
	free (uwa->tau0[0]);
	free (uwa->tau0[1]);
	free (uwa->buf_tau0[0]);
	free (uwa->buf_tau0[1]);
	return 0;
}

static const snd_pcm_extplug_callback_t uwa_callback = {
	.transfer = uwa_transfer,
	.init = uwa_init,
	.close = uwa_close,
};


SND_PCM_PLUGIN_DEFINE_FUNC(uwa)
{
	snd_config_iterator_t i, next;
	struct snd_pcm_uwa *uwa_plug;
	snd_config_t *sconf = NULL;
	int err, verbose;
	long tau0_1_us,tau0_2_us,ncoef,cir_update_rate ;

	snd_config_for_each(i, next, conf) {
		snd_config_t *n = snd_config_iterator_entry(i);
		const char *id;
		if (snd_config_get_id(n, &id) < 0)
			continue;
		if (strcmp(id, "comment") == 0 || strcmp(id, "type") == 0 ||
		    strcmp(id, "hint") == 0)
			continue;
		if (strcmp(id, "slave") == 0) {
			sconf = n;
			continue;
		}
		if (strcmp(id, "tau0_1_us") == 0) {
			long val;
			err = snd_config_get_integer(n, &val);
			if (err < 0) {
				SNDERR("Invalid value for %s", id);
				return err;
			}
			tau0_1_us = val;
			continue;
		}
		if (strcmp(id, "tau0_2_us") == 0) {
			long val;
			err = snd_config_get_integer(n, &val);
			if (err < 0) {
				SNDERR("Invalid value for %s", id);
				return err;
			}
			tau0_2_us = val;
			continue;
		}
		if (strcmp(id, "ncoef") == 0) {
			long val;
			err = snd_config_get_integer(n, &val);
			if (err < 0) {
				SNDERR("Invalid value for %s", id);
				return err;
			}
			ncoef = val;
			continue;
		}
		if (strcmp(id, "cir_update_rate_us") == 0) {
			long val;
			err = snd_config_get_integer(n, &val);
			if (err < 0) {
				SNDERR("Invalid value for %s", id);
				return err;
			}
			cir_update_rate = val;
			continue;
		}
		if (strcmp(id, "verbose") == 0) {
			int val;
			err = snd_config_get_integer(n, &val);
			if (err < 0) {
				SNDERR("Invalid value for %s", id);
				return err;
			}
			verbose = val;
			continue;
		}
		SNDERR("Unknown field %s", id);
                return -EINVAL;
	}

	if (! sconf) {
		SNDERR("No slave configuration for uwa pcm");
		return -EINVAL;
	}

	uwa_plug = calloc(1, sizeof(*uwa_plug));
        if (uwa_plug == NULL)
                return -ENOMEM;

	uwa_plug->ext.version = SND_PCM_EXTPLUG_VERSION;
	uwa_plug->ext.name = "UWA Channel Emulator Plugin";
	uwa_plug->ext.callback = &uwa_callback;
	uwa_plug->ext.private_data = uwa_plug;

	// Limit Propagation delay to 1 - 3000000us (3 Sec)
	if (tau0_1_us <= 0)
		tau0_1_us = 1;
	else if (tau0_1_us > 3000000)
		tau0_1_us = 3000000;
	uwa_plug->tau0_us[0] = tau0_1_us;

	if (tau0_2_us <= 0)
		tau0_2_us = 1;
	else if (tau0_2_us > 3000000)
		tau0_2_us = 3000000;

	uwa_plug->tau0_us[1] = tau0_2_us;
	uwa_plug->init_done = 0;
	uwa_plug->ncoef = ncoef;

	if (cir_update_rate <= 30000 )
		cir_update_rate = 30000;
	else if (cir_update_rate > 3000000)
		cir_update_rate = 3000000;
	uwa_plug->cir_update_rate_us =cir_update_rate;

	if (verbose <= 0 )
		verbose = 0;
	else if (verbose > 1)
		verbose = 1;
	uwa_plug->verbose =verbose;

	err = snd_pcm_extplug_create(&uwa_plug->ext, name, root, sconf, stream, mode);
	if (err < 0) {
			free(uwa_plug);
			return err;
	}

	snd_pcm_extplug_set_param_minmax(&uwa_plug->ext,
					 SND_PCM_EXTPLUG_HW_CHANNELS,
					 2,2); // manual set to 2 channels
	snd_pcm_extplug_set_slave_param_minmax (&uwa_plug->ext, 
					 	SND_PCM_EXTPLUG_HW_CHANNELS, 
						2,2); // manual set to 2 channels 
	
	snd_pcm_extplug_set_param(&uwa_plug->ext, SND_PCM_EXTPLUG_HW_FORMAT,
				  SND_PCM_FORMAT_S32);
	snd_pcm_extplug_set_slave_param(&uwa_plug->ext, SND_PCM_EXTPLUG_HW_FORMAT,
					SND_PCM_FORMAT_S32);

        *pcmp = uwa_plug->ext.pcm;

	return 0;
}

SND_PCM_PLUGIN_SYMBOL(uwa);
