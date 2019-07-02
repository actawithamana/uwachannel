#include "xuwachannel_accelerator.h"

/********************************* DMA **************************************
 * This header file is shared between the DMA Proxy test application and 
 * the DMA Proxy device driver. It defines the shared interface to allow 
 * DMA transfers to be done from user space.
 * 
 * Note: the buffer in the data structure should be 1st in the channel 
 * interface so that the buffer is cached aligned, otherwise there may be 
 * issues when using cached memory. The issues were typically the 1st 32 
 * bytes of the buffer not working in the driver test.
 ****************************************************************************/

#define BUFFER_SIZE ( 1024 * 1024 )

typedef struct uwa_chan_user_interface {
	unsigned char buffer[BUFFER_SIZE]; 		
	enum proxy_status { PROXY_NO_ERROR = 0, 
			    PROXY_BUSY = 1, 
			    PROXY_TIMEOUT = 2, 
			    PROXY_ERROR = 3 } status;
	size_t length;
} uwa_dma_t;


/**************************** HLS IP CTRL************************************
 * Functions to remap IP Ctrl attached to AXI-Lite Interface under Generic
 * UIO framework. 
 ****************************************************************************/


/***************** Macros (Inline Functions) Definitions ********************/
#define MAX_UIO_PATH_SIZE       256
#define MAX_UIO_NAME_SIZE       64
#define MAX_UIO_MAPS            5
#define UIO_INVALID_ADDR        0

/**************************** Type Definitions ******************************/
typedef struct {
    u32 addr;
    u32 size;
} XUwachannel_accelerator_uio_map;


typedef struct {
    int  uio_fd;
    int  uio_num;
    char name[ MAX_UIO_NAME_SIZE ];
    char version[ MAX_UIO_NAME_SIZE ];
    XUwachannel_accelerator_uio_map maps[ MAX_UIO_MAPS ];
} XUwachannel_accelerator_uio_info;


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
    int autorestart;
	snd_pcm_uframes_t cir_update_rate_frames;
	// Propagation Delay (tau0) for two communication link
	snd_pcm_channel_area_t *tau0[2];
	int *buf_tau0[2];
	unsigned int curpos[2];
	long tau0_us[2];
	snd_pcm_uframes_t tau0_frames[2];
	//Zeros
	snd_pcm_channel_area_t *zeros;
	unsigned char  *buf_zeros[2];
	snd_pcm_channel_area_t *dummy;
	unsigned char *buf_dummy[2];
} snd_pcm_uwa_t;

