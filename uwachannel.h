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


