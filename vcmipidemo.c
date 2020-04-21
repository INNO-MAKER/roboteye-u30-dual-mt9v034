/**********************************************************************//**
***************************************************************************
*** @file    vcmipidemo.c
***
*** @brief   image Acuisition Example for Mipi Devices.
***
*** @author  Copyright (c) 2018 Vision Components.
*** @author  All rights reserved.
*** @author  This software embodies materials and concepts which are
***          confidential to Vision Components.
***
*** @revisionHistory
***
***    Date          Version    Author  Changes
***    17.10.2018    0.0.0      MBE     Initial Version.
***
*** @endRevisionHistory
***************************************************************************
***************************************************************************/
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include <syslog.h>
#include <time.h>
#include <sys/time.h>

#include "vclib-excerpt.h"
#include "vcimgnet.h"


//#define DURATION_TEST


#define  DEMO_NAME          "vcmipidemo"
#define  DEMO_MAINVERSION    (  0)  /**<  Main Version: X.-.-   */
#define  DEMO_VERSION        (  0)  /**<       Version: -.X.-   */
#define  DEMO_SUBVERSION     (  3)  /**<    Subversion: -.-.X   */



/*--*STRUCT*----------------------------------------------------------*/
/**
*  @brief  Capture Queue Slot.
*
*    This structure represents a capture queue slot
*    through which data of captured images is accessible.
*/
typedef struct
{
	void    *st;          /*!<   Start Address of Image Data          */
	size_t   byteCount;   /*!<   Size of the buffer in Bytes          */
} QBuf;
#define  NULL_QBuf { NULL, 0 }


/*--*STRUCT*----------------------------------------------------------*/
/**
*  @brief  Sensor Access and Attributes, Image Capture Queue Slots.
*
*    This structure holds the file descriptor of the sensor for access
*    as well as sensor specific information, like width and height of
*    the pixel image. Also available are the capture queue slots to
*    be able to access the recorded images.
*/
typedef struct
{
	int      fd;  /*!<  File Descriptor of the opened Sensor Device.  */

	QBuf    *qbuf; /*!<  Queue Buffers where Images are recorded to.  */
	U32      qbufCount; /*!<  Number of Queue Buffers available.      */

	struct v4l2_pix_format  pix;  /*!<  Sensor Attributes.            */
} VCMipiSenCfg;
#define NULL_VCMipiSenCfg  { -1, NULL,0, {0} }


int  change_options_by_commandline(int argc, char *argv[], int *shutter, float *gain, int *fbOutIff1, char *pcFramebufferDev, int *stdOutIff1, int *fileOutIff1, int *bufCount);
int  sensor_open(char *dev_video_device, VCMipiSenCfg *sen, int qBufCount);
int  sensor_close(VCMipiSenCfg *sen);
int  sensor_set_parameters(VCMipiSenCfg  *sen, int newGain, int newShutter);
int  sensor_streaming_start(VCMipiSenCfg *sen);
int  sensor_streaming_stop(VCMipiSenCfg *sen);
int  capture_buffer_enqueue(I32 bufIdx, VCMipiSenCfg *sen);
int  capture_buffer_dequeue(I32 *bufIdx, VCMipiSenCfg *sen);
int  wait_for_next_capture(VCMipiSenCfg  *sen, int timeoutUS);
int  imgnet_connect(VCImgNetCfg *imgnetCfg, U32 pixelformat, int dx, int dy);
int  imgnet_disconnect(VCImgNetCfg *imgnetCfg);
int  process_capture(unsigned int pixelformat, void *st, int dx, int dy, int pitch, int stdOutIff1, int netSrvOutIff1, VCImgNetCfg *imgnetCfg, int fbOutIff1, int fileOutIff1, int frameNr, char *pcFramebufferDev);
I32  copy_grey_to_image(image *imgOut, char *bufIn, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  convert_raw10_to_image(image *imgOut, char *bufIn, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  convert_raw10_and_debayer_image(image *imgOut, char *bufIn, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
I32  simple_debayer_to_image(image *imgOut, char *bufIn, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes);
int  copy_image(image *in, image *out);
int  copy_image_to_framebuffer(char *pcFramebufferDev, const void *pvDataGREY_OR_R, const void *pvDataGREY_OR_G, const void *pvDataGREY_OR_B, I32 dy, I32 pitch);
I32  write_image_as_pnm(char *path, image *img);
void print_image_to_stdout(image *img, int stp, int goUpIff1);
void timemeasurement_start(struct  timeval *timer);
void timemeasurement_stop(struct  timeval *timer, I64 *s, I64 *us);





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Main Function of vcmipidemo.
*
*  This is the main function of the vcmipidemo.
*/
/*-----------------------------------------------------------------------------*/
int  main(int argc, char *argv[])
{
	char           acVideoDev[]       = "/dev/video0";
	char           acFramebufferDev[] = "/dev/fb0";
	int            timeoutUS          = 10000;

	#ifdef DURATION_TEST
		struct timeval timer;
		I64            seconds, useconds;
		int            timerCycles = 100;
		int            run=0;
	#endif

	int            ee, rc=0, bufIdx;
	int            netSrvIff1 = 0;
	int            frameNr=0;
	int            optShutter, optFBOutIff1, optStdOutIff1, optBufCount, optFileOutIff1;
	float          optGain;
	VCMipiSenCfg   sen       = NULL_VCMipiSenCfg;
	VCImgNetCfg    imgnetCfg = NULL_VCImgNetCfg;

	// Set up configuration and apply command line parameters if set.
	{
		optStdOutIff1= +1;
		optFBOutIff1 = -1;
		optShutter   = 5000;
		optGain      = 10;
		optBufCount  = 3;

		rc =  change_options_by_commandline(argc, argv, &optShutter, &optGain, &optFBOutIff1, acFramebufferDev, &optStdOutIff1, &optFileOutIff1, &optBufCount);
		if(rc>0){ee=0; goto quit;}
		if(rc<0){ee=-1+100*rc; goto quit;}
	}


	// Gets capture dimensions for imgnet_connect().
	rc =  sensor_open(acVideoDev, &sen, optBufCount);
	if(rc<0){ee=-2+100*rc; goto quit;}

	// If vcimgnetsrv is started in background, this connects to it to transfer the captures.
	rc =  imgnet_connect(&imgnetCfg, sen.pix.pixelformat, sen.pix.width, sen.pix.height);
	if(rc!=0){ netSrvIff1=0; }
	else     { netSrvIff1=1; }


	// Apply new Shutter and Gain Settings
	{
		rc =  sensor_set_parameters(&sen, optGain, optShutter);
		if(rc<0){ee=-3+100*rc; goto quit;}
	}


	// Pre-Enqueue all capture buffers into the capture queue
	{
		for(bufIdx= 0; bufIdx< sen.qbufCount; bufIdx++)
		{
			rc =  capture_buffer_enqueue(bufIdx, &sen);
			if(rc<0){ee=-4+100*rc; goto quit;}
		}
	}

	rc =  sensor_streaming_start(&sen);
	if(rc<0){ee=-5+100*rc; goto quit;}

	#ifdef DURATION_TEST
		timemeasurement_start(&timer);
	#endif
	while(1)
	{
		rc =  wait_for_next_capture(&sen, timeoutUS);
		if(rc<0){ee=-6+100*rc; goto quit;}

		rc =  capture_buffer_dequeue(&bufIdx, &sen);
		if(rc>0){continue;} //buffer not yet available, wait again.
		if(rc<0){ee=-7+100*rc; goto quit;}

		rc =  process_capture(sen.pix.pixelformat, sen.qbuf[bufIdx].st, sen.pix.width, sen.pix.height, sen.pix.bytesperline, optStdOutIff1, netSrvIff1, &imgnetCfg, optFBOutIff1, optFileOutIff1, frameNr++, acFramebufferDev);
		if(rc<0){ee=-8+100*rc; goto quit;}

		rc =  capture_buffer_enqueue(bufIdx, &sen);
		if(rc<0){ee=-9+100*rc; goto quit;}

		#ifdef DURATION_TEST
			// Print Out Duration.
			if(((timerCycles)-1)==(run%(timerCycles)))
			{
				timemeasurement_stop(&timer, &seconds, &useconds);
				printf("Acquisiton&Copy Duration:%11llds%11lldus  for %d Cycles ==  %ffps.\n\n", seconds, useconds, (run%timerCycles)+1, (F32)1000000 * ((run%timerCycles)+1)/(seconds * 1000000 + useconds));
				if(1!=netSrvIff1){ printf("\033[%dA", 2); }
				timemeasurement_start(&timer);
			}
			run++;
		#endif
	}

	rc =  sensor_streaming_stop(&sen);
	if(rc<0){ee=-10+100*rc; goto quit;}


	ee=0;
quit:
	if(ee!=0){ printf("\n  '%s' quits with error code: %d\n\n", argv[0], ee); }

	sensor_close(&sen);

	if(1==netSrvIff1)
	{
		imgnet_disconnect(&imgnetCfg);
	}

	return(0);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Processes a Capture: Copy it to several Outputs.
*
*  This function processes a capture image by copying it to selected outputs.
*/
/*-----------------------------------------------------------------------------*/
int  process_capture(unsigned int pixelformat, void *st, int dx, int dy, int pitch, int stdOutIff1, int netSrvOutIff1, VCImgNetCfg *imgnetCfg, int fbOutIff1, int fileOutIff1, int frameNr, char *pcFramebufferDev)
{
	int    rc, ee;
	image  imgConverted = NULL_IMAGE;
	char   acFilename[256];

	// Allocate temporary image
	{
		imgConverted.type = (V4L2_PIX_FMT_SRGGB10P==pixelformat)?(IMAGE_RGB):(IMAGE_GREY);
		imgConverted.dx   = dx;
		imgConverted.dy   = dy;
		imgConverted.pitch= imgConverted.dx;
		imgConverted.ccmp1= malloc(sizeof(U8) * imgConverted.dy * imgConverted.pitch);
		if(NULL==imgConverted.ccmp1){ee=-1; goto fail;}
		imgConverted.ccmp2= malloc(sizeof(U8) * imgConverted.dy * imgConverted.pitch);
		if(NULL==imgConverted.ccmp2){ee=-2; goto fail;}
		imgConverted.st   =  malloc(sizeof(U8) * imgConverted.dy * imgConverted.pitch);
		if(NULL==imgConverted.st   ){ee=-3; goto fail;}
	}

	switch(pixelformat)
	{
		case V4L2_PIX_FMT_GREY:
				rc =  copy_grey_to_image(&imgConverted,  st,  0, 0, dx, dy, dx, 0);
				if(rc<0){ee=-4+100*rc; goto fail;}
			break;
		case V4L2_PIX_FMT_Y10:
				rc =  convert_raw10_to_image(&imgConverted,  st, 0,  0, 0, dx, dy, dx, pitch - (10 * dx)/8);
				if(rc<0){ee=-5+100*rc; goto fail;}
			break;
		case V4L2_PIX_FMT_SRGGB10P:
				rc =  convert_raw10_and_debayer_image(&imgConverted, st, 0,  0, 0, dx, dy, dx, pitch - (10 * dx)/8);
				if(rc<0){ee=-6+100*rc; goto fail;}
			break;
		default:
			printf("Error, Pixelformat unsupported: %c%c%c%c (0x%08x)\n",
					(char)((pixelformat >>  0)&0xFF),
					(char)((pixelformat >>  8)&0xFF),
					(char)((pixelformat >> 16)&0xFF),
					(char)((pixelformat >> 24)&0xFF),
					pixelformat);
			ee=-7; goto fail;
	}


	if(1==stdOutIff1)
	{
		print_image_to_stdout(&imgConverted, 50, 1);
	}

	if(1==netSrvOutIff1)
	{
		rc =  copy_image(&imgConverted, &(imgnetCfg->img));
		if(rc<0){ee=-8+100*rc; goto fail;}
	}

	if(1==fbOutIff1)
	{
		if(IMAGE_GREY==imgConverted.type)
		{
			rc =  copy_image_to_framebuffer(pcFramebufferDev, imgConverted.st, imgConverted.st,    imgConverted.st,    imgConverted.dy, imgConverted.pitch);
		}
		else
		{
			rc =  copy_image_to_framebuffer(pcFramebufferDev, imgConverted.st, imgConverted.ccmp1, imgConverted.ccmp2, imgConverted.dy, imgConverted.pitch);
		}
		if(rc<0){ee=-9+100*rc; goto fail;}
	}

	if(1==fileOutIff1)
	{
		snprintf(acFilename, 255, "img%05d", frameNr);

		rc =  write_image_as_pnm(acFilename, &imgConverted);
		if(rc<0){ee=-10+100*rc; goto fail;}
	}


	ee=0;
fail:
	if(NULL!=imgConverted.st   ){ free(imgConverted.st   );  imgConverted.st   =NULL; }
	if(NULL!=imgConverted.ccmp1){ free(imgConverted.ccmp1);  imgConverted.ccmp1=NULL; }
	if(NULL!=imgConverted.ccmp2){ free(imgConverted.ccmp2);  imgConverted.ccmp2=NULL; }

	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Parses Command Line Parameters.
*
*  This function parses command line parameters.
*/
/*-----------------------------------------------------------------------------*/
int  change_options_by_commandline(int argc, char *argv[], int *shutter, float *gain, int *fbOutIff1, char *pcFramebufferDev, int *stdOutIff1, int *fileOutIff1, int *bufCount)
{
	int  opt;

	while((opt =  getopt(argc, argv, "g:s:fab:o")) != -1)
	{
		switch(opt)
		{
			default:
				printf("_______________________________________________________________________________\n");
				printf("                                                                               \n");
				printf("  %s v.%d.%d.%d.\n", DEMO_NAME, DEMO_MAINVERSION, DEMO_VERSION, DEMO_SUBVERSION);
				printf("  -----------------------------------------------------------------------------\n");
				printf("                                                                               \n");
				printf("  Usage: %s [-s sh] [-g gain] [-f] [-a]\n", argv[0]);
				printf("                                                                               \n");
				printf("  -s,  Shutter Time.                                                           \n");
				printf("  -g,  Gain Value.                                                             \n");
				printf("  -b,  Buffer Count to use.                                                    \n");
				printf("  -f,  Output Capture to framebuffer %s.                                       \n", pcFramebufferDev);
				printf("  -o,  Output Captures to file in PGM or PPM format (openable by e.g. GIMP)    \n");
				printf("  -a,  Suppress ASCII capture at stdout.                                       \n");
				printf("_______________________________________________________________________________\n");
				printf("                                                                               \n");
				return(+1);
			case 's':  *shutter    = atol(optarg);  printf("Setting Shutter Value to %d.\n",*shutter);  break;
			case 'g':  *gain       = atof(optarg);  printf("Setting Gain Value to %f.\n",   *gain   );  break;
			case 'f':  *fbOutIff1  = 1;             printf("Activating /dev/fb0 framebuffer output.\n");break;
			case 'a':  *stdOutIff1 = 0;             printf("Suppressing ASCII capture at stdout.\n" );  break;
			case 'o':  *fileOutIff1=1;             printf("Activating file output of captures.\n" );   break;
			case 'b':  *bufCount   = atol(optarg);  printf("Setting Buffer Count to %d.\n",*bufCount);  break;
		}
	}

	if(argc<2)
	{
		printf("  Hint: Activate framebuffer output by command line option (see:  %s -? )\n", argv[0]);
	}

	return(0);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Connects to a running vcimgnetsrv.
*
*  This function connects to a running vcimgnetsrv.
*  Normally the vcimgnetsrv application is started beforehand in background
*  using the following command:
*
*   vcimgnetsrv &
*/
/*-----------------------------------------------------------------------------*/
int  imgnet_connect(VCImgNetCfg *imgnetCfg, U32 pixelformat, int dx, int dy)
{
	//predefine dimensions for the image to be transferred
	imgnetCfg->img.type  = (V4L2_PIX_FMT_SRGGB10P==pixelformat)?(IMAGE_RGB):(IMAGE_GREY);
	imgnetCfg->img.dx    = dx;
	imgnetCfg->img.dy    = dy;
	imgnetCfg->img.pitch = imgnetCfg->img.dx;
	imgnetCfg->img.st    = NULL;
	imgnetCfg->img.ccmp1 = NULL;
	imgnetCfg->img.ccmp2 = NULL;

	return(vcimgnet_attach(&(imgnetCfg->img), imgnetCfg));
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Disconnects from a running vcimgnetsrv.
*
*  This function disconnects from a running vcimgnetsrv.
*  The vcimgnetsrv application will keep running.
*/
/*-----------------------------------------------------------------------------*/
int  imgnet_disconnect(VCImgNetCfg *imgnetCfg)
{
	return(vcimgnet_detach(imgnetCfg));
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Tells the Sensor to Start Streaming Recordings.
*
*  This function tells the sensor to start streaming recordings.
*  To get access to the streamed images
*  buffers (which will be filled by the stream) must be enqueued,
*  waited until one is filled, and dequeued.
*/
/*-----------------------------------------------------------------------------*/
int  sensor_streaming_start(VCMipiSenCfg *sen)
{
	I32                 ee, rc;
	enum v4l2_buf_type  type;


	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	rc =  ioctl(sen->fd, VIDIOC_STREAMON, &type);
	if(rc<0){ee=-1; goto fail;}


 	ee=0;
fail:
	switch(ee)
	{
		case 0:
			break;
		case -1:
			syslog(LOG_ERR, "%s():  ioctl(VIDIOC_STREAMON) throws Error!\n", __FUNCTION__);
			break;
	}
	return(ee);
}




/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Tells the Sensor to Stop Streaming Recordings.
*
*  This function tells the sensor to stop streaming recordings.
*/
/*-----------------------------------------------------------------------------*/
int  sensor_streaming_stop(VCMipiSenCfg *sen)
{
	I32                 ee, rc;
	enum v4l2_buf_type  type;


	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	rc =  ioctl(sen->fd, VIDIOC_STREAMOFF, &type);
	if(rc<0){ee=-1; goto fail;}


 	ee=0;
fail:
	switch(ee)
	{
		case 0:
			break;
		case -1:
			syslog(LOG_ERR, "%s():  ioctl(VIDIOC_STREAMOFF) throws Error!\n", __FUNCTION__);
			break;
	}
	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Opens the Capture Device and Retreives its Attributes.
*
*  This function opens the capture device and retreives its attributes.
*/
/*-----------------------------------------------------------------------------*/
int  sensor_open(char *dev_video_device, VCMipiSenCfg *sen, int qbufCount)
{
	I32    ee, rc, i;


	// Reset Allocation Markers
	// prohibits closing of un-open device and prevents wrong deallocation or unmapping.
	{
		sen->fd        =   -1;
		sen->qbuf      = NULL;
		sen->qbufCount =   -1;
	}


	// Open the Device.
	{
		sen->fd =  open(dev_video_device, O_RDWR, 0);
		if(sen->fd<0){ee=-1; goto fail;}
	}


	// Check the Capabilities of the Device.
	{
		struct v4l2_capability  cap;

		rc =  ioctl(sen->fd, VIDIOC_QUERYCAP, &cap);
		if(rc<0){ee=-2; goto fail;}

		// Check if device can Capture Videos and supports mmap access.
		if(0==(cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING)))
		{ee=-3; goto fail;}

		syslog(LOG_DEBUG, "%s:  Initialized Capture Device '%s' (fd:%d):\n", __FILE__, dev_video_device, sen->fd);
	}


	// Retreive Dimensions of the Camera Device.
	{
		struct v4l2_format  format;

		format.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.width       = UINT_MAX;
		format.fmt.pix.height      = UINT_MAX;
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y10;

		rc = ioctl(sen->fd, VIDIOC_G_FMT, &format);
		if(rc<0){ee=-4; goto fail;}

		sen->pix = format.fmt.pix;

		syslog(LOG_DEBUG, "%s:    Pixel Format used:        %d          \n", __FILE__, sen->pix.pixelformat );
		syslog(LOG_DEBUG, "%s:    Maximum Pixel Width:      %d          \n", __FILE__, sen->pix.width       );
		syslog(LOG_DEBUG, "%s:    Maximum Pixel Height:     %d          \n", __FILE__, sen->pix.height      );
		syslog(LOG_DEBUG, "%s:    Bytes Per Line:           %d          \n", __FILE__, sen->pix.bytesperline);
	}


	// Allocate Capture Buffer Pointer 'Array'
	{
		QBuf  imgBufNuller = NULL_QBuf;

		sen->qbuf =  malloc(sizeof(QBuf) * qbufCount);
		if(NULL==sen->qbuf){ee=-99; goto fail;}

		for(i= 0; i< qbufCount; i++)
		{
			sen->qbuf[i] = imgBufNuller;
		}
	}

	// Request the Count of Capture Buffers
	{
		struct v4l2_requestbuffers  req;

		memset(&req, 0, sizeof(struct v4l2_requestbuffers));
		req.memory = V4L2_MEMORY_MMAP;
		req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		req.count  = qbufCount;

		rc =  ioctl(sen->fd, VIDIOC_REQBUFS, &req);
		if(rc<0){ee=-5; goto fail;}

		if(req.count != qbufCount){ee=-6; goto fail;}
	}

	// Map the capture buffer memory addresses to the Capture Buffer Pointer 'Array'
	{
		struct v4l2_buffer  buf;

		sen->qbufCount = 0;
		for(i= 0; i< qbufCount; i++)
		{
			memset(&buf, 0, sizeof(struct v4l2_buffer));
			buf.memory = V4L2_MEMORY_MMAP;
			buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.index  = sen->qbufCount;

			rc =  ioctl(sen->fd, VIDIOC_QUERYBUF, &buf);
			if(rc<0){ee=-7; goto fail;}

			sen->qbuf[sen->qbufCount].byteCount = buf.length;
			sen->qbuf[sen->qbufCount].st        = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, sen->fd, buf.m.offset);
			if(MAP_FAILED==sen->qbuf[sen->qbufCount].st){ee=-8; goto fail;}

			sen->qbufCount++;
		}
	}


	ee = 0;
fail:
	if(ee<0)
	{
		sensor_close(sen);
	}
	switch(ee)
	{
		case 0:
			break;
		case -1:
			syslog(LOG_ERR, "%s():  Could not open device '%s' for Reading/Writing!\n", __FUNCTION__, dev_video_device);
			break;
		case -2:
			if(errno == EINVAL)
				syslog(LOG_ERR, "%s():  Device '%s' is no V4L2 Device!\n", __FUNCTION__, dev_video_device);
			else
				syslog(LOG_ERR, "%s():  ioctl(VIDIOC_QUERYCAP) throws Error on Device '%s'!\n", __FUNCTION__, dev_video_device);
			break;
		case -3:
			syslog(LOG_ERR, "%s():  Device '%s' is no Capture Device or doesn't support mmap as IO Method!\n", __FUNCTION__, dev_video_device);
			break;
		case -4:
			syslog(LOG_ERR, "%s():  ioctl(VIDIOC_G_FMT) throws Error on Device '%s'!\n", __FUNCTION__, dev_video_device);
			break;
		case -5:
			syslog(LOG_ERR, "%s():  ioctl(VIDIOC_REQBUFS) throws Error on Device '%s'!\n", __FUNCTION__, dev_video_device);
			break;
		case -6:
			syslog(LOG_ERR, "%s():  Requested buffers differ from returned buffers!\n", __FUNCTION__);
			break;
		case -7:
			syslog(LOG_ERR, "%s():  ioctl(VIDIOC_QUERYBUF) throws Error on Device '%s'!\n", __FUNCTION__, dev_video_device);
			break;
		case -8:
			syslog(LOG_ERR, "%s():  mmap() failed for Buffer %d!\n", __FUNCTION__, sen->qbufCount);
			break;
		case -99:
			syslog(LOG_ERR, "%s():  Out of Memory!\n", __FUNCTION__);
			break;
		default:
			syslog(LOG_ERR, "%s():  Unknown error code: %d!\n", __FUNCTION__, ee);
			break;
	}

	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Closes the Capture Device.
*
*  This function closes the capture device.
*/
/*-----------------------------------------------------------------------------*/
int  sensor_close(VCMipiSenCfg *sen)
{
	I32  ee, rc, i;


	// checks if device has been opened (see sensor_open() and NULL_VCMipiSenCfg)
	if(sen->fd>=0)
	{
		//rc = uninit_mmap();
		if(NULL!=sen->qbuf)
		{
			for(i=sen->qbufCount; i> 0; i--)
			{
				rc =  munmap(sen->qbuf[i-1].st, sen->qbuf[i-1].byteCount);
				if(rc<0){ee=-1-10*i; goto fail;}

				sen->qbufCount--;
			}

			free(sen->qbuf);
			sen->qbuf = NULL;
		}

		// Close Video Device.
		{
			rc =  close(sen->fd);
			if(rc<0){return -2;}

			sen->fd = -1;
		}
	}

	ee=0;
fail:
	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Enqueues a Buffer into the Capture Queue.
*
*  This function enqueues a buffer into the capture queue.
*
*  If the sensor acquires images, e.g. by being in streaming mode,
*  buffers enqueued at the capture queue will be filled with recorded data.
*  After successful waiting for a recording,
*  the image can be accessed by dequeuing the buffer from the capture queue.
*/
/*-----------------------------------------------------------------------------*/
int  capture_buffer_enqueue(I32 bufIdx, VCMipiSenCfg *sen)
{
	I32                 ee, rc;
	struct v4l2_buffer  buf;

	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.memory = V4L2_MEMORY_MMAP;
	buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.index  = bufIdx;

	rc =  ioctl(sen->fd, VIDIOC_QBUF, &buf);
	if(rc<0){ee=-9; goto fail;}


	ee = 0;
fail:
	switch(ee)
	{
		case 0:
			break;
		case -9:
			syslog(LOG_ERR, "%s():  ioctl(VIDIOC_QBUF) throws Error!\n", __FUNCTION__);
			break;
		default:
			syslog(LOG_ERR, "%s():  Unknown error code: %d!\n", __FUNCTION__, ee);
			break;
	}

	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Dequeues a Buffer from the Capture Queue.
*
*  This function dequeues a buffer from the capture queue.
*
*  If the sensor acquires images, e.g. by being in streaming mode,
*  buffers enqueued at the capture queue will be filled with recorded data.
*  After successful waiting for a recording,
*  the image can be accessed by dequeuing the buffer from the capture queue.
*/
/*-----------------------------------------------------------------------------*/
int  capture_buffer_dequeue(I32 *bufIdx, VCMipiSenCfg *sen)
{
	I32                 ee, rc;
	struct v4l2_buffer  buf;

	memset(&buf, 0, sizeof(struct v4l2_buffer));
	buf.memory = V4L2_MEMORY_MMAP;
	buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	rc =  ioctl(sen->fd, VIDIOC_DQBUF, &buf);
	if(rc<0)
	{
		if(EAGAIN==errno){ee=+1; goto fail;} // Buffer is not available.
		else             {ee=-1; goto fail;}
	}

	if(buf.index >= sen->qbufCount)
	{
		ee=-2; goto fail;
	}

	*bufIdx = buf.index;


	ee = 0;
fail:
	switch(ee)
	{
		case +1:
			syslog(LOG_ERR, "%s():  Buffer not available.\n", __FUNCTION__);
			break;
		case 0:
			break;
		case -1:
			syslog(LOG_ERR, "%s():  ioctl(VIDIOC_DQBUF) throws Error!\n", __FUNCTION__);
			break;
		case -2:
			syslog(LOG_ERR, "%s():  Buffer index returned exceeds limits: %d>=%d!\n", __FUNCTION__, buf.index, sen->qbufCount);
			break;
		default:
			syslog(LOG_ERR, "%s():  Unknown error code: %d!\n", __FUNCTION__, ee);
			break;
	}

	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Waits for a Buffer at the Capture Queue to be filled with captured Data.
*
*  This function waits for a buffer at the capture queue to be filled with
*  data of a recording.
*
*  If the sensor acquires images, e.g. by being in streaming mode,
*  buffers enqueued at the capture queue will be filled with recorded data.
*  After successful waiting for a recording,
*  the image can be accessed by dequeuing the buffer from the capture queue.
*/
/*-----------------------------------------------------------------------------*/
int  wait_for_next_capture(VCMipiSenCfg  *sen, int timeoutUS)
{
	I32             ee, rc;
	fd_set          fdSet;
	struct timeval  tv;

	while(1)
	{
		FD_ZERO(&fdSet);
		FD_SET(sen->fd, &fdSet);
		tv.tv_sec  = (timeoutUS/1000000);
		tv.tv_usec = (timeoutUS%1000000);

		// Wait for data.
		rc =  select(sen->fd + 1, &fdSet, NULL, NULL, &tv);
		if(rc<0)
		{
			// Ignore interrupt based select returns.
			if(EINTR==errno){continue;        }
			else            {ee=-1; goto fail;}
		}
		if(0==rc){ee=+1; goto fail;} //select() timeout.

		// New data is available.
		break;
	}


	ee = 0;
fail:
	switch(ee)
	{
		case +1:
			syslog(LOG_ERR, "%s():  select() timeout.\n", __FUNCTION__);
			break;
		case 0:
			break;
		case -1:
			syslog(LOG_ERR, "%s():  select() failed!\n", __FUNCTION__);
			break;
	}

	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Requests new Settings to the Sensor Device.
*
*  This function requests new Settings to the Sensor Device
*  like an exposure time or gain.
*  When the settings become operational depends on the sensor and its configuration.
*/
/*-----------------------------------------------------------------------------*/
int  sensor_set_parameters(VCMipiSenCfg  *sen, int newGain, int newShutter)
{
	I32    ee, rc, target;
	U32    ctlID, val;
	char   a10cTarget[11];
	struct v4l2_control  ctl;


	for(target= 0; target< 2; target++)
	{
		switch(target)
		{
			case 0:  sprintf(a10cTarget,"Gain"    );  ctlID = V4L2_CID_GAIN;      val = newGain;     break;
			case 1:  sprintf(a10cTarget,"Exposure");  ctlID = V4L2_CID_EXPOSURE;  val = newShutter;  break;
		}

		// Only needed for debugging: Get old value.
		{
			memset(&ctl, 0, sizeof(ctl));
			ctl.id = ctlID;

			rc =  ioctl(sen->fd, VIDIOC_G_CTRL, &ctl);
			if(rc<0)
			{
				if(EINVAL!=errno){ee=-1; goto fail;} //general error.
				else             {ee=-2; goto fail;} //unsupported.
			}

			syslog(LOG_DEBUG, "%s():  Old %s Value: %d.\n", __FUNCTION__, a10cTarget, ctl.value);
		}

		// Set new value.
		{
			memset(&ctl, 0, sizeof(ctl));
			ctl.id    = ctlID;
			ctl.value = val;

			rc = ioctl(sen->fd, VIDIOC_S_CTRL, &ctl);
			if(rc<0)
			{
				if((EINVAL!=errno)&&(ERANGE!=errno)){ee=-3; goto fail;} //general error.
				else                                {ee=-4; goto fail;} //Value out of Range Error.
			}

			syslog(LOG_DEBUG, "%s():  Requested New %s Value: %d.\n", __FUNCTION__, a10cTarget, val);
		}

		// Only needed for debugging: Get new value.
		{
			memset(&ctl, 0, sizeof(ctl));
			ctl.id = ctlID;

			rc =  ioctl(sen->fd, VIDIOC_G_CTRL, &ctl);
			if(rc<0)
			{
				if(EINVAL!=errno){ee=-5; goto fail;} //general error.
				else             {ee=-6; goto fail;} //unsupported.
			}

			syslog(LOG_DEBUG, "%s():  New %s Value: %d.\n", __FUNCTION__, a10cTarget, ctl.value);
		}
	}


	ee = 0;
fail:
	switch(ee)
	{
		case 0:
			break;
		case -1:
		case -3:
		case -5:
			syslog(LOG_ERR, "%s():  ioctl(%s) throws Error (%d(%s))!\n", __FUNCTION__, (-3==ee)?("VIDIOC_S_CTRL"):("VIDIOC_G_CTRL"), errno, strerror(errno));
			break;
		case -2:
		case -6:
			syslog(LOG_ERR, "%s():  V4L2_CID_.. is unsupported!\n", __FUNCTION__);
			break;
		case -4:
			syslog(LOG_ERR, "%s():  %s Value is out of range (or V4L2_CID_.. is invalid)!\n", __FUNCTION__, a10cTarget);
			break;
	}

	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Copies an Image Buffer to another Image Buffer.
*
*  This function copies an image buffer to another image buffer.
*/
/*-----------------------------------------------------------------------------*/
int copy_image(image *in, image *out)
{
	int  ee, y;
	int  dx=min(in->dx,out->dx);
	int  dy=min(in->dy,out->dy);

	if(in->type != out->type) { ee=-1; goto fail; }

	#if _OPENMP
	#   pragma omp parallel for
	#endif
	for(y= 0; y< dy; y++)
	{
		memcpy((U8*)out->st + y * out->pitch, (U8*)in->st + y * in->pitch, dx);
	}
	if(IMAGE_RGB==out->type)
	{
		#if _OPENMP
		#   pragma omp parallel for
		#endif
		for(y= 0; y< dy; y++)
		{
			memcpy((U8*)out->ccmp1 + y * out->pitch,  (U8*)in->ccmp1 + y * in->pitch, dx);
		}

		#if _OPENMP
		#   pragma omp parallel for
		#endif
		for(y= 0; y< dy; y++)
		{
			memcpy((U8*)out->ccmp2 + y * out->pitch,  (U8*)in->ccmp2 + y * in->pitch, dx);
		}
	}

	ee=0;
fail:
	return(ee);
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Outputs Image Pixels to a Framebuffer Device.
*
*  This function outputs Image Pixels to to a framebuffer device.
*
* @param  stp        Print only each  @p stp  pixel.
* @param  goUpIff1   Overwrite old ASCII image, works only on recent terminals,
*                    and needs no other data lines to be printed.
*/
/*-----------------------------------------------------------------------------*/
int  copy_image_to_framebuffer(char *pcFramebufferDev, const void *pvDataGREY_OR_R, const void *pvDataGREY_OR_G, const void *pvDataGREY_OR_B, I32 dy, I32 pitch)
{
	I32                       rc, ee;

	U8                       *pInR   = NULL, *pInG   = NULL, *pInB   = NULL;
	U8                       *pOut   = NULL;
	int                       fbufFd = 0;
	U8                       *fbufSt = NULL;
	struct fb_var_screeninfo  fbufVars;
	struct fb_fix_screeninfo  fbufConsts;
	U32                       fbufByteCount;
	I32                       x, y, scaler;


	// Open the framebuffer for reading and writing
	{
		fbufFd =  open(pcFramebufferDev, O_RDWR);
		if(fbufFd<0){ee=-1; goto fail;}
	}

	// Get framebuffer information
	{
		// Variable information
		rc =  ioctl(fbufFd, FBIOGET_VSCREENINFO, &fbufVars  );
		if(rc<0){ee=-2+10*rc; goto fail;}

		// Constant information
		rc =  ioctl(fbufFd, FBIOGET_FSCREENINFO, &fbufConsts);
		if(rc<0){ee=-3+10*rc; goto fail;}
	}

	// Map the framebuffer to memory
	{
		fbufByteCount = fbufVars.xres * fbufVars.yres * fbufVars.bits_per_pixel / 8;

		fbufSt =  mmap(NULL, fbufByteCount, PROT_READ | PROT_WRITE, MAP_SHARED, fbufFd, 0);
		if(fbufSt<0){ee=-4; goto fail;}
	}

	// Approx. scale up/down to framebuffer size.
	scaler =  max(1,  min(pitch/fbufVars.xres, dy/fbufVars.yres));

	// Write pixel per pixel (slow)
	for(y = 0; y < min(fbufVars.yres, dy); y++)
	{
		pInR = ((U8*)pvDataGREY_OR_R) + (scaler * y) * pitch;
		pInG = ((U8*)pvDataGREY_OR_G) + (scaler * y) * pitch;
		pInB = ((U8*)pvDataGREY_OR_B) + (scaler * y) * pitch;
		pOut =       fbufSt  + (y + fbufVars.yoffset) * fbufConsts.line_length
		                     +      fbufVars.xoffset  * fbufVars.bits_per_pixel/8;

		for(x = 0; x < min(fbufVars.xres, pitch); x++)
		{
			*((U32*) pOut) = ((*pInR) << 16) | ((*pInG) << 8) | ((*pInB) << 0);

			pInR += scaler;
			pInG += scaler;
			pInB += scaler;
			pOut += fbufVars.bits_per_pixel/8;
		}
	}


	ee=0;
fail:
	if(NULL!=fbufSt){  munmap(fbufSt, fbufByteCount);  fbufSt = NULL; }
	close(fbufFd);

	return ee;
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Outputs Image Pixels to stdout.
*
*  This function outputs Image Pixels to stdout.
*
* @param  stp        Print only each  @p stp  pixel.
* @param  goUpIff1   Overwrite old ASCII image, works only on recent terminals,
*                    and needs no other data lines to be printed.
*/
/*-----------------------------------------------------------------------------*/
void  print_image_to_stdout(image *img,  int stp, int goUpIff1)
{
	static U8 noUpAtFirst = 1;

	I32 y,x;
	unsigned char *px=NULL;
	U8  c;

	if((1==goUpIff1)&&(noUpAtFirst!=1))
		printf("\033[%dA", img->dy/stp+1 +1);

	if(1==noUpAtFirst){ noUpAtFirst=0; }

	printf("//");
	for(x= 0; x< img->dx; x+=stp)
	{
		printf("==");
	}
	printf("\\\\\n");

	for(y= 0; y< img->dy; y+=stp)
	{
		px = (unsigned char *) img->st + y * img->pitch;

		printf("||");

		for(x= 0; x< img->dx; x+=stp)
		{
			c =  (*(px+x)< 40)?(' ')
				:(*(px+x)< 89)?('-')
				:(*(px+x)<138)?('+')
				:(*(px+x)<178)?('*')
				:(*(px+x)<216)?('X')
				:(              '#');

			printf("%c%c", c, c);
		}

		printf("||\n");
	}

	printf("\\\\");
	for(x= 0; x< img->dx; x+=stp)
	{
		printf("==");
	}
	printf("//\r\n");
}
















/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Converts One Line from RAW10 to 8 Bit Grey Value (Offset-Free).
*
*  This function converts one line of an image from RAW10 to an 8 bit grey value.
*
*  The RAW10 format has four bytes with each containing the uppermost 8 bits
*  of the pixels followed by one byte with the two lowermost bits of each of
*  the four preceeding pixels packed together:
*
*    X0, X1, X2, X3, LowerBitsOfX0..3,  X4, X5, X6, X7, LowerBitsOfX4..7, etc.
*
*  The algorithm simply copies the first four Bytes and skips over the following
*  byte with the lowermost bit information.
*
*  Since the starting position is relevant to know where the byte with the
*  lower bits is and to jump over it, this function needs the byte with the
*  lower bits at fifth position, like being shown at the example above.
*
*   there is   so it is important where the buffer begins: trackOffset==0->X0,trackOffset<4->X1..3.
*
* @param  count      Length of the conversion, should be the width of the output image.
* @param  bufIn      RAW10 encoded data: if the output image has width bytes,
*                    this buffer should have at least (width * 10)/8 bytes.
* @param  bufOut     8 Bit Grey Value Output Address.
*/
/*-----------------------------------------------------------------------------*/
inline void  FL_CPY_RAW10P_U8P_NOOFFS(U32 count, char *bufIn, U8 *bufOut)
{
	while(count >= 4)
	{
		*((U32*)bufOut) = *((U32*)bufIn);
		bufIn+=5;
		bufOut+=4;

		count -= 4;
	}

	while(count--)
	{
		*bufOut = (U8)(*bufIn);
		bufIn++;
		bufOut++;
	}
}





/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Converts One Line from RAW10 to 8 Bit Grey Value.
*
*  This function converts one line of an image from RAW10 to an 8 bit grey value.
*
*  The RAW10 format has four bytes with each containing the uppermost 8 bits
*  of the pixels followed by one byte with the two lowermost bits of each of
*  the four preceeding pixels packed together:
*
*    X0, X1, X2, X3, LowerBitsOfX0..3,  X4, X5, X6, X7, LowerBitsOfX4..7, etc.
*
*  The algorithm simply copies the first four Bytes and skips over the following
*  byte with the lowermost bit information.
*
*  Since the starting position is relevant to know where the byte with the
*  lower bits is and to jump over it, the trackOffset parameter encodes this:
*
*  - If trackOffset==0 the first byte is X0.
*  - If trackOffset==1 the first byte is X1.
*  - If trackOffset==2 the first byte is X2.
*  - If trackOffset==3 the first byte is X3.
*  - Values of trackOffset > 3 are not allowed, especially the input buffer
*    is not allowed to start at the lowermost bits byte.
*
* @param  count       Length of the conversion, should be the width of the output image.
* @param  bufIn       RAW10 encoded data: if the output image has width bytes,
*                     this buffer should have at least (width * 10)/8 bytes.
* @param  trackOffset See text.
* @param  bufOut      8 Bit Grey Value Output Address.
*/
/*-----------------------------------------------------------------------------*/
inline void  FL_CPY_RAW10P_U8P(U32 count, U8 trackOffset, char *bufIn, U8 *bufOut)
{
	while((trackOffset<4)&&(count-- > 0))
	{
		*bufOut = (U8)(*bufIn);
		bufIn++;
		bufOut++;
		trackOffset+=1;
	}

	bufIn++;

	while(count >= 4)
	{
		*((U32*)bufOut) = *((U32*)bufIn);
		bufIn+=5;
		bufOut+=4;

		count -= 4;
	}

	while(count--)
	{
		*bufOut = (U8)(*bufIn);
		bufIn++;
		bufOut++;
	}
}






/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Converts Image from RAW10 Format to 8 Bit Grey Value.
*
*  This function converts an image from RAW10 format to 8 bit grey value.
*
*  The RAW10 format has four bytes with each containing the uppermost 8 bits
*  of the pixels followed by one byte with the two lowermost bits of each of
*  the four preceeding pixels packed together:
*
*    X0, X1, X2, X3, LowerBitsOfX0..3,  X4, X5, X6, X7, LowerBitsOfX4..7, etc.
*
*  Since the starting position is relevant to know where the byte with the
*  lower bits is and to jump over it, the trackOffset parameter encodes this:
*
*  - If trackOffset==0 the first byte is X0.
*  - If trackOffset==1 the first byte is X1.
*  - If trackOffset==2 the first byte is X2.
*  - If trackOffset==3 the first byte is X3.
*  - Values of trackOffset > 3 are not allowed, especially the input buffer
*    is not allowed to start at the lowermost bits byte.
*
* @param  bufIn       RAW10 encoded data: if the output image has width*height bytes,
*                     this buffer should have at least height*(width * 10)/8 bytes.
* @param  trackOffset See text.
* @param  v4lX0,v4lY0 Offset of the top-left pixel relative to the current bufIn pointer.
* @param  v4lDx,v4lDy Dimensions of the input buffer.
* @param  v4lPitch    Currently the same as v4lDx.
* @param  v4lPaddingBytes  Additional bytes to v4lDx to get to a pixel one row down.
* @param  imgOut      8 Bit Grey Value Output image.
*/
/*-----------------------------------------------------------------------------*/
I32  convert_raw10_to_image(image *imgOut, char *bufIn, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes)
{
	I32   dx = min(imgOut->dx, v4lDx - v4lX0);
	I32   y;

	if(IMAGE_GREY!=imgOut->type)
	{
		return(ERR_TYPE);
	}
	if(v4lDx > v4lPitch)
	{
		return(ERR_PARAM);
	}
	if((v4lX0 >= v4lDx)||(v4lY0 >= v4lDy))
	{
		return(ERR_PARAM);
	}
	if(trackOffset > 3)
	{
		return(ERR_PARAM);
	}


	if((0==trackOffset)&&(0==v4lX0)&&(0==v4lY0))
	{
		#if _OPENMP
		#   pragma omp parallel for
		#endif
		for(y= 0; y< min(imgOut->dy,v4lDy); y++)
		{
			char *in  =       bufIn + y * ((v4lPitch*5)/4 + v4lPaddingBytes);
			U8   *out =  imgOut->st + y * imgOut->pitch;

			FL_CPY_RAW10P_U8P_NOOFFS(dx, in, out);
		}
	}
	else
	{
		char *in  = bufIn;
		U8   *out = imgOut->st;

		for(y= 0; y< v4lDy; y++)
		{
			in          =          in  + (         v4lX0) + ((trackOffset%4)+(         v4lX0))/4;
			trackOffset = (trackOffset + (         v4lX0))%4;

			if((y >= v4lY0)&&(y - v4lY0 < imgOut->dy))
			{
				FL_CPY_RAW10P_U8P(dx, trackOffset, in, out);

				out = out + imgOut->pitch;
			}

			in          =          in  + (v4lPitch-v4lX0) + ((trackOffset%4)+(v4lPitch-v4lX0))/4 + v4lPaddingBytes;
			trackOffset = (trackOffset + (v4lPitch-v4lX0))%4;
		}
	}

	return(ERR_NONE);
}


/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Copys Image from GREY Format to 8 Bit Grey Value.
*
*  This function copys an image from GREY format to 8 bit grey value.
*  It is just a copy routine, since the data format is already the same.
*
* @param  bufIn       GREY encoded data.
* @param  ignoredTrackOffset Ignored.
* @param  v4lX0,v4lY0 Offset of the top-left pixel relative to the current bufIn pointer.
* @param  v4lDx,v4lDy Dimensions of the input buffer.
* @param  v4lPitch    Currently the same as v4lDx.
* @param  v4lPaddingBytes  Additional bytes to v4lDx to get to a pixel one row down.
* @param  imgOut      8 Bit Grey Value Output image.
*/
/*-----------------------------------------------------------------------------*/
I32  copy_grey_to_image(image *imgOut, char *bufIn, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes)
{
	I32   dx = min(imgOut->dx, v4lDx - v4lX0);
	I32   y;

	if(IMAGE_GREY!=imgOut->type)
	{
		return(ERR_TYPE);
	}
	if(v4lDx > v4lPitch)
	{
		return(ERR_PARAM);
	}
	if((v4lX0 >= v4lDx)||(v4lY0 >= v4lDy))
	{
		return(ERR_PARAM);
	}


	#if _OPENMP
	#   pragma omp parallel for
	#endif
	for(y= 0; y< min(imgOut->dy,v4lDy); y++)
	{
		char *in  =       bufIn + v4lX0 + (y + v4lY0) * (v4lPitch + v4lPaddingBytes);
		U8   *out =  imgOut->st + y * imgOut->pitch;

		memcpy(out, in, dx);
	}


	return(ERR_NONE);
}




/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Direct conversion from raw10 Bayer RGB data to IMAGE_RGB.
*
*  This function does a direct conversion from raw10 Bayer RGB data to IMAGE_RGB.
*
* @param  bufIn       Bayer encoded data.
* @param  ignoredTrackOffset Ignored.
* @param  v4lX0,v4lY0 Offset of the top-left pixel relative to the current bufIn pointer.
* @param  v4lDx,v4lDy Dimensions of the input buffer.
* @param  v4lPitch    Currently the same as v4lDx.
* @param  v4lPaddingBytes  Additional bytes to v4lDx to get to a pixel one row down.
* @param  imgOut      8 Bit RGB Value Output image.
*/
/*-----------------------------------------------------------------------------*/
I32  convert_raw10_and_debayer_image(image *imgOut, char *bufIn, U8 trackOffset, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes)
{
	int    rc, ee;
	image  imgU8 = NULL_IMAGE;

	// Allocate temporary image
	{
		imgU8.type = IMAGE_GREY;
		imgU8.dx   = v4lDx;
		imgU8.dy   = v4lDy;
		imgU8.pitch= imgU8.dx;
		imgU8.ccmp1= NULL;
		imgU8.ccmp2= NULL;
		imgU8.st   =  malloc(sizeof(U8) * imgU8.dy * imgU8.pitch);
		if(NULL==imgU8.st){ee=-1; goto fail;}
	}

	rc =  convert_raw10_to_image(&imgU8,  bufIn, trackOffset,  v4lX0, v4lY0, v4lDx, v4lDy, v4lPitch, v4lPaddingBytes);
	if(rc<0){ee=-2+10*rc; goto fail;}

	rc =  simple_debayer_to_image(imgOut, (char*)imgU8.st,  0, 0, imgU8.dx, imgU8.dy, imgU8.pitch, 0);
	if(rc<0){ee=-3+10*rc; goto fail;}

 	ee=0;
fail:
	if(NULL!=imgU8.st){ free(imgU8.st);  imgU8.st=NULL; }

	return(ee);
}


/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Simple and SLOW debayering for sensor image data.
*
*  This function does a simple and SLOW debayering for sensor image data.
*
* @param  bufIn       GREY encoded data.
* @param  ignoredTrackOffset Ignored.
* @param  v4lX0,v4lY0 Offset of the top-left pixel relative to the current bufIn pointer.
* @param  v4lDx,v4lDy Dimensions of the input buffer.
* @param  v4lPitch    Currently the same as v4lDx.
* @param  v4lPaddingBytes  Additional bytes to v4lDx to get to a pixel one row down.
* @param  imgOut      8 Bit Grey Value Output image.
*/
/*-----------------------------------------------------------------------------*/
I32  simple_debayer_to_image(image *imgOut, char *bufIn, I32 v4lX0, I32 v4lY0, I32 v4lDx, I32 v4lDy, I32 v4lPitch, I32 v4lPaddingBytes)
{
	I32   dx = min(imgOut->dx, v4lDx - v4lX0);
	I32   dy = min(imgOut->dy, v4lDy - v4lY0);
	I32   y;

	if(IMAGE_RGB!=imgOut->type)
	{
		return(ERR_TYPE);
	}
	if(v4lDx > v4lPitch)
	{
		return(ERR_PARAM);
	}
	if((v4lX0 >= v4lDx)||(v4lY0 >= v4lDy))
	{
		return(ERR_PARAM);
	}

	#if _OPENMP
	#   pragma omp parallel for
	#endif
	for(y= 0; y< dy; y+=2)
	{
		I32   x;
		U8   *in = NULL, *out= NULL;
		U8   *or= NULL, *og= NULL, *ob= NULL;

		//first line input
		in  =  (U8*)bufIn + v4lX0 + ((y+0) + v4lY0) * (v4lPitch + v4lPaddingBytes);
		or  =  imgOut->st    + (y+0) * imgOut->pitch;
		og  =  imgOut->ccmp1 + (y+0) * imgOut->pitch;
		ob  =  imgOut->ccmp2 + (y+0) * imgOut->pitch;

		for(x= 0; x< dx; x+=2)
		{
			*or = *in;
			or++;
			*or = *in;
			or++;

			in++;

			*og = *in;
			og++;
			*og = *in;
			og++;

			in++;
		}

		//next line input
		in  =  (U8*)bufIn + v4lX0 + ((y+1) + v4lY0) * (v4lPitch + v4lPaddingBytes);
		og  =  imgOut->ccmp1 + (y+1) * imgOut->pitch;

		for(x= 0; x< dx; x+=2)
		{
			*og = *in;
			og++;
			*og = *in;
			og++;

			in++;

			*ob = *in;
			ob++;
			*ob = *in;
			ob++;

			in++;
		}

		//copy to second line
		//Red
		in  =  imgOut->st    + (y+0) * imgOut->pitch;
		out =  imgOut->st    + (y+1) * imgOut->pitch;

		memcpy(out, in, dx);

		//Blue
		in  =  imgOut->ccmp2 + (y+0) * imgOut->pitch;
		out =  imgOut->ccmp2 + (y+1) * imgOut->pitch;

		memcpy(out, in, dx);
	}

	return(ERR_NONE);
}

void  timemeasurement_start(struct  timeval *timer)
{
	gettimeofday(timer,(struct timezone *)0);
}

void  timemeasurement_stop(struct  timeval *timer, I64 *s, I64 *us)
{
	struct  timeval  end;

	gettimeofday(&end,(struct timezone *)0);
	*s  = end.tv_sec  - timer->tv_sec;
	*us = end.tv_usec - timer->tv_usec;
	if(*us < 0){ *us += 1000000; *s = *s - 1; }
}











/*--*FUNCTION*-----------------------------------------------------------------*/
/**
* @brief  Stores an Image as Portable Graymap or Portable Pixmap (open with GIMP).
*
*  This function stores an image as Portable Graymap (PGM) or Portable Pixmap (PPM).
*  You can open this files for example with the GIMP (Gnu Image Manipulation Program).
*
* @param  path        The Filename with its path, extension will be added by type.
* @param  img         8 Bit Grey or RGB Value image to be stored.
*/
/*-----------------------------------------------------------------------------*/
I32  write_image_as_pnm(char *path, image *img)
{
	I32   ee, x, y;
	I32   fd=-1;
	I32   headerBytes, wroteBytes;
	char  acHeader[256], *pcFilename=NULL;
	char *pcLine=NULL;

	if((IMAGE_GREY!=img->type)&&(IMAGE_RGB!=img->type)){ee=-1; goto fail;}


	pcFilename =  malloc(sizeof(char) * (strlen(path)+4+2));
	if(NULL==pcFilename){ee=-2; goto fail;}

	snprintf(pcFilename,strlen(path)+4+1,"%s.%s",path,(IMAGE_GREY==img->type)?("pgm"):("ppm"));


	fd =  open(pcFilename, O_WRONLY | O_CREAT, 00644);
	if(fd<0){ee=-3; goto fail;}


	headerBytes =  snprintf(acHeader,255,"P%c %d %d %d ",(IMAGE_GREY==img->type)?('5'):('6'), img->dx, img->dy, 255);
	if(headerBytes<0){ee=-4; goto fail;}

	wroteBytes =  write(fd, acHeader, headerBytes);
	if(wroteBytes!=headerBytes){ee=-5; goto fail;}


	switch(img->type)
	{
		case IMAGE_GREY:
		{
			for(y= 0; y< img->dy; y++)
			{
				wroteBytes =  write(fd, (U8*)img->st + y * img->pitch, img->dx);
				if(wroteBytes!=img->dx){ee=-6; goto fail;}
			}
		}
		break;
		case IMAGE_RGB:
		{
			pcLine =  malloc(sizeof(U8) * 3 * img->dx);
			if(NULL==pcLine){ee=-7; goto fail;}

			for(y= 0; y< img->dy; y++)
			{
				for(x= 0; x< img->dx; x++)
				{
					pcLine[3*x+0] = *((U8*)img->st    + y * img->pitch + x);
					pcLine[3*x+1] = *((U8*)img->ccmp1 + y * img->pitch + x);
					pcLine[3*x+2] = *((U8*)img->ccmp2 + y * img->pitch + x);
				}

				wroteBytes =  write(fd, pcLine, 3 * img->dx);
				if(wroteBytes!=3 * img->dx){ee=-8; goto fail;}
			}
		}
		break;
		default: ee=-9; goto fail;
	}

	ee=0;
fail:
	if(fd>=0)
	{
		close(fd);
	}
	if(NULL!=pcLine    ){ free(pcLine    ); pcLine    =NULL; }
	if(NULL!=pcFilename){ free(pcFilename); pcFilename=NULL; }

	return(ee);
}

