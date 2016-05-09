#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
//#include <linux/ion.h>
//#include <ion/ion.h>
#include <linux/fb.h>

//#define LOG_NDEBUG 0
#define LOG_TAG "FrameBuffer"

#include <cutils/log.h>
#include <sys/time.h>
#include <utils/Timers.h>
#include <cutils/atomic.h>
#include <cutils/properties.h>
#include <hardware/hardware.h>
#include <hardware/gralloc.h>
#include "gralloc_priv.h"
#include "framebuffer.h"

#include <hardware/hwcomposer_defs.h>
#include <GLES/gl.h>

#ifndef __gl_h_
#error a
#endif

#ifdef MALI_VSYNC_EVENT_REPORT_ENABLE
#include "gralloc_vsync_report.h"
#endif

#include "gralloc_priv.h"
#include "gralloc_helper.h"


#ifdef DEBUG_EXTERNAL_DISPLAY_ON_PANEL
 // 3  for panel, 3 for hdmi
#define NUM_BUFFERS (6)

#else

// numbers of buffers for page flipping
#ifndef NUM_BUFFERS
#define NUM_BUFFERS (2)
#endif

#endif

enum
{
	PAGE_FLIP = 0x00000001,
};
/*
 framebuffer interface api to other module to use.
*/

int bits_per_pixel()
{
	char fb_bits[PROPERTY_VALUE_MAX];
	int bits_per_pixel = 16;

	if (property_get("sys.fb.bits", fb_bits, NULL) > 0 && atoi(fb_bits) == 32)
	{
		return 32;
	}
	return 16;
}

#ifndef SINGLE_EXTERNAL_DISPLAY_USE_FB1
int update_cursor_buffer_locked(struct framebuffer_info_t* cbinfo, int xres, int yres)
{
	char const * const device_template[] =
	{
		"/dev/graphics/fb%u",
		"/dev/fb%u",
		NULL
	};

	int i = 0;
	char name[64];

	while ((cbinfo->fd == -1) && device_template[i])
	{
		snprintf(name, 64, device_template[i], cbinfo->fbIdx);
		cbinfo->fd = open(name, O_RDWR, 0);
		i++;
	}

	ALOGE("update_cursor_buffer_locked of fb idx (%d)",cbinfo->fbIdx);

	if (cbinfo->fd < 0)
	{
		return -errno;
	}

	struct fb_fix_screeninfo finfo;
	if (ioctl(cbinfo->fd, FBIOGET_FSCREENINFO, &finfo) == -1)
	{
		return -errno;
	}

	struct fb_var_screeninfo info;
	if (ioctl(cbinfo->fd, FBIOGET_VSCREENINFO, &info) == -1)
	{
		return -errno;
	}

	ALOGE("vinfo. %d %d", info.xres, info.yres);

	info.xoffset = info.yoffset = 0;
	info.bits_per_pixel = 32;

	info.xres_virtual = info.xres = xres;
	info.yres_virtual = info.yres = yres;

	if (ioctl(cbinfo->fd, FBIOPUT_VSCREENINFO, &info) == -1)
    {
		ALOGE("set vinfo fail\n");
	}

	if (ioctl(cbinfo->fd, FBIOGET_VSCREENINFO, &info) == -1)
    {
		ALOGE("get info fail\n");
		return -errno;
	}

	if (int(info.width) <= 0 || int(info.height) <= 0)
	{
		// the driver doesn't return that information
		// default to 160 dpi
		info.width  = ((info.xres * 25.4f)/160.0f + 0.5f);
		info.height = ((info.yres * 25.4f)/160.0f + 0.5f);
	}

	AINF("using (fd=%d)\n"
		"id           = %s\n"
		"xres         = %d px\n"
		"yres         = %d px\n"
		"xres_virtual = %d px\n"
		"yres_virtual = %d px\n"
		"bpp          = %d\n",
		cbinfo->fd,
		finfo.id,
		info.xres,
		info.yres,
		info.xres_virtual,
		info.yres_virtual,
		info.bits_per_pixel);

	AINF("width        = %d mm \n"
			"height       = %d mm \n",
			info.width,
			info.height);

	if (ioctl(cbinfo->fd, FBIOGET_FSCREENINFO, &finfo) == -1)
	{
		return -errno;
	}

	if (finfo.smem_len <= 0)
	{
		return -errno;
	}

	cbinfo->info = info;
	cbinfo->finfo = finfo;
	ALOGD("update_cursor_buffer_locked: finfo.line_length is 0x%x,info.yres_virtual is 0x%x", finfo.line_length, info.yres_virtual);
	cbinfo->fbSize = round_up_to_page_size(finfo.line_length * info.yres_virtual);

	return 0;
}


int init_cursor_buffer_locked(struct framebuffer_info_t* cbinfo)
{
	char const * const device_template[] =
	{
		"/dev/graphics/fb%u",
		"/dev/fb%u",
		NULL
	};

	int fd = -1;
	int i = 0;
	char name[64];

	while ((fd == -1) && device_template[i])
	{
		snprintf(name, 64, device_template[i], cbinfo->fbIdx);
		fd = open(name, O_RDWR, 0);
		i++;
	}

	ALOGE("init_cursor_buffer_locked of dev:(%s),fb idx (%d)",name,cbinfo->fbIdx);

	if (fd < 0)
	{
		return -errno;
	}

	struct fb_fix_screeninfo finfo;
	if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo) == -1)
	{
		return -errno;
	}

	struct fb_var_screeninfo info;
	if (ioctl(fd, FBIOGET_VSCREENINFO, &info) == -1)
	{
		return -errno;
	}

	ALOGE("vinfo. %d %d", info.xres, info.yres);

	info.xoffset = info.yoffset = 0;
	info.bits_per_pixel = 32;

	if (ioctl(fd, FBIOPUT_VSCREENINFO, &info) == -1)
	{
		ALOGE("set vinfo fail\n");
	}

	if (ioctl(fd, FBIOGET_VSCREENINFO, &info) == -1)
	{
		ALOGE("get info fail\n");
		return -errno;
	}

	if (int(info.width) <= 0 || int(info.height) <= 0)
	{
		// the driver doesn't return that information
		// default to 160 dpi
		info.width  = ((info.xres * 25.4f)/160.0f + 0.5f);
		info.height = ((info.yres * 25.4f)/160.0f + 0.5f);
	}

	//float xdpi = (info.xres * 25.4f) / info.width;
	//float ydpi = (info.yres * 25.4f) / info.height;

	AINF("using (fd=%d)\n"
		"id           = %s\n"
		"xres         = %d px\n"
		"yres         = %d px\n"
		"xres_virtual = %d px\n"
		"yres_virtual = %d px\n"
		"bpp          = %d\n",
		fd,
		finfo.id,
		info.xres,
		info.yres,
		info.xres_virtual,
		info.yres_virtual,
		info.bits_per_pixel);

	AINF("width        = %d mm \n"
		"height       = %d mm \n",
		info.width,
		info.height);

	if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo) == -1)
	{
		return -errno;
	}

	if (finfo.smem_len <= 0)
	{
		return -errno;
	}

	cbinfo->info = info;
	cbinfo->finfo = finfo;
	cbinfo->fd = fd;
	ALOGE("init_cursor_buffer_locked: finfo.line_length is 0x%x,info.yres_virtual is 0x%x", finfo.line_length, info.yres_virtual);
	cbinfo->fbSize = round_up_to_page_size(finfo.line_length * info.yres_virtual);

	return 0;
}
#endif


int init_frame_buffer_locked(struct framebuffer_info_t* fbinfo)
{
	char const * const device_template[] =
	{
		"/dev/graphics/fb%u",
		"/dev/fb%u",
		NULL
	};

	int fd = -1;
	int i = 0;
	char name[64];

	while ((fd == -1) && device_template[i])
	{
		snprintf(name, 64, device_template[i], fbinfo->fbIdx);
		fd = open(name, O_RDWR, 0);
		i++;
	}

	ALOGE("init_frame_buffer_locked of dev:(%s),fb idx (%d)",name,fbinfo->fbIdx);

	if (fd < 0)
	{
		return -errno;
	}

	struct fb_fix_screeninfo finfo;
	if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo) == -1)
	{
		return -errno;
	}

	struct fb_var_screeninfo info;
	if (ioctl(fd, FBIOGET_VSCREENINFO, &info) == -1)
	{
		return -errno;
	}

	info.reserved[0] = 0;
	info.reserved[1] = 0;
	info.reserved[2] = 0;
	info.xoffset = 0;
	info.yoffset = 0;
	info.activate = FB_ACTIVATE_NOW;

	if (bits_per_pixel() == 16)
	{
		/*
		 * Explicitly request 5/6/5
		 */
		info.bits_per_pixel = 16;
		info.red.offset     = 11;
		info.red.length     = 5;
		info.green.offset   = 5;
		info.green.length   = 6;
		info.blue.offset    = 0;
		info.blue.length    = 5;
		info.transp.offset  = 0;
		info.transp.length  = 0;
	}
	else
	{
		/*
		* Explicitly request 8/8/8/8
		*/
		info.bits_per_pixel = 32;
		info.red.offset     = 0;
		info.red.length     = 8;
		info.green.offset   = 8;
		info.green.length   = 8;
		info.blue.offset    = 16;
		info.blue.length    = 8;
		info.transp.offset  = 24;
		info.transp.length  = 8;
	}

	/*
	 * Request NUM_BUFFERS screens (at lest 2 for page flipping)
	 */
	info.yres_virtual = info.yres * NUM_BUFFERS;

	uint32_t flags = PAGE_FLIP;
	if (ioctl(fd, FBIOPUT_VSCREENINFO, &info) == -1)
	{
		info.yres_virtual = info.yres;
		flags &= ~PAGE_FLIP;
		AWAR( "FBIOPUT_VSCREENINFO failed, page flipping not supported fd: %d", fd );
	}

	if (info.yres_virtual < info.yres * 2)
	{
		// we need at least 2 for page-flipping
		info.yres_virtual = info.yres;
		flags &= ~PAGE_FLIP;
		AWAR( "page flipping not supported (yres_virtual=%d, requested=%d)", info.yres_virtual, info.yres*2 );
	}

	if (ioctl(fd, FBIOGET_VSCREENINFO, &info) == -1)
	{
		return -errno;
	}

	int refreshRate = 0;
	if ( info.pixclock > 0 )
	{
		refreshRate = 1000000000000000LLU /
		(
			uint64_t( info.upper_margin + info.lower_margin + info.yres + info.hsync_len )
			* ( info.left_margin  + info.right_margin + info.xres + info.vsync_len )
			* info.pixclock
		);
	}
	else
	{
		AWAR( "fbdev pixclock is zero for fd: %d", fd );
	}

	if (refreshRate == 0)
	{
		//refreshRate = 50*1000;  // 50 Hz
		refreshRate = 60*1000;  // 60 Hz
	}

	if (int(info.width) <= 0 || int(info.height) <= 0)
	{
		// the driver doesn't return that information
		// default to 160 dpi
		info.width  = ((info.xres * 25.4f)/160.0f + 0.5f);
		info.height = ((info.yres * 25.4f)/160.0f + 0.5f);
	}

	float xdpi = (info.xres * 25.4f) / info.width;
	float ydpi = (info.yres * 25.4f) / info.height;
	float fps  = refreshRate / 1000.0f;

	AINF("using (fd=%d)\n"
	     "id           = %s\n"
	     "xres         = %d px\n"
	     "yres         = %d px\n"
	     "xres_virtual = %d px\n"
	     "yres_virtual = %d px\n"
	     "bpp          = %d\n"
	     "r            = %2u:%u\n"
	     "g            = %2u:%u\n"
	     "b            = %2u:%u\n",
	     fd,
	     finfo.id,
	     info.xres,
	     info.yres,
	     info.xres_virtual,
	     info.yres_virtual,
	     info.bits_per_pixel,
	     info.red.offset, info.red.length,
	     info.green.offset, info.green.length,
	     info.blue.offset, info.blue.length);

	AINF("width        = %d mm (%f dpi)\n"
	     "height       = %d mm (%f dpi)\n"
	     "refresh rate = %.2f Hz\n",
	     info.width,  xdpi,
	     info.height, ydpi,
	     fps);

	if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo) == -1)
	{
		return -errno;
	}

    if (finfo.smem_len <= 0)
	{
		return -errno;
	}

    fbinfo->info = info;
	fbinfo->finfo = finfo;
	fbinfo->xdpi = xdpi;
	fbinfo->ydpi = ydpi;
	fbinfo->fps = fps;
    fbinfo->fd = fd;
    fbinfo->flipFlags = flags;
    fbinfo->fbSize = round_up_to_page_size(finfo.line_length * info.yres_virtual);

	return 0;
}
int fb_post_with_fence_locked(struct framebuffer_info_t* fbinfo,buffer_handle_t hnd,int in_fence)
{
#define  FBIOPUT_OSD_SYNC_ADD	0x4518
    typedef  struct{
	unsigned int  xoffset;
	unsigned int  yoffset;
	int  in_fen_fd;
	int  out_fen_fd;
    }fb_sync_request_t;
    private_handle_t const* buffer = reinterpret_cast<private_handle_t const*>(hnd);

    //wait fence sync
    //sync_wait(in_fence, 3000);
    //close(in_fence);
    //in_fence = -1;
    //set sync request

    fb_sync_request_t sync_req;
    memset(&sync_req,0,sizeof(fb_sync_request_t));
    sync_req.xoffset=fbinfo->info.xoffset;
    sync_req.yoffset= buffer->offset / fbinfo->finfo.line_length;
    sync_req.in_fen_fd=in_fence;
    //ALOGD( "req offset:%d\n",sync_req.yoffset);
    ioctl(fbinfo->fd, FBIOPUT_OSD_SYNC_ADD, &sync_req);
     //ALOGD( "post offset:%d\n",buffer->offset/fbinfo->finfo.line_length);
    //TODO:: need update with kernel change.
    //fb_post_locked(fbinfo,hnd);

    int out_fence = sync_req.out_fen_fd;
    /*nsecs_t origin=systemTime(CLOCK_MONOTONIC);
    ALOGD( "--sync wait: %d,begin:%lld\n",out_fence,origin);
    sync_wait(out_fence, 3000);
    close(out_fence);
    nsecs_t diff=systemTime(CLOCK_MONOTONIC)-origin;
    ALOGD( "++sync wait: %d,wait_delay:%lld\n",out_fence,diff);*/

    return out_fence;
}

int fb_post_locked(struct framebuffer_info_t* fbinfo, buffer_handle_t hnd)
{
	private_handle_t const* buffer = reinterpret_cast<private_handle_t const*>(hnd);
	fbinfo->info.activate = FB_ACTIVATE_VBL;
	fbinfo->info.yoffset = buffer->offset / fbinfo->finfo.line_length;

    //ALOGD("fbpost on slot (%d)",fbinfo->info.yoffset/fbinfo->info.yres);

#ifdef STANDARD_LINUX_SCREEN
	int interrupt;
#define FBIO_WAITFORVSYNC       _IOW('F', 0x20, __u32)
#define S3CFB_SET_VSYNC_INT	_IOW('F', 206, unsigned int)
	if (ioctl(fbinfo->fd, FBIOPAN_DISPLAY, &fbinfo->info) == -1)
	{
		AERR( "FBIOPAN_DISPLAY failed for fd: %d", fbinfo->fd );
		return 0;
	}
#if PLATFORM_SDK_VERSION >= 16
	if (swapInterval == 1 && !(buffer->usage & GRALLOC_USAGE_HW_COMPOSER))
#else
	if (swapInterval == 1)
#endif
	{
		// enable VSYNC
		interrupt = 1;
		if (ioctl(fbinfo->fd, S3CFB_SET_VSYNC_INT, &interrupt) < 0)
		{
			AERR( "S3CFB_SET_VSYNC_INT enable failed for fd: %d", fbinfo->fd );
			return 0;
		}
		// wait for VSYNC
#ifdef MALI_VSYNC_EVENT_REPORT_ENABLE
		gralloc_mali_vsync_report(MALI_VSYNC_EVENT_BEGIN_WAIT);
#endif
		int crtc = 0;
		if (ioctl(fbinfo->fd, FBIO_WAITFORVSYNC, &crtc) < 0)
		{
			AERR( "FBIO_WAITFORVSYNC failed for fd: %d", fbinfo->fd );
#ifdef MALI_VSYNC_EVENT_REPORT_ENABLE
			gralloc_mali_vsync_report(MALI_VSYNC_EVENT_END_WAIT);
#endif
			return 0;
		}
#ifdef MALI_VSYNC_EVENT_REPORT_ENABLE
		gralloc_mali_vsync_report(MALI_VSYNC_EVENT_END_WAIT);
#endif
		// disable VSYNC
		interrupt = 0;
		if (ioctl(fbinfo->fd, S3CFB_SET_VSYNC_INT, &interrupt) < 0)
		{
			AERR( "S3CFB_SET_VSYNC_INT disable failed for fd: %d", fbinfo->fd );
			return 0;
		}
	}
#else
	/*Standard Android way*/
#ifdef MALI_VSYNC_EVENT_REPORT_ENABLE
	gralloc_mali_vsync_report(MALI_VSYNC_EVENT_BEGIN_WAIT);
#endif
	ALOGD("current yoffset %d\n",fbinfo->info.yoffset);
	//if (ioctl(fbinfo->fd, FBIOPUT_VSCREENINFO, &fbinfo->info) == -1)
	if (ioctl(fbinfo->fd, FBIOPAN_DISPLAY, &fbinfo->info) == -1)
	{
		AERR( "FBIOPUT_VSCREENINFO failed for fd: %d", fbinfo->fd );
#ifdef MALI_VSYNC_EVENT_REPORT_ENABLE
		gralloc_mali_vsync_report(MALI_VSYNC_EVENT_END_WAIT);
#endif
		return -errno;
	}
#ifdef MALI_VSYNC_EVENT_REPORT_ENABLE
	gralloc_mali_vsync_report(MALI_VSYNC_EVENT_END_WAIT);
#endif
#endif

	fbinfo->currentBuffer = buffer;

	return 0;
}

int getOsdIdx(int display_type)
{
#ifdef DEBUG_EXTERNAL_DISPLAY_ON_PANEL
	return 0;
#else
	if (display_type == HWC_DISPLAY_PRIMARY)
		return 0;
	if (display_type == HWC_DISPLAY_EXTERNAL)
	{
#ifndef SINGLE_EXTERNAL_DISPLAY_USE_FB1
            return 2;
#else
            return 1;
#endif
	}
#endif
	return -1;
}


unsigned int get_num_fb_buffers() {
	ALOGD("****************************** %d\n",NUM_BUFFERS);
	return NUM_BUFFERS;
}

