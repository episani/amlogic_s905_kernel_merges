/*
 * Copyright (C) 2010 ARM Limited. All rights reserved.
 *
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include "gralloc_priv.h"
#include <hardware/hwcomposer_defs.h>

#include <cutils/log.h>
#include <cutils/atomic.h>
#include <hardware/hardware.h>
#include <hardware/gralloc.h>

#include <GLES/gl.h>

#include "alloc_device.h"
#include "gralloc_priv.h"
#include "gralloc_helper.h"
#include "gralloc_vsync.h"

static int fb_set_swap_interval(struct framebuffer_device_t* dev, int interval)
{
	if (interval < dev->minSwapInterval)
	{
		interval = dev->minSwapInterval;
	}
	else if (interval > dev->maxSwapInterval)
	{
		interval = dev->maxSwapInterval;
	}

	private_module_t* m = reinterpret_cast<private_module_t*>(dev->common.module);
	m->swapInterval = interval;

	if (0 == interval) gralloc_vsync_disable(dev);
	else gralloc_vsync_enable(dev);

	return 0;
}

static int init_frame_buffer(struct private_module_t* module,struct framebuffer_t* fb)
{
	if (fb->fb_hnd != NULL)
	{
	    ALOGD("init already called before.");
	    return 0;
	}
	pthread_mutex_lock(&module->lock);
	framebuffer_info_t* fbinfo = &(fb->fb_info);
	fbinfo->displayType = HWC_DISPLAY_PRIMARY;
	fbinfo->fbIdx = getOsdIdx(fbinfo->displayType);

	int err = init_frame_buffer_locked(fbinfo);
	int bufferSize = fbinfo->finfo.line_length * fbinfo->info.yres;

	// Create a "fake" buffer object for the entire frame buffer memory, and store it in the module
	fb->fb_hnd = new private_handle_t(private_handle_t::PRIV_FLAGS_FRAMEBUFFER, 0, fbinfo->fbSize, 0,
											0, fbinfo->fd, bufferSize, 0);
	ALOGD("init_frame_buffer get frame size %d",bufferSize);

	//init fb_info
	framebuffer_mapper_t* m = NULL;
	private_handle_t *hnd = (private_handle_t *)fb->fb_hnd;
	if (hnd->usage & GRALLOC_USAGE_EXTERNAL_DISP)
	{
		m = &(module->fb_external);
	}
	else
	{
		m = &(module->fb_primary);
	}
	m->fb_info = fb->fb_info;
	//m->fb_info = &(fb->fb_info);

	//Register the handle.
	module->base.registerBuffer(&(module->base),fb->fb_hnd);

	pthread_mutex_unlock(&module->lock);
	return err;
}

static int fb_post(struct framebuffer_device_t* dev, buffer_handle_t buffer){
	private_module_t* priv_t = reinterpret_cast<private_module_t*>(dev->common.module);
	framebuffer_t* fb = reinterpret_cast<framebuffer_t*>(dev);
	framebuffer_info_t* fbinfo = &(fb->fb_info);
	int display_type = fbinfo->displayType;

/*	framebuffer_mapper_t* m = &(priv_t->fb_primary);
#ifdef DEBUG_EXTERNAL_DISPLAY_ON_PANEL
	if (display_type == HWC_DISPLAY_EXTERNAL)
		ALOGD("fbpost hdmi on panel");
#else
	if (display_type == HWC_DISPLAY_EXTERNAL)
		m = &(priv_t->fb_external);
#endif
*/
	if (private_handle_t::validate(buffer) < 0)
	{
		return -EINVAL;
	}
    if (fbinfo->currentBuffer)
	{
		priv_t->base.unlock(&priv_t->base, fbinfo->currentBuffer);
		fbinfo->currentBuffer = 0;
	}
	private_handle_t const* hnd = reinterpret_cast<private_handle_t const*>(buffer);
	if (hnd->flags & private_handle_t::PRIV_FLAGS_FRAMEBUFFER)
	{
		priv_t->base.lock(&priv_t->base, buffer, private_module_t::PRIV_USAGE_LOCKED_FOR_POST,
									0, 0, fbinfo->info.xres, fbinfo->info.yres, NULL);
		int rtn = fb_post_locked(fbinfo,buffer);
		if (rtn < 0)
		{
			//post fail.
			ALOGD("fb_post_locked return error %d",rtn);
			priv_t->base.unlock(&priv_t->base, buffer);
			return rtn;
		}
	} else {
		void* fb_vaddr;
		void* buffer_vaddr;
		priv_t->base.lock(&priv_t->base, priv_t->fb_primary.framebuffer, GRALLOC_USAGE_SW_WRITE_RARELY,
				0, 0, fbinfo->info.xres, fbinfo->info.yres, &fb_vaddr);
		priv_t->base.lock(&priv_t->base, buffer, GRALLOC_USAGE_SW_READ_RARELY,
				0, 0, fbinfo->info.xres, fbinfo->info.yres, &buffer_vaddr);
		memcpy(fb_vaddr, buffer_vaddr, fbinfo->finfo.line_length * fbinfo->info.yres);
		priv_t->base.unlock(&priv_t->base, buffer);
		priv_t->base.unlock(&priv_t->base, priv_t->fb_primary.framebuffer);
	}
	return 0;
}

static int fb_close(struct hw_device_t *device)
{
	framebuffer_t* dev = reinterpret_cast<framebuffer_t*>(device);
	if (dev)
	{
#if GRALLOC_ARM_UMP_MODULE
		ump_close();
#endif
		if (dev->fb_hnd)
		{
			#if 0
			hw_module_t * pmodule = NULL;
			private_module_t *m = NULL;
			if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&pmodule) == 0)
			{
				m = reinterpret_cast<private_module_t *>(pmodule);
				m->base.unregisterBuffer(&(m->base),dev->fb_hnd);
			}
			close(dev->fb_info.fd);
			dev->fb_info.fd= -1;
			#endif

			delete dev->fb_hnd;
			dev->fb_hnd = 0;
		}

		delete dev;
	}
	return 0;
}

int compositionComplete(struct framebuffer_device_t* dev)
{
	/* By doing a finish here we force the GL driver to start rendering
	   all the drawcalls up to this point, and to wait for the rendering to be complete.*/
	glFinish();
	/* The rendering of the backbuffer is now completed.
	   When SurfaceFlinger later does a call to eglSwapBuffer(), the swap will be done
	   synchronously in the same thread, and not asynchronoulsy in a background thread later.
	   The SurfaceFlinger requires this behaviour since it releases the lock on all the
	   SourceBuffers (Layers) after the compositionComplete() function returns.
	   However this "bad" behaviour by SurfaceFlinger should not affect performance,
	   since the Applications that render the SourceBuffers (Layers) still get the
	   full renderpipeline using asynchronous rendering. So they perform at maximum speed,
	   and because of their complexity compared to the Surface flinger jobs, the Surface flinger
	   is normally faster even if it does everyhing synchronous and serial.
	   */
	return 0;
}

int framebuffer_device_open(hw_module_t const* module, const char* name, hw_device_t** device)
{
	int status = -EINVAL;

#if 0
	alloc_device_t* gralloc_device;
#if DISABLE_FRAMEBUFFER_HAL == 1
	AERR("Framebuffer HAL not support/disabled %s",
#ifdef MALI_DISPLAY_VERSION
	"with MALI display enable");
#else
	"");
#endif
	return -ENODEV;
#endif
	status = gralloc_open(module, &gralloc_device);
	if (status < 0)
	{
		return status;
	}
#endif
#if DISABLE_FRAMEBUFFER_HAL == 1
	AERR("Framebuffer HAL not support/disabled %s",
#ifdef MALI_DISPLAY_VERSION
	"with MALI display enable");
#else
	"");
#endif
	return -ENODEV;
#endif

	/*Init the framebuffer data*/
	framebuffer_t *fb = new framebuffer_t();
	memset(fb, 0, sizeof(*fb));

	framebuffer_device_t *dev = &(fb->base);
	framebuffer_info_t *fbinfo = &(fb->fb_info);

	/*get gralloc module to register framebuffer*/
	private_module_t* priv_t = (private_module_t*)module;
	framebuffer_mapper_t* m =&(priv_t->fb_primary);
	status = init_frame_buffer(priv_t,fb);
	if (status < 0)
	{
	#if 0
		gralloc_close(gralloc_device);
	#endif
		delete fb;
		return status;
	}

	/* initialize the procs */
	dev->common.tag = HARDWARE_DEVICE_TAG;
	dev->common.version = 0;
	dev->common.module = const_cast<hw_module_t*>(module);
	dev->common.close = fb_close;
	dev->setSwapInterval = fb_set_swap_interval;
	dev->post = fb_post;
	dev->setUpdateRect = 0;
	dev->compositionComplete = &compositionComplete;

	int stride = fbinfo->finfo.line_length / (fbinfo->info.bits_per_pixel >> 3);
	const_cast<uint32_t&>(dev->flags) = 0;
	const_cast<uint32_t&>(dev->width) = fbinfo->info.xres;
	const_cast<uint32_t&>(dev->height) = fbinfo->info.yres;
	const_cast<int&>(dev->stride) = stride;
	const_cast<int&>(dev->format) = (bits_per_pixel() == 16) ? HAL_PIXEL_FORMAT_RGB_565 : HAL_PIXEL_FORMAT_RGBX_8888;
	const_cast<float&>(dev->xdpi) = fbinfo->xdpi;
	const_cast<float&>(dev->ydpi) = fbinfo->ydpi;
	const_cast<float&>(dev->fps) = fbinfo->fps;
	const_cast<int&>(dev->minSwapInterval) = 0;
	const_cast<int&>(dev->maxSwapInterval) = 1;
	*device = &dev->common;

	gralloc_vsync_enable(dev);

	return status;
}
