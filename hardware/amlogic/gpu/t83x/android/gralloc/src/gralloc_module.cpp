/*
 * Copyright (C) 2010 ARM Limited. All rights reserved.
 *
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * You may not use this file except in compliance with the License.
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
#define LOG_NDEBUG 0
#define LOG_TAG "Gralloc"

#include <errno.h>
#include <pthread.h>

#include <cutils/log.h>
#include <cutils/atomic.h>
#include <hardware/hardware.h>
#include <hardware/gralloc.h>

#include <hardware/hwcomposer_defs.h>

#include "gralloc_priv.h"
#include "alloc_device.h"
#include "framebuffer_device.h"

#include "gralloc_module_allocator_specific.h"

#if MALI_AFBC_GRALLOC == 1
#include "gralloc_buffer_priv.h"
#endif

#include "format_chooser.h"

static pthread_mutex_t s_map_lock = PTHREAD_MUTEX_INITIALIZER;

static int gralloc_device_open(const hw_module_t* module, const char* name, hw_device_t** device)
{
	int status = -EINVAL;

	if (!strncmp(name, GRALLOC_HARDWARE_GPU0, MALI_GRALLOC_HARDWARE_MAX_STR_LEN))
	{
		status = alloc_device_open(module, name, device);
	}
	else if (!strncmp(name, GRALLOC_HARDWARE_FB0, MALI_GRALLOC_HARDWARE_MAX_STR_LEN))
	{
		status = framebuffer_device_open(module, name, device);
	}

	return status;
}

static int gralloc_register_buffer(gralloc_module_t const* module, buffer_handle_t handle)
{
	if (private_handle_t::validate(handle) < 0)
	{
		AERR("Registering invalid buffer %p, returning error", handle);
		return -EINVAL;
	}

	// if this handle was created in this process, then we keep it as is.
	private_handle_t* hnd = (private_handle_t*)handle;

	if (hnd->pid == getpid())
	{
		// If the handle is created and registered in the same process this is valid,
		// but it could also be that application is registering twice which is illegal.
		AWAR("Registering handle %p coming from the same process: %d.", hnd, hnd->pid);
	}

	int retval = -EINVAL;

	pthread_mutex_lock(&s_map_lock);

	hnd->pid = getpid();

	if (hnd->flags & private_handle_t::PRIV_FLAGS_FRAMEBUFFER) 
	{
		ALOGD("gralloc_register_buffer register framebuffer");
		hw_module_t * pmodule = NULL;
		private_module_t *m = NULL;
		if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&pmodule) == 0)
		{
			m = reinterpret_cast<private_module_t *>(pmodule);
		}
		else
		{
			AERR("Could not get gralloc module for handle: 0x%x", (unsigned int)hnd);
			retval = -errno;
			goto cleanup;
		}

		framebuffer_mapper_t* fbMaper = &(m->fb_primary);
		if (hnd->usage & GRALLOC_USAGE_EXTERNAL_DISP)
		{
			ALOGD("register external display");
			fbMaper = &(m->fb_external);
		}
		if (!fbMaper->framebuffer)
		{
			fbMaper->framebuffer = new private_handle_t(hnd->flags, hnd->usage, hnd->size, hnd->base, 0, dup(hnd->fd), 0, 0);
			fbMaper->bufferSize = hnd->offset;
			fbMaper->numBuffers = fbMaper->framebuffer->size / fbMaper->bufferSize;
			fbMaper->bufferMask = 0;

			/*
			* map the framebuffer
			*/
			void* vaddr = mmap(0, fbMaper->framebuffer->size, PROT_READ|PROT_WRITE, MAP_SHARED, fbMaper->framebuffer->fd, 0);
			if (vaddr == MAP_FAILED)
			{
				AERR( "Error mapping the framebuffer (%s)", strerror(errno) );
				return -errno;
			}
			memset(vaddr, 0, fbMaper->framebuffer->size);
			fbMaper->framebuffer->base = vaddr;

			#if GRALLOC_ARM_UMP_MODULE
			#ifdef IOCTL_GET_FB_UMP_SECURE_ID
			ioctl(fbMaper->framebuffer->fd, IOCTL_GET_FB_UMP_SECURE_ID, &fbMaper->framebuffer->ump_id);
			#endif
			if ( (int)UMP_INVALID_SECURE_ID != fbMaper->framebuffer->ump_id )
			{
				AINF("framebuffer accessed with UMP secure ID %i\n", fbMaper->framebuffer->ump_id);
			}
			#endif
			ALOGD("register frame buffer count %d ",fbMaper->numBuffers );
		} else {
			ALOGE("ERROR::register frambuffer again!!!");
		}
	}
	else if (hnd->flags & (private_handle_t::PRIV_FLAGS_USES_UMP |
	                       private_handle_t::PRIV_FLAGS_USES_ION))
	{
		retval = gralloc_backend_register(hnd);
	}
	else
	{
		AERR("registering non-UMP buffer not supported. flags = %d", hnd->flags );
	}

cleanup:
	pthread_mutex_unlock(&s_map_lock);
	return retval;
}

static int gralloc_unregister_buffer(gralloc_module_t const* module, buffer_handle_t handle)
{
	if (private_handle_t::validate(handle) < 0)
	{
		AERR("unregistering invalid buffer %p, returning error", handle);
		return -EINVAL;
	}

	private_handle_t* hnd = (private_handle_t*)handle;

	AERR_IF(hnd->lockState & private_handle_t::LOCK_STATE_READ_MASK, "[unregister] handle %p still locked (state=%08x)", hnd, hnd->lockState);

	if (hnd->flags & private_handle_t::PRIV_FLAGS_FRAMEBUFFER)
	{
		pthread_mutex_lock(&s_map_lock);

		ALOGD("unregister framebuffer ");
		//AERR( "Can't unregister buffer 0x%x as it is a framebuffer", (unsigned int)handle );
		hw_module_t * pmodule = NULL;
		private_module_t *m = NULL;
		if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&pmodule) == 0)
		{
			m = reinterpret_cast<private_module_t *>(pmodule);
			framebuffer_mapper_t* fbMaper = &(m->fb_primary);
			if (hnd->usage & GRALLOC_USAGE_EXTERNAL_DISP)
			{
				ALOGD("unregister external display");
				fbMaper = &(m->fb_external);
			}

			if (fbMaper->framebuffer)
			{
				munmap((void*)fbMaper->framebuffer->base,fbMaper->framebuffer->size);
				close(fbMaper->framebuffer->fd);
				//reset framebuffer info
				delete fbMaper->framebuffer;
				fbMaper->framebuffer = 0;
				fbMaper->bufferMask = 0;
				fbMaper->numBuffers = 0;
			} else {
				AERR("Can't unregister a not exist buffers: 0x%x", (unsigned int)hnd);
			}
		} else {
			AERR("Could not get gralloc module for handle: 0x%x", (unsigned int)hnd);
		}
	// never unmap buffers that were not created in this process
	} else if (hnd->pid == getpid()) {
		pthread_mutex_lock(&s_map_lock);

		if (hnd->flags & (private_handle_t::PRIV_FLAGS_USES_UMP |
							private_handle_t::PRIV_FLAGS_USES_ION))
		{
			gralloc_backend_unregister(hnd);
		}
		else
		{
			AERR("Unregistering unknown buffer is not supported. Flags = %d", hnd->flags);
		}

#if MALI_AFBC_GRALLOC == 1
		/*
		 * Close shared attribute region file descriptor. It might seem strange to "free"
		 * this here since this can happen in a client process, but free here is nothing
		 * but unmapping and closing the duplicated file descriptor. The original ashmem
		 * fd instance is still open until alloc_device_free() is called. Even sharing
		 * of gralloc buffers within the same process should have fds dup:ed.
		 */
		gralloc_buffer_attr_free( hnd );

#endif
		hnd->base = 0;
		hnd->lockState  = 0;
		hnd->writeOwner = 0;

		pthread_mutex_unlock(&s_map_lock);
	}
	else
	{
		AERR( "Trying to unregister buffer %p from process %d that was not created in current process: %d", hnd, hnd->pid, getpid());
	}

	return 0;
}

static int gralloc_lock(gralloc_module_t const* module, buffer_handle_t handle, int usage, int l, int t, int w, int h, void** vaddr)
{
	if (private_handle_t::validate(handle) < 0)
	{
		AERR("Locking invalid buffer %p, returning error", handle );
		return -EINVAL;
	}

	private_handle_t* hnd = (private_handle_t*)handle;
	if (hnd->flags & private_handle_t::PRIV_FLAGS_USES_UMP || hnd->flags & private_handle_t::PRIV_FLAGS_USES_ION)
	{
		hnd->writeOwner = usage & GRALLOC_USAGE_SW_WRITE_MASK;
	}
	if (usage & (GRALLOC_USAGE_SW_READ_MASK | GRALLOC_USAGE_SW_WRITE_MASK))
	{
		*vaddr = (void*)hnd->base;
	}
	return 0;
}

static int gralloc_unlock(gralloc_module_t const* module, buffer_handle_t handle)
{
	if (private_handle_t::validate(handle) < 0)
	{
		AERR( "Unlocking invalid buffer %p, returning error", handle );
		return -EINVAL;
	}

	private_handle_t* hnd = (private_handle_t*)handle;

	if (hnd->flags & (private_handle_t::PRIV_FLAGS_USES_UMP |
	                  private_handle_t::PRIV_FLAGS_USES_ION)
	    && hnd->writeOwner)
	{
		gralloc_backend_sync(hnd);
	}

	return 0;
}

// There is one global instance of the module

static struct hw_module_methods_t gralloc_module_methods =
{
	open: gralloc_device_open
};

private_module_t::private_module_t()
{
#define INIT_ZERO(obj) (memset(&(obj),0,sizeof((obj))))

	base.common.tag = HARDWARE_MODULE_TAG;
	base.common.version_major = 1;
	base.common.version_minor = 0;
	base.common.id = GRALLOC_HARDWARE_MODULE_ID;
	base.common.name = "Graphics Memory Allocator Module";
	base.common.author = "ARM Ltd.";
	base.common.methods = &gralloc_module_methods;
	base.common.dso = NULL;
	INIT_ZERO(base.common.reserved);

	base.registerBuffer = gralloc_register_buffer;
	base.unregisterBuffer = gralloc_unregister_buffer;
	base.lock = gralloc_lock;
	base.unlock = gralloc_unlock;
	base.perform = NULL;
	INIT_ZERO(base.reserved_proc);

	INIT_ZERO(fb_primary);
	INIT_ZERO(fb_external);

	pthread_mutex_init(&(lock), NULL);
	swapInterval = 1;

	initialize_blk_conf();

#undef INIT_ZERO
};

/*
 * HAL_MODULE_INFO_SYM will be initialized using the default constructor
 * implemented above
 */ 
struct private_module_t HAL_MODULE_INFO_SYM;

