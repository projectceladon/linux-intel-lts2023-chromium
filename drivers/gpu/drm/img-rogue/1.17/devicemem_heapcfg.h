/**************************************************************************/ /*!
@File
@Title          Device Heap Configuration Helper Functions
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Device memory management
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /***************************************************************************/

#ifndef DEVICEMEMHEAPCFG_H
#define DEVICEMEMHEAPCFG_H

#include <powervr/mem_types.h>

#include "img_types.h"
#include "pvrsrv_error.h"

/*
 *  Supported log2 page size values for RGX_GENERAL_NON_4K_HEAP_ID
 */
#define RGX_HEAP_PAGE_SHIFTS_DEF \
	X(4KB, 12U) \
	X(16KB, 14U) \
	X(64KB, 16U) \
	X(256KB, 18U) \
	X(1MB, 20U) \
	X(2MB, 21U)

typedef enum RGX_HEAP_PAGE_SHIFTS_TAG
{
#define X(_name, _shift) RGX_HEAP_ ## _name ## _PAGE_SHIFT = _shift,
	RGX_HEAP_PAGE_SHIFTS_DEF
#undef X
} RGX_HEAP_PAGE_SHIFTS;

struct _PVRSRV_DEVICE_NODE_;
struct _CONNECTION_DATA_;


/*
  A "heap config" is a blueprint to be used for initial setting up of heaps
  when a device memory context is created.

  We define a data structure to define this, but it's really down to the
  caller to populate it. This is all expected to be in-kernel. We provide an
  API that client code can use to enquire about the blueprint, such that it may
  do the heap set-up during the context creation call on behalf of the user.
*/

/* Blueprint for a single heap */
typedef struct _DEVMEM_HEAP_BLUEPRINT_
{
	/* Name of this heap - for debug purposes, and perhaps for lookup
	by name */
	const IMG_CHAR *pszName;

	/* Virtual address of the beginning of the heap.  This _must_ be a
	multiple of the data page size for the heap.  It is
	_recommended_ that it be coarser than that - especially, it
	should begin on a boundary appropriate to the MMU for the
	device.  For Rogue, this is a Page Directory boundary, or 1GB
	(virtual address a multiple of 0x0040000000). */
	IMG_DEV_VIRTADDR sHeapBaseAddr;

	/* Length of the heap.  Given that the END address of the heap has
	a similar restriction to that of the _beginning_ of the heap.
	That is the heap length _must_ be a whole number of data pages.
	Again, the recommendation is that it ends on a 1GB boundary.
	Again, this is not essential, but we do know that (at the time
	of writing) the current implementation of mmu_common.c is such
	that no two heaps may share a page directory, thus the
	remaining virtual space would be wasted if the length were not
	a multiple of 1GB */
	IMG_DEVMEM_SIZE_T uiHeapLength;

	/* VA space starting sHeapBaseAddr to uiReservedRegionLength-1 are reserved
	for statically defined addresses (shared/known between clients and FW).
	Services never maps allocations into this reserved address space _unless_
	explicitly requested via PVRSRVMapToDeviceAddress by passing sDevVirtAddr
	which falls within this reserved range. Since this range is completely for
	clients to manage (where allocations are page granular), it _must_ again be
	a whole number of data pages. Additionally, another constraint enforces this
	to be a multiple of DEVMEM_HEAP_RESERVED_SIZE_GRANULARITY (which evaluates to
	max page size supported) to support varied pages sizes */
	IMG_DEVMEM_SIZE_T uiReservedRegionLength;

	/* Data page size.  This is the page size that is going to get
	programmed into the MMU, so it needs to be a valid one for the
	device.  Importantly, the start address and length _must_ be
	multiples of this page size.  Note that the page size is
	specified as the log 2 relative to 1 byte (e.g. 12 indicates
	4kB) */
	IMG_UINT32 uiLog2DataPageSize;

	/* Import alignment.  Force imports to this heap to be
	aligned to at least this value */
	IMG_UINT32 uiLog2ImportAlignment;

} DEVMEM_HEAP_BLUEPRINT;

void HeapCfgBlueprintInit(const IMG_CHAR        *pszName,
	                      IMG_UINT64             ui64HeapBaseAddr,
	                      IMG_DEVMEM_SIZE_T      uiHeapLength,
	                      IMG_DEVMEM_SIZE_T      uiReservedRegionLength,
	                      IMG_UINT32             ui32Log2DataPageSize,
	                      IMG_UINT32             uiLog2ImportAlignment,
	                      DEVMEM_HEAP_BLUEPRINT *psHeapBlueprint);

/* Entire named heap config */
typedef struct _DEVMEM_HEAP_CONFIG_
{
    /* Name of this heap config - for debug and maybe lookup */
    const IMG_CHAR *pszName;

    /* Number of heaps in this config */
    IMG_UINT32 uiNumHeaps;

    /* Array of individual heap blueprints as defined above */
    DEVMEM_HEAP_BLUEPRINT *psHeapBlueprintArray;
} DEVMEM_HEAP_CONFIG;


PVRSRV_ERROR
HeapCfgHeapConfigCount(struct _CONNECTION_DATA_ *psConnection,
    const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
    IMG_UINT32 *puiNumHeapConfigsOut
);

PVRSRV_ERROR
HeapCfgHeapCount(struct _CONNECTION_DATA_ *psConnection,
    const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
    IMG_UINT32 uiHeapConfigIndex,
    IMG_UINT32 *puiNumHeapsOut
);

PVRSRV_ERROR
HeapCfgHeapConfigName(struct _CONNECTION_DATA_ *psConnection,
    const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
    IMG_UINT32 uiHeapConfigIndex,
    IMG_UINT32 uiHeapConfigNameBufSz,
    IMG_CHAR *pszHeapConfigNameOut
);

PVRSRV_ERROR
HeapCfgHeapDetails(struct _CONNECTION_DATA_ *psConnection,
    const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
    IMG_UINT32 uiHeapConfigIndex,
    IMG_UINT32 uiHeapIndex,
    IMG_UINT32 uiHeapNameBufSz,
    IMG_CHAR *pszHeapNameOut,
    IMG_DEV_VIRTADDR *psDevVAddrBaseOut,
    IMG_DEVMEM_SIZE_T *puiHeapLengthOut,
    IMG_DEVMEM_SIZE_T *puiReservedRegionLengthOut,
    IMG_UINT32 *puiLog2DataPageSizeOut,
    IMG_UINT32 *puiLog2ImportAlignmentOut
);

#endif
