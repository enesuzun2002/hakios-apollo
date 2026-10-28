/*
 * ApolloGPU: the Mi 10T's Adreno 650 for programs to draw with. A copy of hakios.gpu.cpp (the POCO X5 Pro's, which
 * has the long explanations), kept apart from it: the same interface of Linux's msm driver -- the one Mesa's
 * freedreno speaks, through the libdrm stand-in of tools/mesa/hakios-drm, which looks for ApolloGPU first -- on
 * the kept-on API of bsd/dev/hakios.apollo_adreno.c (apollo_gpu_*) instead of hakios.adreno.c's. The selectors of our own
 * (0x7d, 0x7e, 0x7f) keep their numbers, so that nothing in userland has to know which phone it is on.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOSubMemoryDescriptor.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <mach/vm_param.h>
#include <sys/systm.h>

#include "MiniIOSurface.h"

extern "C" void hakios_tl(const char * format, ...) __printflike(1, 2);

extern "C" {
/* bsd/dev/hakios.apollo_adreno.c ("the GPU kept on"; declared here only: no header of bsd/sys is seen from here) */
bool apollo_gpu_ready(void);
bool apollo_gpu_parameter(uint32_t which, uint64_t * value);
uint64_t apollo_gpu_address(uint64_t size);
void apollo_gpu_address_free(uint64_t address, uint64_t size);
void apollo_gpu_space(uint64_t * used, uint32_t * live, uint64_t * largest, unsigned * holes, unsigned * most,
    uint64_t * lost, uint32_t * refused);
extern uint32_t apollo_gpu_va;                 /* boot-arg gpu_va: 0 as before, 1 describe the addresses, 2 also guard them */
bool apollo_gpu_map(uint64_t address, uint64_t phys, bool read_only);
void apollo_gpu_mapped(void);
bool apollo_gpu_unmap(uint64_t address, uint64_t size);       /* false: the SMMU may still see the pages: keep them */
void apollo_gpu_to_memory(void * start, uint64_t size);
uint32_t apollo_gpu_submit(const uint64_t * addresses, const uint32_t * words, uint32_t count);
extern uint64_t apollo_gpu_phase[4];           /* the last submission's ticks: ring room, times, ring writes, doorbell */
extern uint32_t apollo_gpu_log;
bool apollo_gpu_done(uint32_t fence);
uint32_t apollo_gpu_last(void);
void apollo_gpu_worked(uint64_t * microseconds, uint32_t * submissions, uint32_t by[7]);
void apollo_gpu_report(const char * why);
bool apollo_gpu_recover(void);
void apollo_gpu_stop(void);

void ApolloGPUStart(void);
void ApolloGPUStop(void);
void MiniIOMobileFramebufferShow(const uint8_t * pixels, uint32_t width, uint32_t height, uint32_t rowBytes);
void MiniIOMobileFramebufferSize(uint32_t * width, uint32_t * height);     /* the display as user space sees it */
/* apollo_gpu_present=1 (bsd/dev/hakios.apollo_display.c): the panel's three frames, which the GPU may draw into */
bool apollo_scanout_gpu_info(unsigned index, uint64_t * phys, uint32_t * stride, uint32_t * width, uint32_t * height,
    uint64_t * bytes, vm_offset_t * va);
int apollo_scanout_acquire(uint32_t timeout_ms);
void apollo_scanout_release(unsigned index);
void apollo_scanout_mark(unsigned index, uint32_t swap);
}

/* Linux's include/uapi/drm/msm_drm.h and drm.h: the requests' numbers and structures. */
enum {
	kDrmGemClose            = 0x09,
	kMsmGetParam            = 0x40,
	kMsmSetParam            = 0x41,
	kMsmGemNew              = 0x42,
	kMsmGemInfo             = 0x43,
	kMsmGemCpuPrep          = 0x44,
	kMsmGemCpuFini          = 0x45,
	kMsmGemSubmit           = 0x46,
	kMsmWaitFence           = 0x47,
	kMsmGemMadvise          = 0x48,
	kMsmSubmitQueueNew      = 0x4a,
	kMsmSubmitQueueClose    = 0x4b,
	kMsmSubmitQueueQuery    = 0x4c,
	kApolloScanout         = 0x7c,         /* ours: the panel's frames as buffers to draw into (apollo_gpu_present=1) */
	kApolloGPUWorked       = 0x7d,         /* ours: how long the 3D core itself has worked, and on how many */
	kApolloSurface         = 0x7e,         /* ours: an IOSurface's memory as a buffer */
	kApolloShow            = 0x7f,         /* ours: pixels of the caller's onto the panel, to see a test's result */
};

struct msm_timespec {
	int64_t seconds, nanoseconds;           /* of the clock that goes on while the machine sleeps */
};
struct msm_param {
	uint32_t pipe, param;
	uint64_t value;
	uint32_t len, pad;
};
struct msm_gem_new {
	uint64_t size;
	uint32_t flags, handle;
};
struct msm_gem_info {
	uint32_t handle, info;
	uint64_t value;
	uint32_t len, pad;
};
struct msm_gem_cpu_prep {
	uint32_t handle, op;
	struct msm_timespec timeout;
};
struct msm_gem_submit_cmd {
	uint32_t type, submit_idx, submit_offset, size, pad, nr_relocs;
	uint64_t relocs;
};
struct msm_gem_submit_bo {
	uint32_t flags, handle;
	uint64_t presumed;
};
struct msm_gem_submit {
	uint32_t flags, fence, nr_bos, nr_cmds;
	uint64_t bos, cmds;
	int32_t fence_fd;
	uint32_t queueid;
	uint64_t in_syncobjs, out_syncobjs;
	uint32_t nr_in_syncobjs, nr_out_syncobjs, syncobj_stride, pad;
};
struct msm_wait_fence {
	uint32_t fence, flags;
	struct msm_timespec timeout;
	uint32_t queueid;
};
struct msm_gem_madvise {
	uint32_t handle, madv, retained;
};
struct msm_submitqueue {
	uint32_t flags, prio, id;
};
struct msm_submitqueue_query {
	uint64_t data;
	uint32_t id, param, len, pad;
};
struct drm_gem_close {
	uint32_t handle, pad;
};
struct apollo_surface {
	uint32_t surface;                       /* the IOSurface's ID */
	uint32_t handle;                        /* kApolloDescribe: no buffer is wanted; else, back: the buffer, */
	uint64_t size;                          /* how big it is, */
	uint32_t offset, rowBytes;              /* where the pixels start in it and how long a row of them is, */
	uint32_t width, height, pixelFormat, planes;    /* and what they are */
};
struct apollo_show {
	uint64_t pixels;
	uint32_t width, height, rowBytes, pad;
};
/*
 * kApolloScanout (apollo_gpu_present=1): the window server's GPU scales CoreAnimation's frame into one of the panel's three
 * frames itself, and the display's driver flips to it instead of scaling with the processor.
 *   kScanoutInfo     out: count (3, or 0: not available -- no apollo_flip=1 / apollo_gpu_present=1), width, height, stride (bytes
 *                    of a GPU frame's row: a multiple of 64), surface (the frame 0's ID: frame i is that + i), displayWidth and
 *                    displayHeight (the size user space believes the display to be: what a swap's rectangles are in).
 *   kScanoutAcquire  in: timeoutMs; out: index, surface. A frame that is neither scanned, nor waiting for its latch, nor another
 *                    caller's, reserved for this one until the swap is shown or kScanoutRelease; waits up to timeoutMs for one
 *                    (EBUSY-like kIOReturnTimeout if there is none). The frame is a buffer by kApolloSurface with `surface` as
 *                    the IOSurface ID (it is what Mesa imports as a texture with glEGLImageTargetTexture2DOES).
 *   kScanoutQueue    in: index, swap. The next SwapEnd with that swap number (the word at 0x50 of its blob; with bit 31 set, whose layer 0
 *                    surface ID, the word at 0x54, is the rest of the number) is of a frame the GPU
 *                    scaled into `index`: the presenter waits for the GPU's last submission that named that buffer, and flips
 *                    to it without drawing. If the swap is not of a shape it can show so (more than the one layer, not a
 *                    full-panel BGRA picture), or no submission named the buffer, it is drawn by the processor as always.
 *   kScanoutRelease  in: index. The frame is given back: it was not used.
 */
enum { kScanoutInfo = 0, kScanoutAcquire = 1, kScanoutQueue = 2, kScanoutRelease = 3 };
#define kScanoutSurface 0x5ca00000u             /* the IDs of the frames as buffers, kApolloSurface: this + index (no IOSurface has them) */
struct apollo_scanout {
	uint32_t op, index, surface, swap;
	uint32_t timeoutMs, count, width, height;
	uint32_t stride, displayWidth, displayHeight, pad;
};
/*
 * kApolloGPUWorked: the GPU's own side of a measurement a program makes with its own clock.
 *
 * A program timing this driver (userland/src/metal_drv_test.m --time) can measure only wall
 * time: the whole of a draw, which is its own work, this ioctl, the submission, the core's
 * execution and the wait. Nothing it can reach says how much of that was the core -- and the
 * kernel has known all along, because bsd/dev/hakios.apollo_adreno.c times every submission off
 * CP_ALWAYS_ON_COUNTER (19.2 ticks a microsecond) and until now only ever printed it. This
 * request hands the same three numbers to the program, so the subtraction happens where the
 * two halves are both in hand instead of in whoever reads the log afterwards.
 *
 * It RESETS what it reports, which is apollo_gpu_worked()'s own contract ("since this was
 * last asked") and is what makes it useful: a caller reads once to zero the counters, does the
 * work it wants to price, and reads again. Two consequences, and they are the whole of the
 * sharp edge: two programs asking at once each get a part of the truth, and the kernel's own
 * five-second report in submit() below shares the same counters -- so a boot that reads this
 * will see gaps in those lines, and that is expected rather than a fault.
 */
struct apollo_worked {
	uint64_t microseconds;                  /* the core's own time on the submissions counted here */
	uint32_t submissions;                   /* how many of them it timed; 0 means nothing finished since */
	uint32_t by[7];                         /* ... of which under 0.1, 0.25, 0.5, 1, 2, 4 ms, and longer */
};

#define kApolloDescribe        0xffffffffu

#define MSM_INFO_GET_OFFSET     0
#define MSM_INFO_GET_IOVA       1
#define MSM_INFO_SET_NAME       2
#define MSM_INFO_SET_IOVA       4
#define MSM_INFO_GET_FLAGS      5
#define MSM_BO_GPU_READONLY     0x00000002
#define MSM_PREP_NOSYNC         0x04
#define MSM_SUBMIT_CMD_BUF      1
#define MSM_SUBMIT_FENCE_FD_OUT 0x20000000

#define GPU_BUFFERS             8192            /* a client's, at most */
#define GPU_BUFFER_MAX          (256u << 20)
#define GPU_SUBMIT_CMDS         128
#define GPU_SUBMIT_BOS          8192
#define GPU_PATIENCE_MS         5000            /* for a buffer or a submission that somebody waits for without a limit */
#define GPU_ASK_AGAIN_US        200             /* while waiting for the core: how long between two looks at how far it is */
#define GPU_PUSH_AGAIN_MS       4               /* a surface is pushed out of the cache again for a submission this long after the last */

static IOLock * gGPULock;                       /* around everything that is the GPU's (apollo_gpu_...) */
static bool gGPUReported;
static bool gGPUCached;                         /* boot-arg gpu_cached=1 */
static bool gGPUSaidSetIova;                    /* MSM_INFO_SET_IOVA has been refused out loud once */

/* What the last seconds' submissions were and cost here, for the log. */
static struct {
	uint64_t since, flushing, waiting, taking;      /* times of the processor's clock */
	uint64_t flushed;                       /* bytes sent out of the cache */
	uint32_t submissions, commands, buffers, surfaces, waits, over;
	uint32_t made, given_again;             /* buffers of their own made for programs, and of those kept ones given again */
	uint64_t making;                        /* ... and the time it took to make the others */
} gGPUStat;

/* Wait for the 3D core to be done with submission `fence`; with the lock held, which is let go of while waiting. */
static bool
gpuWait(uint32_t fence, uint64_t milliseconds)
{
	uint64_t began = mach_absolute_time(), patience;

	nanoseconds_to_absolutetime(milliseconds * NSEC_PER_MSEC, &patience);
	for (bool waited = false;; waited = true) {
		if (apollo_gpu_done(fence)) {
			if (waited) {
				gGPUStat.waits++;
				gGPUStat.waiting += mach_absolute_time() - began;
			}
			return true;
		}
		if (mach_absolute_time() - began >= patience || gGPUReported) {
			break;                  /* (a GPU that has stopped answering is not waited for again) */
		}
		IOLockUnlock(gGPULock);
		delay_for_interval(GPU_ASK_AGAIN_US, NSEC_PER_USEC);
		IOLockLock(gGPULock);
	}
	/*
	 * A wait of a second or more that came to nothing: the core has stopped answering. What it
	 * and its SMMU context say is put down, and then, with boot-arg gpu_recover=1, the core is
	 * reset and started again, which gives up the submission that hung and leaves later ones
	 * the core's own. Without that -- or if it does not work -- nothing is waited for again in
	 * this run: a GPU that may yet write memory of ours is given nothing more. Either way this
	 * wait has failed, and says so: the submission did not finish.
	 */
	if (milliseconds >= 1000 && !gGPUReported) {
		apollo_gpu_report("no answer");
		gGPUReported = !apollo_gpu_recover();
	}
	return false;
}

/*
 * The last submission that named each of the panel's frames (0: none since it was acquired), set under gGPULock by submit() and
 * cleared by kScanoutAcquire. A frame that was scaled into by the GPU is shown once that submission is done.
 */
static uint32_t gScanFence[3];

/* true: the GPU is done with the frame, or has no submission for it (then *submitted is false: nothing was drawn) */
bool
ApolloGPUScanoutDrawn(unsigned index, uint64_t milliseconds, bool * submitted)
{
	uint32_t fence;
	bool done = true;

	if (index > 2 || gGPULock == NULL) {
		*submitted = false;
		return true;
	}
	IOLockLock(gGPULock);
	fence = gScanFence[index];
	*submitted = fence != 0;
	if (fence != 0) {
		done = !apollo_gpu_ready() || gpuWait(fence, milliseconds);
	}
	IOLockUnlock(gGPULock);
	return done;
}

bool
ApolloGPUSurfaceDrawn(MiniIOSurface * surface, uint64_t milliseconds)
{
	uint32_t fence = surface->drawn;
	bool done = true;

	if (fence != 0 && gGPULock != NULL) {
		IOLockLock(gGPULock);
		done = !apollo_gpu_ready() || gpuWait(fence, milliseconds);
		IOLockUnlock(gGPULock);
	}
	return done;
}

/*
 * Boot-arg touch_lat=1: a line for every wait of a program's for the GPU that took a millisecond or
 * more ("tl: f KIND US at US"), for telling apart what holds a frame up (tools/frames.py).
 */
static TUNABLE(uint32_t, gWaitLines, "touch_lat", 0);

static void
slowWait(const char * kind, uint64_t began)
{
	uint64_t now = mach_absolute_time(), ns, at;

	if (!gWaitLines) {
		return;
	}
	absolutetime_to_nanoseconds(now - began, &ns);
	if (ns >= NSEC_PER_MSEC) {
		absolutetime_to_nanoseconds(now, &at);
		hakios_tl("tl: f %s %llu at %llu\n", kind, ns / 1000, at / 1000);
	}
}

/* How long from now a time of msm's is, in milliseconds; 0: now or past. */
static uint64_t
millisecondsUntil(const struct msm_timespec * when)
{
	uint64_t now;
	int64_t left;

	absolutetime_to_nanoseconds(mach_continuous_time(), &now);
	left = when->seconds * (int64_t) NSEC_PER_SEC + when->nanoseconds - (int64_t) now;
	if (left <= 0) {
		return 0;
	}
	return (uint64_t) left / NSEC_PER_MSEC > GPU_PATIENCE_MS ? GPU_PATIENCE_MS : (uint64_t) left / NSEC_PER_MSEC;
}

/*
 * A wait that was to end at a time that had passed when it was asked for, and the GPU was not
 * done: whoever asked goes on as if it were (a picture is shown half drawn). That is what comes
 * of a caller whose clock is not this one, so the first few are told.
 */
static void
waitOver(const struct msm_timespec * when, uint32_t fence)
{
	static uint32_t told;
	uint64_t now;

	gGPUStat.over++;
	if (told < 4) {
		told++;
		absolutetime_to_nanoseconds(mach_continuous_time(), &now);
		IOLog("ApolloGPU: a wait for submission %u, which is not done, until %lld.%09lld s, asked for at %llu.%09llu s: "
		    "over before it began\n", fence, when->seconds, when->nanoseconds, now / NSEC_PER_SEC, now % NSEC_PER_SEC);
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class ApolloGPU : public IOService
{
	OSDeclareDefaultStructors(ApolloGPU);

public:
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(ApolloGPU, IOService);

bool
ApolloGPU::start(IOService * provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	setProperty(kIOUserClientClassKey, "ApolloGPUUserClient");
	setName("ApolloGPU");
	registerService();
	return true;
}

void
ApolloGPUStart(void)
{
	IOService * provider = IOService::getPlatform();
	ApolloGPU * gpu;
	uint32_t cached = 0;

	gGPUCached = PE_parse_boot_argn("gpu_cached", &cached, sizeof(cached)) && cached != 0;
	gGPULock = IOLockAlloc();
	gpu = OSTypeAlloc(ApolloGPU);
	if (gpu == NULL || provider == NULL || !gpu->init() || !gpu->attach(provider)) {
		OSSafeReleaseNULL(gpu);
		IOLog("ApolloGPU: cannot create the service\n");
		return;
	}
	if (!gpu->start(provider)) {
		gpu->detach(provider);
		IOLog("ApolloGPU: the service did not start\n");
	}
	gpu->release();
}

/* The run is ending (hakios.apollo_adreno.c): nobody is in the middle of anything when the GPU goes. */
void
ApolloGPUStop(void)
{
	IOLockLock(gGPULock);
	apollo_gpu_stop();
	IOLockUnlock(gGPULock);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

struct gpu_buffer {
	IOMemoryDescriptor *            memory;         /* NULL: this handle is free */
	uint8_t *                       kernel;         /* where it is in the kernel */
	MiniIOSurface *                 surface;        /* whose memory it is, when not its own */
	uint64_t                        address;        /* the GPU's for it */
	uint64_t                        size;
	uint32_t                        flags;
	uint32_t                        seed;           /* the surface's when it was last pushed out of the cache, */
	uint64_t                        pushed;         /* ... and when that was */
	bool                            seen;           /* by the program: mapped, and from then on written by it */
	uint32_t                        fence;          /* the last submission that names it (0: none has) */
	uint32_t                        scanout;        /* one of the panel's frames (apollo_scanout_gpu_info), 1 + its index; else 0 */
};

/*
 * A buffer given back while the GPU is still at it: its memory and its place in the GPU's
 * address space are kept until the submission is done, and nobody waits for that. (A window
 * server that makes a surface for every frame gives the last frame's back while the GPU draws
 * this one: waiting there, for all the GPU had been given, cost it some five milliseconds a
 * frame.)
 */
struct gpu_left {
	IOMemoryDescriptor *            memory;
	MiniIOSurface *                 surface;
	uint64_t                        address, size;
	uint32_t                        fence, flags;
	uint32_t                        scanout;
};
#define GPU_LEFT                64

/*
 * A buffer of the program's own given back, and done with: kept, memory and place in the GPU's
 * address space, for the next one of the same size and kind. CoreAnimation makes a new texture to
 * draw a blur into for every frame of an animation with one (Search: 1082 x 1226, 6 MB with Mesa's
 * rounding), and gives them back only later, a score at a time; Mesa keeps what is given back for
 * a second and then lets it go. Each made afresh here took 2 to 3.5 ms of the window server's frame
 * (memory got, cleared, made write-combined, and entered page by page into the GPU's address
 * space). A buffer kept is given again as it is, not cleared: it was the same program's (Mesa's own
 * cache gives buffers back the same way). At most GPU_KEPT_BYTES are kept (the one kept longest
 * makes way), none for longer than GPU_KEPT_SECONDS. rc.jobs.fps, Search: 8 such textures made
 * slowly instead of 77, the window server's frames drawn in 6.8 ms instead of 9.3-10 (median).
 */
struct gpu_kept {
	IOMemoryDescriptor *            memory;         /* NULL: this place is free */
	uint8_t *                       kernel;
	uint64_t                        address, size, since;
	uint32_t                        flags;
};
#define GPU_KEPT                48
#define GPU_KEPT_BYTES          (160ull << 20)
#define GPU_KEPT_SECONDS        60              /* a buffer not asked for again in so long is let go */
#define GPU_KEEP_FROM           (256u << 10)    /* bytes: smaller ones are not worth it */
static TUNABLE(uint32_t, gGPUKeep, "gpu_keep", 1);      /* boot-arg gpu_keep=0 (a test): none kept; gpu_keep=2: kept ones cleared when given again */
/*
 * The most a client may have in buffers of its own making (kMsmGemNew), in MB: boot-arg gpu_client_mb, 1024 if
 * not given. The memory is wired, so one program could otherwise take all of the phone's; the window server's
 * frames and textures are a few hundred MB at the most. (fBufferBytes, which this is held against, also counts
 * the IOSurfaces and the panel's frames a client has as buffers.)
 */
static TUNABLE(uint32_t, gGPUClientMB, "gpu_client_mb", 1024);

class ApolloGPUUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(ApolloGPUUserClient);

	task_t                  fTask;
	struct gpu_buffer *     fBuffers;               /* handle n is fBuffers[n - 1] */
	uint32_t                fNextQueue;
	uint32_t                fNextBuffer;            /* where the search for a free handle starts: one given back is not the next given */
	uint32_t                fBufferCount;
	uint64_t                fBufferBytes;
	uint32_t                fSubmits;
	uint32_t                fStrangers;             /* requests that named a buffer the client does not have */
	struct gpu_left         fLeft[GPU_LEFT];
	uint32_t                fLeftCount;
	struct gpu_kept         fKept[GPU_KEPT];
	uint64_t                fKeptBytes;
	uint32_t                fScanReserved;          /* panel frames this client has acquired and not yet queued or released: a bit each */

	struct gpu_buffer * buffer(uint32_t handle);
	struct gpu_buffer * newBuffer(IOMemoryDescriptor * memory, uint64_t from, uint64_t size, uint32_t flags, uint32_t * handle);
	void freeBuffer(struct gpu_buffer * b);
	bool keep(IOMemoryDescriptor * memory, uint8_t * kernel, uint64_t address, uint64_t size, uint32_t flags);
	void dropKept(struct gpu_kept * k);
	void reap(void);
	IOReturn gemNew(struct msm_gem_new * request);
	IOReturn surfaceBuffer(struct apollo_surface * request);
	IOReturn scanoutBuffer(struct apollo_surface * request);
	IOReturn scanout(struct apollo_scanout * request);
	IOReturn gemInfo(struct msm_gem_info * request);
	IOReturn submit(struct msm_gem_submit * request);
	IOReturn show(const struct apollo_show * request);

public:
	virtual bool initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService * provider) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits * options, IOMemoryDescriptor ** memory) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments * args,
	    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(ApolloGPUUserClient, IOUserClient);

bool
ApolloGPUUserClient::initWithTask(task_t owningTask, void * securityID, UInt32 type, OSDictionary * properties)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type, properties)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
ApolloGPUUserClient::start(IOService * provider)
{
	if (!apollo_gpu_ready() || !IOUserClient::start(provider)) {
		return false;
	}
	fBuffers = IONewZero(struct gpu_buffer, GPU_BUFFERS);
	return fBuffers != NULL;
}

/*
 * A buffer's addresses out of the GPU's space and free for another. false: the SMMU has not said that it
 * has forgotten them (apollo_gpu_unmap()): the addresses are not handed out again, and the caller keeps the
 * memory the GPU's for good (gpuKeepForGood()), as it does a buffer the core is not done with. With the lock held.
 */
static bool
gpuUnmap(uint64_t address, uint64_t size)
{
	if (!apollo_gpu_unmap(address, size)) {
		return false;
	}
	apollo_gpu_address_free(address, size);
	return true;
}

/* Memory the GPU may still reach: a reference that is never let go of, so that it is nobody else's. */
static void
gpuKeepForGood(IOMemoryDescriptor * memory, MiniIOSurface * surface)
{
	if (surface == NULL) {
		memory->retain();
	} else {
		surface->memory->retain();
	}
}

/*
 * With the lock held. A buffer the GPU is not done with is left (fLeft) until it is, or, with
 * nowhere to leave it, waited for; if the GPU has stopped answering, the buffer's memory stays
 * the GPU's for good: it may yet be written, and must not be somebody else's by then.
 *
 * The handle is free from the start, before the wait lets go of the lock: what is given back is a
 * copy of the slot, so another thread giving back the same handle meanwhile finds nothing there,
 * and one making a buffer may have the slot again.
 */
void
ApolloGPUUserClient::freeBuffer(struct gpu_buffer * slot)
{
	struct gpu_buffer copy = *slot, * b = &copy;
	bool busy, lost;

	slot->memory = NULL;
	slot->surface = NULL;
	fBufferCount--;
	fBufferBytes -= b->size;
	busy = apollo_gpu_ready() && b->fence != 0 && !apollo_gpu_done(b->fence);
	if (busy && fLeftCount == GPU_LEFT) {
		busy = !gpuWait(b->fence, 1000);        /* (nowhere to keep it: this one is waited for; fLeftCount is looked at again after) */
	}
	if (busy && fLeftCount < GPU_LEFT) {
		struct gpu_left * left = &fLeft[fLeftCount++];

		left->memory = b->memory;
		left->surface = b->surface;
		left->address = b->address;
		left->size = b->size;
		left->fence = b->fence;
		left->flags = b->flags;
		left->scanout = b->scanout;
	} else {
		lost = busy;
		if (busy) {
			static uint32_t said;

			if (said++ < 8) {
				IOLog("ApolloGPU: the GPU is not done with what it was given: a buffer of %llu bytes stays its own\n", b->size);
			}
		} else if (b->surface == NULL && b->scanout == 0 && apollo_gpu_ready() &&
		    keep(b->memory, b->kernel, b->address, b->size, b->flags)) {
			b->memory = NULL;               /* (now the kept one's) */
		} else if (apollo_gpu_ready()) {
			lost = !gpuUnmap(b->address, b->size);
		}
		if (lost) {
			gpuKeepForGood(b->memory, b->surface);
		}
		if (b->memory != NULL) {
			b->memory->release();
		}
		if (b->surface != NULL) {
			MiniIOSurfaceDrop(b->surface);
		}
	}
}

/* The buffers that were given back early and that the GPU is done with now: freed. With the lock held. */
void
ApolloGPUUserClient::reap(void)
{
	for (uint32_t i = 0; i < fLeftCount;) {
		struct gpu_left * left = &fLeft[i];

		if (apollo_gpu_ready() && !apollo_gpu_done(left->fence)) {
			i++;
			continue;
		}
		if (left->surface == NULL && left->scanout == 0 && apollo_gpu_ready() &&
		    keep(left->memory, NULL, left->address, left->size, left->flags)) {
			left->memory = NULL;
		} else if (apollo_gpu_ready() && !gpuUnmap(left->address, left->size)) {
			gpuKeepForGood(left->memory, left->surface);
		}
		if (left->memory != NULL) {
			left->memory->release();
		}
		if (left->surface != NULL) {
			MiniIOSurfaceDrop(left->surface);
		}
		*left = fLeft[--fLeftCount];
	}
}

IOReturn
ApolloGPUUserClient::clientClose(void)
{
	uint32_t reserved;

	/* (taken under the lock, given back outside it: the display's driver takes gGPULock under locks of its own) */
	IOLockLock(gGPULock);
	reserved = fScanReserved;
	fScanReserved = 0;
	IOLockUnlock(gGPULock);
	for (unsigned i = 0; i < 3; i++) {
		if (reserved & (1u << i)) {
			apollo_scanout_release(i);              /* (a window server that is gone has no frame to be waited for) */
		}
	}
	if (fBuffers != NULL) {
		IOLockLock(gGPULock);
		/* what the client had the GPU do last may still be going on, in these buffers */
		if (apollo_gpu_ready()) {
			gpuWait(apollo_gpu_last(), 1000);
		}
		for (uint32_t i = 0; i < GPU_BUFFERS; i++) {
			if (fBuffers[i].memory != NULL) {
				freeBuffer(&fBuffers[i]);
			}
		}
		reap();
		for (uint32_t i = 0; i < GPU_KEPT; i++) {
			dropKept(&fKept[i]);
		}
		if (fLeftCount != 0) {
			IOLog("ApolloGPU: the GPU is not done with %u buffers of a program that is gone: they stay its own\n", fLeftCount);
		}
		IOLockUnlock(gGPULock);
	}
	if (!isInactive()) {
		terminate();
	}
	return kIOReturnSuccess;
}

void
ApolloGPUUserClient::free(void)
{
	if (fBuffers != NULL) {
		IODelete(fBuffers, struct gpu_buffer, GPU_BUFFERS);
		fBuffers = NULL;
	}
	IOUserClient::free();
}

struct gpu_buffer *
ApolloGPUUserClient::buffer(uint32_t handle)
{
	if (handle == 0 || handle > GPU_BUFFERS || fBuffers[handle - 1].memory == NULL) {
		/* (a program that names a buffer it has given back draws into somebody else's next: worth knowing) */
		if (handle != 0 && fStrangers++ < 16) {
			IOLog("ApolloGPU: buffer %u is asked for and is not there\n", handle);
		}
		return NULL;
	}
	return &fBuffers[handle - 1];
}

/*
 * A handle and addresses of the GPU's for `size` bytes of a memory descriptor, from byte `from`
 * of it on (wired memory; both in whole pages). With the lock held; NULL: there is no room.
 */
struct gpu_buffer *
ApolloGPUUserClient::newBuffer(IOMemoryDescriptor * memory, uint64_t from, uint64_t size, uint32_t flags, uint32_t * handle)
{
	struct gpu_buffer * b = NULL;
	bool mapped = true;

	if (!apollo_gpu_ready()) {
		return NULL;                            /* (given back while the caller was not holding the lock: the run is ending) */
	}
	for (uint32_t n = 0; n < GPU_BUFFERS; n++) {
		uint32_t i = (fNextBuffer + n) % GPU_BUFFERS;

		if (fBuffers[i].memory == NULL) {
			b = &fBuffers[i];
			*handle = i + 1;
			fNextBuffer = i + 1;
			break;
		}
	}
	if (b == NULL || (b->address = apollo_gpu_address(size)) == 0) {
		IOLog("ApolloGPU: no %s left for a buffer of %llu bytes\n", b ? "addresses" : "handles", size);
		return NULL;
	}
	for (uint64_t offset = 0; offset < size && mapped; offset += 4096) {
		IOByteCount length = 0;
		addr64_t phys = memory->getPhysicalSegment(from + offset, &length, kIOMemoryMapperNone);

		mapped = phys != 0 && apollo_gpu_map(b->address + offset, phys, flags & MSM_BO_GPU_READONLY);
	}
	apollo_gpu_mapped();
	if (!mapped) {
		if (!gpuUnmap(b->address, size)) {
			memory->retain();               /* (what was entered may still be seen: stays the GPU's, gpuKeepForGood()) */
		}
		IOLog("ApolloGPU: a buffer of %llu bytes cannot be entered into the GPU's address space\n", size);
		return NULL;
	}
	b->size = size;
	b->flags = flags;
	b->seed = 0;
	b->pushed = 0;
	b->seen = false;
	b->fence = 0;
	b->surface = NULL;
	b->scanout = 0;
	fBufferCount++;
	fBufferBytes += size;
	return b;
}

/* With the lock held. */
void
ApolloGPUUserClient::dropKept(struct gpu_kept * k)
{
	if (k->memory == NULL) {
		return;
	}
	if (apollo_gpu_ready() && !gpuUnmap(k->address, k->size)) {
		gpuKeepForGood(k->memory, NULL);
	}
	k->memory->release();
	k->memory = NULL;
	fKeptBytes -= k->size;
}

/* With the lock held, the GPU done with the buffer: kept if it is worth it (those kept longest make way). */
bool
ApolloGPUUserClient::keep(IOMemoryDescriptor * memory, uint8_t * kernel, uint64_t address, uint64_t size, uint32_t flags)
{
	struct gpu_kept * place = NULL;

	if (gGPUKeep == 0 || size < GPU_KEEP_FROM || size > GPU_KEPT_BYTES / 4 || memory == NULL ||
	    (kernel == NULL && OSDynamicCast(IOBufferMemoryDescriptor, memory) == NULL)) {
		return false;
	}
	for (;;) {
		struct gpu_kept * oldest = NULL;

		place = NULL;
		for (uint32_t i = 0; i < GPU_KEPT; i++) {
			if (fKept[i].memory == NULL) {
				place = place != NULL ? place : &fKept[i];
			} else if (oldest == NULL || fKept[i].since < oldest->since) {
				oldest = &fKept[i];
			}
		}
		if (place != NULL && fKeptBytes + size <= GPU_KEPT_BYTES) {
			break;
		}
		dropKept(oldest);
	}
	place->memory = memory;
	place->kernel = kernel != NULL ? kernel : (uint8_t *) OSDynamicCast(IOBufferMemoryDescriptor, memory)->getBytesNoCopy();
	place->address = address;
	place->size = size;
	place->flags = flags;
	place->since = mach_absolute_time();
	fKeptBytes += size;
	return true;
}

IOReturn
ApolloGPUUserClient::gemNew(struct msm_gem_new * request)
{
	uint64_t size = round_page_64(request->size), began = mach_absolute_time();
	struct gpu_buffer * b;
	IOBufferMemoryDescriptor * memory;

	if (size == 0 || size > GPU_BUFFER_MAX) {
		return kIOReturnBadArgument;
	}
	/* one kept of this size and kind, if there is one (the one kept last: likeliest still in the caches' way) */
	IOLockLock(gGPULock);
	struct gpu_kept * found = NULL;
	uint64_t old;

	if (!apollo_gpu_ready()) {
		IOLockUnlock(gGPULock);
		return kIOReturnNotReady;                       /* (given back since externalMethod() looked: the run is ending) */
	}
	if (fBufferBytes + size > ((uint64_t) gGPUClientMB << 20)) {
		static uint32_t said;

		IOLockUnlock(gGPULock);
		if (said++ < 8) {
			IOLog("ApolloGPU: a buffer of %llu bytes refused: the client has %llu MB in buffers, gpu_client_mb=%u\n", size,
			    fBufferBytes >> 20, gGPUClientMB);
		}
		return kIOReturnNoMemory;
	}

	nanoseconds_to_absolutetime(GPU_KEPT_SECONDS * NSEC_PER_SEC, &old);
	for (uint32_t i = 0; i < GPU_KEPT; i++) {
		struct gpu_kept * k = &fKept[i];

		if (k->memory != NULL && began - k->since > old) {
			dropKept(k);
		}

		if (k->memory != NULL && k->size == size && k->flags == request->flags && (found == NULL || k->since > found->since)) {
			found = k;
		}
	}
	for (uint32_t n = 0; found != NULL && n < GPU_BUFFERS; n++) {
		uint32_t h = (fNextBuffer + n) % GPU_BUFFERS;

		if (fBuffers[h].memory == NULL) {
			if (gGPUKeep == 2) {
				bzero(found->kernel, (size_t) found->size);     /* (before the handle is the program's: under the lock) */
			}
			b = &fBuffers[h];
			bzero(b, sizeof(*b));
			b->memory = found->memory;
			b->kernel = found->kernel;
			b->address = found->address;
			b->size = found->size;
			b->flags = found->flags;
			fNextBuffer = h + 1;
			fBufferCount++;
			fBufferBytes += size;
			request->handle = h + 1;
			found->memory = NULL;
			fKeptBytes -= size;
			gGPUStat.given_again++;
			IOLockUnlock(gGPULock);
			return kIOReturnSuccess;
		}
	}
	IOLockUnlock(gGPULock);
	/* (the memory comes zeroed, and making it write-combined pushes the zeroes out of the cache) */
	memory = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut | kIOMemoryKernelUserShared |
	    (gGPUCached ? 0 : kIOMapWriteCombineCache), (vm_size_t) size, PAGE_SIZE);
	if (memory == NULL) {
		IOLog("ApolloGPU: no memory for a buffer of %llu bytes (%u buffers, %llu MB with this client)\n", size, fBufferCount,
		    fBufferBytes >> 20);
		return kIOReturnNoMemory;
	}
	if (gGPUCached) {
		apollo_gpu_to_memory(memory->getBytesNoCopy(), size);
	}

	IOLockLock(gGPULock);
	/* (the lock was let go of: the limit again, against what other threads of the client made meanwhile; newBuffer() sees to the GPU being there) */
	b = fBufferBytes + size > ((uint64_t) gGPUClientMB << 20) ? NULL : newBuffer(memory, 0, size, request->flags, &request->handle);
	if (b != NULL) {
		b->memory = memory;
		b->kernel = (uint8_t *) memory->getBytesNoCopy();
		gGPUStat.made++;
		gGPUStat.making += mach_absolute_time() - began;
	}
	IOLockUnlock(gGPULock);
	if (b == NULL) {
		memory->release();
		return kIOReturnNoMemory;
	}
	return kIOReturnSuccess;
}

/*
 * The memory of an IOSurface as a buffer: its pixels, from the page after the surface's shared
 * state on (MiniIOSurface.h). The surface is held for as long as the buffer is there. A surface
 * that is a buffer of this client's already is that buffer again, as with Linux's handles: so
 * the client must not be giving that buffer back at the same moment (freedreno asks under the
 * lock it gives buffers back under). Asked only to describe (kApolloDescribe), no buffer is made.
 */
IOReturn
ApolloGPUUserClient::surfaceBuffer(struct apollo_surface * request)
{
	MiniIOSurface * s;
	struct gpu_buffer * b = NULL;
	IOMemoryDescriptor * memory;
	uint64_t size, began = mach_absolute_time();

	if ((request->surface & 0xffff0000u) == kScanoutSurface) {
		return scanoutBuffer(request);
	}
	s = MiniIOSurfaceCopy(request->surface);
	if (s == NULL) {
		return kIOReturnNotFound;
	}
	if (s->client != NULL) {
		/* (memory of a program's own with a surface made around it: not in whole pages, and seldom in rows the GPU takes) */
		MiniIOSurfaceDrop(s);
		return kIOReturnUnsupported;
	}
	size = round_page_64(s->info.allocSize);
	memory = IOSubMemoryDescriptor::withSubRange(s->memory, PAGE_SIZE, size, kIODirectionInOut);
	if (memory == NULL || size == 0 || size > GPU_BUFFER_MAX) {
		OSSafeReleaseNULL(memory);
		MiniIOSurfaceDrop(s);
		return kIOReturnNoMemory;
	}
	request->size = size;
	request->offset = s->info.offset;
	request->rowBytes = s->info.bytesPerRow;
	request->width = s->info.width;
	request->height = s->info.height;
	request->pixelFormat = s->info.pixelFormat;
	request->planes = s->info.planeCount;
	if (request->handle == kApolloDescribe) {
		request->handle = 0;
		memory->release();
		MiniIOSurfaceDrop(s);
		return kIOReturnSuccess;
	}

	IOLockLock(gGPULock);
	for (uint32_t i = 0; i < GPU_BUFFERS; i++) {
		if (fBuffers[i].memory != NULL && fBuffers[i].surface == s) {
			b = &fBuffers[i];
			request->handle = i + 1;
			break;
		}
	}
	if (b != NULL) {
		memory->release();
		MiniIOSurfaceDrop(s);
	} else if ((b = newBuffer(s->memory, PAGE_SIZE, size, 0, &request->handle)) != NULL) {
		b->memory = memory;
		b->kernel = s->kbase + PAGE_SIZE;
		b->surface = s;
		gGPUStat.surfaces++;
	}
	gGPUStat.taking += mach_absolute_time() - began;
	IOLockUnlock(gGPULock);
	if (b == NULL) {
		memory->release();
		MiniIOSurfaceDrop(s);
		return kIOReturnNoMemory;
	}
	return kIOReturnSuccess;
}

/*
 * One of the panel's frames (kScanoutSurface + index) as a buffer, as an IOSurface's memory is one (surfaceBuffer() above): its
 * physical pages, which are the display's reservation (bsd/dev/hakios.apollo_display.c), entered into the GPU's address space. The
 * 3D core's page table entries are PTE_CORE, attribute 0 of MAIR (Linux's: uncached, SMMU_MAIR0 in hakios.apollo_adreno.c), so what
 * the core writes is in memory for the display controller, which reads memory with nothing in between. Rows are of the GPU
 * stride (64-byte multiples; Mesa takes no other pitch of a render target). The same frame asked for again is the same buffer.
 * It is made whether or not the client has the frame acquired (the window server makes all three when it starts,
 * userland/src/eagl.c): what is checked is that a submission naming one is of the client that has it acquired (submit()).
 */
IOReturn
ApolloGPUUserClient::scanoutBuffer(struct apollo_surface * request)
{
	unsigned index = request->surface & 0xffffu;
	uint64_t phys, bytes;
	uint32_t stride, width, height;
	vm_offset_t va;
	IOMemoryDescriptor * memory;
	struct gpu_buffer * b = NULL;

	if (index > 2 || !apollo_scanout_gpu_info(index, &phys, &stride, &width, &height, &bytes, &va)) {
		return kIOReturnUnsupported;
	}
	request->size = bytes;
	request->offset = 0;
	request->rowBytes = stride;
	request->width = width;
	request->height = height;
	request->pixelFormat = 0x42475241;              /* 'BGRA' */
	request->planes = 0;
	if (request->handle == kApolloDescribe) {
		request->handle = 0;
		return kIOReturnSuccess;
	}
	IOLockLock(gGPULock);
	for (uint32_t i = 0; i < GPU_BUFFERS; i++) {
		if (fBuffers[i].memory != NULL && fBuffers[i].scanout == index + 1) {
			request->handle = i + 1;
			IOLockUnlock(gGPULock);
			return kIOReturnSuccess;
		}
	}
	IOLockUnlock(gGPULock);
	memory = IOMemoryDescriptor::withPhysicalAddress((IOPhysicalAddress) phys, (IOByteCount) bytes, kIODirectionInOut);
	if (memory == NULL) {
		return kIOReturnNoMemory;
	}
	IOLockLock(gGPULock);
	b = newBuffer(memory, 0, bytes, 0, &request->handle);
	if (b != NULL) {
		b->memory = memory;
		b->kernel = (uint8_t *) va;
		b->scanout = index + 1;
	}
	IOLockUnlock(gGPULock);
	if (b == NULL) {
		memory->release();
		return kIOReturnNoMemory;
	}
	return kIOReturnSuccess;
}

IOReturn
ApolloGPUUserClient::scanout(struct apollo_scanout * request)
{
	uint64_t phys, bytes;
	uint32_t stride, width, height;
	vm_offset_t va;

	switch (request->op) {
	case kScanoutInfo:
		request->count = 0;
		if (apollo_scanout_gpu_info(0, &phys, &stride, &width, &height, &bytes, &va)) {
			request->count = 3;
			request->width = width;
			request->height = height;
			request->stride = stride;
			request->surface = kScanoutSurface;
			MiniIOMobileFramebufferSize(&request->displayWidth, &request->displayHeight);
		}
		return kIOReturnSuccess;
	case kScanoutAcquire: {
		int index = apollo_scanout_acquire(request->timeoutMs > 1000 ? 1000 : request->timeoutMs);

		if (index < 0) {
			return kIOReturnTimeout;
		}
		IOLockLock(gGPULock);
		gScanFence[index] = 0;                  /* (nothing has drawn into it yet) */
		fScanReserved |= 1u << index;
		IOLockUnlock(gGPULock);
		request->index = (uint32_t) index;
		request->surface = kScanoutSurface + (uint32_t) index;
		return kIOReturnSuccess;
	}
	/*
	 * fScanReserved is looked at and changed under gGPULock (submit() reads it there too), so that of two
	 * threads queueing or releasing one frame, one does; the display's driver is called outside the lock.
	 */
	case kScanoutQueue:
	case kScanoutRelease: {
		bool reserved;

		if (request->index > 2) {
			return kIOReturnBadArgument;
		}
		IOLockLock(gGPULock);
		reserved = (fScanReserved & (1u << request->index)) != 0;
		fScanReserved &= ~(1u << request->index);       /* (the display's driver has it from here: apollo_scanout_*) */
		IOLockUnlock(gGPULock);
		if (!reserved) {
			return kIOReturnBadArgument;
		}
		if (request->op == kScanoutQueue) {
			apollo_scanout_mark(request->index, request->swap);
		} else {
			apollo_scanout_release(request->index);
		}
		return kIOReturnSuccess;
	}
	}
	return kIOReturnBadArgument;
}

IOReturn
ApolloGPUUserClient::gemInfo(struct msm_gem_info * request)
{
	struct gpu_buffer * b;
	IOReturn ret = kIOReturnSuccess;

	IOLockLock(gGPULock);
	b = buffer(request->handle);
	if (b == NULL) {
		ret = kIOReturnBadArgument;
	} else if (request->info == MSM_INFO_GET_OFFSET) {
		request->value = (uint64_t) request->handle << 32;      /* nobody maps by it here */
	} else if (request->info == MSM_INFO_GET_IOVA) {
		/*
		 * The address the buffer was given when it was made, and the buffer's until it is
		 * given back. Nothing here ever moves a live buffer, so a caller may ask once and
		 * keep the answer -- which is what freedreno does (fd_bo_init_common).
		 */
		request->value = b->address;
	} else if (request->info == MSM_INFO_SET_IOVA) {
		/*
		 * Where a buffer goes is not the program's to choose here: every client is in the
		 * one SMMU context 0, so an address one program picked would be a place in every
		 * other program's space as well. Linux draws the line in the same place for the
		 * two parameters that describe a space -- VA_START and VA_SIZE are refused unless
		 * the client has page tables of its own (adreno_gpu.c, adreno_get_param) -- and
		 * placing a buffer is the stronger request of the two. Said once, by name, rather
		 * than returned as a bare code nobody can look up.
		 */
		if (!gGPUSaidSetIova) {
			gGPUSaidSetIova = true;
			IOLog("ApolloGPU: a program asked to place buffer %u at %#llx itself: refused. There is one "
			    "page table here for every client, so addresses are the kernel's to give and "
			    "MSM_INFO_GET_IOVA is how to learn them\n", request->handle, request->value);
		}
		ret = kIOReturnUnsupported;
	} else if (request->info == MSM_INFO_GET_FLAGS) {
		request->value = b->flags;
	} else if (request->info != MSM_INFO_SET_NAME) {            /* a name for somebody's debugger: not kept */
		ret = kIOReturnUnsupported;
	}
	IOLockUnlock(gGPULock);
	return ret;
}

IOReturn
ApolloGPUUserClient::submit(struct msm_gem_submit * request)
{
	struct msm_gem_submit_bo * bos = NULL;
	struct msm_gem_submit_cmd * cmds = NULL;
	uint64_t addresses[GPU_SUBMIT_CMDS] = { 0 };     /* (the first is put in the log even when no command buffer is taken) */
	uint32_t words[GPU_SUBMIT_CMDS] = { 0 }, count = 0;
	uint64_t began, nanos, again;
	uint64_t at_start = mach_absolute_time(), at_copied = at_start, at_locked = at_start, at_flushed = at_start;
	uint64_t at_submitted = at_start, at_reaped = at_start;
	IOReturn ret = kIOReturnBadArgument;

	if (request->nr_cmds == 0 || request->nr_cmds > GPU_SUBMIT_CMDS || request->nr_bos == 0 || request->nr_bos > GPU_SUBMIT_BOS) {
		IOLog("ApolloGPU: a submission of %u command buffers in %u buffers: not taken\n", request->nr_cmds, request->nr_bos);
		return kIOReturnBadArgument;
	}
	bos = IONew(struct msm_gem_submit_bo, request->nr_bos);
	cmds = IONew(struct msm_gem_submit_cmd, request->nr_cmds);
	if (bos == NULL || cmds == NULL) {
		ret = kIOReturnNoMemory;
		goto out;
	}
	if (copyin((user_addr_t) request->bos, bos, request->nr_bos * sizeof(*bos)) != 0 ||
	    copyin((user_addr_t) request->cmds, cmds, request->nr_cmds * sizeof(*cmds)) != 0) {
		ret = kIOReturnVMError;
		goto out;
	}
	at_copied = mach_absolute_time();
	IOLockLock(gGPULock);
	at_locked = mach_absolute_time();
	/*
	 * Looked at again under the lock (externalMethod() looked without it): a GPU given back meanwhile takes
	 * nothing, and one that has stopped answering and was not recovered (gGPUReported) is given nothing more in
	 * this run -- as gpuWait() waits for nothing more.
	 */
	if (!apollo_gpu_ready() || gGPUReported) {
		IOLockUnlock(gGPULock);
		ret = apollo_gpu_ready() ? kIOReturnNotResponding : kIOReturnNotReady;
		goto out;
	}
	/*
	 * A submission that names one of the panel's frames is not checked against kScanoutAcquire: Mesa sends the
	 * blit into a frame after eagl.c has queued it (glClientWaitSync does not always flush, userland/src/eagl.c),
	 * and keeps the frames in later submissions, so such a check refused about 1600 good submissions in a
	 * 240 s run (27 October, -22 in MESA's flush_submit_list). Keeping other programs off the frames needs
	 * eagl.c to hold a frame until its blit is submitted.
	 */
	/*
	 * The GPU reads memory and writes memory. What this processor's cache holds of an IOSurface
	 * (somebody has written it, or has read it, as the display's driver does, and the GPU is to
	 * draw into it again) goes out of the cache now: when the surface is said to have changed
	 * (its seed: who writes one is to lock it, and so does a program that has had the GPU draw
	 * into one: userland/src/eagl.c), and in any case once in a few milliseconds, which is once
	 * a frame and not once a submission: not everybody who draws into a surface says so (an
	 * application drawing a picture that is on the screen meanwhile). A surface this program
	 * has mapped, every time. The buffers made here need none of that, unless they are
	 * ordinary memory (gpu_cached=1).
	 */
	began = mach_absolute_time();
	nanoseconds_to_absolutetime(GPU_PUSH_AGAIN_MS * NSEC_PER_MSEC, &again);
	for (uint32_t i = 0; i < request->nr_bos; i++) {
		struct gpu_buffer * b = buffer(bos[i].handle);

		if (b == NULL) {
			continue;
		}
		if (b->surface != NULL) {
			uint32_t seed = b->surface->shared()->seed;

			if (seed == b->seed && !b->seen && began - b->pushed < again) {
				continue;
			}
			b->seed = seed;
			b->pushed = began;
		} else if (!b->seen || !gGPUCached || b->scanout != 0) {
			continue;               /* (the panel's frames are write-combined for the processor and uncached for the GPU) */
		}
		apollo_gpu_to_memory(b->kernel, b->size);
		gGPUStat.flushed += b->size;
	}
	apollo_gpu_to_memory(NULL, 0);                 /* (and what was written last is in memory: a barrier) */
	gGPUStat.flushing += mach_absolute_time() - began;
	gGPUStat.buffers += request->nr_bos;
	at_flushed = mach_absolute_time();
	for (uint32_t i = 0; i < request->nr_cmds; i++) {
		struct gpu_buffer * b = cmds[i].submit_idx < request->nr_bos ? buffer(bos[cmds[i].submit_idx].handle) : NULL;

		if (cmds[i].type != MSM_SUBMIT_CMD_BUF) {
			continue;                               /* the other kinds are for a GPU shared between address spaces */
		}
		if (b == NULL || cmds[i].nr_relocs != 0 || (cmds[i].size & 3) || (cmds[i].submit_offset & 3) ||
		    (uint64_t) cmds[i].submit_offset + cmds[i].size > b->size) {
			IOLog("ApolloGPU: command buffer %u of a submission: buffer %u, offset %u, %u bytes, %u relocations: not taken\n",
			    i, cmds[i].submit_idx, cmds[i].submit_offset, cmds[i].size, cmds[i].nr_relocs);
			IOLockUnlock(gGPULock);
			goto out;
		}
		addresses[count] = b->address + cmds[i].submit_offset;
		words[count++] = cmds[i].size / 4;
	}
	request->fence = apollo_gpu_submit(addresses, words, count);
	at_submitted = mach_absolute_time();
	/*
	 * The ring is full and the core is not reading it: it has stopped. With gpu_recover=1 it is
	 * reset and given this submission again -- after a reset the ring is empty, so there is
	 * room for it. (A wait of a second or more notices a stopped core first, as a rule; this is
	 * for a program that submits without waiting.)
	 */
	if (request->fence == 0 && !gGPUReported) {
		apollo_gpu_report("the ring is full");
		gGPUReported = !apollo_gpu_recover();
		if (!gGPUReported) {
			request->fence = apollo_gpu_submit(addresses, words, count);
		}
	}
	if (apollo_gpu_log && request->fence != 0) {
		/* "tl: Q FENCE at US: N buffers, B of 4 MB or more, W words of commands": what was given (tl: q: what it took) */
		uint32_t big = 0, all_words = 0;
		uint64_t ns;

		for (uint32_t i = 0; i < request->nr_bos; i++) {
			struct gpu_buffer * b = buffer(bos[i].handle);

			big += b != NULL && b->size >= (4u << 20);
		}
		for (uint32_t i = 0; i < count; i++) {
			all_words += words[i];
		}
		absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
		hakios_tl("tl: Q %u at %llu: %u buffers, %u big, %u words\n", request->fence, ns / 1000, request->nr_bos, big, all_words);
	}
	if (request->flags & MSM_SUBMIT_FENCE_FD_OUT) {
		request->fence_fd = -1;
	}
	/*
	 * The surfaces named are being drawn into, or read, until this submission is done: the
	 * display's driver waits for that before it puts one on the panel (ApolloGPUSurfaceDrawn()).
	 * (freedreno names every buffer for reading and writing alike, so which it is cannot be
	 * told; the last submission of a frame names the surface the frame is drawn into.)
	 */
	for (uint32_t i = 0; request->fence != 0 && i < request->nr_bos; i++) {
		struct gpu_buffer * b = buffer(bos[i].handle);

		if (b != NULL) {
			b->fence = request->fence;
			if (b->surface != NULL) {
				b->surface->drawn = request->fence;
			}
			if (b->scanout != 0) {
				gScanFence[b->scanout - 1] = request->fence;    /* (what the display's driver waits for, ApolloGPUScanoutDrawn) */
			}
		}
	}
	reap();
	at_reaped = mach_absolute_time();
	if (fSubmits++ < 4) {
		IOLog("ApolloGPU: submission %u: %u command buffers, the first at %#llx, %u words\n", request->fence, count,
		    addresses[0], words[0]);
	}
	/* every five seconds in which something was submitted: how much, and what it cost here */
	gGPUStat.submissions++;
	gGPUStat.commands += count;
	if (gGPUStat.since == 0) {
		gGPUStat.since = began;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - gGPUStat.since, &nanos);
	if (nanos >= 5 * NSEC_PER_SEC) {
		uint64_t flushing, waiting, taking, worked;
		uint32_t timed, by[7];

		absolutetime_to_nanoseconds(gGPUStat.flushing, &flushing);
		absolutetime_to_nanoseconds(gGPUStat.waiting, &waiting);
		absolutetime_to_nanoseconds(gGPUStat.taking, &taking);
		apollo_gpu_worked(&worked, &timed, by);
		uint64_t making;

		absolutetime_to_nanoseconds(gGPUStat.making, &making);
		IOLog("ApolloGPU: buffers: %u made in %llu us each, %u kept ones given again (%llu MB kept)\n", gGPUStat.made,
		    gGPUStat.made ? making / NSEC_PER_USEC / gGPUStat.made : 0, gGPUStat.given_again, fKeptBytes >> 20);
		IOLog("ApolloGPU: %u submissions in %llu ms: %u command buffers and %u buffers named, %llu KB out of the cache "
		    "in %llu us a submission; the GPU worked %llu us on each of %u (%u under 0.1 ms, %u under 0.25, %u under 0.5, "
		    "%u under 1, %u under 2, %u under 4, %u longer; %llu ms in all); %u waits of %llu us, %u over before they "
		    "began; %u IOSurfaces taken as buffers in %llu us each; %u buffers, %llu MB\n",
		    gGPUStat.submissions, nanos / NSEC_PER_MSEC, gGPUStat.commands, gGPUStat.buffers,
		    (gGPUStat.flushed >> 10) / gGPUStat.submissions, flushing / NSEC_PER_USEC / gGPUStat.submissions,
		    timed ? worked / timed : 0, timed, by[0], by[1], by[2], by[3], by[4], by[5], by[6], worked / 1000,
		    gGPUStat.waits, gGPUStat.waits ? waiting / NSEC_PER_USEC / gGPUStat.waits : 0, gGPUStat.over,
		    gGPUStat.surfaces, gGPUStat.surfaces ? taking / NSEC_PER_USEC / gGPUStat.surfaces : 0, fBufferCount,
		    fBufferBytes >> 20);
		if (apollo_gpu_va != 0) {
			/*
			 * The address space itself, because nothing else said anything about it: what
			 * argument buffers and heaps are built on is that this allocator holds, and
			 * "lost" and "refused" are the two ways it stops holding (bsd/dev/
			 * hakios.apollo_adreno.c, apollo_gpu_address_free).
			 */
			uint64_t used, largest, lost;
			uint32_t live, refused;
			unsigned holes, most;

			apollo_gpu_space(&used, &live, &largest, &holes, &most, &lost, &refused);
			IOLog("ApolloGPU: addresses: %llu MB in %u buffers, longest run free %llu MB, %u holes "
			    "(%u at most, of 1024), %llu KB lost, %u allocations refused\n", used >> 20, live,
			    largest >> 20, holes, most, lost >> 10, refused);
		}
		bzero(&gGPUStat, sizeof(gGPUStat));
	}
	IOLockUnlock(gGPULock);
	{
		/*
		 * A submission that took 10 ms or more: where the time went (the first 20, then every 200th), in
		 * microseconds: the copies from the caller, waiting for the clients' lock, the cache flushes of the
		 * buffers, naming and checking the commands, the ring (waiting for room, adding up the core's times,
		 * writing the commands, the doorbell: apollo_gpu_phase), the rest of the submission (the reaping of
		 * buffers, the five-second report), and the whole of it.
		 */
		static uint32_t slow;
		uint64_t end = mach_absolute_time(), total, parts[9];

		absolutetime_to_nanoseconds(end - at_start, &total);
		if (total >= 10 * NSEC_PER_MSEC && (slow++ < 20 || slow % 200 == 0)) {
			uint64_t ticks[5] = { at_copied - at_start, at_locked - at_copied, at_flushed - at_locked,
				at_submitted - at_flushed, at_reaped - at_submitted };

			for (unsigned i = 0; i < 5; i++) {
				absolutetime_to_nanoseconds(ticks[i], &parts[i]);
			}
			for (unsigned i = 0; i < 4; i++) {
				absolutetime_to_nanoseconds(apollo_gpu_phase[i], &parts[5 + i]);
			}
			hakios_tl("tl: slow-submit %u (#%u): %llu us: copyin %llu, lock %llu, flush %llu, submit %llu (ring room %llu, times %llu, "
			    "writes %llu, doorbell %llu), reap %llu, report %llu\n", request->fence, slow, total / 1000, parts[0] / 1000,
			    parts[1] / 1000, parts[2] / 1000, parts[3] / 1000, parts[5] / 1000, parts[6] / 1000, parts[7] / 1000,
			    parts[8] / 1000, parts[4] / 1000, (total - parts[0] - parts[1] - parts[2] - parts[3] - parts[4]) / 1000);
		}
	}
	ret = request->fence != 0 ? kIOReturnSuccess : kIOReturnTimeout;
out:
	if (bos != NULL) {
		IODelete(bos, struct msm_gem_submit_bo, request->nr_bos);
	}
	if (cmds != NULL) {
		IODelete(cmds, struct msm_gem_submit_cmd, request->nr_cmds);
	}
	return ret;
}

/* Pixels of the caller's (32 bits each, blue first) copied to the panel: for a test to show what it has drawn. */
IOReturn
ApolloGPUUserClient::show(const struct apollo_show * request)
{
	uint64_t size = (uint64_t) request->rowBytes * request->height;
	uint8_t * pixels;
	IOReturn ret = kIOReturnSuccess;

	if (request->width == 0 || request->width > 4096 || request->height == 0 || request->height > 4096 ||
	    request->rowBytes < 4 * request->width) {
		return kIOReturnBadArgument;
	}
	pixels = (uint8_t *) IOMallocData(size);
	if (pixels == NULL) {
		return kIOReturnNoMemory;
	}
	if (copyin((user_addr_t) request->pixels, pixels, size) != 0) {
		ret = kIOReturnVMError;
	} else {
		MiniIOMobileFramebufferShow(pixels, request->width, request->height, request->rowBytes);
	}
	IOFreeData(pixels, size);
	return ret;
}

IOReturn
ApolloGPUUserClient::clientMemoryForType(UInt32 type, IOOptionBits * options, IOMemoryDescriptor ** memory)
{
	struct gpu_buffer * b;
	IOReturn ret = kIOReturnBadArgument;

	IOLockLock(gGPULock);
	b = buffer(type);
	if (b != NULL) {
		b->memory->retain();
		b->seen = true;
		*memory = b->memory;
		*options = 0;
		ret = kIOReturnSuccess;
	}
	IOLockUnlock(gGPULock);
	return ret;
}

IOReturn
ApolloGPUUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments * args,
    IOExternalMethodDispatch * dispatch, OSObject * target, void * reference)
{
	union {
		struct msm_param param;
		struct msm_gem_new gemNew;
		struct msm_gem_info gemInfo;
		struct msm_gem_cpu_prep cpuPrep;
		struct msm_gem_submit submit;
		struct msm_wait_fence waitFence;
		struct msm_gem_madvise madvise;
		struct msm_submitqueue queue;
		struct msm_submitqueue_query query;
		struct drm_gem_close close;
		struct apollo_surface surface;
		struct apollo_show show;
		struct apollo_worked worked;
		struct apollo_scanout scanout;
	} r;
	size_t size = args->structureInputSize;
	struct gpu_buffer * b;
	IOReturn ret = kIOReturnSuccess;
	uint64_t zero = 0, patience, waitBegan;

	if (args->structureInput == NULL || size > sizeof(r) || fBuffers == NULL) {
		return kIOReturnBadArgument;
	}
	if (!apollo_gpu_ready()) {
		return kIOReturnNotReady;                       /* given back: the run is ending (kApolloShow too: the panel is not ours then) */
	}
	bzero(&r, sizeof(r));
	memcpy(&r, args->structureInput, size);

	switch (selector) {
	case kMsmGetParam:
		ret = apollo_gpu_parameter(r.param.param, &r.param.value) ? kIOReturnSuccess : kIOReturnBadArgument;
		break;
	case kMsmSetParam:                              /* the program's name, for Linux's lists of who uses the GPU */
	case kMsmGemCpuFini:
	case kMsmSubmitQueueClose:
		break;
	case kMsmGemNew:
		ret = gemNew(&r.gemNew);
		break;
	case kMsmGemInfo:
		ret = gemInfo(&r.gemInfo);
		break;
	case kMsmGemCpuPrep:                            /* is the GPU done with a buffer: with all it has been given, here */
		waitBegan = mach_absolute_time();
		IOLockLock(gGPULock);
		patience = (r.cpuPrep.op & MSM_PREP_NOSYNC) ? 0 : millisecondsUntil(&r.cpuPrep.timeout);
		if (!gpuWait(apollo_gpu_last(), patience)) {
			ret = kIOReturnBusy;
			if (patience == 0 && !(r.cpuPrep.op & MSM_PREP_NOSYNC)) {
				waitOver(&r.cpuPrep.timeout, apollo_gpu_last());
			}
		} else if ((b = buffer(r.cpuPrep.handle)) != NULL && b->scanout == 0 && (b->surface != NULL || gGPUCached)) {
			/*
			 * The GPU is done, and the program is about to look: memory that this processor caches (an
			 * IOSurface's; a buffer of the program's own with gpu_cached=1) may have lines in the cache
			 * from before the GPU wrote it -- read ahead while the GPU was at work -- which would be seen
			 * instead of what it wrote. Out of the cache. (Write-combined buffers, the usual, and the
			 * panel's frames are not cached: nothing to do.)
			 */
			apollo_gpu_to_memory(b->kernel, b->size);
		}
		IOLockUnlock(gGPULock);
		slowWait("cpu-prep", waitBegan);
		break;
	case kMsmGemSubmit:
		ret = submit(&r.submit);
		break;
	case kMsmWaitFence:
		waitBegan = mach_absolute_time();
		IOLockLock(gGPULock);
		patience = millisecondsUntil(&r.waitFence.timeout);
		if (!gpuWait(r.waitFence.fence, patience)) {
			ret = kIOReturnTimeout;
			if (patience == 0) {
				waitOver(&r.waitFence.timeout, r.waitFence.fence);
			}
		}
		IOLockUnlock(gGPULock);
		slowWait("fence", waitBegan);
		break;
	case kMsmGemMadvise:
		r.madvise.retained = 1;
		break;
	case kMsmSubmitQueueNew:
		r.queue.id = ++fNextQueue;
		break;
	case kMsmSubmitQueueQuery:                      /* how often the GPU has faulted on this client's commands */
		if (r.query.data != 0 && r.query.len >= sizeof(zero)) {
			copyout(&zero, (user_addr_t) r.query.data, sizeof(zero));
		}
		break;
	case kDrmGemClose:
		IOLockLock(gGPULock);
		b = buffer(r.close.handle);
		if (b == NULL) {
			ret = kIOReturnBadArgument;
		} else {
			freeBuffer(b);
			reap();
		}
		IOLockUnlock(gGPULock);
		break;
	case kApolloGPUWorked:
		/*
		 * Under gGPULock because apollo_gpu_worked() calls gpu_times(), which reads the
		 * scratch page and advances the same counters the submit path advances -- and
		 * submit() holds this lock while it does exactly that. The output goes back
		 * through the common copy at the end of this method, so a caller that passes a
		 * struct of this size in gets it filled on the way out; one that passes fewer
		 * bytes gets as many as it asked for, and the rest of the numbers are lost with
		 * the reset, which is the one way to misuse this request.
		 */
		IOLockLock(gGPULock);
		apollo_gpu_worked(&r.worked.microseconds, &r.worked.submissions, r.worked.by);
		IOLockUnlock(gGPULock);
		break;
	case kApolloSurface:
		ret = surfaceBuffer(&r.surface);
		break;
	case kApolloShow:
		ret = show(&r.show);
		break;
	case kApolloScanout:
		ret = scanout(&r.scanout);
		break;
	default:
		IOLog("ApolloGPU: request %#x (%zu bytes) is not known here\n", selector, size);
		ret = kIOReturnUnsupported;
		break;
	}
	if (ret == kIOReturnSuccess && args->structureOutput != NULL) {
		if (args->structureOutputSize > size) {
			args->structureOutputSize = (uint32_t) size;
		}
		memcpy(args->structureOutput, &r, args->structureOutputSize);
	}
	return ret;
}
