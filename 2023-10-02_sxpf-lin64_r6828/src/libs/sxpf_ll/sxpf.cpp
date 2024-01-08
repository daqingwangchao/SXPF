/**
 * @file    sxpf.cpp
 *
 * SX proFRAME frame grabber user space library implementation.
 *
 * The basic usage scenario of this low-level part of the library looks like
 * this: @image html grabber-API.png
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "sxpf.h"
#include "sxpf_regs.h"
#include "driver_acc.h"

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <dirent.h>

#ifdef WIN32
#include <windows.h>
#endif

#define CHECK_PARAM(cond)       if (!(cond)) return -1;
#define CHECK_PARAM_VOID(cond)  if (!(cond)) return;


typedef enum sxpf_i2c_flag_e
{
    I2C_NONE = 0,
    I2C_START_ONLY = 1,
    I2C_STOP_ONLY = 2,
    I2C_START_STOP = 3,

} sxpf_i2c_flag_t;


static int def_sxpf_i2c_result_filter(int result, sxpf_hdl pf, int port,
                                  unsigned char dev_id,
                                  const __u8 *wbuf, unsigned int wbytes,
                                  __u8 *rbuf, unsigned int rbytes);


static sxpf_i2c_result_filter_t g_i2c_result_filter = def_sxpf_i2c_result_filter;


/** Open a grabber instance.
 *
 * @param grabber_id    The 0-based ID/number of the grabber device to open
 *
 * @return  A grabber handle, if sucecssful; \c NULL on error.
 */
sxpf_hdl sxpf_open(int grabber_id)
{
    sxpf_hdl pf = (sxpf_hdl)malloc(sizeof(struct sxpf_instance_s));

    if (pf)
    {
        char    devname[PATH_MAX];

        memset(pf, 0, sizeof(*pf));

        sprintf(devname, SXPF_DEVICE_NAME, grabber_id);

        pf->dev = DEV_OPEN(devname, O_RDWR);

        if (!DEV_VALID(pf->dev))
        {
            free(pf);
            return NULL;
        }

        uint32_t    drv_version = 0;
        uint32_t    lib_version = 0;
        uint32_t const  major_minor_mask = (0xff << 24) | (0xff << 16);

        sxpf_get_sw_versions(&lib_version, &drv_version);

        if ((drv_version & major_minor_mask) != (lib_version & major_minor_mask))
        {
            fprintf(stderr,
                    "Driver version %d.%d.%d doesn't match client (%d.%d.%d)\n",
                    (drv_version >> 24) & 0xff, (drv_version >> 16) & 0xff,
                    drv_version & 0xffff, (lib_version >> 24) & 0xff,
                    (lib_version >> 16) & 0xff, lib_version & 0xffff);

            sxpf_close(pf);
            return NULL;
        }

        /* sxpf_get_device_fd() will return this handle */
        pf->dev_user = DEV_DUP(pf->dev);

        /* get buffer size */
        if (DEV_IOCTL(pf->dev, IOCTL_SXPF_GET_CARD_PROPS, &pf->props))
        {
            sxpf_close(pf);
            return NULL;
        }

        pf->num_dma_buffers = pf->props.num_buffers + pf->props.num_i2c_buffers;

        uint32_t num_drv_img_bufs =
            pf->props.num_buffers - pf->props.num_user_buffers;

        for (unsigned i = 0; i < pf->num_dma_buffers; i++)
        {
            sxpf_frame_data_t  *fr = pf->frames + i;

            if (i < num_drv_img_bufs ||     // image buffer allocated by driver
                i >= pf->props.num_buffers) // I2C buffer (always from driver)
            {
                fr->type = SXPF_ALLOC_DRIVER;
            }
            else
            {
                fr->type = SXPF_BUF_UNUSED; // not initialized by user, yet
            }
        }
    }

    return pf;
}


/** Set the grabber's global (or channel 0) configuration.
 *
 * @note    This should be done in idle mode (before sxpf_start, or after
 *          sxpf_stop have been called).
 *
 * @param pf        The grabber handle.
 * @param config    The configuration structure to write to the hardware
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_write_config(sxpf_hdl pf, sxpf_config_t *config)
{
    return sxpf_write_channel_config(pf, 0, config);
}


/** Read the grabber's global (or channel 0) configuration.
 *
 * @note    This should be done in idle mode (before sxpf_start, or after
 *          sxpf_stop have been called).
 *
 * @note    If manual exposure is requested, calling this function also causes
 *          a single trigger pulse to be generated.
 *
 * @param pf        The grabber handle.
 * @param config    The configuration structure to fill with current values
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_read_config(sxpf_hdl pf, sxpf_config_t *config)
{
    return sxpf_read_channel_config(pf, 0, config);
}


/** Set the grabber's configuration for an individual channel.
 *
 * @note    This should be done in idle mode (before sxpf_start, or after
 *          sxpf_stop have been called).
 *
 * If the addressed hardware doesn't have the individual trigger config
 * feature, this function returnes en error when trig_chan != 0.
 *
 * @param pf        The grabber handle.
 * @param trig_chan The trigger channel to operate on (0..3).
 * @param config    The configuration structure to write to the hardware
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_write_channel_config(sxpf_hdl pf, __u32 trig_chan,
                              sxpf_config_t *config)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    config->channel = trig_chan;

    __u32   trig_mode = config->trig_mode;

    if (trig_chan > 0)
    {
        config->trig_mode =
            trig_mode & ~(SXPF_TRIG_MODULE_ENABLE | SXPF_TRIG_ENABLE);

        if (trig_mode & SXPF_TRIG_ENABLE)
            config->trig_mode |= SXPF_TRIG_MODULE_ENABLE;
    }

    int res = DEV_IOCTL(pf->dev, IOCTL_SXPF_WRITE_CONFIG, config);

    config->trig_mode = trig_mode;   // restore user setting

    return res;
}


/** Read the grabber's configuration for an individual channel.
 *
 * @note    This should be done in idle mode (before sxpf_start, or after
 *          sxpf_stop have been called).
 *
 * @note    If manual exposure is requested, calling this function also causes
 *          a single trigger pulse to be generated.
 *
 * If the addressed hardware doesn't have the individual trigger config
 * feature, this function returnes en error when trig_chan != 0.
 *
 * @param pf        The grabber handle.
 * @param trig_chan The trigger channel to operate on (0..3).
 * @param config    The configuration structure to fill with current values
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_read_channel_config(sxpf_hdl pf, __u32 trig_chan,
                             sxpf_config_t *config)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    config->channel = trig_chan;

    int res = DEV_IOCTL(pf->dev, IOCTL_SXPF_READ_CONFIG, config);

    if (res == 0 && trig_chan > 0)
    {
        __u32   trig_mode = config->trig_mode;

        config->trig_mode =
            trig_mode & ~(SXPF_TRIG_MODULE_ENABLE | SXPF_TRIG_ENABLE);

        if (trig_mode & SXPF_TRIG_MODULE_ENABLE)
            config->trig_mode |= SXPF_TRIG_ENABLE;
    }

    return res;
}


/** Fill a user-provided structure with information about the queried card.
 *
 * @param pf    The grabber handle.
 * @param props The property structure to fill with card capability information
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_get_card_properties(sxpf_hdl pf, sxpf_card_props_t *props)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    *props = pf->props;

    return 0;
}


/** Retrieve the current hardware timestamp.
 *
 * @param pf        The grabber handle.
 * @param timestamp Pointer to variable that is to receive the timestamp value
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_get_timestamp(sxpf_hdl pf, __s64 *timestamp)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_GET_TIMESTAMP, timestamp);
}


/** Retrieve the current secondary hardware timestamp (e.g., GPS).
 *
 * @param pf        The grabber handle.
 * @param timestamp Pointer to variable that is to receive the secondary
 *                  timestamp value
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_get_secondary_timestamp(sxpf_hdl pf, __s64 *timestamp)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_GET_TIMESTAMP_SEC, timestamp);
}


/** Retrieve the current hardware timestamp including system time
 *
 * @param pf                The grabber handle.
 * @param timestamp_sync    Pointer to struct with time synchronization info
 *
 * The Caller must enter a clock source in the argument's \c clockSelect member
 * to specify which system time source to compare the card's hardware time to.
 *
 * After The function returns, the members \c timestamp and \c systemTime
 * contain time values for the same point in time for the card's hardware time
 * and the selected clock's time, respectively.
 * Additionally, the member \c slack gives an indication of the accuracy of the
 * synchronization, by telling how long (in the domain of the selected clock) it
 * took to retrieve the hardware timestamp. If \c slack is 0, this is due to an
 * imprecise/coarse clock that hasn't ticked while reading the hardware time.
 *
 * To aid in querying and interpreting system time values, see the functions
 * \ref sxpf_get_system_clock_rate() and \ref sxpf_get_system_time(). Together
 * with the clock source \c SXPF_CLOCK_DEFAULT the allow platform-independent
 * high-resolution measurement of time differences.
 *
 * @note    Querying the hardware timestamp and system time themselfes are
 *          subject to certain latencies, which depend on the system (PCIe bus,
 *          CPU,...) and can be in the order of a few micro seconds.
 * @note    The selector values for the POSIX clocks equal their respective
 *          \c CLOCK_REALTIME, \c CLOCK_MONOTONIC & \c CLOCK_MONOTONIC_RAW
 *          values, so they can be used interchangably. The \c systemTime value
 *          returned for these clocks is given in nanoseconds:
 *          tv_sec * 1000000000LL + tv_nsec.
 * @note    When running on Windows 7 or earlier, the \c systemTime returned for
 *          \c SXPF_CLOCK_FILETIME is very inaccurate. It should only be used if
 *          running on Windows 10 or newer. The \c systemTime value is a 64 bit
 *          representation of a \c FILETIME structure. Its resolution is 100ns.
 * @note    To get the resolution of the Windows clock SXPF_CLOCK_QPC, the user
 *          must call \c QueryPerformanceFrequency().
 *
 * @return  0 on success; negative value on error (e.g., if a clock source was
 *          selected that isn't available in the running operating system).
 */
int sxpf_timestamp_sync(sxpf_hdl pf, sxpf_timestamp_sync_t *timestamp_sync)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_TIMESTAMP_SYNC, timestamp_sync);
}


/** Get the current system time from the specified clock source.
 *
 * @param clock_source  The clock to query. @see sxpf_clock_select_t
 *
 * @returns The current system time according to the given clock, as it would
 *          be returned in the sxpf_timestamp_sync_t::systemTime member by a
 *          call to \ref sxpf_timestamp_sync().
 * @return  -1 on error.
 */
__s64 sxpf_get_system_time(sxpf_clock_select_t clock_source)
{
    __s64           result = -1;
#ifdef _WIN32
    typedef VOID(WINAPI* PFNGETSYSTEMTIME)(LPFILETIME);

    static PFNGETSYSTEMTIME get_system_time_func = NULL;

    FILETIME        ft;
    LARGE_INTEGER   now;
#else
    struct timespec ts = { 0, 0 };
#endif

    switch (clock_source)
    {
#ifdef _WIN32
    case SXPF_CLOCK_REALTIME:
    case SXPF_CLOCK_MONOTONIC:
    case SXPF_CLOCK_MONOTONIC_RAW:
#else
    case SXPF_CLOCK_QPC:
    case SXPF_CLOCK_FILETIME:
#endif
    default:
        break;

#ifdef _WIN32
    case SXPF_CLOCK_QPC:
        if (QueryPerformanceCounter(&now))
            result = now.QuadPart;
        break;

    case SXPF_CLOCK_FILETIME:
        if (!get_system_time_func)
        {
            static HMODULE hkernel32 = LoadLibrary(TEXT("kernel32.dll"));

            if (hkernel32)
            {
                // if GetSystemTimePreciseAsFileTime is available use that for
                // higher system time precision
                FARPROC func = GetProcAddress(hkernel32,
                                              "GetSystemTimePreciseAsFileTime");
                if (func)
                {
                    get_system_time_func = (PFNGETSYSTEMTIME)func;

                    atexit([]()
                    {
                        if (hkernel32)
                            FreeLibrary(hkernel32);
                        hkernel32 = NULL;
                    });
                }
            }
            if (get_system_time_func == NULL)
            {
                // imprecise default
                get_system_time_func = GetSystemTimeAsFileTime;

                fprintf(stderr,
                        "WARNING: FILETIME-based timestamps are imprecise!\n");
            }
        }

        get_system_time_func(&ft);

        now.HighPart = ft.dwHighDateTime;
        now.LowPart = ft.dwLowDateTime;

        result = now.QuadPart;
        break;
#else
    case SXPF_CLOCK_REALTIME:
    case SXPF_CLOCK_MONOTONIC:
    case SXPF_CLOCK_MONOTONIC_RAW:
        if (clock_gettime(clock_source, &ts))
            break;

        result = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        break;
#endif
    }

    return result;
}


/** Retrieve the selected system clock's tick rate.
 *
 * @param clock_source  The clock to query. @see sxpf_clock_select_t
 *
 * @return  The number of ticks per second for the selected clock.
 * @return  -1 on error (wrong clock source for the current operating system)
 */
__s64 sxpf_get_system_clock_rate(sxpf_clock_select_t clock_source)
{
    __s64           result = -1;
#ifdef _WIN32
    LARGE_INTEGER   rate;
#endif

    switch (clock_source)
    {
#ifdef _WIN32
    case SXPF_CLOCK_REALTIME:
    case SXPF_CLOCK_MONOTONIC:
    case SXPF_CLOCK_MONOTONIC_RAW:
#else
    case SXPF_CLOCK_QPC:
    case SXPF_CLOCK_FILETIME:
#endif
    default:
        break;

#ifdef _WIN32
    case SXPF_CLOCK_QPC:
        if (QueryPerformanceFrequency(&rate))
            result = rate.QuadPart;
        break;

    case SXPF_CLOCK_FILETIME:
        result = 10000000;      // 100ns per counter step
        break;
#else
    case SXPF_CLOCK_REALTIME:
    case SXPF_CLOCK_MONOTONIC:
    case SXPF_CLOCK_MONOTONIC_RAW:
        result = 1000000000;    // 1ns per counter step
        break;
#endif
    }

    return result;
}


/** Start receiving data.
 *
 * @param pf            The grabber handle.
 * @param channel_mask  The subchannels to enable. A bit mask combining
 *                      values from enum sxpf_channel_t.
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_start_record(sxpf_hdl pf, __u32 channel_mask)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_START_RECORD, channel_mask);
}


/** Start sending data.
 *
 * @param pf            The grabber handle.
 * @param channel_mask  The subchannels to enable. A bit mask combining
 *                      values from enum sxpf_channel_t.
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_start_playback(sxpf_hdl pf, __u32 channel_mask)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_START_PLAYBACK, channel_mask);
}


/** Execute a low-level command on the SX proFRAME's embedded controller.
 *
 * @param pf    The grabber handle.
 * @param req   Pointer to the command request & result data.
 *
 * @return  0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_cmd_sync(sxpf_hdl pf, sxpf_cmd_fifo_el_t *req)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, req);
}


/** Allocate a driver buffer for exclusive use by the calling process during
 *  playback.
 *
 * The buffer is sent to the hardware for playback when sxpf_release_frame() is
 * called on it with data_size set unequal to 0 in the passed sxpf_frame_info_t
 * structure.
 *
 * Exclusive use must be relinquished by calling sxpf_release_frame() with
 * data_size set to 0 in the passed sxpf_frame_info_t structure.
 *
 * @param pf            The grabber handle.
 * @param slot          Pointer to the variable where to store the allocated
 *                      slot number.
 * @param timeout_us    Number of microseconds to wait befor giving up
 *
 * @return  0 on success; negative value on error. errno is set to ETIME if
 *          the timeout expired without a buffer becoming available.
 */
int sxpf_alloc_playback_frame(sxpf_hdl pf, sxpf_buf_t *slot, __u32 timeout_us)
{
    int res;

    CHECK_PARAM(pf && DEV_VALID(pf->dev) && slot != NULL);

    // timeout is param to IOCTL, slot number is returned back
    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_ALLOC_PLAY_FRAME, &timeout_us);

    if (!res)
        *slot = timeout_us;

    return res;
}


/** Release a driver buffer back to the driver after exclusive use on playback
 *  cards is ended.
 *
 * The frame slot to be released must not currently by owned by the hardware
 * (DMA in progress).
 *
 * @param pf    The grabber handle.
 * @param slot  The buffer slot number returned by \c sxpf_alloc_playback_frame
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_free_playback_frame(sxpf_hdl pf, sxpf_buf_t slot)
{
    // data size 0 on playback cards signals that exlusive access is given up
    return sxpf_release_frame_ex(pf, slot, 0, SXPF_FREE_BUFFER);
}


/** Tell the driver to use a heap memory block as a video DMA buffer.
 *
 * The memory passed in by \c header and \c payload is permanently mapped for
 * DMA by the driver. Before this memory can be freed by the user, the returned
 * buffer slot must be unmapped from the grabber's DMA engine by calling
 * \c sxpf_free_user_buffer.
 *
 * @param pf        The grabber handle.
 * @param header    Pointer to the user-allocated header buffer for video DMA.
 * @param payload   Pointer to the user-allocated payload buffer for video DMA.
 *                  This parameter may be NULL; in this case the payload is
 *                  part of the memory block defined by \c header and follows
 *                  directly after the 64 bytes herader data.
 *                  If used, this address must be aligned to 64 bytes.
 * @param size      Payload size of the buffer (must be at least as big as the
 *                  grabber's configured frame_size).
 *
 * @return  Buffer slot index (sxpf_buf_t as an int) on success
 * @return  Negative value on error.
 */
int sxpf_declare_heap_buffer(sxpf_hdl pf, void *header, void *payload,
                             size_t size)
{
    return sxpf_declare_user_buffer(pf, SXPF_ALLOC_HEAP, header, payload,
                                    (__u32)size);
}


/** Tell the driver to use a user-provided memory block as a video DMA buffer.
 *
 * The memory passed in by \c header and \c payload is permanently mapped for
 * DMA by the driver. Before this memory can be freed by the user, the returned
 * buffer slot must be unmapped from the grabber's DMA engine by calling
 * \c sxpf_free_user_buffer.
 *
 * @param pf        The grabber handle.
 * @param type      Type of buffer allocation
 * @param header    Pointer to the user-allocated header buffer for video DMA.
 * @param payload   Pointer to the user-allocated payload buffer for video DMA
 *                  This parameter may be NULL; in this case the payload is
 *                  part of the memory block defined by \c header and follows
 *                  directly after the 64 bytes herader data.
 *                  If used, this address must be aligned to 64 bytes.
 * @param size      Payload size of the buffer (must be at least as big as the
 *                  grabber's configured frame_size).
 *
 * @return  Buffer slot index (sxpf_buf_t as an int) on success
 * @return  Negative value on error.
 */
int sxpf_declare_user_buffer(sxpf_hdl pf, sxpf_buf_alloc_type_t type,
                             void *header, void *payload, uint32_t size)
{
    sxpf_user_buffer_declare_t  req;

    req.slot = ~0;
    req.type = type;
    req.header = (__u64)header;
    req.payload = (__u64)payload;
    req.size = size;

    CHECK_PARAM(pf && DEV_VALID(pf->dev));
    CHECK_PARAM(req.payload % 64 == 0);

    int res = DEV_IOCTL(pf->dev, IOCTL_SXPF_INIT_USER_BUFFER, &req);

    if (res >= 0)
    {
        sxpf_frame_data_t  *fr = pf->frames + req.slot;

        fr->header = header;
        fr->payload = payload;
        fr->size = size;
        fr->type = type;
    }

    return req.slot;
}


/** Unmap a user-allocated buffer from to the driver after exclusive use has
 *  ended.
 *
 * The frame slot to be released must not currently by owned by the hardware
 * (DMA in progress).
 *
 * @param pf    The grabber handle.
 * @param slot  The buffer slot number returned by sxpf_declare_user_buffer
 *
 * @return  0 on success; negative value on error, errno will be EAGAIN if the
 *          passed buffer is still accessed by the grabber's DMA engine.
 */
int sxpf_free_user_buffer(sxpf_hdl pf, sxpf_buf_t slot)
{
    return sxpf_release_frame_ex(pf, slot, 0, SXPF_FREE_BUFFER);
}


/** Stop streaming.
 *
 * @param pf            The grabber handle.
 * @param channel_mask  The subchannels to disable. A bit mask combining
 *                      values from enum sxpf_channel_t.
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_stop(sxpf_hdl pf, __u32 channel_mask)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_STOP, channel_mask);
}


/** Set exposure time (i.e., the trigger pulse width) for global trigger chan 0.
 *
 * In manual trigger mode (see sxpf_trigger_mode_t), calling this function
 * triggers a single exposure.
 *
 * @param pf        The grabber handle.
 * @param exposure  The desired exposure time in units of 1e-8 seconds.
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_trigger_exposure(sxpf_hdl pf, __u32 exposure)
{
    return sxpf_trigger_channel_exposure(pf, 0, exposure);
}


/** Set exposure time (i.e., the trigger pulse width) for the selected trigger
 *  channel.
 *
 * In manual trigger mode (see sxpf_trigger_mode_t), calling this function
 * triggers a single exposure.
 *
 * If the addressed hardware doesn't have the individual trigger config
 * feature, this function returnes en error when trig_chan != 0.
 *
 * @param pf        The grabber handle.
 * @param trig_chan The trigger channel to operate on (0..3).
 * @param exposure  The desired exposure time in units of 1e-8 seconds.
 *
 * @return  0 on success; negative value on error.
 */
int sxpf_trigger_channel_exposure(sxpf_hdl pf, __u32 trig_chan, __u32 exposure)
{
    sxpf_trigger_exposure_t trig;

    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    trig.channel = trig_chan;
    trig.exposure = exposure;

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_TRIG_EXPOSURE, &trig);
}


/** Close a grabber instance.
 *
 * @param pf    The grabber handle.
 */
void sxpf_close(sxpf_hdl pf)
{
    CHECK_PARAM_VOID(pf && DEV_VALID(pf->dev));

    if (pf)
    {
        __u32   i;

        /* unmap DMA buffers... */
        for (i = 0; i < pf->num_dma_buffers; ++i)
        {
            sxpf_frame_data_t   *fr = &pf->frames[i];

            switch (fr->type)
            {
            default:
                fprintf(stderr, "internal error: buffer %d "
                        "has invalid alloc type (%d)\n", i, fr->type);
                break;

            case SXPF_BUF_UNUSED:
                break;

            case SXPF_ALLOC_DRIVER:
                /* but only the ones that had been mmapped before */
                if (fr->header)
                {
                    DEV_MUNMAP(fr->header, fr->size);
                }
                break;

            case SXPF_ALLOC_HEAP:
            case SXPF_ALLOC_NV_DVP:
            case SXPF_ALLOC_NV_P2P:
                /* or the ones that were provided by the user */
                // TODO do this in a loop until successful - we don't want DMA
                //      to be still writing to this memory
                sxpf_free_user_buffer(pf, i);
                break;
            }
        }

        DEV_CLOSE(pf->dev);     /* equals pipe's read end in simulation mode */
        DEV_CLOSE_WAIT_DEV(pf->dev_user);

        free(pf);
    }
}


/** Wait for events from some frame grabber(s).
 *
 * This function is just a convenience wrapper around select() or
 * WaitForMultipleObjects().
 *
 * @param nfds          Number of device handles to wait on.
 * @param fds           Pointer to array of device handles to wait on.
 *                      If a positive value is returned, this number of ready
 *                      handles will be moved to the beginning of this array.
 * @param timeout_ms    Wait timeout in milli seconds. May be -1 for an
 *                      infinite wait time (not recommended).
 *
 * @return -1 on error
 * @return 0 if no device became ready to be read from during the timeout period
 * @return A positive value as the number of ready devices. (On Windows this can
 *         only be 1.)
 *
 * @note The wait handles passed in the \c fds array may also be generic handles
 *       of files, sockets, etc. that can be checked for readable data.
 */
int sxpf_wait_events(int nfds, HWAITSXPF *fds, int timeout_ms)
{
    int len = 0;

    if (nfds < 1 || fds == NULL)
        return -1;

#ifdef WIN32
    if (nfds > MAXIMUM_WAIT_OBJECTS)
        return -1;

    DWORD   timeout = timeout_ms < 0 ? INFINITE : (DWORD)timeout_ms;
    auto    res = WaitForMultipleObjects(nfds, fds, false, timeout);

    if (res >= WAIT_OBJECT_0 && res < (WAIT_OBJECT_0 + nfds))
    {
        // object was signaled: event(s) available
        len = 1;
        fds[0] = fds[res - WAIT_OBJECT_0];
    }
    else if (res == WAIT_TIMEOUT)
    {
        len = 0;
    }
    else
    {
        // case WAIT_FAILED:
        // case WAIT_ABANDONED:
        // default:
        len = -1;   // error waiting for events
    }
#else
    int             maxfd = -1;
    fd_set          fdin;
    struct timeval  timeout =
    {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };

    FD_ZERO(&fdin);

    for (int i = 0; i < nfds; i++)
    {
        FD_SET(fds[i], &fdin);
        if (fds[i] > maxfd)
            maxfd = fds[i];
    }

    len = select(maxfd + 1, &fdin, NULL, NULL,
                 timeout_ms < 0 ? NULL : &timeout);
    if (len > 0)
    {
        for (int in = 0, out = 0; in < len; in++)
        {
            if (FD_ISSET(in, &fdin))
                fds[out++] = fds[in];
        }
    }
#endif

    return len;
}


/** Receive events from the frame grabber.
 *
 * This function is just a convenience wrapper around read().
 *
 * @param pf            The grabber handle.
 * @param evt_buf       User event buffer for storing received events
 * @param max_events    The number of events that fit into the buffer
 *
 * @return The number of events delivered, if >=0.
 * @return A negative errno value on error
 */
int sxpf_read_event(sxpf_hdl pf, sxpf_event_t *evt_buf, int max_events)
{
    int ret = DEV_READ(pf->dev, evt_buf, max_events * sizeof(sxpf_event_t));

    if (ret > 0)
        ret /= sizeof(sxpf_event_t);

    return ret;
}


/** Retrieve the address of the data buffer assigned to a slot.
 *
 * This function can only be used if the actual payload data is located at the
 * first memory address directly following the header itself. This is the case
 * if the buffer was allocated by the grabber driver. If the buffer was provided
 * by @ref sxpf_declare_user_buffer, this function will fail if the payload was
 * not placed directly after the header.
 *
 * @param pf    The grabber handle.
 * @param slot  The slot number to query (0..SXPF_MAX_DMA_BUFFERS-1).
 *
 * @return  The address on success; \c NULL on error.
 */
void *sxpf_get_frame_ptr(sxpf_hdl pf, sxpf_buf_t slot)
{
    sxpf_image_header_t *addr = NULL;

    if (pf && DEV_VALID(pf->dev) && slot < pf->num_dma_buffers)
    {
        sxpf_frame_data_t  *fr = pf->frames + slot;

        addr = (sxpf_image_header_t*)fr->header;

        if (addr)
        {
            // fail, if payload doesn't follow directly after header
            if (fr->payload != NULL && fr->payload != addr + 1)
                return NULL;
        }
        else if (fr->type == SXPF_ALLOC_DRIVER)
        {
            /* map stream buffer into this process's memory space */
            void               *bufptr;
            size_t              length;
            off_t               offset;

            if (slot < pf->props.num_buffers)
            {
                /* video */
                length = pf->props.buffer_size;
                offset = slot * pf->props.buffer_size;
            }
            else
            {
                /* I2C: skip video buffers */
                length = pf->props.i2c_buffer_size;
                offset = pf->props.num_buffers * pf->props.buffer_size +
                    (slot - pf->props.num_buffers) * pf->props.i2c_buffer_size;
            }

            bufptr = DEV_MMAP(0, length, PROT_READ | PROT_WRITE, MAP_SHARED,
                              pf->dev, offset);
            
            if (bufptr == MAP_FAILED)
                return NULL;    /* no more buffers available for this device */

            /* cache pointer for later */
            fr->header = bufptr;
            fr->payload = NULL;
            fr->size = length;

            addr = (sxpf_image_header_t*)bufptr;
        }
    }

    return addr;
}


/** Retrieve the address of the data buffer assigned to an image buffer slot.
 *
 * @note It is not necessarily an error if this function stores a NULL pointer
 * at the address given by \c paddr_payload. This may be the case if the buffer
 * is only backed by memory in a GPU.
 *
 * @param pf            The grabber handle.
 * @param slot          The slot number to query.
 * @param paddr_payload [OUT] Address of a pointer wher to store the address
 *                      of the buffer's payload part.
 *
 * @return  The address of the buffer's header part on success; \c NULL on error.
 */
sxpf_image_header_t *sxpf_get_image_ptr(sxpf_hdl pf, sxpf_buf_t slot,
                                        void **paddr_payload)
{
    sxpf_image_header_t *header = NULL;
    void                *payload = NULL;

    if (!paddr_payload)
        return NULL;

    if (pf && DEV_VALID(pf->dev) && slot < pf->props.num_buffers)
    {
        sxpf_frame_data_t  *fr = pf->frames + slot;

        if (fr->type == SXPF_ALLOC_DRIVER)
        {
            header = (sxpf_image_header_t*)sxpf_get_frame_ptr(pf, slot);

            if (header)
                payload = header + 1;
        }
        else if (fr->type != SXPF_BUF_UNUSED)
        {
            // these pointers must have been provided by the current user
            header = (sxpf_image_header_t*)fr->header;
            payload = fr->payload;
        }
    }

    *paddr_payload = payload;

    return header;
}


/** Release a frame slot after received data has been processed.
 *
 * Record mode:
 *
 * This function makes the buffer available for further receptions. It /em must
 * be called as soon as possible after the event SXPF_EVENT_FRAME_RECEIVED has
 * delivered the slot number of the received frame to the application. If the
 * proFRAME FPGA runs out of buffers, received frames are dropped.
 *
 * Each individual session (represented by different sxpf_hdl instances) that
 * receives frames from a selected channel must release them - it is \em not
 * enough to release a frame only from one of the sessions.
 *
 * Playback mode:
 *
 * Pass the buffer to the hardware for sending. The buffer's ownership passes to
 * the hardware. Only after an event has been received that gives back this
 * buffer may it be filled with new playback data for further transmissions.
 *
 * @param pf   The grabber handle.
 * @param slot The slot number to release (0..SXPF_MAX_DMA_BUFFERS-1).
 * @param size Number of bytes to send in playback mode; ignored when recording.
 *             For playback, the channel number 0..3 to send on must be encoded
 *             in bits 30..31 of the size value!
 *
 * @return  0 on success; non-0 on error (see ioctl()).
 */
int sxpf_release_frame(sxpf_hdl pf, sxpf_buf_t slot, __u32 size)
{
    uint32_t target = SXPF_RELEASE_TO_ALL_CHAN;

    CHECK_PARAM(pf && DEV_VALID(pf->dev) && slot < pf->num_dma_buffers);

    if (pf->props.capabilities & SXPF_CAP_VIDEO_PLAYBACK)
    {
        if (size == 0)
        {
            target = SXPF_FREE_BUFFER;
        }
        else
        {
            uint32_t    size_mask_bits = pf->props.num_channels <= 4 ? 30 : 29;

            target = size >> size_mask_bits;
            size  &= (1u << size_mask_bits) - 1;
        }
    }

    return sxpf_release_frame_ex(pf, slot, size, target);
}


/** Release a frame slot after received data has been processed with extended
 *  options.
 *
 * Record mode:
 *
 * This function makes the buffer available for further receptions. It /em must
 * be called as soon as possible after the event SXPF_EVENT_FRAME_RECEIVED has
 * delivered the slot number of the received frame to the application. If the
 * proFRAME FPGA runs out of buffers, received frames are dropped.
 *
 * Each individual session (represented by different sxpf_hdl instances) that
 * receives frames from a selected channel must release them - it is \em not
 * enough to release a frame only from one of the sessions.
 *
 * If target==SXPF_RELEASE_TO_ALL_CHAN, this function acts like
 * @ref sxpf_release_frame.
 *
 * If target==SXPF_FREE_BUFFER, this function unmaps a buffer from the
 * grabber's DMA engine that was announced before by calling
 * sxpf_declare_user_buffer().
 *
 * If target is >=0 it specifies the video channel on which this buffer shall be
 * enqueued for reception.
 *
 * Playback mode:
 *
 * Pass the buffer to the hardware for sending. The buffer's ownership passes to
 * the hardware. Only after an event has been received that gives back this
 * buffer may it be filled with new playback data for further transmissions.
 *
 * If target==SXPF_FREE_BUFFER, this function unmaps a buffer from the
 * grabber's DMA engine that was announced before by calling @ref
 * sxpf_declare_user_buffer() or @ref sxpf_alloc_playback_frame().
 *
 * The only other values allowed for target are the available channel numbers to
 * specify on which video interface the frame shall be sent.
 *
 * @param pf        The grabber handle.
 * @param slot      The slot number to release (0..SXPF_MAX_DMA_BUFFERS-1).
 * @param size      Number of bytes to send in playback mode; ignored when
 *                  recording.
 * @param target    Specifies where the buffer should go.
 *                  See also @ref sxpf_release_buf_target_t
 *                  - negative value: generic target: @see
 *                    sxpf_release_buf_target_t
 *                      * SXPF_FREE_BUFFER: unmap buffer from DMA engine (only
 *                        for buffers that were set up with
 *                        \ref sxpf_declare_user_buffer() or allocated with
 *                        \ref sxpf_alloc_playback_frame())
 *                      * SXPF_RELEASE_TO_ALL_CHAN: use buffer for reception on
 *                        any video channel
 *                  - value >= 0: specific channel number to post the buffer to
 *
 * @return  0 on success; non-0 on error (see ioctl()).
 */
int sxpf_release_frame_ex(sxpf_hdl pf, sxpf_buf_t slot, __u32 size,
                          int32_t target)
{
    sxpf_frame_info_t    request;

    CHECK_PARAM(pf && DEV_VALID(pf->dev) && slot < pf->num_dma_buffers);

    if (pf->props.capabilities & SXPF_CAP_VIDEO_PLAYBACK)
    {
        uint32_t    size_mask_bits = pf->props.num_channels <= 4 ? 30 : 29;

        if (!(target == SXPF_FREE_BUFFER ||
              (target >= 0 && target < (int)pf->props.num_channels)))
        {
            return -EINVAL;
        }

        // TODO put this logic into the driver
        size |= target << size_mask_bits;   // for older hardware revisions
    }
    else
    {
        if (!(target == SXPF_FREE_BUFFER ||
              target == SXPF_RELEASE_TO_ALL_CHAN ||
              (target >= 0 && target < (int)pf->props.num_channels)))
        {
            return -EINVAL;
        }
    }

    request.slot = slot;        /* provide slot to driver */
    request.data_size = size;   /* playback data size */
    request.target = target;

    int ret = DEV_IOCTL(pf->dev, IOCTL_SXPF_RELEASE_FRAME, &request);
    printf("(extern)Target is %d", request.target);
    return ret;
}


/** Get the contents of a buffer's header area copied directly after it was
 *  received from the FPGA.
 *
 * @param pf        The grabber handle.
 * @param slot      The slot number to release (0..SXPF_MAX_DMA_BUFFERS-1).
 * @param header    Pointer to memory large enough (64 bytes) to receive the
 *                  buffer's header data.
 *
 * @return  0 on success; non-0 on error (see ioctl()).
 */
int sxpf_get_buffer_header(sxpf_hdl pf, sxpf_buf_t slot, void *header)
{
    sxpf_get_buf_hdr_t  request;
    int                 ret;

    CHECK_PARAM(pf && DEV_VALID(pf->dev) && slot < pf->num_dma_buffers);

    request.slot = slot;

    ret = DEV_IOCTL(pf->dev, IOCTL_SXPF_GET_BUF_HDR, &request);

    if (!ret)
        memcpy(header, request.header, 64);

    return ret;
}


/** Retrieve the device file descriptor for use in select or blocking reads, to
 *  wait for a new image to become available.
 *
 * @note
 *  - This function returns a duplicate of the internally used device file
 *    descriptor.
 *  - This descriptor is automatically closed by sxpf_close().
 *  - The returned file descriptor may have the flag O_NONBLOCK set. If the user
 *    needs blocking reads, he should call fcntl(fd, F_SETFL, 0) to reset the
 *    flag. (on Linux)
 *
 * @param pf    The grabber handle.
 * @param fd    Pointer to variable for returning file descriptor
 *
 * @return 0 on success; -1 on error (due to invalid parameter)
 */
int sxpf_get_device_fd(sxpf_hdl pf, HWAITSXPF *fd)
{
    CHECK_PARAM(pf && DEV_VALID(pf->dev));
    CHECK_PARAM(fd);

    *fd = pf->dev_user;

    return 0;
}


/** Low-level function to read a device register value.
 *
 * @param pf        The grabber handle.
 * @param bar       The BAR number to read from (only 0, 2 & 4 allowed).
 * @param reg_off   Register offset. See FPGA documentation for details.
 * @param value     Pointer to register (return) value
 *
 * @return  0 on success; non-0 on error (see ioctl()).
 */
int sxpf_read_register(sxpf_hdl pf, __u32 bar, __u32 reg_off, __u32 *value)
{
    sxpf_rw_register_t   rw;
    int                 res;

    CHECK_PARAM(pf && DEV_VALID(pf->dev) && value != NULL);

    rw.bar = bar;
    rw.offs = reg_off;
    rw.size = 4;        /* 32bit access */

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_READ_REG, &rw);

    if (res == 0)
    {
        *value = rw.data;
    }

    return res;
}


/** Low-level function to write a device register value.
 *
 * @param pf        The grabber handle.
 * @param bar       The BAR number to write to (only 0, 2 & 4 allowed).
 * @param reg_off   Register offset. See FPGA documentation for details.
 * @param value     Register value to write
 *
 * @return  0 on success; non-0 on error (see ioctl()).
 */
int sxpf_write_register(sxpf_hdl pf, __u32 bar, __u32 reg_off, __u32 value)
{
    sxpf_rw_register_t   rw;

    CHECK_PARAM(pf && DEV_VALID(pf->dev));

    rw.bar  = bar;
    rw.offs = reg_off;
    rw.data = value;
    rw.size = 4;        /* 32bit access */

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_WRITE_REG, &rw);
}


/** Send a generic command on the I2C bus of the given video channel.
 *
 * The order of actions performed by this function is as follows:
 * 1. If \c wbuf is not \c NULL and wbytes is not 0, send a start condition
 *    follwoed by a device select command on the I2C bus with the R/W bit
 *    cleared (select device for writing).
 * 2. If \c wbuf is not \c NULL, send \c wbytes number of bytes from \c wbuf on
 *    the I2C bus, as long as the addressed device acknowledges them.
 * 3. If \c rbuf is \c NULL or \c rbytes is 0, goto 6.
 * 4. Send a repeated start condidtion, followed by a device select command with
 *    the R/W bit set (select the device again, this time for reading).
 * 5. Read \c rbytes number of bytes from the I2C device, acknowledging all but
 *    the last of them and store them at the address given by \c rbuf.
 * 6. Send a stop condition on the I2C bus.
 * 7. Return.
 *
 * For register write accesses, \c wbuf must contain the complete I2C
 * transaction consisting of register address byte(s) and data bytes.
 *
 * For register read accesses, \c wbuf contains only the register address while
 * the read data bytes are stored in \c rbuf.
 *
 * @param pf        The grabber handle.
 * @param chan      The video channel to address (0...3).
 * @param dev_id    The addressed device's I2C slave id
 * @param wbuf      The I2C command bytes to write, or \c NULL if only reading.
 * @param wbytes    The number of command bytes in wbuf.
 * @param rbuf      Pointer to the receive buffer, or \c NULL if only writing.
 * @param rbytes    The number of bytes to read from the device.
 *
 * @note    The maximum number of bytes that can be written and/or read in one
 *          go (i.e., the maximum value for wbytes and rbytes) is 251.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
static int sxpf_i2c_xfer_internal(sxpf_hdl pf, int chan, unsigned char dev_id,
                                  const unsigned char *wbuf, unsigned wbytes,
                                  unsigned char *rbuf, unsigned rbytes)
{
    sxpf_cmd_fifo_el_t  req;
    long                res;

    // in addition to the user data read and/or written, we need to put 5
    // command bytes into the argument buffer
    if (wbytes + 5 > SXPF_NUM_ARGS_PER_CMD ||
        rbytes + 5 > SXPF_NUM_ARGS_PER_CMD)
    {
        return 1;
    }

    req.timeout_ms = 1000;
    req.cmd_stat = SXPF_CMD_I2C_TRANSFER | SXPF_CMD_STATUS_REQUEST;
    req.num_args = 5 + wbytes;
    req.num_ret = 5 + rbytes;

    // clear unused argument bytes
    memset(req.args, 0, sizeof(req.args));

    req.args[0] = 0;    // result (overwritten by Plasma)
    req.args[1] = chan;
    req.args[2] = dev_id;
    req.args[3] = wbytes;
    req.args[4] = rbytes;
    if (wbytes > 0)
        memcpy(req.args + 5, wbuf, wbytes);

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    if (res == 0)
    {
        if (rbuf != NULL && rbytes > 0)
        {
            memcpy(rbuf, req.args + 5, rbytes);
        }
    }

    return res;
}


/** Default I2C transfer result filter that passes on the low-level result
 *  unmodified.
 *
 * The user application may install a custom filter that modifies the the result
 * of every I2C transaction.
 *
 * @param result    The result of the low-level I2C transaction as it is
 *                  returned by sxpf_i2c_xfer().
 * @param pf        The grabber handle (context info)
 * @param port      The addressed I2C port (context info)
 * @param dev_id    The addressed iI2C device ID (context info)
 * @param wbuf      The I2C command bytes to write (context info)
 * @param wbytes    The number of command bytes in wbuf (context info)
 * @param rbuf      Pointer to the receive buffer (context info)
 * @param rbytes    The number of bytes to read from the device (context info)
 *
 * @result  The translated result code. This filter returns what was passed in
 *          as \c result.
 */
int def_sxpf_i2c_result_filter(int result, sxpf_hdl pf, int port, __u8 dev_id,
                               const __u8 *wbuf, unsigned int wbytes,
                               __u8 *rbuf, unsigned int rbytes)
{
    (void)pf;
    (void)port;
    (void)dev_id;
    (void)wbuf;
    (void)wbytes;
    (void)rbuf;
    (void)rbytes;

    return result;
}


/** Install a custom I2C transfer result filter.
 *
 * This can be used to let the execution of init sequences continue after
 * low-level I2C errors that are benign and should be ignored. The filter gets
 * passed all arguments that were passed to the sxpf_i2c_xfer() function as
 * context information. This allows the filter to only modify specific I2C
 * transactions and let all other results pass unmodified.
 *
 * @param filter User I2C result filter function; if \c NULL, the default filter
 *               is (re)installed, which passes on the low-level result code
 *               unmodified.
 */
void sxpf_set_i2c_result_filter(sxpf_i2c_result_filter_t filter)
{
    g_i2c_result_filter = filter ? filter : def_sxpf_i2c_result_filter;
}


/** Send a generic command on the I2C bus of the given video channel.
 *
 * The order of actions performed by this function is as follows:
 * 1. If \c wbuf is not \c NULL and wbytes is not 0, send a start condition
 *    follwoed by a device select command on the I2C bus with the R/W bit
 *    cleared (select device for writing).
 * 2. If \c wbuf is not \c NULL, send \c wbytes number of bytes from \c wbuf on
 *    the I2C bus, as long as the addressed device acknowledges them.
 * 3. If \c rbuf is \c NULL or \c rbytes is 0, goto 6.
 * 4. Send a repeated start condidtion, followed by a device select command with
 *    the R/W bit set (select the device again, this time for reading).
 * 5. Read \c rbytes number of bytes from the I2C device, acknowledging all but
 *    the last of them and store them at the address given by \c rbuf.
 * 6. Send a stop condition on the I2C bus.
 * 7. Return.
 *
 * For register write accesses, \c wbuf must contain the complete I2C
 * transaction consisting of register address byte(s) and data bytes.
 *
 * For register read accesses, \c wbuf contains only the register address while
 * the read data bytes are stored in \c rbuf.
 *
 * @param pf        The grabber handle.
 * @param chan      The video channel to address (0...3).
 * @param dev_id    The addressed device's I2C slave id
 * @param wbuf      The I2C command bytes to write, or \c NULL if only reading.
 * @param wbytes    The number of command bytes in wbuf.
 * @param rbuf      Pointer to the receive buffer, or \c NULL if only writing.
 * @param rbytes    The number of bytes to read from the device.
 *
 * @note    The maximum number of bytes that can be written and/or read in one
 *          I2C transfer (i.e., the maximum value for wbytes and rbytes) is 251
 *          payload bytes. If more bytes should be read/written the complete I2C
 *          command is split into multiple I2C transfers.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_i2c_xfer(sxpf_hdl pf, int chan, unsigned char dev_id,
                  const unsigned char *wbuf, unsigned int wbytes,
                  unsigned char *rbuf, unsigned int rbytes)
{
    int res = 0;

    if (wbytes + 5 <= SXPF_NUM_ARGS_PER_CMD &&
        rbytes + 5 <= SXPF_NUM_ARGS_PER_CMD)
    {
        // fast path
        res = sxpf_i2c_xfer_internal(pf, chan, dev_id, wbuf, wbytes,
                                     rbuf, rbytes);

        return g_i2c_result_filter(res, pf, chan, dev_id, wbuf, wbytes,
                                   rbuf, rbytes);
    }

    bool start = true;
    int busI2cFlag = I2C_START_STOP;

    unsigned char* wbufChunk = (unsigned char*)wbuf;
    unsigned char* rbufChunk = (unsigned char*)rbuf;

    while (wbytes > 0 || rbytes > 0)
    {
        int i2cFlag;

        if (start)
        {
            start = false;
            i2cFlag = I2C_START_ONLY;
        }
        else
        {
            i2cFlag = I2C_NONE;
        }

        int wbytesChunk = 0;
        int rbytesChunk = 0;
        if (wbytes > (SXPF_NUM_ARGS_PER_CMD - 5))
        {
            wbytesChunk = (SXPF_NUM_ARGS_PER_CMD - 5);
        }
        else
        {
            wbytesChunk = wbytes;
            if (rbytes == 0)
            {
                i2cFlag |= I2C_STOP_ONLY;
            }
            else
            {
                if (wbytes != 0)
                {
                    start = true;
                }
            }
        }

        if (wbytesChunk == 0)
        {
            if (rbytes > (SXPF_NUM_ARGS_PER_CMD - 5))
            {
                rbytesChunk = (SXPF_NUM_ARGS_PER_CMD - 5);
            }
            else
            {
                rbytesChunk = rbytes;
                i2cFlag |= I2C_STOP_ONLY;
            }
        }
        else
        {
            rbytesChunk = 0;
        }

        if (i2cFlag != busI2cFlag)
        {
            static int plasma_flag[4] =
            { //NO_START_STOP, NO_STOP, NO_START, DO_START_STOP
                2,              1,          3,      0
            };

            sxpf_plasma_writeRegister(pf, 0x00100010 + (4 * chan),
                                      plasma_flag[i2cFlag]);
            busI2cFlag = i2cFlag;
        }

        // perform I2C transfer of the current write/read chunk
        res = sxpf_i2c_xfer_internal(pf, chan, dev_id,
                                     wbufChunk, wbytesChunk,
                                     rbufChunk, rbytesChunk);
        if (res != 0)
            break;

        wbufChunk += wbytesChunk;
        wbytes -= wbytesChunk;

        rbufChunk += rbytesChunk;
        rbytes -= rbytesChunk;
    }

    if (res != 0)
    {
        // try to reset I2C bus to idle state by sending a STOP
        sxpf_plasma_writeRegister(pf, 0x00100010 + (4 * chan), 3); // STOP only

        sxpf_i2c_xfer_internal(pf, chan, dev_id, NULL, 0, NULL, 0);
    }

    sxpf_plasma_writeRegister(pf, 0x00100010 + (4 * chan), 0);

    return g_i2c_result_filter(res,
                               pf, chan, dev_id, wbuf, wbytes, rbuf, rbytes);
}


/** Store a user-defined INI sequence at the given location in the card's user
 *  sequence RAM.
 *
 * @param pf         The grabber handle.
 * @param ram_offset The offset from the start of the card's user sequence RAM
 * @param sequence   Binary INI sequence data
 * @param len        Size in bytes of the new user sequence
 *
 * @return 0 on success, non-null on error
 *
 * @see sxpf_trigger_i2c_update()
 */
int sxpf_store_user_sequence(sxpf_hdl pf, __u32 ram_offset, __u8 *sequence,
                             __u32 len)
{
    sxpf_user_sequence_t    req;

    if (len > sizeof(req.sequence))
        return -1;

    // clear unused argument bytes
    memset(req.sequence, 0, sizeof(req.sequence));

    req.ram_offset = ram_offset;
    req.seq_size   = len;
    memcpy(req.sequence, sequence, len);

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_SET_USR_SEQUENCE, &req);
}


/** Request execution of a user-defined INI sequence when a certain scanline is
 *  hit during reception of video.
 *
 * @param pf            The grabber handle.
 * @param chan          The channel on the card to bind to the sequence (0..3)
 * @param scanline      Line number that shall trigger the sequence's execution
 * @param repeat        Flag: 0=single-shot, 1=repeat for each video frame
 * @param ram_offset    Offset from the start of the user sequence RAM to the
 *                      sequence's binary data.
 *
 * @return 0 on success, non-null on error
 *
 * @see sxpf_store_user_sequence()
 */
int sxpf_trigger_i2c_update(sxpf_hdl pf, int chan, int scanline, int repeat,
                            __u32 ram_offset)
{
    sxpf_enable_user_sequence_t req;

    if (chan > 3 || chan < 0 ||
        scanline > 16384) // limit: "big enough" line number that fits in 16 bit
    {
        return -1;
    }

    req.channel    = chan;
    req.repeat     = repeat;
    req.ram_offset = ram_offset;
    req.scanline   = scanline;

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_ENABLE_USER_SEQ, &req);
}


/** Disable execution of a user-defined INI sequence when a certain scanline is
 *  hit during reception of video.
 *
 * @param pf    The grabber handle.
 * @param chan  The channel on the card to disable the sequence (0..3) for.
 *
 * @return 0 on success, non-null on error
 *
 * @see sxpf_store_user_sequence()
 */
int sxpf_disable_i2c_update(sxpf_hdl pf, int chan)
{
    sxpf_enable_user_sequence_t req;

    if (chan > 3 || chan < 0)
    {
        return -1;
    }

    req.channel    = chan;
    req.repeat     = 0;
    req.ram_offset = 0;
    req.scanline   = -1;    // disable scnaline IRQ

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_ENABLE_USER_SEQ, &req);
}


/** Set the I2C baudrate (transmision speed) on the given video channel.
 *
 * @param pf        The grabber handle.
 * @param chan      The video channel to address (0...3).
 * @param baudrate  The baudrate to set in Hz (min.\ 10000, max.\ 400000).
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_i2c_baudrate(sxpf_hdl pf, int chan, __u32 baudrate)
{
    sxpf_cmd_fifo_el_t  req;
    long                res;

    req.timeout_ms = 1000;
    req.cmd_stat = SXPF_CMD_I2C_SET_BAUDRATE | SXPF_CMD_STATUS_REQUEST;
    req.num_args = 4;
    req.num_ret = 0;

    req.args[0] = (baudrate >>  0) & 0xff;
    req.args[1] = (baudrate >>  8) & 0xff;
    req.args[2] = (baudrate >> 16) & 0xff;
    req.args[3] = chan;

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    return res;
}


/** Set up a new I2C slave.
 *
 * @param pf        The grabber handle.
 * @param i2c_chan  The I2C channel to which to connect the slave device.
 * @param dev_id    The 8bit slave device ID to use.
 *
 * @return negative value on error
 * @return 0 or positive value: handle of the I2C slave (@see sxpf_i2c_slave_
 */
int sxpf_i2c_slave_init(sxpf_hdl pf, __u32 i2c_chan, __u8 dev_id)
{
    sxpf_i2c_slave_init_t   req;

    req.channel = i2c_chan;
    req.dev_id = dev_id;
    req.handle = -1;

    int res = DEV_IOCTL(pf->dev, IOCTL_SXPF_INIT_I2C_SLAVE, &req);

    if (res < 0)
        return -1;

    return res;
}


/** Remove a user-defined I2C slave device.
 *
 * @param pf        The grabber handle.
 * @param handle    The handle of the slave device to remove (return value of
 *                  \ref sxpf_i2c_slave_init).
 *
 * @return 0 un success; non-0 on error
 */
int sxpf_i2c_slave_remove(sxpf_hdl pf, __u8 handle)
{
    return DEV_IOCTL(pf->dev, IOCTL_SXPF_REMOVE_I2C_SLAVE, handle);
}


/** Provide I2C slave device with TX data and acknowledment info
 *
 * @param pf        The grabber handle.
 * @param handle    The handle of the slave device to access (return value of
 *                  \ref sxpf_i2c_slave_init).
 * @param tx_data   Pointer to data to transmit to the I2C master
 * @param tx_size   Number of bytes to transmit.
 */
int sxpf_i2c_slave_tx_ack(sxpf_hdl pf, __u8 handle,
                          __u8 *tx_data, __u32 tx_size)
{
    sxpf_i2c_slave_tx_ack_t req;

    if ((tx_size > sizeof(req.tx_data)) || (tx_size > 0 && !tx_data))
        return -1;

    req.handle = handle;
    req.ack_data = 0;       // TODO check what could be useful here
    req.tx_size = tx_size;

    if (tx_size > 0)
        memcpy(req.tx_data, tx_data, tx_size);

    return DEV_IOCTL(pf->dev, IOCTL_SXPF_I2C_SLAVE_TX_ACK, &req);
}


/** Get the size of the configuration EEPROM on the adapter of a given video
 *  channel.
 *
 * @param pf        The grabber handle.
 * @param channel   The video channel to address (0...3).
 * @param size      The read size will be written here.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_eeprom_getSize(sxpf_hdl pf, __u32 channel, __u32* size)
{
    sxpf_cmd_fifo_el_t  req;
    int                 res;

    req.timeout_ms = 1000;
    req.cmd_stat = SXPF_CMD_EEPROM_GET_SIZE;
    req.num_args = 4; // write 8bit-words
    req.num_ret = 4; // read 8bit-words

    req.args[0] = 0;
    req.args[1] = 0;
    req.args[2] = 0;
    req.args[3] = channel;

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    if (0 == res)
    {
        *size = req.args[0] | (req.args[1] << 8) | (req.args[2] << 16);
    }

    return res;
}


/** Read a number of bytes from the EEPROM on the adapter of the given video
 *  channel.
 *
 * @param pf        The grabber handle.
 * @param channel   The video channel to address (0...3).
 * @param offset    The read start offset.
 * @param bytes     The read bytes will be written here.
 * @param length    The number of bytes to read.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_eeprom_readBytes(sxpf_hdl pf, __u32 channel, __u32 offset,
                          __u8* bytes, __u32 length)
{
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_EEPROM_READ;
    req.num_args = 4; // write 8bit-words
    req.num_ret = 4; // read 8bit-words

    while (length)
    {
        int     res;
        __u32   chunkSize =
            (length > (SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH - 4)) ?
            (SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH - 4) : length;

        req.args[0] = (offset >> 0) & 0xff;
        req.args[1] = (offset >> 8) & 0xff;
        req.args[2] = chunkSize;
        req.args[3] = channel;

        req.num_ret = 4 + chunkSize;

        res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);
        if (res)
        {
            return res;
        }

        memcpy(bytes, &req.args[4], chunkSize);

        length -= chunkSize;
        offset += chunkSize;
        bytes += chunkSize;
    }

    return 0;
}


/** Write a number of bytes to the EEPROM on the adapter of the given video
 *  channel.
 *
 * @param pf        The grabber handle.
 * @param channel   The video channel to address (0...3).
 * @param offset    The write start offset.
 * @param bytes     The bytes to write to write.
 * @param length    The number of bytes to write.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_eeprom_writeBytes(sxpf_hdl pf, __u32 channel, __u32 offset,
        __u8* bytes, __u32 length)
{
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_EEPROM_WRITE;
    req.num_args = 4; // write 8bit-words
    req.num_ret = 4; // read 8bit-words

    while (length)
    {
        int     res;
        __u32   chunkSize =
            (length > (SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH - 4)) ?
            (SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH - 4) : length;

        req.args[0] = (offset >> 0) & 0xff;
        req.args[1] = (offset >> 8) & 0xff;
        req.args[2] = chunkSize;
        req.args[3] = channel;

        memcpy(&req.args[4], bytes, chunkSize);

        req.num_args = 4 + chunkSize;

        res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);
        if (res)
        {
            return res;
        }

        length -= chunkSize;
        offset += chunkSize;
        bytes += chunkSize;
    }

    return 0;
}


/** Prepare the internal SPI flash information so it can be read out by
 *  sxpf_flash_get_info().
 *
 * @param pf        The grabber handle.
 *
 * @return  0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_flash_init(sxpf_hdl pf)
{
    #define FLASH_ID_W25Q128FV_SPI  0xEF4018
    #define FLASH_ID_W25Q128FV_QPI  0xEF6018
    #define FLASH_ID_N25Q128_SPI    0x20BA18
    #define FLASH_ID_NOR_INTEL      0x890062    // 0x89     ^= XFL_MANUFACTURER_ID_INTEL
                                                // 0x0062   ^= XFL_INTEL_PLATFORM_MANUFACTURE_ID
                                                // 0x506B   ^= XFL_INTEL_PLATFORM_DEVICE_ID
    #define FLASH_ID_MT25QU256_QPI  0x2019BB    // 0x20XXXX ^= Manufacturer Micron
                                                // 0xXX19XX ^= Memory capacity 256Mb
                                                // 0xXXXXBB ^= Memory type 1.8V
    __u32 version = 0;
    int                 res;
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_SPI_GET_ID;
    req.num_args = 0; // write 8bit-words
    req.num_ret = 4; // read 8bit-words

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    if (0 == res)
    {
        #define REG_PLASMA_VERSION 0xC
    sxpf_plasma_readRegister(pf, REG_PLASMA_VERSION, &version);
        // the result doesn't care
        // - if it goes wrong, the version is 0x0, that always happens before FW 15100102
        // - otherwise we got the current FW version
    }

    if (0 == res)
    {
        pf->flash_info.id = req.args[0] | (req.args[1] << 8UL) |
            (req.args[2] << 16UL) | (req.args[3] << 24UL);

        switch (pf->flash_info.id)
        {
        case FLASH_ID_W25Q128FV_SPI:
        case FLASH_ID_N25Q128_SPI:
            // since FW 15100102, we use the universal SPI driver,
            // that use sector-sizes of 64K
            if (version >= 0x15100102) {
                pf->flash_info.sectorSize = 64*1024;
            } else {
                pf->flash_info.sectorSize =  4*1024;
            }
            pf->flash_info.flashSize  = 16*1024*1024;
            pf->flash_info.pageSize   = 256;
            break;
        case FLASH_ID_NOR_INTEL:
            pf->flash_info.sectorSize = 128*1024; // more exactly: 127*1024 + 4*256
            pf->flash_info.flashSize  = 128*1024*1024; // 134.217.728 bytes
            pf->flash_info.pageSize   = 256; // no pages, but max available internal buffer is 256
            break;
        case FLASH_ID_MT25QU256_QPI:
            pf->flash_info.sectorSize = 2*64*1024;
            pf->flash_info.flashSize  = 64*1024*1024;
            pf->flash_info.pageSize   = 256;
            break;
        default:
            fprintf(stderr, "Flash ID (0x%x) not supported.\n",
                    pf->flash_info.id);
            res = -1;
            break;
        }
    }

    return res;
}


/** Get information about the SPI flash that is needed for write operations.
 *
 * @param pf        The grabber handle.
 * @param info      The flash informations will be written here.
 *
 * @return  0 on success, non-null on error
 */
int sxpf_flash_get_info(sxpf_hdl pf, sxpf_flash_info_t* info)
{
    memcpy(info, &pf->flash_info, sizeof(sxpf_flash_info_t));
    return 0;
}


/** Write a number of bytes to the proFRAME board's SPI flash.
 *
 * @note    Make sure that sector/page-boundaries match and the written range is
 *          erased.
 *
 * @note    This function performs a verification of the written data.
 *
 * @param pf        The grabber handle.
 * @param address   The write address (offset) in the flash.
 * @param bytes     The data to be written.
 * @param size      The number of bytes to write.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_flash_write(sxpf_hdl pf, __u32 address, __u8* bytes, __u32 size)
{
    int res = 0;
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_SPI_WRITE_VERIFY;
    req.num_args = size + 4;    // written 8bit-words
    req.num_ret = 0;            // read 8bit-words

    if (size > (SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH - 4))
    {
        fprintf(stderr, "Too many bytes to write to SPI Flash: %d\n", size);
        return -1;
    }

    if (address + size > pf->flash_info.flashSize)
    {
        fprintf(stderr, "Out of bounds: Can't write %d bytes at address 0x%x\n",
                size, address);
        return -1;
    }

    if (address > 0xFFFFFF) {
        fprintf(stderr, "Out of bounds: address > 0xFFFFFF not supported\n");
        return -1;
    }

    req.args[0] = (address >>  0) & 0xff;
    req.args[1] = (address >>  8) & 0xff;
    req.args[2] = (address >> 16) & 0xff;
    req.args[3] = size;

    memcpy(&req.args[4], bytes, size);

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    return res;
}


/** Read a number of bytes from the proFRAME board's SPI flash.
 *
 * @param pf        The grabber handle.
 * @param address   The read address (offset) of flash.
 * @param bytes     The read data is written here.
 * @param size      The number of bytes to read.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_flash_read(sxpf_hdl pf, __u32 address, __u8* bytes, __u32 size)
{
    int                 res;
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_SPI_READ;
    req.num_args = 4; // write 8bit-words
    req.num_ret = size+4; // read 8bit-words

    if (size > (SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH - 4))
    {
        fprintf(stderr, "Too many bytes to read from SPI Flash: %d\n", size);
        return -1;
    }

    if (address+size > pf->flash_info.flashSize)
    {
        fprintf(stderr, "Out of bounds: Can't read %d bytes at address 0x%x\n",
                size, address);
        return -1;
    }

    req.args[0] = (address >>  0) & 0xff;
    req.args[1] = (address >>  8) & 0xff;
    req.args[2] = (address >> 16) & 0xff;
    req.args[3] = size;

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    if (0 == res)
    {
        memcpy(bytes, &req.args[4], size);
    }

    return res;
}


/** Erase a sector at the given address within the SPI flash.
 *
 * Obsolete function: 4K sector erase is only supported for FW before 0x15100102!
 *
 * @param pf        The grabber handle.
 * @param address   The sector address (offset) in the flash.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_flash_eraseSector4KB(sxpf_hdl pf, __u32 address)
{
    int                 res;
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_SPI_ERASE_4KB;
    req.num_args = 8; // write 8bit-words
    req.num_ret = 0; // read 8bit-words

    if (address >= pf->flash_info.flashSize)
    {
        fprintf(stderr, "Out of bounds: Can't erase sector at address 0x%x\n",
                address);
        return -1;
    }

    address &= ~(pf->flash_info.sectorSize - 1);
    req.args[0] = (address >>  0) & 0xff;
    req.args[1] = (address >>  8) & 0xff;
    req.args[2] = (address >> 16) & 0xff;
    req.args[3] = (address >> 24) & 0xff;

    req.args[4] = (pf->flash_info.sectorSize >>  0) & 0xff;
    req.args[5] = (pf->flash_info.sectorSize >>  8) & 0xff;
    req.args[6] = (pf->flash_info.sectorSize >> 16) & 0xff;
    req.args[7] = (pf->flash_info.sectorSize >> 24) & 0xff;

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    return res;
}

/** Erase a sector at the given address within the SPI flash.
 *
 * @param pf        The grabber handle.
 * @param address   The sector address (offset) in the flash.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_flash_eraseSector64KB(sxpf_hdl pf, __u32 address)
{
    int                 res;
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_SPI_ERASE_64KB;
    req.num_args = 8; // write 8bit-words
    req.num_ret = 0; // read 8bit-words

    if (address >= pf->flash_info.flashSize)
    {
        fprintf(stderr, "Out of bounds: Can't erase sector at address 0x%x\n",
                address);
        return -1;
    }

    address &= ~(pf->flash_info.sectorSize - 1);
    req.args[0] = (address >>  0) & 0xff;
    req.args[1] = (address >>  8) & 0xff;
    req.args[2] = (address >> 16) & 0xff;
    req.args[3] = (address >> 24) & 0xff;

    req.args[4] = (pf->flash_info.sectorSize >>  0) & 0xff;
    req.args[5] = (pf->flash_info.sectorSize >>  8) & 0xff;
    req.args[6] = (pf->flash_info.sectorSize >> 16) & 0xff;
    req.args[7] = (pf->flash_info.sectorSize >> 24) & 0xff;

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    return res;
}

/** Erase a sector at the given address within the NOR flash.
 *
 * @param pf        The grabber handle.
 * @param address   The sector address (offset) in the flash.
 *
 * @return 0 on success, non-null on error
 */
int sxpf_flash_eraseSector128KB(sxpf_hdl pf, __u32 address)
{
    // note: the implementation of eraseSector64KB do send
    //       the sector-size (number of bytes) to erase
    // --> thus the native plasma-implementation use
    //       64KByte for SPI-Flashes  and
    //      128KByte for NOR-Flashes
    return sxpf_flash_eraseSector64KB(pf, address);
}

/** Write a register value of bar 2 by the internal soft core.
 *
 * @param pf        The grabber handle.
 * @param reg       Register offset in bar 2.
 * @param val       Value to be written.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_plasma_writeRegister(sxpf_hdl pf, __u32 reg, __u32 val)
{
    int                 res;
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_REGISTER_WRITE;
    req.num_args = 8; // write 8bit-words
    req.num_ret = 0; // read 8bit-words

    // clear unused argument bytes
    memset(req.args, 0, sizeof(req.args));

    req.args[0] = (reg >>  0) & 0xff;
    req.args[1] = (reg >>  8) & 0xff;
    req.args[2] = (reg >> 16) & 0xff;
    req.args[3] = (reg >> 24) & 0xff;

    req.args[4] = (val >>  0) & 0xff;
    req.args[5] = (val >>  8) & 0xff;
    req.args[6] = (val >> 16) & 0xff;
    req.args[7] = (val >> 24) & 0xff;

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    return res;
}

/** Read a register value of bar 2 by the internal soft core.
 *
 * @param pf        The grabber handle.
 * @param reg       Register offset in bar 2.
 * @param val       read Value.
 *
 * @return 0 on success, non-null on error (see @ref sxpf_plasma_cmd_status_e)
 */
int sxpf_plasma_readRegister(sxpf_hdl pf, __u32 reg, __u32* val)
{
    int                 res;
    sxpf_cmd_fifo_el_t  req;

    req.timeout_ms = 3000;
    req.cmd_stat = SXPF_CMD_REGISTER_READ;
    req.num_args = 4; // write 8bit-words
    req.num_ret = 8; // read 8bit-words

    req.args[0] = (reg >>  0) & 0xff;
    req.args[1] = (reg >>  8) & 0xff;
    req.args[2] = (reg >> 16) & 0xff;
    req.args[3] = (reg >> 24) & 0xff;

    res = DEV_IOCTL(pf->dev, IOCTL_SXPF_CMD_SYNC, &req);

    if (0 == res)
    {
        *val = (req.args[4] << 0)
             | (req.args[5] << 8)
             | (req.args[6] << 16)
             | (req.args[7] << 24);
    }

    return res;
}
