#ifndef SXPF_INSTANCE_H_
#define SXPF_INSTANCE_H_

/** Frame data pointers returned by sxpf_acquire_frame_ptr and
 *  sxpf_acquire_frame_header_payload
 */
typedef struct
{
    void                  *header; /**< Frame header pointer */
    void                  *payload;/**< Payload Data pointer */
    size_t                 size;   /**< Data size in bytes (including header) */
    sxpf_buf_alloc_type_t  type;   /**< allocation type */

} sxpf_frame_data_t;

/** Grabber instance management */
struct sxpf_instance_s
{
    HSXPFDEV            dev;            /**< Driver file descriptor */

    /** User accessible low-level device handle for select (Linux) or
     *  WaitForMultipleObjects (Windows).
     */
    HWAITSXPF           dev_user;

    sxpf_frame_data_t   frames[SXPF_MAX_DMA_BUFFERS];
    sxpf_flash_info_t   flash_info;
    sxpf_card_props_t   props;          /**< card properties from driver */
    uint32_t            num_dma_buffers;/**< sum of video & I2C buffers */
};

#endif /* SXPF_INSTANCE_H_ */
