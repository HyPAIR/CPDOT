// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm.h>

#ifndef _bot_core_image_t_h
#define _bot_core_image_t_h

#ifdef __cplusplus
extern "C" {
#endif

#include "bot_core_image_metadata_t.h"
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_UYVY 1498831189
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUYV 1448695129
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU1 827677001
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_IYU2 844454217
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV420 842093913
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_YUV411P 1345401140
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_I420 808596553
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_NV12 842094158
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY 1497715271
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB 859981650
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGR 861030210
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGBA 876758866
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BGRA 877807426
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_BGGR 825770306
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GBRG 844650584
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_GRBG 861427800
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BAYER_RGGB 878205016
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_BGGR 826360386
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GBRG 843137602
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_GRBG 859914818
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_BAYER16_RGGB 876692034
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_BGGR 826360396
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GBRG 843137612
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_GRBG 859914828
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_BAYER16_RGGB 876692044
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG 1196444237
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_GRAY16 357
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_GRAY16 909199180
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_RGB16 358
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_LE_RGB16 1279412050
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_GRAY16 359
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_BE_SIGNED_RGB16 360
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32 842221382
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_INVALID -2
#define BOT_CORE_IMAGE_T_PIXEL_FORMAT_ANY -1

/**
 * Describes an image
 *
 * This type should be kept compatible with camlcm.image_t, as defined
 * in camunits-extra
 */
typedef struct _bot_core_image_t bot_core_image_t;
struct _bot_core_image_t
{
    int64_t    utime;
    int32_t    width;
    int32_t    height;
    int32_t    row_stride;
    int32_t    pixelformat;
    int32_t    size;
    uint8_t    *data;
    int32_t    nmetadata;
    bot_core_image_metadata_t *metadata;
};

/**
 * Create a deep copy of a bot_core_image_t.
 * When no longer needed, destroy it with bot_core_image_t_destroy()
 */
bot_core_image_t* bot_core_image_t_copy(const bot_core_image_t* to_copy);

/**
 * Destroy an instance of bot_core_image_t created by bot_core_image_t_copy()
 */
void bot_core_image_t_destroy(bot_core_image_t* to_destroy);

/**
 * Identifies a single subscription.  This is an opaque data type.
 */
typedef struct _bot_core_image_t_subscription_t bot_core_image_t_subscription_t;

/**
 * Prototype for a callback function invoked when a message of type
 * bot_core_image_t is received.
 */
typedef void(*bot_core_image_t_handler_t)(const lcm_recv_buf_t *rbuf,
             const char *channel, const bot_core_image_t *msg, void *userdata);

/**
 * Publish a message of type bot_core_image_t using LCM.
 *
 * @param lcm The LCM instance to publish with.
 * @param channel The channel to publish on.
 * @param msg The message to publish.
 * @return 0 on success, <0 on error.  Success means LCM has transferred
 * responsibility of the message data to the OS.
 */
int bot_core_image_t_publish(lcm_t *lcm, const char *channel, const bot_core_image_t *msg);

/**
 * Subscribe to messages of type bot_core_image_t using LCM.
 *
 * @param lcm The LCM instance to subscribe with.
 * @param channel The channel to subscribe to.
 * @param handler The callback function invoked by LCM when a message is received.
 *                This function is invoked by LCM during calls to lcm_handle() and
 *                lcm_handle_timeout().
 * @param userdata An opaque pointer passed to @p handler when it is invoked.
 * @return 0 on success, <0 if an error occured
 */
bot_core_image_t_subscription_t* bot_core_image_t_subscribe(lcm_t *lcm, const char *channel, bot_core_image_t_handler_t handler, void *userdata);

/**
 * Removes and destroys a subscription created by bot_core_image_t_subscribe()
 */
int bot_core_image_t_unsubscribe(lcm_t *lcm, bot_core_image_t_subscription_t* hid);

/**
 * Sets the queue capacity for a subscription.
 * Some LCM providers (e.g., the default multicast provider) are implemented
 * using a background receive thread that constantly revceives messages from
 * the network.  As these messages are received, they are buffered on
 * per-subscription queues until dispatched by lcm_handle().  This function
 * how many messages are queued before dropping messages.
 *
 * @param subs the subscription to modify.
 * @param num_messages The maximum number of messages to queue
 *  on the subscription.
 * @return 0 on success, <0 if an error occured
 */
int bot_core_image_t_subscription_set_queue_capacity(bot_core_image_t_subscription_t* subs,
                              int num_messages);

/**
 * Encode a message of type bot_core_image_t into binary form.
 *
 * @param buf The output buffer.
 * @param offset Encoding starts at this byte offset into @p buf.
 * @param maxlen Maximum number of bytes to write.  This should generally
 *               be equal to bot_core_image_t_encoded_size().
 * @param msg The message to encode.
 * @return The number of bytes encoded, or <0 if an error occured.
 */
int bot_core_image_t_encode(void *buf, int offset, int maxlen, const bot_core_image_t *p);

/**
 * Decode a message of type bot_core_image_t from binary form.
 * When decoding messages containing strings or variable-length arrays, this
 * function may allocate memory.  When finished with the decoded message,
 * release allocated resources with bot_core_image_t_decode_cleanup().
 *
 * @param buf The buffer containing the encoded message
 * @param offset The byte offset into @p buf where the encoded message starts.
 * @param maxlen The maximum number of bytes to read while decoding.
 * @param msg Output parameter where the decoded message is stored
 * @return The number of bytes decoded, or <0 if an error occured.
 */
int bot_core_image_t_decode(const void *buf, int offset, int maxlen, bot_core_image_t *msg);

/**
 * Release resources allocated by bot_core_image_t_decode()
 * @return 0
 */
int bot_core_image_t_decode_cleanup(bot_core_image_t *p);

/**
 * Check how many bytes are required to encode a message of type bot_core_image_t
 */
int bot_core_image_t_encoded_size(const bot_core_image_t *p);

// LCM support functions. Users should not call these
int64_t __bot_core_image_t_get_hash(void);
uint64_t __bot_core_image_t_hash_recursive(const __lcm_hash_ptr *p);
int __bot_core_image_t_encode_array(void *buf, int offset, int maxlen, const bot_core_image_t *p, int elements);
int __bot_core_image_t_decode_array(const void *buf, int offset, int maxlen, bot_core_image_t *p, int elements);
int __bot_core_image_t_decode_array_cleanup(bot_core_image_t *p, int elements);
int __bot_core_image_t_encoded_array_size(const bot_core_image_t *p, int elements);
int __bot_core_image_t_clone_array(const bot_core_image_t *p, bot_core_image_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif
