#pragma once

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <libavutil/error.h>

#include "mpv_talloc.h"
#include "audio/aframe.h"
#include "audio/chmap.h"
#include "audio/format.h"
#include "common/msg.h"
#include "demux/stheader.h"
#include "filters/filter_internal.h"

#include "starmine_ad.h"

enum {
    STARMINE_RENDER_714_CHANNELS = STARMINE_AD_RENDER_714_CHANNEL_COUNT,
};

static const struct mp_chmap starmine_714_chmap =
    MP_CHMAP12(FL, FR, FC, LFE, BL, BR, SL, SR, TFL, TFR, TBL, TBR);

static inline int starmine_output_index(starmine_ad_bed_channel channel)
{
    switch (channel) {
    case STARMINE_AD_BED_CHANNEL_FRONT_LEFT:
        return 0;
    case STARMINE_AD_BED_CHANNEL_FRONT_RIGHT:
        return 1;
    case STARMINE_AD_BED_CHANNEL_CENTER:
        return 2;
    case STARMINE_AD_BED_CHANNEL_LOW_FREQUENCY_EFFECTS:
        return 3;
    case STARMINE_AD_BED_CHANNEL_REAR_LEFT:
        return 4;
    case STARMINE_AD_BED_CHANNEL_REAR_RIGHT:
        return 5;
    case STARMINE_AD_BED_CHANNEL_SURROUND_LEFT:
        return 6;
    case STARMINE_AD_BED_CHANNEL_SURROUND_RIGHT:
        return 7;
    case STARMINE_AD_BED_CHANNEL_TOP_FRONT_LEFT:
        return 8;
    case STARMINE_AD_BED_CHANNEL_TOP_FRONT_RIGHT:
        return 9;
    case STARMINE_AD_BED_CHANNEL_TOP_REAR_LEFT:
        return 10;
    case STARMINE_AD_BED_CHANNEL_TOP_REAR_RIGHT:
        return 11;
    default:
        return -1;
    }
}

static inline int starmine_build_frame(struct mp_filter *ad,
                                       struct mp_codec_params *codec,
                                       struct mp_aframe_pool *pool,
                                       const starmine_ad_render_714_frame *frame,
                                       double pts,
                                       struct mp_frame *out)
{
    struct mp_aframe *mpframe = mp_aframe_create();
    uint8_t **planes = NULL;
    float *dst = NULL;
    bool seen[STARMINE_RENDER_714_CHANNELS] = {0};

    if (!mpframe)
        return AVERROR(ENOMEM);

    struct mp_chmap chmap = starmine_714_chmap;
    if (!mp_aframe_set_format(mpframe, AF_FORMAT_FLOAT) ||
        !mp_aframe_set_chmap(mpframe, &chmap) ||
        !mp_aframe_set_rate(mpframe, frame->sample_rate) ||
        mp_aframe_pool_allocate(pool, mpframe, frame->samples_per_channel) < 0)
    {
        talloc_free(mpframe);
        return AVERROR(ENOMEM);
    }

    planes = mp_aframe_get_data_rw(mpframe);
    if (!planes || !planes[0]) {
        talloc_free(mpframe);
        return AVERROR(ENOMEM);
    }

    dst = (float *)planes[0];
    memset(dst, 0, frame->samples_per_channel * chmap.num * sizeof(float));

    for (size_t channel_index = 0; channel_index < frame->channel_count;
         channel_index++)
    {
        int out_index = starmine_output_index(frame->channel_order[channel_index]);
        const float *src = frame->channels[channel_index];

        if (out_index < 0 || out_index >= chmap.num || !src) {
            MP_ERR(ad, "unsupported 7.1.4 channel id %d in rendered frame\n",
                   frame->channel_order[channel_index]);
            talloc_free(mpframe);
            return AVERROR_INVALIDDATA;
        }

        seen[out_index] = true;
        for (size_t sample_index = 0; sample_index < frame->samples_per_channel;
             sample_index++)
        {
            dst[sample_index * chmap.num + out_index] = src[sample_index];
        }
    }

    for (int index = 0; index < chmap.num; index++) {
        if (!seen[index]) {
            MP_ERR(ad, "rendered frame is missing 7.1.4 channel index %d\n",
                   index);
            talloc_free(mpframe);
            return AVERROR_INVALIDDATA;
        }
    }

    mp_aframe_set_pts(mpframe, pts);
    codec->channels = chmap;
    codec->samplerate = frame->sample_rate;
    *out = MAKE_FRAME(MP_FRAME_AUDIO, mpframe);
    return 0;
}

static inline double starmine_render_frame_duration(
    const starmine_ad_render_714_frame *frame)
{
    if (frame->has_frame && frame->sample_rate && frame->samples_per_channel)
        return (double)frame->samples_per_channel / frame->sample_rate;
    return 0.0;
}

static inline bool starmine_status_is_oamd_warmup(starmine_ad_status status)
{
    return status == STARMINE_AD_STATUS_MISSING_OAMD ||
           status == STARMINE_AD_STATUS_OAMD_STATE_UNINITIALIZED;
}

static inline bool starmine_truehd_packet_is_single_access_unit(
    const uint8_t *data, size_t len)
{
    uint16_t header = 0;
    size_t access_unit_len = 0;

    if (!data || len < 2)
        return false;

    header = (uint16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);
    access_unit_len = (size_t)((header & 0x0fffU) << 1);
    return access_unit_len != 0 && access_unit_len == len;
}
