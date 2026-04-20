/*
 * This file is part of mpv.
 *
 * mpv is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * mpv is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with mpv.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <libavcodec/avcodec.h>

#include "config.h"

#include "mpv_talloc.h"
#include "audio/aframe.h"
#include "audio/chmap.h"
#include "audio/format.h"
#include "common/av_common.h"
#include "common/codecs.h"
#include "common/msg.h"
#include "demux/packet.h"
#include "demux/packet_pool.h"
#include "demux/stheader.h"
#include "filters/f_decoder_wrapper.h"
#include "filters/filter_internal.h"

#include "starmine_ad.h"

enum {
    STARMINE_RENDER_714_CHANNELS = STARMINE_AD_RENDER_714_CHANNEL_COUNT,
};

static const struct mp_chmap starmine_714_chmap =
    MP_CHMAP12(FL, FR, FC, LFE, BL, BR, SL, SR, TFL, TFR, TBL, TBR);

struct priv {
    struct mp_codec_params *codec;
    AVRational codec_timebase;
    AVCodecParserContext *parser;
    AVCodecContext *parser_ctx;
    struct demux_packet *pending_packet;
    size_t pending_packet_offset;
    bool pending_packet_pts_used;
    bool draining;
    double next_pts;
    struct mp_aframe_pool *pool;
    starmine_ad_renderer_714 *renderer;
    struct lavc_state state;

    struct mp_decoder public;
};

static void clear_pending_packet(struct mp_filter *ad)
{
    struct priv *ctx = ad->priv;

    if (ctx->pending_packet) {
        demux_packet_pool_push(ad->packet_pool, ctx->pending_packet);
        ctx->pending_packet = NULL;
    }
    ctx->pending_packet_offset = 0;
    ctx->pending_packet_pts_used = false;
}

static bool init_parser(struct mp_filter *ad)
{
    struct priv *ctx = ad->priv;
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_EAC3);

    ctx->parser = av_parser_init(AV_CODEC_ID_EAC3);
    if (!ctx->parser) {
        MP_ERR(ad, "failed to create E-AC-3 parser\n");
        return false;
    }

    ctx->parser_ctx = avcodec_alloc_context3(codec);
    if (!ctx->parser_ctx) {
        MP_ERR(ad, "failed to allocate parser codec context\n");
        av_parser_close(ctx->parser);
        ctx->parser = NULL;
        return false;
    }

    ctx->parser_ctx->codec_type = AVMEDIA_TYPE_AUDIO;
    ctx->parser_ctx->codec_id = AV_CODEC_ID_EAC3;
    ctx->parser_ctx->pkt_timebase = ctx->codec_timebase;
    mp_set_avctx_codec_headers(ctx->parser_ctx, ctx->codec);
    return true;
}

static bool reinit_parser(struct mp_filter *ad)
{
    struct priv *ctx = ad->priv;

    if (ctx->parser) {
        av_parser_close(ctx->parser);
        ctx->parser = NULL;
    }
    avcodec_free_context(&ctx->parser_ctx);
    return init_parser(ad);
}

static int starmine_output_index(starmine_ad_bed_channel channel)
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

static double access_unit_duration(const starmine_ad_access_unit_info *info,
                                   const starmine_ad_render_714_frame *frame)
{
    if (info->sample_rate && info->num_blocks) {
        return (256.0 * info->num_blocks) / info->sample_rate;
    }
    if (frame->has_frame && frame->sample_rate && frame->samples_per_channel) {
        return (double)frame->samples_per_channel / frame->sample_rate;
    }
    return 0.0;
}

static int build_frame(struct mp_filter *ad,
                       const starmine_ad_render_714_frame *frame,
                       double pts,
                       struct mp_frame *out)
{
    struct priv *ctx = ad->priv;
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
        mp_aframe_pool_allocate(ctx->pool, mpframe, frame->samples_per_channel) < 0)
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
    ctx->codec->channels = chmap;
    ctx->codec->samplerate = frame->sample_rate;
    *out = MAKE_FRAME(MP_FRAME_AUDIO, mpframe);
    return 0;
}

static int decode_access_unit(struct mp_filter *ad,
                              const uint8_t *data, size_t len,
                              struct mp_frame *out)
{
    struct priv *ctx = ad->priv;
    starmine_ad_access_unit_info info;
    starmine_ad_render_714_frame frame;
    starmine_ad_status status;
    double pts = ctx->next_pts;
    double duration = 0.0;

    if (starmine_ad_access_unit_info_init(&info) != STARMINE_AD_STATUS_OK ||
        starmine_ad_render_714_frame_init(&frame) != STARMINE_AD_STATUS_OK)
    {
        MP_ERR(ad, "failed to initialize Starmine output structs\n");
        return AVERROR(EINVAL);
    }

    status = starmine_ad_renderer_714_push_access_unit(ctx->renderer, data, len,
                                                       &info, &frame);
    if (status != STARMINE_AD_STATUS_OK) {
        MP_ERR(ad, "Starmine decode failed: %s\n",
               starmine_ad_status_string(status));
        return AVERROR_INVALIDDATA;
    }

    duration = access_unit_duration(&info, &frame);
    if (pts != MP_NOPTS_VALUE && duration > 0.0)
        ctx->next_pts = pts + duration;

    if (!frame.has_frame)
        return AVERROR(EAGAIN);

    if (pts == MP_NOPTS_VALUE && ctx->pending_packet &&
        ctx->pending_packet->pts != MP_NOPTS_VALUE)
    {
        pts = ctx->pending_packet->pts;
    }

    return build_frame(ad, &frame, pts, out);
}

static bool init(struct mp_filter *ad, struct mp_codec_params *codec,
                 const char *decoder)
{
    struct priv *ctx = ad->priv;

    if (strcmp(decoder, "eac3joc") != 0)
        return false;
    if (!codec->codec || strcmp(codec->codec, "eac3") != 0)
        return false;

    ctx->codec = codec;
    ctx->codec_timebase = mp_get_codec_timebase(codec);
    ctx->pool = mp_aframe_pool_create(ctx);
    ctx->renderer = starmine_ad_renderer_714_new();
    if (!ctx->renderer) {
        MP_ERR(ad, "failed to create E-AC-3 JOC renderer\n");
        return false;
    }

    ctx->next_pts = MP_NOPTS_VALUE;
    if (!init_parser(ad)) {
        starmine_ad_renderer_714_free(ctx->renderer);
        ctx->renderer = NULL;
        return false;
    }

    codec->decoder = "eac3joc";
    codec->decoder_desc = "E-AC-3 JOC decoder";
    return true;
}

static void ad_eac3joc_destroy(struct mp_filter *ad)
{
    struct priv *ctx = ad->priv;

    clear_pending_packet(ad);
    if (ctx->parser) {
        av_parser_close(ctx->parser);
        ctx->parser = NULL;
    }
    avcodec_free_context(&ctx->parser_ctx);
    if (ctx->renderer) {
        starmine_ad_renderer_714_free(ctx->renderer);
        ctx->renderer = NULL;
    }
}

static void ad_eac3joc_reset(struct mp_filter *ad)
{
    struct priv *ctx = ad->priv;

    clear_pending_packet(ad);
    ctx->draining = false;
    ctx->next_pts = MP_NOPTS_VALUE;
    ctx->state = (struct lavc_state){0};
    if (ctx->renderer)
        starmine_ad_renderer_714_reset(ctx->renderer);
    reinit_parser(ad);
}

static int send_packet(struct mp_filter *ad, struct demux_packet *mpkt)
{
    struct priv *ctx = ad->priv;

    if (!ctx->parser || !ctx->parser_ctx)
        return AVERROR(EINVAL);

    if (mpkt) {
        if (ctx->pending_packet) {
            MP_ERR(ad, "received a new packet before the previous one drained\n");
            clear_pending_packet(ad);
        }
        ctx->pending_packet = demux_copy_packet(ad->packet_pool, mpkt);
        if (!ctx->pending_packet)
            return AVERROR(ENOMEM);
        ctx->pending_packet_offset = 0;
        ctx->pending_packet_pts_used = false;
        ctx->draining = false;
        if (ctx->next_pts == MP_NOPTS_VALUE && mpkt->pts != MP_NOPTS_VALUE)
            ctx->next_pts = mpkt->pts;
        return 0;
    }

    ctx->draining = true;
    return 0;
}

static int receive_packet_frame(struct mp_filter *ad, struct mp_frame *out)
{
    struct priv *ctx = ad->priv;
    uint8_t *access_unit = NULL;
    int access_unit_size = 0;
    const uint8_t *data = ctx->pending_packet->buffer + ctx->pending_packet_offset;
    int size = ctx->pending_packet->len - ctx->pending_packet_offset;
    int64_t pts = AV_NOPTS_VALUE;
    int64_t dts = AV_NOPTS_VALUE;
    int64_t pos = -1;

    if (!ctx->pending_packet_pts_used) {
        pts = mp_pts_to_av(ctx->pending_packet->pts, &ctx->codec_timebase);
        dts = mp_pts_to_av(ctx->pending_packet->dts, &ctx->codec_timebase);
        pos = ctx->pending_packet->pos;
    }

    int consumed = av_parser_parse2(ctx->parser, ctx->parser_ctx, &access_unit,
                                    &access_unit_size, data, size,
                                    pts, dts, pos);
    if (consumed < 0) {
        MP_ERR(ad, "E-AC-3 parser failed\n");
        clear_pending_packet(ad);
        return AVERROR_INVALIDDATA;
    }

    if (consumed == 0 && access_unit_size == 0 && size > 0) {
        MP_ERR(ad, "E-AC-3 parser made no progress\n");
        clear_pending_packet(ad);
        return AVERROR_INVALIDDATA;
    }

    ctx->pending_packet_offset += consumed;
    ctx->pending_packet_pts_used = true;
    if (ctx->pending_packet_offset >= ctx->pending_packet->len)
        clear_pending_packet(ad);

    if (access_unit_size <= 0)
        return AVERROR(EAGAIN);

    return decode_access_unit(ad, access_unit, access_unit_size, out);
}

static int receive_flush_frame(struct mp_filter *ad, struct mp_frame *out)
{
    struct priv *ctx = ad->priv;
    uint8_t *access_unit = NULL;
    int access_unit_size = 0;
    int consumed = av_parser_parse2(ctx->parser, ctx->parser_ctx, &access_unit,
                                    &access_unit_size, NULL, 0,
                                    AV_NOPTS_VALUE, AV_NOPTS_VALUE, -1);

    if (consumed < 0) {
        MP_ERR(ad, "E-AC-3 parser flush failed\n");
        ctx->draining = false;
        return AVERROR_INVALIDDATA;
    }

    if (access_unit_size <= 0) {
        ctx->draining = false;
        return AVERROR_EOF;
    }

    return decode_access_unit(ad, access_unit, access_unit_size, out);
}

static int receive_frame(struct mp_filter *ad, struct mp_frame *out)
{
    struct priv *ctx = ad->priv;

    while (1) {
        int ret = AVERROR(EAGAIN);

        if (ctx->pending_packet) {
            ret = receive_packet_frame(ad, out);
        } else if (ctx->draining) {
            ret = receive_flush_frame(ad, out);
        } else {
            return AVERROR(EAGAIN);
        }

        if (out->type)
            return 0;
        if (ret == AVERROR(EAGAIN))
            continue;
        return ret;
    }
}

static void ad_eac3joc_process(struct mp_filter *ad)
{
    struct priv *ctx = ad->priv;

    lavc_process(ad, &ctx->state, send_packet, receive_frame);
}

static const struct mp_filter_info ad_eac3joc_filter = {
    .name = "ad_eac3joc",
    .priv_size = sizeof(struct priv),
    .process = ad_eac3joc_process,
    .reset = ad_eac3joc_reset,
    .destroy = ad_eac3joc_destroy,
};

static struct mp_decoder *create(struct mp_filter *parent,
                                 struct mp_codec_params *codec,
                                 const char *decoder)
{
    struct mp_filter *ad = mp_filter_create(parent, &ad_eac3joc_filter);
    if (!ad)
        return NULL;

    mp_filter_add_pin(ad, MP_PIN_IN, "in");
    mp_filter_add_pin(ad, MP_PIN_OUT, "out");

    ad->log = mp_log_new(ad, parent->log, NULL);

    struct priv *ctx = ad->priv;
    ctx->public.f = ad;

    if (!init(ad, codec, decoder)) {
        talloc_free(ad);
        return NULL;
    }

    return &ctx->public;
}

static void add_decoders(struct mp_decoder_list *list)
{
    mp_add_decoder(list, "eac3", "eac3joc",
                   "E-AC-3 JOC decoder");
}

const struct mp_decoder_fns ad_eac3joc = {
    .create = create,
    .add_decoders = add_decoders,
};
