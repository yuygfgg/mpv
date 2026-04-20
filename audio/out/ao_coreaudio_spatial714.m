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

#include <stddef.h>
#include <string.h>
#include <TargetConditionals.h>

#include <libavutil/mathematics.h>

#include "ao.h"
#include "internal.h"
#include "audio/format.h"
#include "audio/chmap.h"
#include "osdep/timer.h"
#include "common/msg.h"
#include "ao_coreaudio_utils.h"
#include "osdep/mac/compat.h"

#if TARGET_OS_IPHONE
#import <AVFoundation/AVFoundation.h>
#endif

#define IDLE_TIME 7 * NSEC_PER_SEC
#define STARMINE_INPUT_BUS 0
#define STARMINE_OUTPUT_BUS 0

struct priv {
    struct coreaudio_cb_sem sem;

#if !TARGET_OS_IPHONE
    AudioDeviceID device;
#endif
    AudioUnit output_unit;
    AudioUnit spatial_mixer;

    uint64_t hw_latency_ns;

    dispatch_block_t idle_work;
    dispatch_queue_t queue;

#if !TARGET_OS_IPHONE
    int hotplug_cb_registration_times;
#endif
};

static const struct mp_chmap spatial714_layout = MP_CHMAP12(FL, FR, FC, LFE,
                                                            BL, BR, SL, SR,
                                                            TFL, TFR, TBL, TBR);

static bool register_hotplug_cb(struct ao *ao);
static void unregister_hotplug_cb(struct ao *ao);

static bool has_explicit_device(struct ao *ao)
{
#if TARGET_OS_IPHONE
    return false;
#else
    return ao->device && ao->device[0];
#endif
}

static AudioStreamBasicDescription make_float_asbd(int samplerate, int channels,
                                                  bool interleaved)
{
    UInt32 bytes_per_sample = sizeof(float);
    UInt32 bytes_per_frame = interleaved ? channels * bytes_per_sample
                                         : bytes_per_sample;
    UInt32 format_flags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;

    if (!interleaved)
        format_flags |= kAudioFormatFlagIsNonInterleaved;

    return (AudioStreamBasicDescription) {
        .mSampleRate       = samplerate,
        .mFormatID         = kAudioFormatLinearPCM,
        .mFormatFlags      = format_flags,
        .mBytesPerPacket   = bytes_per_frame,
        .mFramesPerPacket  = 1,
        .mBytesPerFrame    = bytes_per_frame,
        .mChannelsPerFrame = channels,
        .mBitsPerChannel   = 8 * bytes_per_sample,
    };
}

static size_t tag_layout_size(void)
{
    return offsetof(AudioChannelLayout, mChannelDescriptions);
}

static AudioChannelLayout make_tag_layout(AudioChannelLayoutTag tag)
{
    return (AudioChannelLayout) {
        .mChannelLayoutTag = tag,
    };
}

static void zero_buffers(AudioBufferList *buffer_list)
{
    if (!buffer_list)
        return;

    for (UInt32 n = 0; n < buffer_list->mNumberBuffers; n++) {
        AudioBuffer *buffer = &buffer_list->mBuffers[n];
        if (buffer->mData && buffer->mDataByteSize)
            memset(buffer->mData, 0, buffer->mDataByteSize);
    }
}

static uint64_t get_unit_latency_ns(struct ao *ao, AudioUnit unit,
                                    const char *name)
{
    Float64 latency_sec = 0.0;
    uint32_t size = sizeof(latency_sec);
    OSStatus err = AudioUnitGetProperty(unit, kAudioUnitProperty_Latency,
                                        kAudioUnitScope_Global, 0,
                                        &latency_sec, &size);
    if (err != noErr) {
        MP_VERBOSE(ao, "%s latency unavailable (%s/%d)\n",
                   name, mp_tag_str(err), (int)err);
        return 0;
    }

    uint64_t latency_ns = MP_TIME_S_TO_NS(latency_sec);
    MP_VERBOSE(ao, "%s latency [ns]: %" PRIu64 "\n", name, latency_ns);
    return latency_ns;
}

static int64_t ca_get_hardware_latency(struct ao *ao)
{
    struct priv *p = ao->priv;
    uint64_t mixer_latency_ns = get_unit_latency_ns(ao, p->spatial_mixer,
                                                    "spatial mixer");
    uint64_t output_latency_ns = get_unit_latency_ns(ao, p->output_unit,
                                                     "output unit");

#if TARGET_OS_IPHONE
    AVAudioSession *instance = AVAudioSession.sharedInstance;
    uint64_t device_latency_ns = MP_TIME_S_TO_NS(instance.outputLatency);
#else
    uint64_t device_latency_ns = ca_get_device_latency_ns(ao, p->device);
#endif

    MP_VERBOSE(ao, "device latency [ns]: %" PRIu64 "\n", device_latency_ns);

    return mixer_latency_ns + output_latency_ns + device_latency_ns;
}

static OSStatus render_cb_spatial_input(void *ctx,
                                        AudioUnitRenderActionFlags *aflags,
                                        const AudioTimeStamp *ts,
                                        UInt32 bus, UInt32 frames,
                                        AudioBufferList *buffer_list)
{
    (void)aflags;
    (void)bus;

    struct ao *ao = ctx;
    struct priv *p = ao->priv;
    void *planes[MP_NUM_CHANNELS] = {0};

    if (!buffer_list || buffer_list->mNumberBuffers < ao->num_planes) {
        zero_buffers(buffer_list);
        return noErr;
    }

    for (int n = 0; n < ao->num_planes; n++)
        planes[n] = buffer_list->mBuffers[n].mData;

    int64_t end = mp_time_ns();
    end += p->hw_latency_ns + ca_get_latency(ts) + ca_frames_to_ns(ao, frames);
    ao_read_data(ao, planes, frames, end, NULL, true, true);
    return noErr;
}

static OSStatus render_cb_output(void *ctx, AudioUnitRenderActionFlags *aflags,
                                 const AudioTimeStamp *ts, UInt32 bus,
                                 UInt32 frames, AudioBufferList *buffer_list)
{
    (void)bus;

    struct ao *ao = ctx;
    struct priv *p = ao->priv;
    OSStatus err = AudioUnitRender(p->spatial_mixer, aflags, ts,
                                   STARMINE_OUTPUT_BUS, frames, buffer_list);
    if (err != noErr) {
        MP_ERR(ao, "spatial mixer render failed (%s/%d)\n",
               mp_tag_str(err), (int)err);
        zero_buffers(buffer_list);
    }

    return noErr;
}

static int get_volume(struct ao *ao, float *vol)
{
#if TARGET_OS_IPHONE
    (void)ao;
    (void)vol;
    return CONTROL_UNKNOWN;
#else
    struct priv *p = ao->priv;
    float auvol = 0.0;
    OSStatus err = AudioUnitGetParameter(p->output_unit, kHALOutputParam_Volume,
                                         kAudioUnitScope_Global, 0, &auvol);
    CHECK_CA_ERROR("could not get HAL output volume");
    *vol = auvol * 100.0;
    return CONTROL_TRUE;
coreaudio_error:
    return CONTROL_ERROR;
#endif
}

static int set_volume(struct ao *ao, float *vol)
{
#if TARGET_OS_IPHONE
    (void)ao;
    (void)vol;
    return CONTROL_UNKNOWN;
#else
    struct priv *p = ao->priv;
    float auvol = *vol / 100.0;
    OSStatus err = AudioUnitSetParameter(p->output_unit, kHALOutputParam_Volume,
                                         kAudioUnitScope_Global, 0, auvol, 0);
    CHECK_CA_ERROR("could not set HAL output volume");
    return CONTROL_TRUE;
coreaudio_error:
    return CONTROL_ERROR;
#endif
}

static int control(struct ao *ao, enum aocontrol cmd, void *arg)
{
    switch (cmd) {
    case AOCONTROL_GET_VOLUME:
        return get_volume(ao, arg);
    case AOCONTROL_SET_VOLUME:
        return set_volume(ao, arg);
    }

    return CONTROL_UNKNOWN;
}

static bool reinit_device(struct ao *ao)
{
#if TARGET_OS_IPHONE
    (void)ao;
    return true;
#else
    struct priv *p = ao->priv;
    OSStatus err = ca_select_device(ao, ao->device, &p->device);
    CHECK_CA_ERROR("failed to select device");
    return true;
coreaudio_error:
    return false;
#endif
}

static void log_optional_property_failure(struct ao *ao, const char *name,
                                          OSStatus err)
{
    if (err == noErr)
        return;
    MP_VERBOSE(ao, "%s unavailable (%s/%d)\n", name, mp_tag_str(err), (int)err);
}

#if TARGET_OS_IPHONE
static bool configure_ios_audio_session(struct ao *ao)
{
    AVAudioSession *instance = AVAudioSession.sharedInstance;
    AVAudioSessionCategoryOptions options = 0;
    NSError *error = nil;

    if (!(ao->init_flags & AO_INIT_EXCLUSIVE))
        options |= AVAudioSessionCategoryOptionMixWithOthers;

    if (![instance setCategory:AVAudioSessionCategoryPlayback
                   mode:AVAudioSessionModeMoviePlayback
                   options:options
                     error:&error]) {
        MP_ERR(ao, "failed to configure AVAudioSession category: %s\n",
               error.localizedDescription.UTF8String);
        return false;
    }

    if ([instance respondsToSelector:@selector(setPreferredOutputNumberOfChannels:error:)])
        [instance setPreferredOutputNumberOfChannels:2 error:nil];

    if (![instance setActive:YES error:&error]) {
        MP_ERR(ao, "failed to activate AVAudioSession: %s\n",
               error.localizedDescription.UTF8String);
        return false;
    }

    return true;
}

static void teardown_ios_audio_session(void)
{
    [AVAudioSession.sharedInstance
        setActive:NO
        withOptions:AVAudioSessionSetActiveOptionNotifyOthersOnDeactivation
        error:nil];
}
#endif

static bool init_spatial_mixer(struct ao *ao)
{
    struct priv *p = ao->priv;
    AudioStreamBasicDescription input_asbd = make_float_asbd(ao->samplerate, 12, false);
    AudioStreamBasicDescription output_asbd = make_float_asbd(ao->samplerate, 2, false);
    AudioChannelLayout input_layout = make_tag_layout(kAudioChannelLayoutTag_Atmos_7_1_4);
    AudioChannelLayout output_layout = make_tag_layout(kAudioChannelLayoutTag_Stereo);
    UInt32 element_count = 1;
    UInt32 algorithm = kSpatializationAlgorithm_UseOutputType;
    UInt32 source_mode = kSpatialMixerSourceMode_AmbienceBed;
    UInt32 output_type = kSpatialMixerOutputType_Headphones;
    AURenderCallbackStruct render_cb = {
        .inputProc = render_cb_spatial_input,
        .inputProcRefCon = ao,
    };
    OSStatus err;

    AudioComponentDescription desc = (AudioComponentDescription) {
        .componentType = kAudioUnitType_Mixer,
        .componentSubType = kAudioUnitSubType_SpatialMixer,
        .componentManufacturer = kAudioUnitManufacturer_Apple,
        .componentFlags = 0,
        .componentFlagsMask = 0,
    };

    AudioComponent comp = AudioComponentFindNext(NULL, &desc);
    if (!comp) {
        MP_ERR(ao, "unable to find spatial mixer component\n");
        return false;
    }

    err = AudioComponentInstanceNew(comp, &p->spatial_mixer);
    CHECK_CA_ERROR("unable to open spatial mixer");

    err = AudioUnitSetProperty(p->spatial_mixer, kAudioUnitProperty_ElementCount,
                               kAudioUnitScope_Input, 0, &element_count,
                               sizeof(element_count));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer input bus count");

    err = AudioUnitSetProperty(p->spatial_mixer,
                               kAudioUnitProperty_SpatialMixerOutputType,
                               kAudioUnitScope_Global, 0,
                               &output_type, sizeof(output_type));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer output type");

    ca_print_asbd(ao, "spatial mixer input format:", &input_asbd);
    err = AudioUnitSetProperty(p->spatial_mixer, kAudioUnitProperty_StreamFormat,
                               kAudioUnitScope_Input, STARMINE_INPUT_BUS,
                               &input_asbd, sizeof(input_asbd));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer input format");

    err = AudioUnitSetProperty(p->spatial_mixer,
                               kAudioUnitProperty_AudioChannelLayout,
                               kAudioUnitScope_Input, STARMINE_INPUT_BUS,
                               &input_layout, tag_layout_size());
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer input layout");

    ca_print_asbd(ao, "spatial mixer output format:", &output_asbd);
    err = AudioUnitSetProperty(p->spatial_mixer, kAudioUnitProperty_StreamFormat,
                               kAudioUnitScope_Output, STARMINE_OUTPUT_BUS,
                               &output_asbd, sizeof(output_asbd));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer output format");

    err = AudioUnitSetProperty(p->spatial_mixer,
                               kAudioUnitProperty_AudioChannelLayout,
                               kAudioUnitScope_Output, STARMINE_OUTPUT_BUS,
                               &output_layout, tag_layout_size());
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer output layout");

    err = AudioUnitSetProperty(p->spatial_mixer,
                               kAudioUnitProperty_SetRenderCallback,
                               kAudioUnitScope_Input, STARMINE_INPUT_BUS,
                               &render_cb, sizeof(render_cb));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer render callback");

    err = AudioUnitSetProperty(p->spatial_mixer,
                               kAudioUnitProperty_SpatializationAlgorithm,
                               kAudioUnitScope_Input, STARMINE_INPUT_BUS,
                               &algorithm, sizeof(algorithm));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatialization algorithm");

    err = AudioUnitSetProperty(p->spatial_mixer,
                               kAudioUnitProperty_SpatialMixerSourceMode,
                               kAudioUnitScope_Input, STARMINE_INPUT_BUS,
                               &source_mode, sizeof(source_mode));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set spatial mixer source mode");

#if defined(__MAC_OS_X_VERSION_MAX_ALLOWED) && __MAC_OS_X_VERSION_MAX_ALLOWED >= 120300
    {
        UInt32 enable_head_tracking = 1;
        err = AudioUnitSetProperty(p->spatial_mixer,
                                   kAudioUnitProperty_SpatialMixerEnableHeadTracking,
                                   kAudioUnitScope_Global, 0,
                                   &enable_head_tracking,
                                   sizeof(enable_head_tracking));
        log_optional_property_failure(ao, "head tracking", err);
    }
#endif

#if defined(__MAC_OS_X_VERSION_MAX_ALLOWED) && __MAC_OS_X_VERSION_MAX_ALLOWED >= 130000
    {
        UInt32 personalized_hrtf = kSpatialMixerPersonalizedHRTFMode_Auto;
        err = AudioUnitSetProperty(p->spatial_mixer,
                                   kAudioUnitProperty_SpatialMixerPersonalizedHRTFMode,
                                   kAudioUnitScope_Global, 0,
                                   &personalized_hrtf,
                                   sizeof(personalized_hrtf));
        log_optional_property_failure(ao, "personalized HRTF", err);
    }
#endif

    err = AudioUnitInitialize(p->spatial_mixer);
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to initialize spatial mixer");

#if defined(__MAC_OS_X_VERSION_MAX_ALLOWED) && __MAC_OS_X_VERSION_MAX_ALLOWED >= 140000
    {
        UInt32 using_personalized = 0;
        uint32_t size = sizeof(using_personalized);
        err = AudioUnitGetProperty(p->spatial_mixer,
                                   kAudioUnitProperty_SpatialMixerAnyInputIsUsingPersonalizedHRTF,
                                   kAudioUnitScope_Global, 0,
                                   &using_personalized, &size);
        if (err == noErr) {
            MP_VERBOSE(ao, "personalized HRTF active: %s\n",
                       using_personalized ? "yes" : "no");
        } else {
            log_optional_property_failure(ao,
                                          "personalized HRTF activity query",
                                          err);
        }
    }
#endif

    MP_VERBOSE(ao, "Spatial Mixer configured for Atmos 7.1.4 -> stereo headphones\n");
    return true;

coreaudio_error_component:
    if (p->spatial_mixer) {
        AudioUnitUninitialize(p->spatial_mixer);
        AudioComponentInstanceDispose(p->spatial_mixer);
        p->spatial_mixer = NULL;
    }
coreaudio_error:
    return false;
}

static bool init_output_unit(struct ao *ao)
{
    struct priv *p = ao->priv;
    AudioStreamBasicDescription asbd = make_float_asbd(ao->samplerate, 2, false);
    AURenderCallbackStruct render_cb = {
        .inputProc = render_cb_output,
        .inputProcRefCon = ao,
    };
    OSStatus err;

    AudioComponentDescription desc = (AudioComponentDescription) {
        .componentType = kAudioUnitType_Output,
#if TARGET_OS_IPHONE
        .componentSubType = kAudioUnitSubType_RemoteIO,
#else
        .componentSubType = has_explicit_device(ao) ?
                            kAudioUnitSubType_HALOutput :
                            kAudioUnitSubType_DefaultOutput,
#endif
        .componentManufacturer = kAudioUnitManufacturer_Apple,
        .componentFlags = 0,
        .componentFlagsMask = 0,
    };

    AudioComponent comp = AudioComponentFindNext(NULL, &desc);
    if (!comp) {
        MP_ERR(ao, "unable to find audio output component\n");
        return false;
    }

    err = AudioComponentInstanceNew(comp, &p->output_unit);
    CHECK_CA_ERROR("unable to open audio output component");

#if !TARGET_OS_IPHONE
    if (has_explicit_device(ao)) {
        err = AudioUnitSetProperty(p->output_unit,
                                   kAudioOutputUnitProperty_CurrentDevice,
                                   kAudioUnitScope_Global, 0,
                                   &p->device, sizeof(p->device));
        CHECK_CA_ERROR_L(coreaudio_error_component, "can't link output unit to selected device");
    }
#endif

    ca_print_asbd(ao, "output unit input format:", &asbd);
    err = AudioUnitSetProperty(p->output_unit, kAudioUnitProperty_StreamFormat,
                               kAudioUnitScope_Input, STARMINE_OUTPUT_BUS,
                               &asbd, sizeof(asbd));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set output unit input format");

#if !TARGET_OS_IPHONE
    AudioChannelLayout layout = make_tag_layout(kAudioChannelLayoutTag_Stereo);
    err = AudioUnitSetProperty(p->output_unit,
                               kAudioUnitProperty_AudioChannelLayout,
                               kAudioUnitScope_Input, STARMINE_OUTPUT_BUS,
                               &layout, tag_layout_size());
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set output unit input layout");
#endif

    err = AudioUnitSetProperty(p->output_unit,
                               kAudioUnitProperty_SetRenderCallback,
                               kAudioUnitScope_Input, STARMINE_OUTPUT_BUS,
                               &render_cb, sizeof(render_cb));
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to set output render callback");

    err = AudioUnitInitialize(p->output_unit);
    CHECK_CA_ERROR_L(coreaudio_error_component, "unable to initialize output unit");

    return true;

coreaudio_error_component:
    if (p->output_unit) {
        AudioUnitUninitialize(p->output_unit);
        AudioComponentInstanceDispose(p->output_unit);
        p->output_unit = NULL;
    }
coreaudio_error:
    return false;
}

static void reinit_latency(struct ao *ao)
{
    struct priv *p = ao->priv;
    p->hw_latency_ns = ca_get_hardware_latency(ao);
}

static void stop(struct ao *ao)
{
    struct priv *p = ao->priv;
    OSStatus err = AudioOutputUnitStop(p->output_unit);
    CHECK_CA_WARN("can't stop output unit");
}

static void cancel_and_release_idle_work(struct priv *p)
{
    if (!p->idle_work)
        return;

    dispatch_block_cancel(p->idle_work);
    Block_release(p->idle_work);
    p->idle_work = NULL;
}

static void stop_after_idle_time(struct ao *ao)
{
    struct priv *p = ao->priv;

    cancel_and_release_idle_work(p);

    p->idle_work = dispatch_block_create(0, ^{
        MP_VERBOSE(ao, "Stopping coreaudio_spatial714 output unit due to idle timeout\n");
        stop(ao);
    });

    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, IDLE_TIME),
                   p->queue, p->idle_work);
}

static void _reset(void *_ao)
{
    struct ao *ao = _ao;
    struct priv *p = ao->priv;
    OSStatus err = AudioUnitReset(p->spatial_mixer, kAudioUnitScope_Global, 0);
    CHECK_CA_WARN("can't reset spatial mixer");

    err = AudioUnitReset(p->output_unit, kAudioUnitScope_Global, 0);
    CHECK_CA_WARN("can't reset output unit");

    stop_after_idle_time(ao);
}

static void reset(struct ao *ao)
{
    struct priv *p = ao->priv;
    if (!p->queue)
        return;
    dispatch_sync_f(p->queue, ao, &_reset);
}

static void _start(void *_ao)
{
    struct ao *ao = _ao;
    struct priv *p = ao->priv;

    if (p->idle_work)
        dispatch_block_cancel(p->idle_work);

#if TARGET_OS_IPHONE
    reinit_latency(ao);
#endif
    OSStatus err = AudioOutputUnitStart(p->output_unit);
    CHECK_CA_WARN("can't start output unit");
}

static void start(struct ao *ao)
{
    struct priv *p = ao->priv;
    if (!p->queue)
        return;
    dispatch_sync_f(p->queue, ao, &_start);
}

static int init(struct ao *ao)
{
    struct priv *p = ao->priv;

    if (!af_fmt_is_pcm(ao->format) || (ao->init_flags & AO_INIT_EXCLUSIVE)) {
        MP_VERBOSE(ao, "redirecting to coreaudio_exclusive\n");
        ao->redirect = "coreaudio_exclusive";
        return CONTROL_ERROR;
    }

    if (!mp_chmap_equals(&ao->channels, &spatial714_layout)) {
        MP_VERBOSE(ao, "coreaudio_spatial714 only accepts 7.1.4 input, redirecting to coreaudio\n");
        ao->redirect = "coreaudio";
        return CONTROL_ERROR;
    }

    ao->channels = spatial714_layout;
    ao->format = AF_FORMAT_FLOATP;

#if TARGET_OS_IPHONE
    if (!configure_ios_audio_session(ao))
        goto coreaudio_error;
#endif

    if (!reinit_device(ao))
        goto coreaudio_error;

    if (!register_hotplug_cb(ao))
        goto coreaudio_error;

    if (!init_spatial_mixer(ao))
        goto coreaudio_error;

    if (!init_output_unit(ao))
        goto coreaudio_error;

    reinit_latency(ao);
    ao->device_buffer = av_rescale(p->hw_latency_ns, ao->samplerate,
                                   1000000000) * 2;

    p->queue = dispatch_queue_create("io.mpv.coreaudio_spatial714_idle",
                                     DISPATCH_QUEUE_SERIAL);
    if (!p->queue)
        goto coreaudio_error;

    return CONTROL_OK;

coreaudio_error:
    if (p->queue) {
        dispatch_release(p->queue);
        p->queue = NULL;
    }
    if (p->output_unit) {
        AudioOutputUnitStop(p->output_unit);
        AudioUnitUninitialize(p->output_unit);
        AudioComponentInstanceDispose(p->output_unit);
        p->output_unit = NULL;
    }
    if (p->spatial_mixer) {
        AudioUnitUninitialize(p->spatial_mixer);
        AudioComponentInstanceDispose(p->spatial_mixer);
        p->spatial_mixer = NULL;
    }
    unregister_hotplug_cb(ao);
#if TARGET_OS_IPHONE
    teardown_ios_audio_session();
#endif
    return CONTROL_ERROR;
}

static void uninit(struct ao *ao)
{
    struct priv *p = ao->priv;

    if (p->queue) {
        dispatch_sync(p->queue, ^{
            cancel_and_release_idle_work(p);
        });
        dispatch_release(p->queue);
        p->queue = NULL;
    }

    if (p->output_unit) {
        AudioOutputUnitStop(p->output_unit);
        AudioUnitUninitialize(p->output_unit);
        AudioComponentInstanceDispose(p->output_unit);
        p->output_unit = NULL;
    }

    if (p->spatial_mixer) {
        AudioUnitUninitialize(p->spatial_mixer);
        AudioComponentInstanceDispose(p->spatial_mixer);
        p->spatial_mixer = NULL;
    }

    unregister_hotplug_cb(ao);
#if TARGET_OS_IPHONE
    teardown_ios_audio_session();
#endif
}

#if !TARGET_OS_IPHONE
static OSStatus hotplug_cb(AudioObjectID id, UInt32 naddr,
                           const AudioObjectPropertyAddress addr[],
                           void *ctx)
{
    (void)id;
    (void)naddr;
    (void)addr;

    struct ao *ao = ctx;
    struct priv *p = ao->priv;
    MP_VERBOSE(ao, "Handling potential coreaudio_spatial714 hotplug event...\n");
    reinit_device(ao);
    if (p->output_unit && p->spatial_mixer)
        reinit_latency(ao);
    ao_hotplug_event(ao);
    return noErr;
}

static uint32_t hotplug_properties[] = {
    kAudioHardwarePropertyDevices,
    kAudioHardwarePropertyDefaultOutputDevice,
};

static int hotplug_init(struct ao *ao)
{
    if (!reinit_device(ao))
        return -1;
    if (!register_hotplug_cb(ao))
        return -1;
    return 0;
}

static void hotplug_uninit(struct ao *ao)
{
    unregister_hotplug_cb(ao);
}

static bool register_hotplug_cb(struct ao *ao)
{
    struct priv *p = ao->priv;

    if (p->hotplug_cb_registration_times++)
        return true;

    OSStatus err = noErr;
    for (int i = 0; i < MP_ARRAY_SIZE(hotplug_properties); i++) {
        AudioObjectPropertyAddress addr = {
            hotplug_properties[i],
            kAudioObjectPropertyScopeGlobal,
            kAudioObjectPropertyElementMain,
        };
        err = AudioObjectAddPropertyListener(kAudioObjectSystemObject, &addr,
                                             hotplug_cb, ao);
        if (err != noErr) {
            MP_ERR(ao, "failed to set device listener %s (%s)\n",
                   mp_tag_str(hotplug_properties[i]), mp_tag_str(err));
            goto coreaudio_error;
        }
    }

    return true;

coreaudio_error:
    return false;
}

static void unregister_hotplug_cb(struct ao *ao)
{
    struct priv *p = ao->priv;

    if (!p->hotplug_cb_registration_times)
        return;
    if (--p->hotplug_cb_registration_times)
        return;

    OSStatus err = noErr;
    for (int i = 0; i < MP_ARRAY_SIZE(hotplug_properties); i++) {
        AudioObjectPropertyAddress addr = {
            hotplug_properties[i],
            kAudioObjectPropertyScopeGlobal,
            kAudioObjectPropertyElementMain,
        };
        err = AudioObjectRemovePropertyListener(kAudioObjectSystemObject, &addr,
                                                hotplug_cb, ao);
        if (err != noErr) {
            MP_ERR(ao, "failed to remove device listener %s (%s)\n",
                   mp_tag_str(hotplug_properties[i]), mp_tag_str(err));
        }
    }
}
#else
static bool register_hotplug_cb(struct ao *ao)
{
    (void)ao;
    return true;
}

static void unregister_hotplug_cb(struct ao *ao)
{
    (void)ao;
}
#endif

#define OPT_BASE_STRUCT struct priv

const struct ao_driver audio_out_coreaudio_spatial714 = {
    .description    = "CoreAudio Spatial Mixer 7.1.4 output",
    .name           = "coreaudio_spatial714",
    .init           = init,
    .uninit         = uninit,
    .control        = control,
    .reset          = reset,
    .start          = start,
#if !TARGET_OS_IPHONE
    .hotplug_init   = hotplug_init,
    .hotplug_uninit = hotplug_uninit,
    .list_devs      = ca_get_device_list,
#endif
    .priv_size      = sizeof(struct priv),
    .priv_defaults  = &(const struct priv){
        .sem = (struct coreaudio_cb_sem){
            .mutex = MP_STATIC_MUTEX_INITIALIZER,
            .cond = MP_STATIC_COND_INITIALIZER,
        },
    },
};
