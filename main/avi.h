#ifndef ANTICOPTER_AVI_H
#define ANTICOPTER_AVI_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_timer.h"

#ifndef AVI_MJPEG_DEFAULT_FPS
#define AVI_MJPEG_DEFAULT_FPS 20
#endif

#ifndef AVI_MJPEG_MAX_INDEX_FRAMES
#define AVI_MJPEG_MAX_INDEX_FRAMES 0
#endif

typedef struct {
    uint32_t offset;
    uint32_t size;
} avi_mjpeg_index_entry_t;

typedef struct {
    FILE *file;
    uint32_t width;
    uint32_t height;
    uint32_t requested_fps;
    uint32_t frame_count;
    uint32_t max_frame_size;
    int64_t start_us;
    int64_t end_us;
    bool use_average_fps;
    bool started;
    bool index_overflow;

    long riff_size_pos;
    long avih_flags_pos;
    long avih_usec_per_frame_pos;
    long total_frames_pos;
    long suggested_buffer_size_pos;
    long strh_scale_pos;
    long strh_rate_pos;
    long stream_frames_pos;
    long stream_buffer_size_pos;
    long strf_size_image_pos;
    long movi_size_pos;
    long movi_start;

#if AVI_MJPEG_MAX_INDEX_FRAMES > 0
    avi_mjpeg_index_entry_t index[AVI_MJPEG_MAX_INDEX_FRAMES];
#endif
} avi_mjpeg_writer_t;

static inline esp_err_t avi_mjpeg_write_bytes(FILE *f, const void *buf, size_t len)
{
    return fwrite(buf, 1, len, f) == len ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t avi_mjpeg_write_fourcc(FILE *f, const char s[4])
{
    return avi_mjpeg_write_bytes(f, s, 4);
}

static inline esp_err_t avi_mjpeg_write_u16(FILE *f, uint16_t v)
{
    uint8_t b[2] = {
        (uint8_t)(v & 0xff),
        (uint8_t)((v >> 8) & 0xff),
    };
    return avi_mjpeg_write_bytes(f, b, sizeof(b));
}

static inline esp_err_t avi_mjpeg_write_u32(FILE *f, uint32_t v)
{
    uint8_t b[4] = {
        (uint8_t)(v & 0xff),
        (uint8_t)((v >> 8) & 0xff),
        (uint8_t)((v >> 16) & 0xff),
        (uint8_t)((v >> 24) & 0xff),
    };
    return avi_mjpeg_write_bytes(f, b, sizeof(b));
}

static inline uint32_t avi_mjpeg_effective_fps(const avi_mjpeg_writer_t *avi)
{
    if (!avi->use_average_fps || avi->frame_count < 2) {
        return avi->requested_fps ? avi->requested_fps : AVI_MJPEG_DEFAULT_FPS;
    }

    int64_t duration_us = avi->end_us - avi->start_us;
    if (duration_us <= 0) {
        return avi->requested_fps ? avi->requested_fps : AVI_MJPEG_DEFAULT_FPS;
    }

    uint32_t fps = (uint32_t)(((int64_t)avi->frame_count * 1000000LL +
                               duration_us / 2) /
                              duration_us);
    if (fps < 1) {
        fps = 1;
    } else if (fps > 120) {
        fps = 120;
    }
    return fps;
}

static inline esp_err_t avi_mjpeg_start(avi_mjpeg_writer_t *avi,
                                        FILE *file,
                                        uint32_t width,
                                        uint32_t height,
                                        uint32_t fps)
{
    if (!avi || !file || width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *avi = (avi_mjpeg_writer_t){0};
    avi->file = file;
    avi->width = width;
    avi->height = height;
    avi->requested_fps = fps ? fps : AVI_MJPEG_DEFAULT_FPS;
    avi->use_average_fps = (fps == 0);
    avi->start_us = esp_timer_get_time();
    avi->end_us = avi->start_us;

    uint32_t usec_per_frame = 1000000U / avi->requested_fps;

#define AVI_MJPEG_CHECK(expr)          \
    do {                               \
        esp_err_t err_ = (expr);       \
        if (err_ != ESP_OK) return err_; \
    } while (0)

    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "RIFF"));
    avi->riff_size_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "AVI "));

    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "LIST"));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 192));
    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "hdrl"));

    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "avih"));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 56));
    avi->avih_usec_per_frame_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, usec_per_frame));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    avi->avih_flags_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    avi->total_frames_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 1));
    avi->suggested_buffer_size_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, width));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, height));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));

    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "LIST"));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 116));
    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "strl"));

    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "strh"));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 56));
    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "vids"));
    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "MJPG"));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    avi->strh_scale_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 1));
    avi->strh_rate_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, avi->requested_fps));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    avi->stream_frames_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    avi->stream_buffer_size_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0xffffffffU));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, (uint16_t)width));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, (uint16_t)height));

    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "strf"));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 40));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 40));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, width));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, height));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, 1));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u16(file, 24));
    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "MJPG"));
    avi->strf_size_image_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));

    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "LIST"));
    avi->movi_size_pos = ftell(file);
    AVI_MJPEG_CHECK(avi_mjpeg_write_u32(file, 0));
    AVI_MJPEG_CHECK(avi_mjpeg_write_fourcc(file, "movi"));
    avi->movi_start = ftell(file);
    avi->started = true;

#undef AVI_MJPEG_CHECK

    return ESP_OK;
}

static inline esp_err_t avi_mjpeg_write_frame(avi_mjpeg_writer_t *avi,
                                              const uint8_t *buf,
                                              size_t len)
{
    if (!avi || !avi->started || !avi->file || !buf || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    long chunk_start = ftell(avi->file);
    if (chunk_start < 0) {
        return ESP_FAIL;
    }

#if AVI_MJPEG_MAX_INDEX_FRAMES > 0
    if (avi->frame_count < AVI_MJPEG_MAX_INDEX_FRAMES) {
        avi->index[avi->frame_count].offset =
            (uint32_t)(chunk_start - avi->movi_start);
        avi->index[avi->frame_count].size = (uint32_t)len;
    } else {
        avi->index_overflow = true;
    }
#endif

    esp_err_t err = avi_mjpeg_write_fourcc(avi->file, "00dc");
    if (err != ESP_OK) return err;
    err = avi_mjpeg_write_u32(avi->file, (uint32_t)len);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_write_bytes(avi->file, buf, len);
    if (err != ESP_OK) return err;

    if (len & 1U) {
        if (fputc(0, avi->file) == EOF) {
            return ESP_FAIL;
        }
    }

    avi->frame_count++;
    if (len > avi->max_frame_size) {
        avi->max_frame_size = (uint32_t)len;
    }
    avi->end_us = esp_timer_get_time();

    return ESP_OK;
}

static inline esp_err_t avi_mjpeg_write_idx1(avi_mjpeg_writer_t *avi)
{
#if AVI_MJPEG_MAX_INDEX_FRAMES > 0
    if (avi->index_overflow || avi->frame_count == 0) {
        return ESP_OK;
    }

    esp_err_t err = avi_mjpeg_write_fourcc(avi->file, "idx1");
    if (err != ESP_OK) return err;
    err = avi_mjpeg_write_u32(avi->file, avi->frame_count * 16U);
    if (err != ESP_OK) return err;

    for (uint32_t i = 0; i < avi->frame_count; i++) {
        err = avi_mjpeg_write_fourcc(avi->file, "00dc");
        if (err != ESP_OK) return err;
        err = avi_mjpeg_write_u32(avi->file, 0x10);
        if (err != ESP_OK) return err;
        err = avi_mjpeg_write_u32(avi->file, avi->index[i].offset);
        if (err != ESP_OK) return err;
        err = avi_mjpeg_write_u32(avi->file, avi->index[i].size);
        if (err != ESP_OK) return err;
    }
#else
    (void)avi;
#endif

    return ESP_OK;
}

static inline esp_err_t avi_mjpeg_patch_u32(FILE *file, long pos, uint32_t value)
{
    if (fseek(file, pos, SEEK_SET) != 0) {
        return ESP_FAIL;
    }
    return avi_mjpeg_write_u32(file, value);
}

static inline esp_err_t avi_mjpeg_finish(avi_mjpeg_writer_t *avi)
{
    if (!avi || !avi->started || !avi->file) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = avi->file;
    long movi_end = ftell(file);
    if (movi_end < 0) {
        return ESP_FAIL;
    }

    esp_err_t err = avi_mjpeg_write_idx1(avi);
    if (err != ESP_OK) {
        return err;
    }

    long file_end = ftell(file);
    if (file_end < 0) {
        return ESP_FAIL;
    }

    uint32_t fps = avi_mjpeg_effective_fps(avi);
    uint32_t usec_per_frame = 1000000U / fps;
    uint32_t riff_size = (uint32_t)(file_end - 8);
    uint32_t movi_size = (uint32_t)(movi_end - avi->movi_size_pos - 4);
    uint32_t has_index = 0;

#if AVI_MJPEG_MAX_INDEX_FRAMES > 0
    if (!avi->index_overflow && avi->frame_count > 0) {
        has_index = 0x10;
    }
#endif

    err = avi_mjpeg_patch_u32(file, avi->riff_size_pos, riff_size);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->avih_usec_per_frame_pos, usec_per_frame);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->avih_flags_pos, has_index);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->total_frames_pos, avi->frame_count);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->suggested_buffer_size_pos, avi->max_frame_size);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->strh_scale_pos, 1);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->strh_rate_pos, fps);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->stream_frames_pos, avi->frame_count);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->stream_buffer_size_pos, avi->max_frame_size);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->strf_size_image_pos, avi->max_frame_size);
    if (err != ESP_OK) return err;
    err = avi_mjpeg_patch_u32(file, avi->movi_size_pos, movi_size);
    if (err != ESP_OK) return err;

    if (fseek(file, file_end, SEEK_SET) != 0) {
        return ESP_FAIL;
    }
    if (fflush(file) != 0) {
        return ESP_FAIL;
    }

    avi->started = false;
    avi->file = NULL;
    return ESP_OK;
}

#endif