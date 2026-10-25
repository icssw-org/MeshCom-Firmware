// zstream.h -- push-model streaming inflate for the Safeboot web OTA upload
//
// Header-only, Arduino-free, no malloc, no printf. The upload handler
// (ElegantOTA.cpp) receives the body in chunks it does not control, so unlike
// fwInflateStream() (fw_apply.h), which pulls compressed bytes from flash, this
// inflater is pushed: every call to push() takes one input chunk, drives the
// decompressor until that chunk is consumed, and hands each produced output
// span to the sink as it appears.
//
// Same Dec convention as fwInflateStream():
//
//   int step(const uint8_t *in, size_t *inSize, uint8_t *dictStart,
//            uint8_t *outNext, size_t *outSize, bool moreInput)
//
// is one tinfl_decompress() call (negative = error, ZS_ST_* otherwise), with
// the output window running from outNext to the end of the circular dictionary
// (dictSize a power of two, 32768 for tinfl). push() always announces more
// input (the stream end is detected by DONE, not by running out of bytes), so
// a stream that stops early simply never reports done() -- the caller maps
// that to "incomplete upload" at the final frame.
//
// Wr: bool write(const uint8_t *src, size_t n)   strictly sequential output
//
// Memory (the 32 KB dictionary, the decompressor state) belongs to the caller.

#ifndef ZSTREAM_H
#define ZSTREAM_H

#include <stddef.h>
#include <stdint.h>

// tinfl_status values (miniz.h), kept here so the loop does not need miniz.
// ElegantOTA.cpp static_asserts them against the real enum.
#define ZS_ST_DONE 0
#define ZS_ST_NEEDS_MORE_INPUT 1
#define ZS_ST_HAS_MORE_OUTPUT 2

enum ZsResult : uint8_t
{
    ZS_MORE = 0, // chunk consumed, the stream is not finished yet
    ZS_DONE,     // the stream is finished (this chunk or an earlier one)
    ZS_DATA,     // the decompressor reported an error (bad header, bad stream, adler32)
    ZS_WRITE,    // the sink failed
    ZS_PARAM     // dictSize not a power of two, no dictionary
};

template <class Dec>
class ZStreamInflater
{
  public:
    // False (and the inflater stays unusable) for a bad dictionary.
    bool init(Dec *dec, uint8_t *dict, size_t dictSize)
    {
        dec_ = nullptr;
        if (dec == nullptr || dict == nullptr || dictSize == 0 || (dictSize & (dictSize - 1)) != 0)
            return false;
        dec_ = dec;
        dict_ = dict;
        dictSize_ = dictSize;
        dictOfs_ = 0;
        total_ = 0;
        state_ = ZS_MORE;
        return true;
    }

    // Feeds one input chunk. Once the stream is DONE every further byte is
    // ignored; after an error the same error is returned again.
    template <class Wr>
    ZsResult push(const uint8_t *in, size_t len, Wr &wr)
    {
        if (dec_ == nullptr)
            return ZS_PARAM;
        if (state_ != ZS_MORE)
            return state_;
        if (len == 0)
            return ZS_MORE;

        size_t pos = 0;
        uint8_t idle = 0; // consecutive calls without any progress
        for (;;)
        {
            size_t inSize = len - pos;
            size_t outSize = dictSize_ - dictOfs_;
            const int st = dec_->step(in + pos, &inSize, dict_, dict_ + dictOfs_, &outSize, true);
            pos += inSize;

            if (outSize > 0)
            {
                if (!wr.write(dict_ + dictOfs_, outSize))
                    return state_ = ZS_WRITE;
                total_ += (uint32_t)outSize;
                dictOfs_ = (dictOfs_ + outSize) & (dictSize_ - 1);
            }

            if (st == ZS_ST_DONE)
                return state_ = ZS_DONE; // bytes after the stream end are ignored
            if (st < 0)
                return state_ = ZS_DATA;
            // Input used up and the decoder wants more: wait for the next chunk.
            // (HAS_MORE_OUTPUT with input left, or with the dictionary just
            // flushed, loops: the window has wrapped and there is room again.)
            if (st == ZS_ST_NEEDS_MORE_INPUT && pos >= len)
                return ZS_MORE;

            // A decompressor that neither consumes nor produces would spin forever.
            if (inSize == 0 && outSize == 0)
            {
                if (++idle >= 3)
                    return state_ = ZS_DATA;
            }
            else
                idle = 0;
        }
    }

    bool done() const { return state_ == ZS_DONE; }
    ZsResult state() const { return state_; }
    uint32_t outTotal() const { return total_; } // inflated bytes handed to the sink

  private:
    Dec *dec_ = nullptr;
    uint8_t *dict_ = nullptr;
    size_t dictSize_ = 0;
    size_t dictOfs_ = 0;
    uint32_t total_ = 0;
    ZsResult state_ = ZS_PARAM;
};

#endif // ZSTREAM_H
