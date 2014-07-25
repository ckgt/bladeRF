/*
 * This test simply receives available and checks that there are no gaps/jumps
 * in the expected timestamp
 *
 * This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2014 Nuand LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <libbladeRF.h>
#include "rel_assert.h"

/* TODO Make sync params configurable */
#define NUM_BUFFERS 16
#define NUM_XFERS   8
#define BUF_SIZE    (64 * 1024)
#define TIMEOUT_MS  1000

struct test_case {
    uint64_t gap;
    unsigned int iterations;
};
static const struct test_case tests[] = {
#if 0
    { 1,    10000000 },
    { 2,    10000000 },
    { 128,  10000000 },
    { 256,  5000000 },
    { 512,  5000000 },
#endif
    { 1023, 10000 },
    { 1024, 10000 },
    { 1025, 10000 },
#if 0
    { 2048, 5000 },
    { 3172, 5000 },
    { 4096, 2500 },
    { 8192, 2500 },
    { 16 * 1024, 1000 },
    { 32 * 1024, 1000 },
    { 64 * 1024, 1000 },
#endif
};
static const size_t num_tests = sizeof(tests) / sizeof(tests[0]);

static int run(struct bladerf *dev, int16_t *samples, const struct test_case *t)
{
    int status;
    struct bladerf_metadata meta;
    uint64_t timestamp;
    unsigned int i;
    bool pass = true;

    assert(t->gap <= BUF_SIZE);

    /* Clear out flags, request timestamp = 0 "Any" */
    memset(&meta, 0, sizeof(meta));

    status = bladerf_sync_config(dev,
                                 BLADERF_MODULE_RX,
                                 BLADERF_FORMAT_SC16_Q11_META,
                                 NUM_BUFFERS,
                                 BUF_SIZE,
                                 NUM_XFERS,
                                 TIMEOUT_MS);

    if (status != 0) {
        fprintf(stderr, "Failed to configure RX sync i/f: %s\n",
                bladerf_strerror(status));
        return status;
    }

    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, true);
    if (status != 0) {
        fprintf(stderr, "Failed to enable RX module: %s\n",
                bladerf_strerror(status));

        goto out;
    }

    printf("\nTest Case: Read size=%"PRIu64" samples, %u iterations\n",
            t->gap, t->iterations);
    printf("--------------------------------------------------------\n");

    /* Initial read to get a starting timestamp */
    status = bladerf_sync_rx(dev, samples, t->gap, &meta, TIMEOUT_MS);
    if (status != 0) {
        fprintf(stderr, "Intial RX failed: %s\n", bladerf_strerror(status));
        goto out;
    }

    printf("Initial timestamp: 0x%016"PRIx64"\n", meta.timestamp);
    printf("Initial status:    0x%08"PRIu32"\n", meta.status);

    for (i = 0; i < t->iterations && status == 0 && pass; i++) {
        /* Calculate the next expected timestamp
         * FIXME We need to change the FPGA to remove the 2x factor */
        timestamp = meta.timestamp + 2 * t->gap;

        /* Reset metadata timestamp value to indicate that we we want
         * whatever is available */
        meta.timestamp = 0;

        status = bladerf_sync_rx(dev, samples, t->gap, &meta, TIMEOUT_MS);
        if (status != 0) {
            fprintf(stderr, "RX %u failed: %s\n", i, bladerf_strerror(status));
            goto out;
        }

        if (meta.timestamp != timestamp) {
            pass = false;
            fprintf(stderr, "Timestamp mismatch @ %u. "
                    "Expected 0x%016"PRIx64", got 0x%016"PRIx64"\n",
                    i, meta.timestamp, timestamp);

        }

        if (meta.status != 0) {
            pass = false;
            fprintf(stderr, "Warning: status=0x%08"PRIu32"\n", meta.status);
        }
    }

    printf("Test %s.\n", pass ? "passed" : "failed");

out:
    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
    if (status != 0) {
        fprintf(stderr, "Failed to disable RX module: %s\n",
                bladerf_strerror(status));
    }

    return status;
}

int test_rx_gaps(struct bladerf *dev)
{
    int status = 0;
    int16_t *samples;
    size_t i;

    samples = malloc(BUF_SIZE * 2 * sizeof(int16_t));
    if (samples == NULL) {
        perror("malloc");
        return BLADERF_ERR_MEM;
    }

    for (i = 0; i < num_tests && status == 0; i++) {
        status = run(dev, samples, &tests[i]);
    }

    free(samples);
    return status;
}
