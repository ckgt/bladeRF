/*
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
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <getopt.h>
#include <libbladeRF.h>
#include "conversions.h"

#define DEFAULT_SAMPLERATE  1000000

struct app_params {
    char *device_str;
    unsigned int samplerate;

    char *test_name;
};


#define OPTSTR "hd:s:t:v:"
static const struct option long_options[] = {
    { "help",       no_argument,        0,      'h' },

    /* Device configuration */
    { "device",             required_argument,  0,      'd' },
    { "samplerate",         required_argument,  0,      's' },

    /* Test configuration */
    { "test",               required_argument,  0,      't' },

    /* Verbosity options */
    { "lib-verbosity",      required_argument,  0,      'v' },

    { 0,                    0,                  0,      0   },
};

const struct numeric_suffix freq_suffixes[] = {
    { "K",   1000 },
    { "kHz", 1000 },
    { "M",   1000000 },
    { "MHz", 1000000 },
    { "G",   1000000000 },
    { "GHz", 1000000000 },
};

const unsigned int num_freq_suffixes =
                            sizeof(freq_suffixes) / sizeof(freq_suffixes[0]);

static void usage(const char *argv0)
{
    printf("Usage: %s [options]\n", argv0);
    printf("Run timestamp tests\n\n");

    printf("Device configuration options:\n");
    printf("    -d, --device <device>     Use the specified device. By default,\n");
    printf("                              any device found will be used.\n");
    printf("    -s, --samplerate <rate>   Set the specified sample rate.\n");
    printf("                              Default = %u\n", DEFAULT_SAMPLERATE);

    printf("Test configuration:\n");
    printf("    -t, --test <name>         Test name to run. Options are:\n");
    printf("         rx_gaps     Check for unexpected gaps in samples.\n");
    printf("\n");

    printf("Misc options:\n");
    printf("    -h, --help                  Show this help text\n");
    printf("    -v, --lib-verbosity <level> Set libbladeRF verbosity (Default: warning)\n");
    printf("\n");
}

static void init_app_params(struct app_params *p)
{
    memset(p, 0, sizeof(p[0]));
    p->samplerate = 1000000;

}

static void deinit_app_params(struct app_params *p)
{
    free(p->device_str);
    free(p->test_name);
}

static int handle_args(int argc, char *argv[], struct app_params *p)
{
    int c, idx;
    bool ok;
    bladerf_log_level log_level;

    /* Print help */
    if (argc == 1) {
        return 1;
    }

    while ((c = getopt_long(argc, argv, OPTSTR, long_options, &idx)) >= 0) {
        switch (c) {
            case 'v':
                log_level = str2loglevel(optarg, &ok);
                if (!ok) {
                    fprintf(stderr, "Invalid log level: %s\n", optarg);
                    return -1;
                } else {
                    bladerf_log_set_verbosity(log_level);
                }
                break;

            case 'h':
                return 1;

            case 'd':
                if (p->device_str != NULL) {
                    fprintf(stderr, "Device already specified: %s\n",
                            p->device_str);
                    return -1;
                } else {
                    p->device_str = strdup(optarg);
                    if (p->device_str == NULL) {
                        perror("strdup");
                        return -1;
                    }
                }
                break;

            case 's':
                p->samplerate = str2uint_suffix(optarg,
                                                BLADERF_SAMPLERATE_MIN,
                                                BLADERF_SAMPLERATE_REC_MAX,
                                                freq_suffixes,
                                                num_freq_suffixes,
                                                &ok);
                if (!ok) {
                    fprintf(stderr, "Invalid sample rate: %s\n", optarg);
                    return -1;
                }
                break;

            case 't':
                p->test_name = strdup(optarg);
                if (p->test_name == NULL) {
                    perror("strdup");
                    return -1;
                }
                break;

            default:
                return -1;
        }
    }

    return 0;
}

static inline int apply_params(struct bladerf *dev, struct app_params *p)
{
    int status;

    status = bladerf_set_sample_rate(dev, BLADERF_MODULE_RX, p->samplerate, NULL);
    if (status != 0) {
        fprintf(stderr, "Failed to set RX sample rate: %s\n",
                bladerf_strerror(status));
        return status;
    }

    /* XXX: Must the RX and TX channels be at the same sample rate for
     *      timestamps to be used? */
    status = bladerf_set_sample_rate(dev, BLADERF_MODULE_TX, p->samplerate, NULL);
    if (status != 0) {
        fprintf(stderr, "Failed to set TX sample rate: %s\n",
                bladerf_strerror(status));
        return status;
    }

    return 0;
}

extern int test_rx_gaps(struct bladerf *dev);

int main(int argc, char *argv[])
{
    int status;
    struct app_params p;
    struct bladerf *dev = NULL;

    init_app_params(&p);

    status = handle_args(argc, argv, &p);
    if (status != 0) {
        if (status == 1) {
            usage(argv[0]);
            status = 0;
        }
        goto out;
    }

    status = bladerf_open(&dev, p.device_str);
    if (status != 0) {
        fprintf(stderr, "Failed to open device: %s\n",
                bladerf_strerror(status));
        goto out;
    }

    status = apply_params(dev, &p);
    if (status != 0) {
        goto out;
    }

    if (!strcasecmp(p.test_name, "rx_gaps")) {
        status = test_rx_gaps(dev);
    } else {
        fprintf(stderr, "Unknown test: %s\n", p.test_name);
        status = -1;
    }

out:
    if (dev != NULL) {
        bladerf_close(dev);
    }

    deinit_app_params(&p);
    return status;
}
