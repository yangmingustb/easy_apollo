/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include "cdl_perf.h"

#define PERFORMANCE_PROFILE_MAX_NAME 128

#define cdl_fequal(x, y) (fabs((x) - (y)) < FLT_EPSILON)
#define cdl_max(x, y) ((x) > (y) ? (x) : (y))
#define cdl_min(x, y) ((x) < (y) ? (x) : (y))
#define cdl_us2ms(t) ((float)(t) / 1000.0)

static const char_t
    cdl_perf_profile_str[CDL_PERF_PROFILE_NUM][PERFORMANCE_PROFILE_MAX_NAME] = {
        "traj polgyon size", "coll detect",   "traj reg",
        "obj mem alloc",     "obj tf calc",   "obj coll_tree reg",
        "veh mem alloc",     "veh tf calc",   "veh coll_tree reg",
        "tree setup",        "collide query", "narrow coll"};


#ifdef CDL_PERFORM_PROFILE
static cdl_performance_profile_t perf_profile;

void init_cdl_perf_prof()
{
    for (int index = 0; index < CDL_PERF_PROFILE_NUM; index++)
    {
        perf_profile.start_time_usec[index] = 0.0;
        perf_profile.end_time_usec[index] = 0.0;
        perf_profile.time_usec[index] = 0.0;

        perf_profile.start_clock[index] = 0;
        perf_profile.end_clock[index] = 0;
        perf_profile.clocks[index] = 0;

        perf_profile.time_usec_sum[index] = 0;
        perf_profile.call_count[index] = 0;
    }

    perf_profile.frm_count = 0;
}

uint64_t cdl_getnow_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

void cdl_perf_prof_set_data(cdl_perf_profile_index_t index, int num)
{
    if (index < 0) return;
    if (index >= CDL_PERF_PROFILE_NUM) return;

    perf_profile.time_usec_sum[index] += num;
    perf_profile.call_count[index]++;

    return;
}

void cdl_perf_prof_start(cdl_perf_profile_index_t index)
{
    if (index < 0) return;
    if (index >= CDL_PERF_PROFILE_NUM) return;

    perf_profile.start_time_usec[index] = cdl_getnow_us();

    return;
}

void cdl_perf_prof_stop(cdl_perf_profile_index_t index)
{
    if (index < 0) return;
    if (index >= CDL_PERF_PROFILE_NUM) return;
    if (cdl_fequal(perf_profile.start_time_usec[index], 0.0)) return;

    perf_profile.end_time_usec[index] = cdl_getnow_us();

    perf_profile.time_usec[index] = cdl_max(
        perf_profile.end_time_usec[index] - perf_profile.start_time_usec[index],
        0.0);
    perf_profile.time_usec_sum[index] += perf_profile.time_usec[index];
    perf_profile.call_count[index]++;

    perf_profile.start_time_usec[index] = 0.0;
    perf_profile.end_time_usec[index] = 0.0;
    perf_profile.start_clock[index] = 0;
    perf_profile.end_clock[index] = 0;

    return;
}

void cdl_perf_prof_updatae_frm_cnt(int clear_prev_data)
{
    int i;
    perf_profile.frm_count++;
    if (clear_prev_data)
    {
        for (i = 0; i < CDL_PERF_PROFILE_NUM; i++)
        {
            perf_profile.time_usec[i] = 0;
            perf_profile.clocks[i] = 0;
            perf_profile.time_usec_sum[i] = 0;
            perf_profile.call_count[i] = 0;
        }
    }
}

static const char *cdl_get_perf_profile_str(cdl_perf_profile_index_t index)
{
    return ((index >= 0 && index < CDL_PERF_PROFILE_NUM)
                ? cdl_perf_profile_str[index]
                : "INVALID PROFILE");
}

#define PERFORMANCE_PROFILE_LOG_MSG_SIZE (1024)
static char_t msg[PERFORMANCE_PROFILE_LOG_MSG_SIZE + 1];
const char_t* cdl_get_perf_profile_result()
{
    int32_t i, size;
    char_t *tmp_msg;
    int32_t num;

    snprintf(msg, PERFORMANCE_PROFILE_LOG_MSG_SIZE,
             "PERFORMANCE PROFILE[%lld]: ", (long long)perf_profile.frm_count);
    size = PERFORMANCE_PROFILE_LOG_MSG_SIZE - strlen(msg);
    tmp_msg = msg + cdl_min(strlen(msg), PERFORMANCE_PROFILE_LOG_MSG_SIZE);
    for (i = 0; i < CDL_PERF_PROFILE_NUM; i++)
    {
        size = PERFORMANCE_PROFILE_LOG_MSG_SIZE - strlen(msg);
        if (i < CDL_PERF_PROFILE_NUM - 1)
        {
            num = snprintf(
                tmp_msg, size, "%s:[time %llu clock %llu],",
                cdl_get_perf_profile_str((cdl_perf_profile_index_t)i),
                (long long)(perf_profile.time_usec_sum[i]), (long long)(perf_profile.clocks[i]));
            tmp_msg += num;
            size -= num;
            num =
                snprintf(tmp_msg, size, "%s callcnt:[time %llu clock 0],",
                         cdl_get_perf_profile_str((cdl_perf_profile_index_t)i),
                         (long long)(perf_profile.call_count[i]));
            tmp_msg += num;
            size -= num;
        }
        else
        {
            num = snprintf(
                tmp_msg, size, "%s:[time %llu clock %llu],",
                cdl_get_perf_profile_str((cdl_perf_profile_index_t)i),
                (long long)(perf_profile.time_usec_sum[i]),
                    (long long)(perf_profile.clocks[i]));
            tmp_msg += num;
            size -= num;
            num =
                snprintf(tmp_msg, size, "%s callcnt:[time %llu clock 0]\n",
                         cdl_get_perf_profile_str((cdl_perf_profile_index_t)i),
                         (long long)(perf_profile.call_count[i]));
            tmp_msg += num;
            size -= num;
        }
    }

    return msg;
}

#endif
