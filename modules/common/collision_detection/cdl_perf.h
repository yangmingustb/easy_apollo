/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CDL_PERFORM_PROFILE

typedef enum Cdl_Perf_Profile_index
{
    CDL_PERFORMANCE_PROFILE_TRAJ_OBJ_SIZE = 0,
    CDL_PERFORMANCE_PROFILE_COLLISION_DETECTION,
    CDL_PERFORMANCE_PROFILE_COLLISION_REGISTER,
    CDL_PERFORMANCE_PROFILE_OBJ_MEM_ALLOC,
    CDL_PERFORMANCE_PROFILE_OBJ_TF_CALC,
    CDL_PERFORMANCE_PROFILE_OBJ_REG_COLL,
    CDL_PERFORMANCE_PROFILE_VEH_MEM_ALLOC,
    CDL_PERFORMANCE_PROFILE_VEH_TF_CALC,
    CDL_PERFORMANCE_PROFILE_VEH_REG_COLL,
    CDL_PERFORMANCE_PROFILE_TREE_SETUP,
    CDL_PERFORMANCE_PROFILE_COLLIDE_QUERY,
    CDL_PERFORMANCE_PROFILE_NARROW_DETECT,
    CDL_PERF_PROFILE_NUM,
} cdl_perf_profile_index_t;

typedef struct Cdl_Performance_Profile
{
    uint64_t start_time_usec[CDL_PERF_PROFILE_NUM]; /* us */
    uint64_t end_time_usec[CDL_PERF_PROFILE_NUM];   /* us */
    uint64_t time_usec[CDL_PERF_PROFILE_NUM];       /* us */

    clock_t start_clock[CDL_PERF_PROFILE_NUM]; /* cpu clock */
    clock_t end_clock[CDL_PERF_PROFILE_NUM];   /* cpu clock */
    clock_t clocks[CDL_PERF_PROFILE_NUM];      /* cpu clock */
    uint64_t time_usec_sum[CDL_PERF_PROFILE_NUM];
    uint64_t call_count[CDL_PERF_PROFILE_NUM];
    uint64_t frm_count;
} cdl_performance_profile_t;

uint64_t cdl_getnow_us(void);

#ifdef CDL_PERFORM_PROFILE
void init_cdl_perf_prof(void);

void cdl_perf_prof_set_data(cdl_perf_profile_index_t index, int num);

void cdl_perf_prof_start(cdl_perf_profile_index_t index);

void cdl_perf_prof_stop(cdl_perf_profile_index_t index);

void cdl_perf_prof_updatae_frm_cnt();

const char_t* cdl_get_perf_profile_result();
#else

#define init_cdl_perf_prof() (void)(NULL)
#define cdl_perf_prof_set_data(i, n) (void)(NULL)
#define cdl_perf_prof_start(i) (void)(NULL)
#define cdl_perf_prof_stop(i) (void)(NULL)
#define cdl_perf_prof_updatae_frm_cnt(i) (void)(NULL)
#define cdl_get_perf_profile_result() (void*)(NULL)

#endif

#ifdef __cplusplus
}
#endif
