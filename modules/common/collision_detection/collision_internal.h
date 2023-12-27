/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include "collision_interface.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct Collision_Internal_Handle
{
    void *bvh_manager_proxy;
} collision_internal_handle_t;

#ifdef __cplusplus
}
#endif

