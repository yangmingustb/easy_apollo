
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#define COLL_DECT_INVALID_OBJ_IDX (-1)

typedef struct Collision_Handle
{
    void *handle;
} collision_handle_t;

typedef enum Collision_Algo
{
    COLLISION_ALGO_PRIM_SAT = 0,
    COLLISION_ALGO_PRIM_GJK2D,
    COLLISION_ALGO_HIERARCHY_AABB,
    COLLISION_ALGO_NUM
} collision_algo_t;

typedef enum Extract_Collision_Pos_Algo
{
    EXTRACT_COLLISION_POS_RAYCAST = 0,
    EXTRACT_COLLISION_POS_SHAPECAST,
    EXTRACT_COLLISION_POS_ALGO_NUM,
} extract_collision_pos_algo_t;

typedef enum Collision_Direction
{
    COLLISION_INVALID_DIRECTION = -1,
    COLLISION_FRONT_RIGHT       = 0,
    COLLISION_FRONT             = 1,
    COLLISION_FRONT_LEFT        = 2,
    COLLISION_LEFT              = 3,
    COLLISION_REAR_LEFT         = 4,
    COLLISION_REAR              = 5,
    COLLISION_REAR_RIGHT        = 6,
    COLLISION_RIGHT             = 7,
    COLLISION_CENTER            = 8,
    COLLISION_MAX_DIRECTION     = 9,
} collision_direction_t;

#ifdef __cplusplus
}
#endif

