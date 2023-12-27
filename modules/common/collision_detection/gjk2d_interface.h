/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include "modules/common/math/polygon_base.h"

namespace apollo
{

typedef enum Raycast_Collision_Info
{
    RAYCAST_SOURCE_POINT_IN_POLYGON,
    RAYCAST_HIT_POLYGON,
    RAYCAST_NO_COLLISION,
    RAYCAST_MAX_COLLISION_TYPE
} raycast_collision_info_t;

/**
 * \brief Directly use gjk2d to calculate the collision information
 * \param output: is_collision
 * \param polygon_p
 * \param polygon_q
 * \return int
 */
int
gjk2d_collision_interface(bool *is_collision,
                          const Polygon2D *polygon_p,
                          const Polygon2D *polygon_q);

/**
 * \brief Directly use gjk2d to calculate the collision and distance information
 * \param output: is_collision
 * \param output: dist, distance of polygon_p and polgyon_q
 * \param polygon_p
 * \param polygon_q
 * \return int
 */
int
gjk2d_distance_interface(bool *is_collision,
                         double *dist,
                         const Polygon2D *polygon_obj,
                         const Polygon2D *polygon_veh);
/**
 * \brief Calculate the accurate distance if the distance between
 *  two polyogns is less than dist_thresh else return distance INF
 * \param output: is_collision
 * \param output: dist, distance between polygon_obj and polgyon_veh
 * \param polygon_obj
 * \param polygon_veh
 * \param dist_thresh
 * \return int
 */
int
gjk2d_collision_detecting_with_dist_thresh(bool *is_collision,
                                          double *dist,
                                          const Polygon2D *polygon_obj,
                                          const Polygon2D *polygon_veh,
                                          const double dist_thresh);

/**
 * \brief Calculate the boolean collision info;
 * \param[out] is_collision     Is collision;
 * \param[in] polygon_obj       Polygon;
 * \param[in] polygon_veh       Polygon;
 * \param[in] dist_thresh       Dist;
 * \return int
 */
int
gjk_fast_collision_detection(bool *is_collision,
                             const Polygon2D *polygon_obj,
                             const Polygon2D *polygon_veh,
                             const double dist_thresh);

/**
 * \brief gjk2d to calculate the collision and distance information
 *  output two points of collision pairs on polyogns
 *
 * \param output: is_collision
 * \param output: dist Distance of polygon_obj1 and polgyon_obj2
 * \param output: pos_obj1 Collision position in polygon_obj1
 * \param output: pos_obj2 Collision position in polygon_obj2
 * \param polygon_obj1
 * \param polygon_obj2
 * \return int
 */
int
gjk2d_collision_detecting_with_dist_pair(bool *is_collision,
                                         double *dist,
                                         Position2D *pos_obj1,
                                         Position2D *pos_obj2,
                                         const Polygon2D *polygon_obj1,
                                         const Polygon2D *polygon_obj2);

/**
 * \brief raycast to extract collision position
 *
 * \param[in] source        Start point of the ray
 * \param[in] direction     Raycast direction, no need to normalize the
 *                          direction for time performance, but if you have
 *                          requirement about the hit distance, then you need to
 *                          normalize the direction (And if you have this
 *                          requirement, please contact me!)
 * \param[in] max_lambda    Max distance that the ray can explore
 * \param[in] polygon_veh   The target that ray will hit (or not hit)
 * \param[out] is_collision If the ray hits target along direction
 *                          within max_lambda range or not
 * \param[out] collision_point Hit position in the polygon_veh
 * \param[out] hit_edge_id  Hit edge of polygon_veh (target)
 * \param[out] ratio        Collision ratio on edge.
 * \param[out] raycastlabel Result label.
 * \return int
 */
int
raycast_extract_collisoin_point(Position2D *source,
                                Position2D *direction,
                                float max_lambda,
                                const Polygon2D *polygon_veh,
                                bool *is_collision,
                                Position2D *collision_point,
                                int32_t *hit_edge_id,
                                float * ratio,
                                raycast_collision_info_t *raycast_label);

/**
 * \brief
 * This is a shape cast interface for gjk2d::shapeCast(), specify your endpoint;
 * Attention please: The vertex order in polygon is counter-clock-wise.
 *
 *                              -----
 *                             *    *
 *                            *----*
 *  polygon A
 *
 *         ------                        ------
 *        |      |                      |      |
 *        |      |    -->               |      |
 *         ------                        ------
 *                   ^
 *                  /
 *                 /
 *
 *             -----
 *            *    *
 *           *----*   polygon B
 *
 * \param[out] is_collision
 * \param[out] distA : The polygon A will hit if it moves distA
 * \param[out] distB : The polygon B will hit if it moves distB
 * \param[out] collision_pointA : When polygon A hits with polygon B,
 *  collision_pointA records the hit spot in the body of polygon A
 * \param[out] collision_pointB
 * \param[in] polygonA_start  Poly A start position, you need to fill
 *                            vertex data and center_pt, no need to fill
 *                            axises, radius, and type data in polygon_t.
 * \param[in] polygonA_end
 * \param[in] polygonB_start
 * \param[in] polygonB_end
 * \return int
 */
int
gjk2d_shape_cast(bool *is_collision,
                 double *distA,
                 double *distB,
                 Position2D *collision_pointA,
                 Position2D *collision_pointB,
                 const Polygon2D *polygonA_start,
                 const Polygon2D *polygonA_end,
                 const Polygon2D *polygonB_start,
                 const Polygon2D *polygonB_end);

/**
 * \brief
 * This is a simple shape cast interface for gjk2d::shapeCast(), you need to
 * specify the moving directions, and the two polygon will hit in the same
 * endpoint;
 * Attention please: The vertex order in polygon is counter-clock-wise.
 *
 *    polygon A
 *
 *         ------                        ------
 *        |      |                      |      |
 *        |      |    -->               |      |
 *         ------                        ------
 *
 *
 *                                         ^
 *                                         |
 *
 *                                       -----
 *                                      |     |
 *                                      |     |
 *                                       -----       polygon B
 *
 * \param[out] is_collision
 * \param[out] distA             The polygon A will hit if it moves distA;
 * \param[out] distB             The polygon B will hit if it moves distB;
 * \param[out] collision_pointA  When polygon A hits with polygon B,
 *                               collision_pointA records the hit spot in the
 *                               body of polygon A;
 * \param[out] collision_pointB
 * \param[in] polygonA_start     Poly A start position, you just only fill
 *                               vertex data, no need to fill axises, center_pt,
 *                               radius, and type data in polygon_t;
 * \param[in] dirA               PolygonA moving direction;
 * \param[in] polygonB_start     Poly B start position;
 * \param[in] dirB               PolygonB moving direction;
 * \return int
 */
int
shapecast_with_direction(bool *is_collision,
                         double *distA,
                         double *distB,
                         Position2D *collision_pointA,
                         Position2D *collision_pointB,
                         const Polygon2D *polygonA_start,
                         const Position2D *dirA,
                         const Polygon2D *polygonB_start,
                         const Position2D *dirB);

/**
 * \brief raycast algorithm
 *

 * \param[out] info             If the ray hits target along direction
 *                              within max_lambda range or not
 * \param[out] collision_point  Hit position in the polygon
 * \param[out] collision_dist   Hit dist
 * \param[in] source            Start point of the ray
 * \param[in] direction         Raycast direction
 *                              requirement, please contact me!)
 * \param[in] max_lambda        Max distance that the ray can explore
 * \param[in] polygon           The target that ray will hit (or not hit)
 * \param[in] get_collision_pt  Whether to get collision point
 * \param[in] get_collide_dist  Whether to get collision dist
 * \param[in] normalized_dir    Whether give one normalized direction.
 *                              You can give this API normalized dir or not. IF
 *                              you don't kown about you dir, you can set
 *                              normalized_dir FALSE.
 * \return 0 if success or 1 if failed;
 */
extern int
raycast_algorithm(raycast_collision_info_t *info,
                  Position2D *collision_point,
                  double *collision_dist,
                  Position2D *source,
                  Position2D *direction,
                  double max_lambda,
                  const Polygon2D *polygon,
                  bool get_collision_pt,
                  bool get_collision_dist,
                  bool normalized_dir);

}