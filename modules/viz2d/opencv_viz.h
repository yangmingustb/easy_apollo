
#pragma once

#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <cmath>
#include "modules/common/configs/proto/vehicle_config.pb.h"


#include "modules/prediction/prediction_component.h"

#include "modules/common/math/polygon_base.h"
#include "modules/common/math/pose.h"

namespace apollo
{

#define MAX_WINDOW_NAME_LEN 64

enum viz2d_color
{
    viz2d_colors_black = 0,
    viz2d_colors_blue,
    viz2d_colors_brown,
    viz2d_colors_cyan,
    viz2d_colors_gold,
    viz2d_colors_gray,
    viz2d_colors_green,
    viz2d_colors_orange,
    viz2d_colors_red,
    viz2d_colors_yellow,
    viz2d_colors_pink,
    viz2d_colors_purple,
    viz2d_colors_silver,
    viz2d_colors_white,
    viz2d_colors_dark_blue,
    viz2d_colors_banana_yelow,
    viz2d_colors_aquamarine2,
    viz2d_colors_lightgreen,
    viz2d_colors_lightblue,
    viz2d_colors_dodgerblue1,
    viz2d_colors_black_gray,
    viz2d_colors_num
};

enum viz2d_font
{
    viz2d_font_hershey_simplex = 0,
    viz2d_font_hershey_plain,
    viz2d_font_hershey_duplex,
    viz2d_font_hershey_complex,
    viz2d_font_hershey_triplex,
    viz2d_font_hershey_complex_small,
    viz2d_font_hershey_script_simplex,
    viz2d_font_hershey_script_complex,
    viz2d_font_num
};

struct viz2d_color_scalar
{
    double b;
    double g;
    double r;
};

struct viz2d_font_setting
{
    viz2d_font font_type;
    double hscale;
    double vscale;
    double shear;
    int thickness;
    int line_type;
};

/**
 *              -------------------------->  x (colume)
 *              |
 *              |
 *              |
 *              |
 *              |
 *              |
 *              |
 *              V  y (row)
 * @note   
 * @retval None
 */
struct viz2d_image
{
    IplImage *image;
    viz2d_font_setting font;
    char win_name[MAX_WINDOW_NAME_LEN];
    int columns;
    int rows;
    // 坐标原点
    int origin_row_index;
    int origin_column_index;

    // 每一个格子的尺寸，如0.1，表示一个格子占据0.1米
    double resolution;
    viz2d_color background_color;

    int min_x;
    int max_x;
    int min_y;
    int max_y;
};

#define viz2d_WAIT_TIME_SECOND(time) \
    {                               \
        cvWaitKey(time * 1000);     \
    }

#define viz2d_WAIT_TIME_MILLISECOND(time) \
    {                                    \
        cvWaitKey(time);                 \
    }

#define viz2d_WAIT_FOR_KEY(ch)          \
    {                                  \
        char key_temp = cvWaitKey(0);  \
        while (true)                   \
        {                              \
            key_temp = cvWaitKey(0);   \
            if (ch == key_temp) break; \
        }                              \
    }



/*
 * Get opencv color
 * notice: use only inline { #ifdef ENABLE_viz2d ... #endif }, so better to
 * wrap it in a header(.h) file to use it!
 */
extern int viz2d_get_color(CvScalar *cvColor, viz2d_color color_index);

/*
 * Get the opencv font for cvPuttext from internal font setting
 */
extern int viz2d_get_cvfont(CvFont *cvfont,
                           const viz2d_font_setting *font_setting);

/*
 * Initialize the image handle with params, call it before viz2d_start
 *
 * \param[in\out]  image_handle     image handle
 * \param[in]  font                 font
 * \param[in]  win_name             window's name
 * \param[in]  columns              columns of the image windows
 * \param[in]  rows                 rows of the image windows
 * \param[in]  origin_column_index  origin's x coordinate in the image pixel
 * frame \param[in]  origin_row_index     origin's y coordinate in the image
 * pixel frame \param[in]  resolution           image resolution \param[in]
 * background_color     background color of the image windows \return 1 or 0
 */
extern int viz2d_init_image_handle(viz2d_image *img_handle,
                                  const viz2d_font_setting *font,
                                  const char *win_name, int columns, int rows,
                                  int origin_column_index, int origin_row_index,
                                  double resolution,
                                  viz2d_color background_color);

/*
 * Set default font
 */
extern int viz2d_set_default_font(viz2d_font_setting *font);

/*
 * Reset the display background
 */
extern int viz2d_clear_image(viz2d_image *img_handle);

/*
 * Release window gui and image resources
 * called by the planner framework loop
 */
extern int viz2d_shutdown(viz2d_image *img_handle);

/*
 * Create window and images for visualization
 */
extern int viz2d_start(viz2d_image *img_handle);

/*
 * Display the image
 */
extern int viz2d_display(viz2d_image *img_handle);

/*
 * Draw xy axises
 */
extern int viz2d_draw_xy_axis(viz2d_image *img_handle);


extern int viz2d_draw_arrow(viz2d_image *img_handle,
                           const Pose2D*ego_pose,
                           viz2d_color color_index);

/*
 * Get image pixel coordinate according to coordinate in ego frame
 *
 * \param[in]  img_handle  image handle
 * \param[in]  index       pixel coordinate
 * \param[in]  x           x coordinate in ego frame
 * \param[in]  y           y coordinate in ego frame
 * \return 1 or 0
 */
extern int viz2d_get_index(viz2d_image *img_handle, CvPoint *index,
                          double x, double y);

/*
 * Draw line according to coordinate in ego frame
 *
 * \param[in]  img_handle  image handle
 * \param[in]  x1           x1 coordinate in ego frame
 * \param[in]  y1           y1 coordinate in ego frame
 * \param[in]  x2           x2 coordinate in ego frame
 * \param[in]  y2           y2 coordinate in ego frame
 * \param[in]  thickness    line thickness
 * \param[in]  color_index  line color
 * \return 1 or 0
 */
extern int viz2d_draw_line(viz2d_image *image_handle, double x1, double y1,
                          double x2, double y2, int thickness,
                          viz2d_color color_index);

/*
 * \param[in]  image_handle image handle
 * \param[in]  x            x coordinate of centre point of circle
 * \param[in]  y            y coordinate of centre point of circle
 * \param[in]  radius       radius of circle
 * fill : -1, fill, 1 not fill
 * \param[in]  color_index  drawing color
 */
extern int viz2d_draw_circle(viz2d_image *img_handle, double x, double y,
                            double radius, viz2d_color color_index,
                            int fill);

/*
 * Draw text according to position in ego frame
 *
 * \param[in]  img_handle        image handle
 * \param[in]  description       text format specifier
 * \param[in]  var               value number text
 * \param[in]  width_percentage  text position x
 * \param[in]  height_percentage text position y
 * \param[in]  color_index       text color
 * \return 1 or 0
 */
extern int viz2d_draw_int_var_text(viz2d_image *img_handle,
                                  const char *description, int var,
                                  double width_percentage,
                                  double height_percentage,
                                  viz2d_color color_index);

// /*
//  * \param[in]  img_handle  image handle
//  * \param[in]  ego_pose    pose in ego frame
//  * \param[in]  vehicle     vehicle params
//  * \param[in]  color_index drawing color
//  * \return 1 or 0
//  */
// extern int
// viz2d_draw_footprint(viz2d_image *img_handle,
//                     const Pose2D*ego_pose,
//                     const vehicle_params_t *vehicle,
//                     viz2d_color color_index);

// /*
//  * Save ogm map to pgm format image file using opencv API.
//  *
//  * \param[in]  log_filename  file name
//  * \param[in]  ogm           ogm map
//  * \return 1 or 0
//  */
// extern int
// viz2d_save_ogm_to_pgm(const char *log_filename,
//                      const ogm_map_t *ogm);

int viz2d_draw_polygon_with_thickness(viz2d_image *image_handle,
                                     const Polygon2D *polygon,
                                     const Pose2D*base_pose,
                                     viz2d_color color_index,
                                     int32_t thickness);

int viz2d_draw_polygon(viz2d_image *image_handle, const Polygon2D *polygon,
                      const Pose2D*base_pose, viz2d_color color_index);

int viz2d_local_planner_draw_filled_polygon(viz2d_image *viz2d,
                                           const Polygon2D *polygon,
                                           viz2d_color color,
                                           const Pose2D*base_pose_global);

void on_Mouse(int event, int x, int y, int flags, void *param);

int transform_cv_point_to_vrf_point(viz2d_image *img_handle,
                                    Position2D *position);

bool check_index_valid(viz2d_image *img_handle, int x, int y, int buffer);

int viz_draw_filled_box(viz2d_image *viz2d, const Position2D *center,
                        double width, viz2d_color color,
                        const Pose2D*base_pose_global);

// 相对于车辆坐标
int viz_draw_local_text(viz2d_image *viz, Position2D *local_text_center,
                        const std::string &str, viz2d_color color);

// 在opencv 坐标系下plot
int viz_draw_text_relative_to_cv_coordinate(viz2d_image *viz2d,
                                            const std::string &str,
                                            viz2d_color color,
                                            CvPoint text_center);

}  // namespace apollo
