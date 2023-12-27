

#include "opencv_viz.h"
#include <string>

namespace apollo
{


#define CVUI_SCALAR(color)                                               \
    cvScalar((viz2d_colors_scalar[color].b), (viz2d_colors_scalar[color].g), \
             (viz2d_colors_scalar[color].r), (0))

static viz2d_color_scalar viz2d_colors_scalar[viz2d_colors_num] = {
        {0.0, 0.0, 0.0},       {255.0, 0.0, 0.0},     {42.0, 42.0, 165.0},
        {255.0, 255.0, 0.0},   {0.0, 215.0, 255.0},   {128.0, 128.0, 128.0},
        {0.0, 128.0, 0.0},     {0.0, 165.0, 255.0},   {0.0, 0.0, 255.0},
        {0.0, 255.0, 255.0},   {203.0, 192.0, 255.0}, {128.0, 0.0, 128.0},
        {192.0, 192.0, 192.0}, {255.0, 255.0, 255.0}, {70.0, 23.0, 11.0},
        {33.0, 145.0, 234.0},  {198.0, 238.0, 118.0}, {144.0, 238.0, 144.0},
        {178.0, 223.0, 238.0}, {255.0, 144.0, 30.0}, {28, 28, 28},
};

static int uos_font_names[viz2d_font_num] = {
        CV_FONT_HERSHEY_SIMPLEX,        CV_FONT_HERSHEY_PLAIN,
        CV_FONT_HERSHEY_DUPLEX,         CV_FONT_HERSHEY_COMPLEX,
        CV_FONT_HERSHEY_TRIPLEX,        CV_FONT_HERSHEY_COMPLEX_SMALL,
        CV_FONT_HERSHEY_SCRIPT_SIMPLEX, CV_FONT_HERSHEY_SCRIPT_COMPLEX};

static bool is_color_index_valid(viz2d_color color_index)
{
    return (color_index < viz2d_colors_num && color_index >= viz2d_colors_black)
                   ? true
                   : false;
}

static bool is_font_index_valid(viz2d_font font_index)
{
    return (font_index < viz2d_font_num &&
            font_index >= viz2d_font_hershey_simplex)
                   ? true
                   : false;
}

int viz2d_draw_xy_axis(viz2d_image *img_handle)
{
    int ret;
    CvPoint origin, x_end, y_end, x_end_left_tail, x_end_right_tail;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    int16_t delta_lateral = 5, delta_longitudinal = 10;
    CvFont windows_font;


    columns = img_handle->columns;
    rows = img_handle->rows;
    origin_row_index = img_handle->origin_row_index;
    origin_column_index = img_handle->origin_column_index;
    img = img_handle->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    x_end.x = origin.x + 20;
    x_end.y = origin_row_index;

    x_end_left_tail.x = x_end.x - delta_longitudinal;
    x_end_left_tail.y = x_end.y - delta_lateral;

    x_end_right_tail.x = x_end.x - delta_longitudinal;
    x_end_right_tail.y = x_end.y + delta_lateral;

    y_end.x = origin_column_index;
    y_end.y = apollo_round(origin_row_index + 20);

    y_end_left_tail.y = y_end.y - delta_longitudinal;
    y_end_right_tail.y = y_end.y - delta_longitudinal;
    y_end_left_tail.x = y_end.x - delta_lateral;
    y_end_right_tail.x = y_end.x + delta_lateral;

    cvLine(img, origin, x_end, CVUI_SCALAR(viz2d_colors_green), 1, CV_AA, 0);
    cvLine(img, x_end, x_end_left_tail, CVUI_SCALAR(viz2d_colors_green), 1,
           CV_AA, 0);
    cvLine(img, x_end, x_end_right_tail, CVUI_SCALAR(viz2d_colors_green), 1,
           CV_AA, 0);

    ret = viz2d_get_cvfont(&windows_font, &img_handle->font);

    cvPutText(img, "y", x_end, &windows_font, CVUI_SCALAR(viz2d_colors_pink));
    cvLine(img, origin, y_end, CVUI_SCALAR(viz2d_colors_green), 1, CV_AA, 0);
    cvLine(img, y_end, y_end_left_tail, CVUI_SCALAR(viz2d_colors_green), 1,
           CV_AA, 0);
    cvLine(img, y_end, y_end_right_tail, CVUI_SCALAR(viz2d_colors_green), 1,
           CV_AA, 0);

    y_end.y += 20;
    cvPutText(img, "x", y_end, &windows_font, CVUI_SCALAR(viz2d_colors_green));

    return 1;
}

int viz2d_init_image_handle(viz2d_image *img_handle,
                           const viz2d_font_setting *font,
                           const char *win_name, int columns, int rows,
                           int origin_column_index, int origin_row_index,
                           double resolution,
                           viz2d_color background_color)
{

    img_handle->font = *font;
    strncpy(img_handle->win_name, win_name, MAX_WINDOW_NAME_LEN);
    img_handle->columns = columns;
    img_handle->rows = rows;
    img_handle->origin_row_index = origin_row_index;
    img_handle->origin_column_index = origin_column_index;
    img_handle->resolution = resolution;
    img_handle->background_color = background_color;

    img_handle->min_x = 0;
    img_handle->max_x = columns;
    img_handle->min_y = 0;
    img_handle->max_y = rows;

    return 1;
}

int viz2d_set_default_font(viz2d_font_setting *font)
{

    font->font_type = viz2d_font_hershey_simplex;
    font->hscale = 0.5;
    font->vscale = 0.5;
    font->shear = 1.0;
    font->thickness = 1;
    font->line_type = CV_AA;

    return 1;
}

int viz2d_get_color(CvScalar *cvColor, viz2d_color color_index)
{
    *cvColor = CVUI_SCALAR(color_index);

    return 1;
}

int viz2d_get_cvfont(CvFont *cvfont, const viz2d_font_setting *font_setting)
{
    viz2d_font font_type;
    font_type = font_setting->font_type;


    cvInitFont(cvfont, uos_font_names[font_type], font_setting->hscale,
               font_setting->vscale, font_setting->shear,
               font_setting->thickness, font_setting->line_type);

    return 1;
}

int viz2d_get_index(viz2d_image *img_handle, CvPoint *index, double x,
                   double y)
{
    int index_x, index_y;
    int columns, rows, origin_row_index, origin_column_index;
    double resolution;


    columns = img_handle->columns;
    rows = img_handle->rows;
    origin_row_index = img_handle->origin_row_index;
    origin_column_index = img_handle->origin_column_index;
    resolution = img_handle->resolution;

    index_x = apollo_floor(y / resolution) + origin_column_index;
    index_y = apollo_floor(x / resolution) + origin_row_index;
    if (index_x < 0)
    {
        index_x = 0;
    }
    else if (index_x >= columns)
    {
        index_x = columns - 1;
    }
    if (index_y < 0)
    {
        index_y = 0;
    }
    else if (index_y >= rows)
    {
        index_y = rows - 1;
    }

    index->x = index_x;
    index->y = index_y;

    return 1;
}

int viz2d_clear_image(viz2d_image *img_handle)
{

    cvSet(img_handle->image, CVUI_SCALAR(img_handle->background_color), 0);

    return 1;
}

int viz2d_shutdown(viz2d_image *img_handle)
{
    cvReleaseImage(&(img_handle->image));
    cvDestroyWindow(img_handle->win_name);

    return 1;
}

int viz2d_start(viz2d_image *img_handle)
{
    int ret;

    cvNamedWindow(img_handle->win_name, CV_WINDOW_AUTOSIZE);
    img_handle->image =
            cvCreateImage(cvSize(img_handle->columns, img_handle->rows), 8, 3);

    ret = viz2d_clear_image(img_handle);

    return 1;
}

int viz2d_display(viz2d_image *img_handle)
{

    cvShowImage(img_handle->win_name, img_handle->image);
    cvWaitKey(2);  // 1 mill second delay

    return 1;
}


int viz2d_draw_circle(viz2d_image *img_handle, double x, double y,
                     double radius, viz2d_color color_index, int fill)
{
    int ret;
    CvPoint position;
    IplImage *img;
    float resolution;
    int R;
    CvScalar color;


    ret = viz2d_get_color(&color, color_index);
    ret = viz2d_get_index(img_handle, &position, x, y);

    resolution = img_handle->resolution;
    R = std::fmax((int)apollo_round(radius / resolution), 1);
    img = img_handle->image;
    cvCircle(img, position, R, color, fill, CV_AA, 0);

    return 1;
}

int viz2d_draw_line(viz2d_image *image_handle, double x1, double y1,
                   double x2, double y2, int thickness,
                   viz2d_color color_index)
{
    int ret;
    CvPoint point1, point2;
    CvScalar color;


    ret = viz2d_get_color(&color, color_index);

    ret = viz2d_get_index(image_handle, &point1, x1, y1);
    ret = viz2d_get_index(image_handle, &point2, x2, y2);

    cvLine(image_handle->image, point1, point2, color, thickness, CV_AA, 0);

    return 1;
}

int viz2d_draw_int_var_text(viz2d_image *img_handle,
                           const char *description, int var,
                           double width_percentage, double height_percentage,
                           viz2d_color color_index)
{
    int ret;
    CvFont font;
    IplImage *img;
    CvScalar color;
    char text[100];
    CvSize text_size;
    int baseline;
    CvPoint text_origin_pt, text_start_pt;


    img = img_handle->image;
    text_start_pt.x = apollo_round(img_handle->columns * width_percentage);
    text_start_pt.y = apollo_round(img_handle->rows * height_percentage);
    ret = viz2d_get_cvfont(&font, &img_handle->font);
    ret = viz2d_get_color(&color, color_index);

    snprintf(text, sizeof(text), description, var);
    cvGetTextSize(text, &font, &text_size, &baseline);
    text_origin_pt.x = text_start_pt.x - text_size.width / 2;
    text_origin_pt.y = text_start_pt.y + text_size.height / 2;
    cvPutText(img, text, text_origin_pt, &font, color);

    return 1;
}


int viz2d_draw_arrow(viz2d_image *img_handle, const Pose2D*ego_pose,
                    viz2d_color color_index)
{
    int ret;
    CvPoint center, center_end, end_left, end_right;
    int delta_length = 10, arrow_slope_length = 3;
    double arrow_angle, delta_theta;
    IplImage *img;
    double arrow_heading_theta;
    CvScalar color;


    ret = viz2d_get_color(&color, color_index);

    ret = viz2d_get_index(img_handle, &center, ego_pose->pos.x, ego_pose->pos.y);

    /* theta conversation from ego frame to image frame */
    arrow_heading_theta = apollo_PI / 2 - ego_pose->theta;
    arrow_heading_theta = apollo_unify_theta(arrow_heading_theta, 0);

    center_end.x = center.x;
    center_end.x += delta_length * apollo_cos(arrow_heading_theta);
    center_end.y = center.y;
    center_end.y += delta_length * apollo_sin(arrow_heading_theta);

    arrow_angle = apollo_PI / 4;
    delta_theta = arrow_heading_theta - arrow_angle;
    delta_theta = apollo_unify_theta(delta_theta, 0);
    end_left.x = center.x;
    end_left.x -= arrow_slope_length * apollo_cos(delta_theta);
    end_left.y = center.y;
    end_left.y -= arrow_slope_length * apollo_sin(delta_theta);

    delta_theta = arrow_heading_theta + arrow_angle;
    delta_theta = apollo_unify_theta(apollo_PI / 2 - delta_theta, 0);
    end_right.x = center.x;
    end_right.x -= arrow_slope_length * apollo_sin(delta_theta);
    end_right.y = center.y;
    end_right.y -= arrow_slope_length * apollo_cos(delta_theta);

    img = img_handle->image;
    cvLine(img, center, center_end, color, 1, CV_AA, 0);
    cvLine(img, center_end, end_left, color, 1, CV_AA, 0);
    cvLine(img, center_end, end_right, color, 1, CV_AA, 0);

    return 1;
}

// int
// viz2d_save_ogm_to_pgm(const char *log_filename, const ogm_map_t *ogm)
// {
//     IplImage* img;
//     int width, height;
//     int i, j, param;

//     uos_check_log_ret(UOS_MOD_PLANNER_BASE, (nullptr != log_filename) &&
//             (nullptr != ogm), "input pointer can not be nullptr!\n", 0);
//     uos_check_log_ret(UOS_MOD_PLANNER_BASE,
//             is_ogm_valid(ogm),
//             "input ogm map is invalid!\n", 0);

//     width = ogm->meta.width;
//     height = ogm->meta.height;
//     img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
//     for (j = 0; j < height; j++)
//     {
//         for (i = 0; i < width; i++)
//         {
//             if (is_ogm_status_passible(ogm->map[j][i]))
//             {
//                 CV_IMAGE_ELEM(img, uchar, j, i) = 0;
//             }
//             else CV_IMAGE_ELEM(img, uchar, j, i) = 255;
//         }
//     }

//     param = 1;
//     cvSaveImage(log_filename, img, &param);

//     return 1;
// }

int viz2d_draw_polygon(viz2d_image *image_handle, const Polygon2D *polygon,
                      const Pose2D*base_pose, viz2d_color color_index)
{
    return viz2d_draw_polygon_with_thickness(image_handle, polygon, base_pose,
                                            color_index, 1);
}

int viz2d_draw_polygon_with_thickness(viz2d_image *image_handle,
                                     const Polygon2D *polygon,
                                     const Pose2D*base_pose,
                                     viz2d_color color_index,
                                     int32_t thickness)
{
    Position2D cur_position, cur_local_position;
    Position2D next_position, next_local_position;
    uint32_t i;

    if (polygon->vertex_num < 1)
    {
        return 1;
    }

    for (i = 0; i < polygon->vertex_num; i++)
    {
        cur_position.x = polygon->vertexes[i].x;
        cur_position.y = polygon->vertexes[i].y;
        cvt_pos_global_to_local(&cur_local_position, &cur_position, base_pose);

        if ((i + 1) < polygon->vertex_num)
        {
            next_position.x = polygon->vertexes[i + 1].x;
            next_position.y = polygon->vertexes[i + 1].y;
        }
        else
        {
            next_position.x = polygon->vertexes[0].x;
            next_position.y = polygon->vertexes[0].y;
        }
        cvt_pos_global_to_local(&next_local_position, &next_position,
                                base_pose);

        viz2d_draw_line(image_handle, cur_local_position.x, cur_local_position.y,
                       next_local_position.x, next_local_position.y, thickness,
                       color_index);
    }
    return 1;
}

int viz2d_local_planner_draw_filled_polygon(viz2d_image *uviz,
                                           const Polygon2D *polygon,
                                           viz2d_color color,
                                           const Pose2D*base_pose_global)
{
    CvScalar vcolor;
    CvPoint points[12];
    uint32_t i;
    int size;
    Position2D uos_pt;
    Polygon2D local_poly;
    int ret;


    ret = cvt_global_polygon_to_local(&local_poly, polygon, base_pose_global);

    for (i = 0; i < polygon->vertex_num; i++)
    {
        uos_pt = local_poly.vertexes[i];
        viz2d_get_index(uviz, &points[i], uos_pt.x, uos_pt.y);
    }
    size = polygon->vertex_num;
    viz2d_get_color(&vcolor, color);
    cvFillConvexPoly(uviz->image, points, size, vcolor, CV_AA, 0);

    return 1;
}

int viz_draw_filled_box(viz2d_image *uviz, const Position2D *center,
                        double width, viz2d_color color,
                        const Pose2D*base_pose_global)
{
    CvScalar internal_color;
    CvScalar line_color;
    CvPoint points[4];
    uint32_t i;
    int size;
    int ret;
    Position2D local_center;
    Position2D tmp;


    cvt_pos_global_to_local(&local_center, center, base_pose_global);


    /**
     * @brief  
     * @note   
     * @retval None
     * 
     * 
     * 
     *                                        x
     *              ------------------------>
     *              |
     *              |
     *              |
     *              |         -----------   1
     *              |        |          \
     *              |        |    *     \
     *              |        |--------- \   0
     *              |
     *        y     V
     */            

    tmp.x = local_center.x + width / 2;
    tmp.y = local_center.y + width / 2;
    viz2d_get_index(uviz, &points[0], tmp.x, tmp.y);

    tmp.x = local_center.x + width / 2;
    tmp.y = local_center.y - width / 2;
    viz2d_get_index(uviz, &points[1], tmp.x, tmp.y);

    tmp.x = local_center.x - width / 2;
    tmp.y = local_center.y - width / 2;
    viz2d_get_index(uviz, &points[2], tmp.x, tmp.y);

    tmp.x = local_center.x - width / 2;
    tmp.y = local_center.y + width / 2;
    viz2d_get_index(uviz, &points[3], tmp.x, tmp.y);


    viz2d_get_color(&internal_color, color);
    viz2d_get_color(&line_color, viz2d_colors_gray);

    cvFillConvexPoly(uviz->image, points, 4, internal_color, CV_AA, 0);

    for (size_t i = 0; i < 4; i++)
    {
        if (i == 3)
        {
            cvLine(uviz->image, points[i], points[0], line_color, 1, CV_AA, 0);
        }
        else
        {
            cvLine(uviz->image, points[i], points[i + 1], line_color, 1, CV_AA,
                   0);
        }
    }

    return 1;
}

CvPoint cv_point_;
bool mouse_event = false;

void on_Mouse(int event, int x, int y, int flags, void *param)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        cv_point_ = cvPoint(x, y);  // 左键按下，记录起始点坐标

        AINFO << "x " << cv_point_.x;
        mouse_event = true;
    }

    return;
}

int transform_cv_point_to_vrf_point(viz2d_image *img_handle,
                                    Position2D *position)
{
    int index_x, index_y;
    int columns, rows, origin_row_index, origin_column_index;
    double resolution;
    double x;
    double y;

    if (mouse_event == false)
    {
        return -1;
    }

    columns = img_handle->columns;
    rows = img_handle->rows;
    origin_row_index = img_handle->origin_row_index;
    origin_column_index = img_handle->origin_column_index;
    resolution = img_handle->resolution;

    x = (cv_point_.y - origin_row_index) * resolution;
    y = (cv_point_.x - origin_column_index) * resolution;

    position->x = x;
    position->y = y;
    mouse_event = false;

    return 0;
}

bool check_index_valid(viz2d_image *img_handle, int x, int y, int buffer)
{
    if (x < img_handle->min_x + buffer)
    {
        return false;
    }
    if (x > img_handle->max_x - buffer)
    {
        return false;
    }
    if (y < img_handle->min_y + buffer)
    {
        return false;
    }
    if (y > img_handle->max_y - buffer)
    {
        return false;
    }

    return true;
}

int viz_draw_local_text(viz2d_image *viz, Position2D *local_text_center,
                        const std::string &str, viz2d_color color)
{
    CvPoint text_pos;

    CvScalar text_color;

    viz2d_get_color(&text_color, color);

    CvFont windows_font;

    viz2d_get_cvfont(&windows_font, &(viz->font));

    viz2d_get_index(viz, &text_pos, local_text_center->x, local_text_center->y);

    cvPutText(viz->image, str.c_str(), text_pos, &windows_font, text_color);

    return 0;
}

int viz_draw_text_relative_to_cv_coordinate(viz2d_image *uviz,
                                            const std::string &str,
                                            viz2d_color color,
                                            CvPoint text_center)
{
    CvScalar text_color;

    if (uviz == nullptr)
    {
        return -1;
    }

    viz2d_get_color(&text_color, color);

    CvFont windows_font;

    viz2d_get_cvfont(&windows_font, &(uviz->font));

    cvPutText(uviz->image, str.c_str(), text_center, &windows_font, text_color);

    return 0;
}

}  // namespace apollo
