
#include <opencv2/opencv.hpp>

#include "viz2d_path.h"
#include "viz2d_geometry.h"
#include "modules/viz2d/viz_window.h"

namespace apollo
{
static viz2d_image main_window_;
static viz2d_image hmap_window_;
static viz2d_image control_window_;
DisplayConfig config;


viz2d_image *get_main_window2d(void) { return &main_window_; }
viz2d_image *get_hmap_window2d(void) { return &hmap_window_; }
viz2d_image *get_control_window2d(void) { return &control_window_; }

DisplayConfig* get_windows2d_config()
{
    return &config;
}

void main_window2d_init(viz2d_color background_color)
{
    int ret;
    int columns, rows, origin_column_index, origin_row_index;
    char window_name[] = "display 2D window";
    double resolution;
    viz2d_font_setting font;

    viz2d_set_default_font(&font);

    columns = 1800;
    rows = 1000;
    resolution = config.main_window_resolution();
    origin_row_index = rows / 2;
    origin_column_index = columns / 4;
    // origin_column_index = columns / 4 * 3;

    ret = viz2d_init_window(&main_window_, &font, window_name,
                                 columns, rows, origin_column_index,
                                 origin_row_index, resolution,
                                 background_color);
}

int viz2d_window_read_config(std::string config_dir)
{
    std::string FLAGS_display_file = config_dir + "/viz2d/conf/display.pb.txt";

    if (!apollo::cyber::common::GetProtoFromFile(FLAGS_display_file, &config))
    {
        AERROR << "failed to load file: " << FLAGS_display_file;
    }
    AINFO << config.DebugString();

    return 0;
}

void hmap_window2d_init()
{
    int ret;
    int columns, rows, origin_column_index, origin_row_index;
    char window_name[] = "hmap window";
    double resolution;
    viz2d_font_setting font;

    viz2d_set_default_font(&font);

    columns = 1000;
    rows = 800;
    resolution = config.hmap_window_resolution();
    origin_row_index = rows / 2;
    origin_column_index = columns / 2;

    ret = viz2d_init_window(&hmap_window_, &font, window_name, columns, rows,
                                 origin_column_index, origin_row_index,
                                 resolution, viz2d_colors_black);
}

void control_viz2d_init()
{
    int ret;
    int columns, rows, origin_column_index, origin_row_index;
    char window_name[] = "control_window";
    double resolution;
    viz2d_font_setting font;

    viz2d_set_default_font(&font);

    columns = 1600;
    rows = 800;
    resolution = config.control_window_resolution();
    origin_row_index = rows / 2;
    origin_column_index = columns / 6;

    ret = viz2d_init_window(&control_window_, &font, window_name, columns,
                                 rows, origin_column_index, origin_row_index,
                                 resolution, viz2d_colors_white);
}

    




int viz2d_draw_system_state(viz2d_image *viz2d, const SystemState &states,
                           const Pose2D*base_pose)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar success_color;
    CvScalar unkown_color;
    CvScalar error_color;
    CvScalar color;
    CvScalar text_color;
    const char *text;

    viz2d_color success_color_index = viz2d_colors_green;
    viz2d_color error_color_index = viz2d_colors_red;
    viz2d_color text_color_index = viz2d_colors_white;
    viz2d_color unkown_color_index = viz2d_colors_banana_yelow;

    if (base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    Pose2D local_pose, global_pose;


    viz2d_get_color(&success_color, success_color_index);
    viz2d_get_color(&error_color, error_color_index);
    viz2d_get_color(&text_color, text_color_index);
    viz2d_get_color(&unkown_color, unkown_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    int16_t delta_lateral = 5, delta_longitudinal = 10;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 800;
    center.y = 20;

    for (int i = 0; i < static_cast<int>(ModuleName::max_module_number); i++)
    {
        ModuleStateCode state = states.moduel_state[i];
        if (state == ModuleStateCodeSuccess)
        {
            color = success_color;
        }
        else if (state == ModuleStateCodeError)
        {
            color = error_color;
        }
        else
        {
            color = unkown_color;
        }
        cvCircle(viz2d->image, center, circle_size, color, -1, CV_AA, 0);

        // text
        text_center = center;
        text_center.x += 20;

        text = get_module_name((ModuleName)i);
        if (text != nullptr)
        {
            cvPutText(viz2d->image, text, text_center, &windows_font,
                      text_color);
        }

        center.y += 20;
    }

    return 0;

}

int viz2d_draw_rtk_state(viz2d_image *viz2d,
                        const localization::LocalizationEstimate &localization,
                        const Pose2D*base_pose)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar success_color;
    CvScalar unkown_color;
    CvScalar error_color;
    CvScalar color;
    CvScalar text_color;
    const char *text;

    viz2d_color success_color_index = viz2d_colors_green;
    viz2d_color error_color_index = viz2d_colors_red;
    viz2d_color text_color_index = viz2d_colors_white;
    viz2d_color unkown_color_index = viz2d_colors_banana_yelow;

    if (base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    Pose2D local_pose, global_pose;


    viz2d_get_color(&success_color, success_color_index);
    viz2d_get_color(&error_color, error_color_index);
    viz2d_get_color(&text_color, text_color_index);
    viz2d_get_color(&unkown_color, unkown_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    int16_t delta_lateral = 5, delta_longitudinal = 10;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 800;
    center.y = 220;

    if (!localization.has_rtk_status() || localization.rtk_status() == 0)
    {
        color = error_color;
    }
    else
    {
        color = success_color;
    }

    cvCircle(viz2d->image, center, circle_size, color, -1, CV_AA, 0);

    // text
    text_center = center;
    text_center.x += 20;

    text = "rtk status";
    if (text != nullptr)
    {
        cvPutText(viz2d->image, text, text_center, &windows_font, text_color);
    }

    return 0;
}

int viz2d_draw_pause_state(viz2d_image *viz2d,
                          const apollo::RunningTimeDebug &debug)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar success_color;
    CvScalar unkown_color;
    CvScalar error_color;
    CvScalar color;
    CvScalar text_color;
    const char *text;

    viz2d_color success_color_index = viz2d_colors_green;
    viz2d_color error_color_index = viz2d_colors_red;
    viz2d_color text_color_index = viz2d_colors_white;
    viz2d_color unkown_color_index = viz2d_colors_banana_yelow;

    if (viz2d == nullptr)
    {
        return -1;
    }

    Pose2D local_pose, global_pose;


    viz2d_get_color(&success_color, success_color_index);
    viz2d_get_color(&error_color, error_color_index);
    viz2d_get_color(&text_color, text_color_index);
    viz2d_get_color(&unkown_color, unkown_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    int16_t delta_lateral = 5, delta_longitudinal = 10;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 1000;
    center.y = 20;

    if (debug.pause_debug.enabled)
    {
        color = unkown_color;
    }
    else
    {
        color = success_color;
    }
    cvCircle(viz2d->image, center, circle_size, color, -1, CV_AA, 0);

    // text
    text_center = center;
    text_center.x += 20;

    text = "pause state";
    if (text != nullptr)
    {
        cvPutText(viz2d->image, text, text_center, &windows_font, text_color);
    }

    return 0;
}

int viz2d_draw_drive_mode(viz2d_image *viz2d,
                         const apollo::canbus::Chassis::DrivingMode &mode)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar success_color;
    CvScalar unkown_color;
    CvScalar error_color;
    CvScalar color;
    CvScalar text_color;
    const char *text;

    viz2d_color success_color_index = viz2d_colors_green;
    viz2d_color error_color_index = viz2d_colors_red;
    viz2d_color text_color_index = viz2d_colors_white;
    viz2d_color unkown_color_index = viz2d_colors_banana_yelow;

    if (viz2d == nullptr)
    {
        return -1;
    }

    Pose2D local_pose, global_pose;


    viz2d_get_color(&success_color, success_color_index);
    viz2d_get_color(&error_color, error_color_index);
    viz2d_get_color(&text_color, text_color_index);
    viz2d_get_color(&unkown_color, unkown_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    int16_t delta_lateral = 5, delta_longitudinal = 10;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 1000;
    center.y = 40;

    if (mode != apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE)
    {
        color = unkown_color;
        text = "Manual mode";
    }
    else
    {
        color = success_color;
        text = "Auto mode";
    }
    cvCircle(viz2d->image, center, circle_size, color, -1, CV_AA, 0);

    // text
    text_center = center;
    text_center.x += 20;

    if (text != nullptr)
    {
        cvPutText(viz2d->image, text, text_center, &windows_font, text_color);
    }

    return 0;
}

int viz2d_draw_run_mode(viz2d_image *viz2d,
                       const apollo::ApolloRunMode &mode)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar success_color;
    CvScalar unkown_color;
    CvScalar error_color;
    CvScalar color;
    CvScalar text_color;
    const char *text;

    viz2d_color success_color_index = viz2d_colors_green;
    viz2d_color error_color_index = viz2d_colors_red;
    viz2d_color text_color_index = viz2d_colors_white;
    viz2d_color unkown_color_index = viz2d_colors_banana_yelow;

    if (viz2d == nullptr)
    {
        return -1;
    }

    Pose2D local_pose, global_pose;


    viz2d_get_color(&success_color, success_color_index);
    viz2d_get_color(&error_color, error_color_index);
    viz2d_get_color(&text_color, text_color_index);
    viz2d_get_color(&unkown_color, unkown_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    int16_t delta_lateral = 5, delta_longitudinal = 10;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 1200;
    center.y = 20;

    color = success_color;
    cvCircle(viz2d->image, center, circle_size, color, -1, CV_AA, 0);

    // text
    text_center = center;
    text_center.x += 20;

    text = get_run_mode_name(mode);
    if (text != nullptr)
    {
        cvPutText(viz2d->image, text, text_center, &windows_font, text_color);
    }

    return 0;
}

int viz2d_draw_localization_time(viz2d_image *viz2d, double time_stamp)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar success_color;
    CvScalar unkown_color;
    CvScalar error_color;
    CvScalar color;
    CvScalar text_color;

    viz2d_color success_color_index = viz2d_colors_green;
    viz2d_color error_color_index = viz2d_colors_red;
    viz2d_color text_color_index = viz2d_colors_white;
    viz2d_color unkown_color_index = viz2d_colors_banana_yelow;

    if (viz2d == nullptr)
    {
        return -1;
    }


    viz2d_get_color(&success_color, success_color_index);
    viz2d_get_color(&error_color, error_color_index);
    viz2d_get_color(&text_color, text_color_index);
    viz2d_get_color(&unkown_color, unkown_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    int16_t delta_lateral = 5, delta_longitudinal = 10;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 1400;
    center.y = 20;

    // text
    text_center = center;

    cyber::Time t1(time_stamp);

    cvPutText(viz2d->image, t1.ToString().c_str(), text_center, &windows_font,
              text_color);

    return 0;
}

int viz2d_draw_replay_info(viz2d_image *viz2d, int64_t ratio)
{
    CvScalar replay_finish_color;
    CvScalar replay_todo_color;

    if (viz2d == nullptr)
    {
        return -1;
    }

    // 没有接收消息
    if (ratio <= -1)
    {
        return 0;
    }


    viz2d_get_color(&replay_finish_color, viz2d_colors_green);
    viz2d_get_color(&replay_todo_color, viz2d_colors_banana_yelow);

    CvPoint start, mid, end;

    // opencv中的长度
    int total_len = 200;
    start.x = 1400;
    start.y = 60;

    mid = start;

    end = start;
    end.x += total_len;

    if (ratio >= total_len)
    {
        mid.x = end.x;
    }
    else if (ratio <= 0)
    {
        mid = start;
    }
    else
    {
        double ratio_double = ratio / 100.0;

        mid.x += int(total_len * ratio_double);
    }

    int line_width = 10;

    cvLine(viz2d->image, start, mid, replay_finish_color, line_width, CV_AA, 0);

    cvLine(viz2d->image, mid, end, replay_todo_color, line_width, CV_AA, 0);

    return 0;
}


int viz2d_draw_chassis_feedback(viz2d_image *viz2d, double v, double wheel)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar text_color;

    viz2d_color text_color_index = viz2d_colors_white;

    if (viz2d == nullptr)
    {
        return -1;
    }

    viz2d_get_color(&text_color, text_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 100;
    center.y = 20;

    // text
    text_center = center;

    char str[128];
    if (v >= 0.0)
    {
        sprintf(str, "chassis info: speed: +%.4f m/s, steer: %.4f", v, wheel);
    }
    else
    {
        sprintf(str, "chassis info: speed: %.4f m/s, steer: %.4f", v, wheel);
    }

    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);


    // fill box
    CvPoint box_center;
    box_center.x = columns / 2;
    box_center.y = rows - 80;

    viz_draw_box_in_cv_frame(viz2d, &box_center, 150, 800,
                             viz2d_colors_black_gray, true);

    // speed value

    text_center.x = columns / 2 - 250;
    text_center.y = rows - 80;

    double v_kmh = v * 3.6;

    sprintf(str, " %.2f", v_kmh);
    windows_font.hscale = 1.5;
    windows_font.vscale = 1.5;
    windows_font.thickness = 2.0;
    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

    // speed unit
    text_center.x = columns / 2 - 220;
    text_center.y += 50;
    sprintf(str, " km/h");
    windows_font.hscale = 1.0;
    windows_font.vscale = 1.0;
    windows_font.thickness = 1.0;
    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

    // wheel

    if (wheel >= 0.0)
    {
        sprintf(str, "+%.2f", wheel);
    }
    else
    {
        sprintf(str, "-%.2f", std::fabs(wheel));
    }
    
    text_center.x += 160;
    text_center.y = rows - 80;

    windows_font.hscale = 1.5;
    windows_font.vscale = 1.5;
    windows_font.thickness = 2.0;
    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

    // speed unit
    text_center.y += 50;
    sprintf(str, " degree");
    windows_font.hscale = 1.0;
    windows_font.vscale = 1.0;
    windows_font.thickness = 1.0;
    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

    return 0;
}

int viz2d_draw_control_commond_info(viz2d_image *viz2d, double acc,
                                   double wheel)
{
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar text_color;

    viz2d_color text_color_index = viz2d_colors_white;

    if (viz2d == nullptr)
    {
        return -1;
    }

    viz2d_get_color(&text_color, text_color_index);

    CvPoint origin, center, text_center;
    CvPoint y_end_left_tail, y_end_right_tail;
    int columns, rows, origin_row_index, origin_column_index;
    IplImage *img;
    CvFont windows_font;
    int circle_size = 8;

    columns = viz2d->columns;
    rows = viz2d->rows;
    origin_row_index = viz2d->origin_row_index;
    origin_column_index = viz2d->origin_column_index;
    img = viz2d->image;
    origin.x = origin_column_index;
    origin.y = origin_row_index;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    center.x = 100;
    center.y = 40;

    // text
    text_center = center;

    char str[128];
    if (acc >= 0.0)
    {
        sprintf(str, "commond info: acc: +%.4f m/s, steer: %.4f", acc, wheel);
    }
    else
    {
        sprintf(str, "commond info: acc: %.4f m/s, steer: %.4f", acc, wheel);
    }

    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

    // acc

    if (acc >= 0.0)
    {
        sprintf(str, "+%.2f", acc);
    }
    else
    {
        sprintf(str, "-%.2f", std::fabs(acc));
    }

    text_center.x = columns / 2 + 170;
    text_center.y = rows - 80;

    windows_font.hscale = 1.5;
    windows_font.vscale = 1.5;
    windows_font.thickness = 2.0;
    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

    text_center.y += 50;
    sprintf(str, " m/s^2");
    windows_font.hscale = 1.0;
    windows_font.vscale = 1.0;
    windows_font.thickness = 1.0;
    cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

    return 0;
}



static double calc_turn_radius(double lfr, double steering)
{
    double rad = steering;

    double radius = lfr / std::tan(rad);

    return radius;
}

// right turn is negative
int viz2d_draw_front_wheel_state(viz2d_image *viz2d, const Pose2D*veh_pose,
                                double wheel_base, double steering,
                                viz2d_color text_color_index)
{
    double radius;
    radius = calc_turn_radius(wheel_base, steering);

    Position2D center_point;

    if (std::fabs(steering) > 0.00001)
    {
        center_point.x =
                veh_pose->pos.x - radius * std::sin(veh_pose->theta);
        center_point.y =
                veh_pose->pos.y + radius * std::cos(veh_pose->theta);
    }
    else
    {
        return 0;
    }

    double inc_dist, sum_dist, inc_angle, sum_angle, start_theta, end_theta;
    Position2D end_point;
    Position2D start_point;

    CvScalar text_color;

    viz2d_get_color(&text_color, text_color_index);

    start_point = veh_pose->pos;

    inc_dist = 0.5;
    sum_dist = 0.0;
    sum_angle = 0.0;


    radius = apollo_fabs(radius);

    if (radius < 0.000001)
    {
        return 0;
    }

    inc_angle = inc_dist / apollo_fabs(radius);
    start_theta = apollo_calc_theta_by_x_y(center_point.x, center_point.y,
                                        start_point.x, start_point.y);

    end_theta = start_theta;

    // right turn
    if (steering < 0.0)
    {
        inc_angle *= -1.0;
    }

    while (apollo_fless(sum_angle, apollo_PI) && apollo_fless(sum_dist, 100.0))
    {
        end_theta += inc_angle;
        end_point.x = center_point.x + radius * apollo_cos(end_theta);
        end_point.y = center_point.y + radius * apollo_sin(end_theta);

        viz2d_draw_line(viz2d, &start_point, &end_point, veh_pose,
                       text_color_index, 2);

        start_point = end_point;
        sum_dist += inc_dist;
        sum_angle += apollo_fabs(inc_angle);
    }

    return 0;
}



}  // namespace apollo
