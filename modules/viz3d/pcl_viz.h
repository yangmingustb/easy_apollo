#pragma once


namespace apollo
{

enum pcl_color_index
{
    pcl_colors_black = 0,
    pcl_colors_blue,
    pcl_colors_brown,
    pcl_colors_cyan,
    pcl_colors_gold,
    pcl_colors_gray,
    pcl_colors_green,
    pcl_colors_orange,
    pcl_colors_red,
    pcl_colors_yellow,
    pcl_colors_pink,
    pcl_colors_purple,
    pcl_colors_silver,
    pcl_colors_white,
    pcl_colors_dark_blue,
    pcl_colors_banana_yelow,
    pcl_colors_aquamarine2,
    pcl_colors_lightgreen,
    pcl_colors_lightblue,
    pcl_colors_dodgerblue1,
    pcl_colors_black_gray,
    pcl_colors_num
};

struct pcl_color
{
    float b;
    float g;
    float r;
};

int pcl_get_color(pcl_color *Color, pcl_color_index color_index);

} // namespace apollo


