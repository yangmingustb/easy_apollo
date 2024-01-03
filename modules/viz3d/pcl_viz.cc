
#include "pcl_viz.h"


namespace apollo
{

static pcl_color pcl_color_list[pcl_colors_num] = {
        {0.0, 0.0, 0.0},       {255.0, 0.0, 0.0},     {42.0, 42.0, 165.0},
        {255.0, 255.0, 0.0},   {0.0, 215.0, 255.0},   {128.0, 128.0, 128.0},
        {0.0, 128.0, 0.0},     {0.0, 165.0, 255.0},   {0.0, 0.0, 255.0},
        {0.0, 255.0, 255.0},   {203.0, 192.0, 255.0}, {128.0, 0.0, 128.0},
        {192.0, 192.0, 192.0}, {255.0, 255.0, 255.0}, {70.0, 23.0, 11.0},
        {33.0, 145.0, 234.0},  {198.0, 238.0, 118.0}, {144.0, 238.0, 144.0},
        {178.0, 223.0, 238.0}, {255.0, 144.0, 30.0},  {28, 28, 28},
        {70, 23, 11},
};

static int64_t pcl_object_id = 0;

int pcl_get_color(pcl_color *Color, pcl_color_index color_index)
{
    if (color_index >= pcl_colors_num)
    {
        *Color = pcl_color_list[0];

        return 0;
    }

    Color->b = pcl_color_list[color_index].b / 255;
    Color->g = pcl_color_list[color_index].g / 255;
    Color->r = pcl_color_list[color_index].r / 255;

    return 0;
}

int64_t get_pcl_object_id()
{
    pcl_object_id++;
    return pcl_object_id;
}

std::string get_pcl_object_string_id()
{
    pcl_object_id++;

    return "id_" + std::to_string(pcl_object_id);
}

void reset_object_id() { pcl_object_id = 0; }

} // namespace apollo
