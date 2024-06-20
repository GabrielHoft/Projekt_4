#include "planar_quadrotor_visualizer.h"
#include <cmath>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualization
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate propellers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    int screen_width, screen_height;
    SDL_GetRendererOutputSize(gRenderer.get(), &screen_width, &screen_height);
    int img_x = static_cast<int>((q_x + 5.0f) / 10.0f * screen_width);
    int img_y = static_cast<int>(screen_height - (q_y + 5.0f) / 10.0f * screen_height);

    /*Oblicznie polozenia rogow prostokata*/
    int body_width = 100;
    int body_height = 20;
    SDL_Point corners[4];
    corners[0] = { img_x + (int)(body_width / 2 * cos(q_theta) + body_height / 2 * sin(q_theta)), img_y + (int)(-body_width / 2 * sin(q_theta) + body_height / 2 * cos(q_theta)) };
    corners[1] = { img_x + (int)(-body_width / 2 * cos(q_theta) + body_height / 2 * sin(q_theta)), img_y + (int)(body_width / 2 * sin(q_theta) + body_height / 2 * cos(q_theta)) };
    corners[2] = { img_x + (int)(-body_width / 2 * cos(q_theta) - body_height / 2 * sin(q_theta)), img_y + (int)(body_width / 2 * sin(q_theta) - body_height / 2 * cos(q_theta)) };
    corners[3] = { img_x + (int)(body_width / 2 * cos(q_theta) - body_height / 2 * sin(q_theta)), img_y + (int)(-body_width / 2 * sin(q_theta) - body_height / 2 * cos(q_theta)) };

    /*Rysowanie ciala drona*/
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    for (int i = 0; i < 4; ++i) {
        SDL_RenderDrawLine(gRenderer.get(), corners[i].x, corners[i].y, corners[(i + 1) % 4].x, corners[(i + 1) % 4].y);
    }

    /*Rysowanie lukow od podstawy drona*/
    int corner_radius = 10;
    arcRGBA(gRenderer.get(), corners[0].x, corners[0].y, corner_radius, 0, 90, 0, 0, 0, 255);
    arcRGBA(gRenderer.get(), corners[1].x, corners[1].y, corner_radius, 90, 180, 0, 0, 0, 255);

    /*Obliczanie srodkow lukow*/
    int arc_midpoint_0_x = corners[0].x + corner_radius / sqrt(2);
    int arc_midpoint_0_y = corners[0].y + corner_radius / sqrt(2);
    int arc_midpoint_1_x = corners[1].x - corner_radius / sqrt(2);
    int arc_midpoint_1_y = corners[1].y + corner_radius / sqrt(2);

    /*Rysowanie laczace dolne rogi prostokata z lukami*/
    SDL_RenderDrawLine(gRenderer.get(), corners[0].x, corners[0].y, arc_midpoint_0_x, arc_midpoint_0_y);
    SDL_RenderDrawLine(gRenderer.get(), corners[1].x, corners[1].y, arc_midpoint_1_x, arc_midpoint_1_y);

    /*Rysowanie smigiel*/
    int propeller_radius_x = 30;
    int propeller_radius_y = 10;
    int propeller_angle = static_cast<int>(SDL_GetTicks() % 360);
    float arm_extension = 20.0f;

    for (int i = 2; i < 4; ++i) {
        float corner_x = corners[i].x;
        float corner_y = corners[i].y;

        /*Obliczanie pozycji smigiel*/
        float arm_dir_x = (corner_x - img_x);
        float arm_dir_y = (corner_y - img_y);
        float length = sqrt(arm_dir_x * arm_dir_x + arm_dir_y * arm_dir_y);
        arm_dir_x /= length;
        arm_dir_y /= length;
        int propeller_x = static_cast<int>(corner_x + arm_extension * arm_dir_x);
        int propeller_y = static_cast<int>(corner_y + arm_extension * arm_dir_y);

        /*Rysowanie polaczenia smigiel z cialem drona*/
        SDL_RenderDrawLine(gRenderer.get(), corner_x, corner_y, propeller_x, propeller_y);

        /*Rysowanie smigiel*/
        filledEllipseColor(gRenderer.get(), propeller_x, propeller_y, propeller_radius_x, propeller_radius_y, 0x0000FFFF);

        /*Animacja smigiel*/
        float propeller_rad = M_PI * propeller_angle / 180.0f;
        int line_x1 = propeller_x - (int)(propeller_radius_x * cos(propeller_rad + q_theta));
        int line_y1 = propeller_y - (int)(propeller_radius_y * sin(propeller_rad + q_theta));
        int line_x2 = propeller_x + (int)(propeller_radius_x * cos(propeller_rad + q_theta));
        int line_y2 = propeller_y + (int)(propeller_radius_y * sin(propeller_rad + q_theta));

        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
        SDL_RenderDrawLine(gRenderer.get(), line_x1, line_y1, line_x2, line_y2);
    }
}