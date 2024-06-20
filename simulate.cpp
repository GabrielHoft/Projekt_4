#include "simulate.h"
#include <vector>
#include <matplot/matplot.h>

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 5e-3, 5e-3, 5e2, 9e-3, 5.5e-2, 5 / 5 / M_PI;
    R.row(0) << 4e1, 8;
    R.row(1) << 8, 4e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    Eigen::VectorXf control_state = quadrotor.GetControlState();
    control_state(2) /= 5;
    quadrotor.SetInput(input - K * control_state);
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);

    const float dt = 0.009;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT)
                {
                    SDL_GetMouseState(&x, &y);
                    /*Zmiana poczatkowego polozenia drona na srodek*/
                    float x_world = static_cast<float>(x) / SCREEN_WIDTH * 10.0f - 5.0f;
                    float y_world = static_cast<float>(SCREEN_HEIGHT - y) / SCREEN_HEIGHT * 10.0f - 5.0f;
                    goal_state << x_world, y_world, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                    std::cout << "Goal state set to: (" << x_world << ", " << y_world << ", 0, 0, 0, 0)" << std::endl;
                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p)
                {
                    using namespace matplot;
                    auto f = figure(true);
                    f->size(1000, 600);

                    subplot(2, 1, 1);
                    plot(x_history, y_history, "-o")->line_width(2).marker_size(6);
                    xlabel("X");
                    ylabel("Y");
                    title("Quadrotor Trajectory");

                    subplot(2, 1, 2);
                    plot(theta_history, "-o")->line_width(2).marker_size(6).color("red");
                    xlabel("Time");
                    ylabel("Theta");
                    title("Theta Over Time");

                    show();
                }
            }

            SDL_Delay((int)dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            /* Store the trajectory */
            state = quadrotor.GetState();
            x_history.push_back(state(0));
            y_history.push_back(state(1));
            theta_history.push_back(state(2));
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}