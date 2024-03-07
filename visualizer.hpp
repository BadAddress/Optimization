#ifndef PVTK_VISUALIZER_HPP
#define PVTK_VISUALIZER_HPP

#include <pangolin/pangolin.h>
#include <memory>
#include <map>
#include <string>
#include <mutex>
#include <stdexcept>
#include <chrono>
#include <queue>
#include <iostream>

#define XY_PLANE_COLOR_R 0.9804
#define XY_PLANE_COLOR_G 0.1569
#define XY_PLANE_COLOR_B 0.2510

#define XZ_PLANE_COLOR_R 0.2510
#define XZ_PLANE_COLOR_G 0.8980
#define XZ_PLANE_COLOR_B 0.2341

#define YZ_PLANE_COLOR_R 0.0941
#define YZ_PLANE_COLOR_G 0.4549
#define YZ_PLANE_COLOR_B 0.9020


namespace pvtk {

// glColor3f(0.0941, 0.4549, 0.9020); // 设置网格颜色，您可以根据需要调整
struct ColorScheme{
    double R = 0.0941,G=0.4549,B=0.9020;
};


class Equation {
public:
    typedef std::function<double(double, double)> FuncType;

    Equation(const std::string& label, FuncType initialFunc)
        : label_(label) {
        // 将初始函数放入队列中
        funcs_.push(std::move(initialFunc));
        GetLatestFunction();
    }

    // 更新方程的函数，将新函数加入队列
    void UpdateFunction(FuncType newFunc) {
        std::lock_guard<std::mutex> lock(mutex_);
        funcs_.push(std::move(newFunc));
    }

    // 获取并更新当前方程的函数（如果有更新的话）
    void GetLatestFunction() {
        std::lock_guard<std::mutex> lock(mutex_);
        // 检查队列中是否有足够的更新
        if (funcs_.size() > 2) {
            funcs_.pop();  // 移除旧的函数
        }
        func = funcs_.front();
    }
    
    // 调用当前方程的函数
    double Evaluate(double x, double y) {
        return func(x, y);  // 使用当前函数计算
    }

    // 获取方程的标签
    std::string GetLabel() const {
        return label_;
    }

    std::string label_;
    std::queue<FuncType> funcs_; // 存储函数的队列
    FuncType func;
    mutable std::mutex mutex_;   // 保护队列的互斥锁
    ColorScheme scheme_;
};




class Visualizer {
public:
    Visualizer() {
        // 初始化Pangolin
        pangolin::CreateWindowAndBind("Pangolin Visualizer", 640, 480);
        glEnable(GL_DEPTH_TEST);
        // 设置相机视角
        s_cam = std::make_unique<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, -2, -2, 0, 0, 0, pangolin::AxisY)
        );

        // 创建交互式视图
        handler = std::make_unique<pangolin::Handler3D>(*s_cam);
        d_cam = &pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                    .SetHandler(handler.get());
    }

    void Run() {
        auto last_update_time = std::chrono::system_clock::now();
        bool refresh_flag = 0;

        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam->Activate(*s_cam);

            // 绘制坐标系
            glLineWidth(4.0); // 设置线宽为3.0，您可以根据需要调整这个值
            pangolin::glDrawAxis(10.0);
            glLineWidth(1.0); // 设置线宽为3.0，您可以根据需要调整这个值
            // 使用预定义的宏颜色绘制三个方向上的网格
            DrawGrid(0.1, 16, 0, 0.3, 0.3, 0.3); // XY平面
            DrawGrid(0.1, 16, 1, 0.3, 0.3, 0.3); // XZ平面
            DrawGrid(0.1, 16, 2, 0.3, 0.3, 0.3); // YZ平面

            auto now = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_update_time).count() >= 0.5) {
                last_update_time = now;
                refresh_flag = !refresh_flag;
            }
            DrawSurfaces(refresh_flag);
            
            DrawPoints();


            pangolin::FinishFrame();
        }
    }

    void AddEquation(std::shared_ptr<Equation> equation) {
        std::lock_guard<std::mutex> lock(equations_mutex);  // 加锁以确保线程安全
        equations.push_back(equation);
    }

    void AddPoints(std::vector<std::vector<double>> point_set) {
        std::lock_guard<std::mutex> lock(equations_mutex);  // 加锁以确保线程安全
        point_sets.push_back(point_set);
    }
    
private:
    std::unique_ptr<pangolin::OpenGlRenderState> s_cam;
    pangolin::View* d_cam;
    std::unique_ptr<pangolin::Handler3D> handler;

    std::vector<std::shared_ptr<Equation>> equations; 
    std::mutex equations_mutex;  // 用于保护 equations 容器的互斥锁
    std::vector<std::vector<std::vector<double>>> point_sets;

    void DrawSurfaces(bool refresh_flag) {
        std::lock_guard<std::mutex> lock(equations_mutex);  // 保护线程安全
        for (auto& eq : equations) {
            if(refresh_flag){
                eq->GetLatestFunction();
            }
            // 定义x和y的范围和分辨率
            double x_min = -2.0, x_max = 2.0;
            double y_min = -2.0, y_max = 2.0;
            int samples = 200; // 定义网格的精度
            double dx = (x_max - x_min) / samples;
            double dy = (y_max - y_min) / samples;

            // glColor3f(0.0941, 0.4549, 0.9020); // 设置网格颜色，您可以根据需要调整
            glColor3f(eq->scheme_.R, eq->scheme_.G, eq->scheme_.B); // 设置网格颜色，您可以根据需要调整 
            glBegin(GL_LINES); // 使用线段绘制网格
            for (int i = 0; i <= samples; i++) {
                for (int j = 0; j <= samples; j++) {
                    double x0 = x_min + i * dx;
                    double y0 = y_min + j * dy;

                    // 仅在网格的边缘绘制线条
                    if (i < samples) {
                        // 绘制水平线
                        double x1 = x_min + (i + 1) * dx;
                        double z00 = eq->Evaluate(x0, y0);
                        double z10 = eq->Evaluate(x1, y0);
                        glVertex3f(x0, y0, z00);
                        glVertex3f(x1, y0, z10);
                    }
                    if (j < samples) {
                        // 绘制垂直线
                        double y1 = y_min + (j + 1) * dy;
                        double z00 = eq->Evaluate(x0, y0);
                        double z01 = eq->Evaluate(x0, y1);
                        glVertex3f(x0, y0, z00);
                        glVertex3f(x0, y1, z01);
                    }
                }
            }
            glEnd();

        }
    }


    void DrawGrid(double grid_size, int num_grid, int plane, double r, double g, double b) {
        glColor3f(r, g, b);
        glBegin(GL_LINES);
        for (int i = -num_grid; i <= num_grid; ++i) {
            double position = grid_size * i;
            if (plane == 0) { // XY plane
                glVertex3f(position, -num_grid * grid_size, 0);
                glVertex3f(position, num_grid * grid_size, 0);
                glVertex3f(-num_grid * grid_size, position, 0);
                glVertex3f(num_grid * grid_size, position, 0);
            } else if (plane == 1) { // XZ plane
                glVertex3f(position, 0, -num_grid * grid_size);
                glVertex3f(position, 0, num_grid * grid_size);
                glVertex3f(-num_grid * grid_size, 0, position);
                glVertex3f(num_grid * grid_size, 0, position);
            } else if (plane == 2) { // YZ plane
                glVertex3f(0, position, -num_grid * grid_size);
                glVertex3f(0, position, num_grid * grid_size);
                glVertex3f(0, -num_grid * grid_size, position);
                glVertex3f(0, num_grid * grid_size, position);
            }
        }
        glEnd();
    }

    void DrawPoints(){
        if(point_sets.size()==0) return;
        glPointSize(10.0f); // 设置点的大小为10.0，根据需求调整
        glColor3f(1.0f, 0.0f, 0.0f); // 设置绘制颜色为红色，根据需求调整

        glBegin(GL_POINTS); // 开始绘制点
        
        for (size_t i = 0; i < point_sets[0][0].size(); ++i) {
            glVertex3f(point_sets[0][0][i],point_sets[0][1][i], point_sets[0][2][i]); // 绘制每个点
        }
        glEnd(); // 结束绘制点
    }



};

} // namespace pvtk

#endif
