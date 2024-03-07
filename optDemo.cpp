
#include "visualizer.hpp"  
#include <math.h>          
#include <thread>         
#include <vector>          
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <vector>
#include <random>


double guassian2D(double x,double y,const std::vector<double>& params){
    double mean_x = params[0];
    double mean_y = params[1];
    double sigma_x = params[2];
    double sigma_y = params[3];
    double rho = params[4]; // 相关系数

    // 计算预先的系数
    double oMinusRhoSquare = 1 - rho * rho;
    double sigma_x_square = sigma_x * sigma_x;
    double sigma_y_square = sigma_y * sigma_y;
    // 计算二维高斯分布的概率密度函数
    return (1.0 / (2 * M_PI * sigma_x * sigma_y * sqrt(oMinusRhoSquare))) *
           exp(-(1.0 / (2 * oMinusRhoSquare)) * (
               ((x - mean_x) * (x - mean_x) / sigma_x_square) +
               ((y - mean_y) * (y - mean_y) / sigma_y_square) -
               (2 * rho * (x - mean_x) * (y - mean_y) / (sigma_x * sigma_y))
           ));
}



void userCodeSpace(pvtk::Visualizer& visualizer){    

    double mu_x = 0.2, mu_y = 0.5, sigma_x = 0.7, sigma_y = 0.4, rho = 0.73;
    double mu_x_est = 0.6, mu_y_est = 0.7, sigma_x_est = 0.42, sigma_y_est = 0.4, rho_est = 0.73;

    auto params = std::vector<double> {mu_x,mu_y,sigma_x,sigma_y,rho};
    auto params_est = std::vector<double> {mu_x_est,mu_y_est,sigma_x_est,sigma_y_est,rho_est};


    auto groundTruth = std::make_shared<pvtk::Equation>("groundTruth", [params](double x, double y) -> double {
        return  guassian2D(x,y,params); 
    });
    visualizer.AddEquation(groundTruth);

    std::vector<double> x_data,y_data,z_data;
    std::random_device rd; // 随机数种子
    std::mt19937 gen(rd()); // 基于Mersenne Twister的随机数生成器
    std::uniform_real_distribution<> dis(0.0, 1.0); // 定义一个在[0.0, 1.0]范围内的均匀分布
    cv::RNG rng;
    double noise_sigma = 0.015;
    int samples = 200;
    
    for(int i=0;i<samples;i++){
        auto x = dis(gen)*2.6-1.3;
        auto y = dis(gen)*2.6-1.3;
        x_data.push_back(x);
        y_data.push_back(y);
        z_data.push_back(guassian2D(x,y,params)+rng.gaussian(pow(noise_sigma,2)));
    }
    visualizer.AddPoints({x_data,y_data,z_data});



    auto estimate = std::make_shared<pvtk::Equation>("estimation", [params_est](double x, double y) -> double {
        return  guassian2D(x,y,params_est); 
    });
    estimate->scheme_={0.2510,0.8980,0.2341};
    visualizer.AddEquation(estimate);

    
    //auto params_est = std::vector<double> {mu_x_est,mu_y_est,sigma_x_est,sigma_y_est,rho_est};

    int iteration = 1000;
    double lastCost = 10000.;

    for(int it=0;it<iteration;it++){
        
        Eigen::Matrix3d H;
        Eigen::Vector3d b;
        H.setZero();
        b.setZero();
        double cost = 0;
        for(int i=0;i<samples;++i){
            double xi = x_data[i];
            double yi = y_data[i];
            double zi_est = guassian2D(xi,yi,params_est);
            double error = z_data[i] - zi_est;

            Eigen::Vector3d J;

            double mu1 = params_est[0];
            double mu2 = params_est[1];
            double sigma1 = params_est[2];
            double sigma2 = params_est[3];
            double rho = params_est[4];

            double rhoSquare = rho * rho;
            double sigma1Square = sigma1 * sigma1;
            double sigma2Square = sigma2 * sigma2;
            double xiMinusMu1 = xi - mu1;
            double yiMinusMu2 = yi - mu2;
            // double oneMinusRhoSquare = 1 - rhoSquare;
             
            // 预先计算常用表达式保持不变

            J[0] = -(xiMinusMu1 / sigma1Square - rho * yiMinusMu2 / (sigma1 * sigma2)) * zi_est;  // 对mu1的偏导
            J[1] = -(yiMinusMu2 / sigma2Square - rho * xiMinusMu1 / (sigma1 * sigma2)) * zi_est;  // 对mu2的偏导
            J[2] = -(xiMinusMu1 * xiMinusMu1 / (sigma1Square * sigma1) - 1/sigma1 + rho * xiMinusMu1 * yiMinusMu2 / (sigma1Square * sigma2)) * zi_est; // 对sigma1的偏导

            H += J*J.transpose();
            b += -error * J;
            cost = error *error;
        }
        
        Eigen::Vector3d paramsGradient = H.ldlt().solve(b);

        if(isnan(paramsGradient[0])){
            std::cout<<"!!!!"<<std::endl;
            break;
        }

        for(int k=0;k<3;++k){
            params_est[k] += 0.01*paramsGradient[k]; 
            std::cout<<params_est[k]<<"---";
        }

        estimate->UpdateFunction([params_est](double x, double y) -> double {
            return  guassian2D(x,y,params_est); 
        });

        if(it>0 && cost>lastCost){
            std::cout<<"???"<<std::endl;
            break;
        }
       
        std::cout<<"    $$$"<<cost<< std::endl;
        lastCost = cost;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }


}



int main() {
    pvtk::Visualizer visualizer;
    std::thread userThread(userCodeSpace,std::ref(visualizer));
    visualizer.Run();
    userThread.join();
    return 0;
}

