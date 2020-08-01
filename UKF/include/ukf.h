//
// Created by renjie on 2020/4/29.
//
#include <Eigen/Core>
#include<Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

#ifndef UKF_UKF_H
#define UKF_UKF_H


namespace ukf {
    class ukf {
    public:
        ukf(Matrix<float, 5, 1> start_state, Matrix<float, 5, 5> start_cov, Matrix<float, 2, 2> processnoise,
            Matrix<float, 5, 5> measureNoise);

        void productSigma();

        void predict();

        void predictMeanAndcov();

        void measure();

        void measureMeanAndcov();

        Matrix<float ,5,1> update(Matrix<float, 5, 1> measure);

    private:
        Matrix<float, 5, 1> estimate_state;  //状态估计
        Matrix<float, 5, 5> estimate_cov;   //估计的协防差
        Matrix<float, 2, 2> processnoise_cov; //处理噪声的协防差
        Matrix<float, 7, 7> sigma_cov;   //sigma_point协防差矩阵7×7
        Matrix<float, 7, 15> sigma_points;   //sigma_point
        Matrix<float, 5, 15> predict_sigma;  //预测sigma_point
        Matrix<float, 5, 1> predict_mean;
        Matrix<float, 5, 5> predict_cov; //预测sigma_point的协防差
        Matrix<float, 5, 15> measure_sigma; //测量sigma_point
        Matrix<float, 5, 1> measure_mean;
        Matrix<float, 5, 5> measure_cov;   //测量sigma_point的协防差
        Matrix<float, 5, 5> measureNoise_cov;  //测量噪声的协防差

        Matrix<float, 5, 5> relate;  //互相关矩阵
        Matrix<float, 5, 5> gain;  //kalman增益

    };
}
#endif //UKF_UKF_H
