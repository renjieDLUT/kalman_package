//
// Created by renjie on 2020/4/29.
//
#include <ukf.h>
float lamda1=0.2,det=0.1;
int count1=0,count2=0;
namespace ukf {
    ukf::ukf(Matrix<float, 5, 1> start_state, Matrix<float, 5, 5> start_cov, Matrix<float, 2, 2> processnoise,
             Matrix<float, 5, 5> measureNoise) {
        estimate_state = start_state;
        estimate_cov = start_cov;
        processnoise_cov = processnoise;
        processnoise_cov(0,0)=2;
        processnoise_cov(1,1)=0.3;
        measureNoise_cov = measureNoise;


    }

    void ukf::productSigma() {
        sigma_cov=Matrix<float,7,7>::Zero();
        sigma_cov.topLeftCorner(5, 5) = estimate_cov;
        sigma_cov(5, 5) = processnoise_cov(0, 0);
        sigma_cov(6, 6) = processnoise_cov(1, 1);
        //cout<<"renjie"<<endl;
        Matrix<float, 7, 1> point;
        point.topLeftCorner(5, 1) = estimate_state;
        point(5, 0) = 0;
        point(6, 0) = 0;
        sigma_points.col(0) = point;
        MatrixXd B = sigma_cov.cast<double>().llt().matrixL();  //类型转化
        MatrixXf A=B.cast<float>();
        for (int i = 1; i <= 7; i++) {
            //Matrix<float, 7, 1> p;
            sigma_points.col(i) = point + sqrt(lamda1 + 7) * A.col(i - 1);
            sigma_points.col(i + 7) = point - sqrt(lamda1 + 7) * A.col(i - 1);
        }

    }

    void ukf::predict() {
        for (int i = 0; i < 15; i++) {
            float x = sigma_points.col(i)(0);
            float y = sigma_points.col(i)(1);
            float v = sigma_points.col(i)(2);
            float theta = sigma_points.col(i)(3);
            float omiga = sigma_points.col(i)(4);
            float a = sigma_points.col(i)(5);
            float omiga_dot = sigma_points.col(i)(6);

            MatrixXf term1 = MatrixXf(5, 1);
            MatrixXf term2 = MatrixXf(5, 1);
            if (abs(omiga) < 0.0001) {
                term1 << v * cos(theta) * det, v * sin(theta) * det, 0, omiga * det, 0;
            }
            else {
                term1 << v / omiga * sin(det * omiga + theta) - v / omiga * sin(theta),
                        -v / omiga * cos(det * omiga + theta) + v / omiga * cos(theta),
                        0,
                        omiga * det,
                        0;
            }
            term2 << 0.5 * det * det * a * cos(theta),
                    0.5 * a * det * det * sin(theta),
                    det * a,
                    0.5 * omiga_dot * det * det,
                    det * omiga_dot;

            predict_sigma.col(i) = sigma_points.col(i).head(5) + term1 + term2;
        }
    }

    void ukf::predictMeanAndcov() {
        Matrix<float, 5, 1> state_sum = MatrixXf::Zero(5, 1);
        float weight[15];
        weight[0] = lamda1 / (lamda1 + 7);
        for (int i = 1; i < 15; i++)
            weight[i] = 1 / (2 * (lamda1 + 7));
        for (int i = 0; i < 15; i++)
            state_sum += weight[i] * predict_sigma.col(i);
        predict_mean = state_sum;
        predict_cov = Matrix<float, 5, 5>::Zero();
        for (int i = 0; i < 15; i++) {
            predict_cov += weight[i] * (predict_sigma.col(i) - predict_mean) *
                           (predict_sigma.col(i) - predict_mean).transpose();
        }
        //count1+=1;
        //cout<<"count1:"<<count1<<predict_mean<<predict_cov<<endl;
        //cout<<"sigma_point:"<<sigma_points<<endl;
       // cout<<"predict_sigma:"<<predict_sigma<<endl;
    }

    void ukf::measure() {
        for (int i = 0; i < 15; i++) {
            float x = predict_sigma.col(i)(0);
            float y = predict_sigma.col(i)(1);
            float v = predict_sigma.col(i)(2);
            float theta = predict_sigma.col(i)(3);
            float omiga = predict_sigma.col(i)(4);

            Matrix<float, 5, 1> measure;
            measure(0, 0) = x;  //lidar的测量x
            measure(1, 0) = y; //lidar的测量y
            measure(2, 0) = sqrt(x * x + y * y); //lidar的测量y
            measure(3, 0) = atan2(y, x);
            if(x * x + y * y<0.00001)
                 measure(4, 0) = 0;
            else
                measure(4, 0) = (x * cos(theta) * v + y * sin(theta) * v) / sqrt(x * x + y * y);
            measure_sigma.col(i) = measure;
        }
    }

    void ukf::measureMeanAndcov() {
        Matrix<float, 5, 1> state_sum = MatrixXf::Zero(5, 1);
        float weight[15];
        weight[0] = lamda1 / (lamda1 + 7);
        for (int i = 1; i < 15; i++)
            weight[i] = 1 / (2 * (lamda1 + 7));
        for (int i = 0; i < 15; i++)
            state_sum += weight[i] * measure_sigma.col(i);
        measure_mean = state_sum;
        measure_cov = Matrix<float, 5, 5>::Zero();
        for (int i = 0; i < 15; i++) {
            measure_cov += weight[i] * (measure_sigma.col(i) - measure_mean) *
                           (measure_sigma.col(i) - measure_mean).transpose();
        }
        measure_cov = measure_cov + measureNoise_cov;
        //count2+=1;
        //cout<<"count2:"<<count2<<measure_sigma<<endl<<measure_mean<<measure_cov<<endl;

    }

    Matrix<float ,5,1> ukf::update(Matrix<float, 5, 1> measure) {
        Matrix<float, 5, 1> state_sum = MatrixXf::Zero(5, 1);
        float weight[15];
        weight[0] = lamda1 / (lamda1 + 15);
        for (int i = 1; i < 15; i++)
            weight[i] = 1 / (2 * (lamda1 + 15));
        relate=Matrix<float,5,5>::Zero();
        for (int i = 0; i < 15; i++) {
            relate += weight[i] * (predict_sigma.col(i) - predict_mean) *
                      (measure_sigma.col(i) - measure_mean).transpose();
        }
        gain = relate * measure_cov.inverse();
        estimate_state = predict_mean + gain * (measure - measure_mean);
        estimate_cov = predict_cov - gain * measure_cov * gain.transpose();
        return estimate_state;
    }
}