#include <iostream>
#include<Eigen/Core>
#include<vector>
#include<random>
#include<Eigen/Dense>
#include<ctime>
using namespace std;
using namespace Eigen;
float step_t=0.1;

Matrix<float,4,1> start_pose;
Matrix<float ,4,4> state_transfer;
Matrix<float,4,4> corviance_process;
Matrix<float,2,4> observation_matrix;
Matrix<float,2,2> corviance_observation;
Matrix<float,4,4> cov_estimate;

void kalman(Matrix<float,2,1> measure)
{
    Matrix<float,4,1> pred_pose;
    pred_pose=state_transfer*start_pose;     //预测状态
    Matrix<float,4,4> predict_cov=state_transfer*cov_estimate*state_transfer.transpose()+corviance_process;  //预测状态的协防差
    Matrix<float,4,2> gain;  //kalman增益
    gain=predict_cov*observation_matrix.transpose()*(observation_matrix*predict_cov*observation_matrix.transpose()+corviance_observation).inverse();
    start_pose=pred_pose+gain*(measure-observation_matrix*pred_pose);  //更新估计
    cov_estimate=(MatrixXf::Identity(4,4)-gain*observation_matrix)*predict_cov;
    //cout<<"pred_pose:"<<pred_pose<<endl;
    //cout<<"predict_cov:"<<predict_cov<<endl;
    //cout<<"gain:"<<gain<<endl;
    //cout<<"cov_estimate:"<<cov_estimate<<endl;

}
int main() {

    start_pose<<0,0,0,0;  //行人初始位置

    state_transfer<<1,0,step_t,0,
            0,1,0,step_t,
            0,0,1,0,
            0,0,0,1;            //状态转移矩阵

    Matrix<float,4,1> a;
    a<<0.5*step_t*step_t,0.5*step_t*step_t,step_t,step_t;
    corviance_process=a*a.transpose()*0.25; //过程噪声的协防差矩阵

    cov_estimate<<1000,0,0,0,
            0,1000,0,0,
            0,0,1000,0,
            0,0,0,1000;

    observation_matrix<<0,0,1,0,
            0,0,0,1;  //观测矩阵

    corviance_observation<<0.1,0,
            0,0.1;  //观测噪声的协防差矩阵

    vector<Matrix<float,4,1>> vec_estimate;
    vector<Matrix<float,2,1>> vec_measure;

    vec_estimate.push_back(start_pose);
    static default_random_engine e(time(0));   //随机数的引擎
    static normal_distribution<float> n(0,10);  //定义一个正太分布
    int i=200;
    while(i--)
    {
        //float vx_measure=float(rand()%100)/100+10;
        //float vy_measure=float(rand()%100)/100+10;
        float vx_measure=n(e)+10;
        float vy_measure=n(e)+10;
        Matrix<float,2,1> measure;
        measure<<vx_measure,vy_measure;
        vec_measure.push_back(measure);
        kalman(measure);
        vec_estimate.push_back(start_pose);
        //cout<<"start_pose:"<<start_pose.transpose()<<endl;
       // cout<<"start_pose:"<<measure.transpose()<<endl;
    }
    freopen("/home/renjie/桌面/kalman/estimate.txt","w",stdout);
    for(int i=0;i<vec_estimate.size();i++)
        cout<<vec_estimate[i].transpose()<<endl;
    freopen("/home/renjie/桌面/kalman/measure.txt","w",stdout);
    for(int i=0;i<vec_measure.size();i++)
        cout<<vec_measure[i].transpose()<<endl;
    //cout<<"start_pose:"<<start_pose.transpose()<<endl;
    //std::cout << "Hello, World!" << std::endl;
    return 0;
}
