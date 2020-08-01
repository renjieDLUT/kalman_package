//
// Created by renjie on 2020/4/29.
//
#include <iostream>
#include<string>
#include<sstream>
#include<vector>
#include<ukf.h>
#include<Eigen/Core>
#include<Eigen/Dense>
using namespace std;
//using namespace ukf;
using namespace Eigen;
int main()
{
    Matrix<float, 5, 1> start_state=Matrix<float, 5, 1>::Zero();
    Matrix<float, 5, 5> start_cov=Matrix<float, 5, 5>::Identity(5,5)*1;
    Matrix<float, 2, 2> processnoise=Matrix<float, 2, 2>::Identity(2,2)*0.02;
    Matrix<float, 5, 5> measureNoise=Matrix<float, 5, 5>::Identity(5,5)*0.02;
    ukf::ukf test(start_state,start_cov,processnoise,measureNoise);
    freopen("/home/renjie/桌面/kalman/UKF/data/data_synthetic.txt","r",stdin);
    string s,s1;
    float f;
    vector<Matrix<float,5,1>> result;
    int i=2;
    while(getline(cin,s))
    {
        stringstream ss1,ss2;
        vector<float> vec_L,vec_R,ver;
        ss1<<s;
        ss1>>s1;
        while(ss1>>f)
        {
            vec_L.push_back(f);
        }
        getline(cin,s);
        ss2<<s;
        ss2>>s1;
        while(ss2>>f)
        {
            vec_R.push_back(f);
        }

        MatrixXf measure(5,1);
        measure<<vec_L[0],
                 vec_L[1],
                 vec_R[0],
                 vec_R[1],
                 vec_R[2];

        test.productSigma();
        test.predict();
        test.predictMeanAndcov();
        test.measure();
        test.measureMeanAndcov();
        test.update(measure);
        result.push_back(test.update(measure));
    }
    freopen("/home/renjie/桌面/kalman/UKF/data/result.txt","w",stdout);
    for(int i=0;i<result.size();i++)
        cout<<result[i].transpose()<<endl;
    /*
    cin.clear();
    freopen("/home/renjie/桌面/kalman/UKF/data/data_synthetic.txt","r",stdin);
    cin>>s;
    cout<<s;
    */
    return 0;
}
