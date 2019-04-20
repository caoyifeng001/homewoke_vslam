#include <iostream>
using namespace std;
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include<ctime>

#define SIZE 100
int main()
{
cout<<"useEigen..."<<endl;
Eigen::Matrix<double, SIZE, SIZE>  A;
A = Eigen::MatrixXd::Random(SIZE, SIZE); 
//A = A.transpose() * A;

Eigen::Matrix<double, SIZE, 1> b;
b = Eigen::MatrixXd::Random(SIZE,1);


//inverse
clock_t time =clock();
Eigen::Matrix<double, SIZE, 1> x1;
x1  = A.inverse() * b;
//cout<<"Direct Inverse Result:\n"<<x1.matrix().transpose()<<endl;
cout<<"using time :"<<1000*(clock()-time)/(double)CLOCKS_PER_SEC<<" ms"<<endl;
//QR
time =clock();
Eigen::Matrix<double, SIZE, 1> x2;
x2 = A.fullPivHouseholderQr().solve(b);
//cout<<"QR Result:\n"<<x2.matrix().transpose()<<endl;
cout<<"using time :"<<1000*(clock()-time)/(double)CLOCKS_PER_SEC<<" ms"<<endl;
//Cholesky
time =clock();
Eigen::Matrix<double, SIZE, 1> x3;
x3 = A.ldlt().solve(b);
//cout<<"Cholesky LDLT Result:\n"<<x3.matrix().transpose()<<endl;
cout<<"using time :"<<1000*(clock()-time)/(double)CLOCKS_PER_SEC<<" ms"<<endl;

return 1;
}
