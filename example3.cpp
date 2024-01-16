#include "eigen_examples.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <memory>

void fillupDiagnal()
{
     // Create a vector with diagonal elements

    Eigen::VectorXd diagonal(3);
    diagonal << 1.0, 2.0, 3.0;
    //diagonal.setConstant(2e-5);

    // Create a matrix with the diagonal vector
    Eigen::MatrixXd matrix = diagonal.asDiagonal();

    // Print the matrix
    std::cout << " matrix \n" <<matrix << std::endl;

    matrix.diagonal()<<4, 5, 7;

     std::cout << " matrix \n"<<matrix << std::endl;


    matrix.diagonal() = diagonal;

     std::cout << " matrix \n"<<matrix << std::endl;

     int size = 3;
     matrix.diagonal(1).segment(0, size - 1).setConstant(42.0);

     std::cout << " matrix \n"<<matrix << std::endl;

}

void easySetoffdiagnal()
{
     int dim = 9 ;
     Eigen::VectorXd  vector(dim);
     vector.setConstant(2*2e-5);

     Eigen::MatrixXd matrix(dim, dim);

     matrix.diagonal() = vector;

     matrix.diagonal(1).segment(0, dim-1).setConstant(-2e-5);
     matrix.diagonal(-1).segment(0,dim-1).setConstant(-2e-5);
     matrix(0,0) = 1e-5;
     matrix(dim-1, dim-1) = 1e8;
     std::cout<<"matrix \n"<<matrix;
}

void eigenGenerateExp()
{
    const double start = -10;
    const double end = 12;
    const int size = 221;     // Specify the desired size of the vector

     // Generate a vector using Eigen with values from 0.01 to 1e12
     Eigen::VectorXd vec = Eigen::VectorXd::LinSpaced(size, start, end);

     // Eigen::VectorXd vec2 = vec.array().pow(-10);
      Eigen::VectorXcd vec_complex = vec.unaryExpr([](double elem) {  return std::complex<double>(std::pow(10, elem), 0); });

    // Display the vector
    //std::cout << "Vector:\n" << vec_complex << std::endl;

    Eigen::Matrix<double, 3, 3> realMatrix;
     realMatrix << 1, 2, 3,
                  4, 5, 6,
                  7, 8, 9;

     Eigen::VectorXcd complexVec(3);
    complexVec << std::complex<double>(1, 2),
                  std::complex<double>(3, 4),
                  std::complex<double>(5, 6);

    Eigen::MatrixXcd result_complex = realMatrix.diagonal().asDiagonal() * complexVec;

    std::cout << "Vector:\n" << result_complex.cols() <<" \n"<<result_complex << std::endl;
     
     // Add complex diagonal to the complex matrix
    Eigen::MatrixXcd complexMatrix = realMatrix.cast<std::complex<double>>();    
    complexMatrix.diagonal() += result_complex;
    std::cout << "Matrix complex:\n" << complexMatrix <<" \n" << std::endl;

    Eigen::MatrixXcd inverseMatrix = complexMatrix.inverse();
    std::cout << "Matrix complex inverse:\n" << inverseMatrix <<" \n" << std::endl;


    std::vector<double> realVector = {1.0, 2.0, 3.0, 4.0, 5.0};
    
    Eigen::VectorXcd complexVector = Eigen::Map<Eigen::VectorXd>(realVector.data(), realVector.size()).cast<std::complex<double>>();
    complexVector = complexVector.array()*2.0;
    // Print the complexVector
    std::cout << "Complex Vector:\n" << complexVector << std::endl;

}

void eigenuaryExpr()
{
     std::vector<double> Barr = {1, 2, 3, 4, 5, 6 , 7, 8};
     
     int end = Barr.size(), mid = 4;
     std::vector<double> subBarr(Barr.begin() + mid, Barr.begin() + end);
     for(auto& item : subBarr){
          std::cout << item <<' ';
     }

     Eigen::VectorXd vec(end-mid);
     auto it = Barr.begin() + mid;

     vec = vec.unaryExpr([&it](double element){ return *it++;});

     std::cout<<" \nvec uaryExpr \n"<<vec<<'\n';
}