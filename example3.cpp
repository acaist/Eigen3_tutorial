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

     Eigen::SparseMatrix<double> sparseMatrix = vec.sparseView();

     std::cout << "Sparse Matrix:\n" << sparseMatrix << std::endl;
     //sparseMatrix.insert(0, 0) = 1.0;
     //sparseMatrix.insert(1, 1) = 2.0;
     //sparseMatrix.insert(2, 2) = 3.0;
     //sparseMatrix.makeCompressed();

    Eigen::VectorXd denseVector = sparseMatrix.toDense();

    std::cout << "Dense Vector:\n" << denseVector << std::endl;

     Eigen::MatrixXd matrix(3, 3);
     matrix << 1, 2, 3,
               4, 5, 6,
               7, 8, 9;

     double scalar = 2.0;
    // Method 1: Using the * operator
    Eigen::MatrixXd result = matrix * scalar;

    std::cout << "Result using * operator:\n" << result << std::endl;

    // Method 2: Using the array() method
    Eigen::MatrixXd result_array = matrix.array() * scalar;

    std::cout << "Result using array() method:\n" << result_array << std::endl;
}



int eigenFillVector() {
    Eigen::VectorXd vec(3);  // Create a vector of size 3

    // Using the `=` operator
    vec = Eigen::VectorXd::Constant(3, 5.0);  // Assign all elements to the value 5.0

    // Using the `setConstant()` method
    vec.setConstant(7.0);  // Assign all elements to the value 7.0

    vec(1) = 100;
    // Printing the vector
    std::cout << "Vector: \n" << vec << std::endl;

    return 0;
}


int setFromTripletList() {
    // Create an empty sparse matrix of size 4x4
    Eigen::SparseMatrix<double> sparseMatrix(4, 4);

    // Define a list of triplets to represent non-zero entries
    std::vector<Eigen::Triplet<double>> triplets;

    // Add some non-zero entries
    triplets.emplace_back(0, 0, 1.0);  // (row, col, value)
    triplets.emplace_back(1, 1, 1.0);
    triplets.emplace_back(1, 0, -1.0);
    triplets.emplace_back(0, 1, -1.0);
    
    triplets.emplace_back(0, 0, 3.0);
    triplets.emplace_back(2, 2, 3.0);
    triplets.emplace_back(2, 0, -3.0);
    triplets.emplace_back(0, 2, -3.0);
    
    triplets.emplace_back(0, 0, 5.0);
    triplets.emplace_back(3, 3, 5.0);
    triplets.emplace_back(3, 0, -5.0);
    triplets.emplace_back(0, 3, -5.0);

    // Set the non-zero entries in the sparse matrix
    sparseMatrix.setFromTriplets(triplets.begin(), triplets.end());

    int dim = sparseMatrix.cols();
    sparseMatrix = sparseMatrix.bottomRightCorner(dim-1, dim -1);
    // Print the sparse matrix
    std::cout << sparseMatrix << std::endl;
    std::cout << sparseMatrix.cols() << std::endl;

    return 0;
}

void eigenSortVector()
{
    Eigen::VectorXi vec(5);
    vec << 5, 2, 8, 1, 3;

    //std::sort(vec.data(), vec.data() + vec.size());
    std::sort(vec.data(), vec.data() + vec.size());

    std::cout << "Sorted vector in ascending order: " << vec << std::endl;
}

void eigenExtractBlocks()
{
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> matrix(4, 6);
    matrix << 1, 2, 3, 4, 5, 6,
              7, 8, 9, 10, 11, 12,
              13, 14, 15, 16, 17, 18,
              19, 20, 21, 22, 23, 24;

    // Extracting a block of columns from index 1 to 3 (inclusive)
   auto block = matrix.middleCols(1, 3);

    // Modifying the block
    block *= 2;

    // Printing the modified block and the original matrix
    std::cout << "Modified block:\n" << block << std::endl << std::endl;
    std::cout << "Original matrix:\n" << matrix << std::endl;

}

 void eigenTrilow() 
  {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix(3, 3);
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;

    Eigen::MatrixXd lowerView = matrix.triangularView<Eigen::Lower>();

    std::cout << "Original Matrix:" << std::endl;
    std::cout << matrix << std::endl;

    std::cout << "Lower Triangular View:" << std::endl;
    std::cout << lowerView << std::endl;

   Eigen::SparseMatrix<double> sparseMatrix(3, 3);
    sparseMatrix.insert(0, 0) = 1.0;
    sparseMatrix.insert(1, 1) = 2.0;
    sparseMatrix.insert(2, 0) = 3.0;
    sparseMatrix.insert(2, 2) = 4.0;
    sparseMatrix.makeCompressed();

    Eigen::SparseMatrix<double> lowerViewSP = sparseMatrix.triangularView<Eigen::Lower>();

    int columnIndex = 0;  // Index of the column to iterate over

    std::cout << "Non-zero elements in column " << columnIndex << ":" << std::endl;
    for (Eigen::SparseMatrix<double>::InnerIterator it(lowerViewSP, columnIndex); it; ++it) {
        std::cout << "Row: " << it.row() << ", Value: " << it.value() << std::endl;
    }


    //int columnIndex = 0;  // Index of the column to retrieve

    Eigen::SparseVector<double> firstColumn = sparseMatrix.innerVector(columnIndex);

    std::cout << "First column of the sparse matrix:" << std::endl;
    for (int i = 0; i < firstColumn.size(); ++i) {
        std::cout << "Row: " << i << ", Value: " << firstColumn.coeff(i) << std::endl;
    }


}



