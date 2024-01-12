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

    // Create a matrix with the diagonal vector
    Eigen::MatrixXd matrix = diagonal.asDiagonal();

    // Print the matrix
    std::cout << matrix << std::endl;

    matrix.diagonal()<<4, 5, 7;

     std::cout << matrix << std::endl;

}