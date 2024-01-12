#include "eigen_examples.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <memory>


void eigenPerm(){
    // Create a sample sparse matrix
    Eigen::SparseMatrix<double, Eigen::ColMajor> matrix(3, 3); //default: Column major, CSC
    matrix.insert(0, 0) = 1.0;
    matrix.insert(1, 2) = 2.0;
    matrix.insert(2, 1) = 3.0;
    matrix.makeCompressed();

    std::cout << "Original matrix:" << std::endl;
    std::cout << matrix.leftCols(3) << std::endl;

    // Create a permutation vector to shuffle the rows and columns
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm(3);
    std::cout<<"perm "<<perm.indices()<<std::endl;
    perm.indices() << 2, 0, 1;

    Eigen::VectorXi data;
    //std::cout<<" permutation "<<perm<<std::endl;

    // Permute the matrix using the permutation vector
    matrix = perm.transpose() * matrix * perm;

    std::cout << "Permuted matrix:" << std::endl;
    std::cout << matrix << std::endl;

   double v=1.2;
   auto* ptr = &v;
   auto p = static_cast<void*>(ptr);
}

void eigenBlock()
{
    using SparseMatrixType = Eigen::SparseMatrix<double>;

    // Create a sparse matrix
    SparseMatrixType matrix(6, 6);
    matrix.insert(1, 1) = 1.0;
    matrix.insert(2, 2) = 2.0;
    matrix.insert(3, 3) = 3.0;
    matrix.insert(4, 4) = 4.0;
    matrix.insert(5, 5) = 5.0;
    matrix.makeCompressed();

    std::cout << "entire-block:\n" << matrix << std::endl;

    // Define the sub-block sizes and positions
    const int blockSize = 2;
    const int numBlocks = matrix.rows() / blockSize;

    // Create a container to hold the sub-blocks
    std::vector<SparseMatrixType> subBlocks(numBlocks * numBlocks);

    // Split the matrix into sub-blocks
    for (int i = 0; i < numBlocks; ++i) {
        for (int j = 0; j < numBlocks; ++j) {
            int startRow = i * blockSize;
            int startCol = j * blockSize;
            int endRow = (i + 1) * blockSize;
            int endCol = (j + 1) * blockSize;

            subBlocks[i * numBlocks + j] = matrix.block(startRow, startCol, endRow - startRow, endCol - startCol);
        }
    }

    // Access and operate on the sub-blocks
    for (const auto& subBlock : subBlocks) {
        std::cout << "Sub-block:\n" << subBlock << std::endl;
        // Perform operations on the sub-block
    }

    return;
}


void eigenMap() {
    std::vector<int> stdVec = {1, 2, 3, 4, 5};
    
    // Create an Eigen::Map to the std::vector's data
    Eigen::Map<Eigen::VectorXi> eigenVec(stdVec.data(), stdVec.size());

    stdVec[0]=10;
    std::vector<int>().swap(stdVec);
    std::cout << "Eigen vector:\n" << eigenVec << std::endl;

    return;
}

void eigenBlockBlock()
{
        Eigen::MatrixXi matrix(6, 6);
    matrix <<
        1, 2, 3, 4, 5, 6,
        7, 8, 9, 10, 11, 12,
        13, 14, 15, 16, 17, 18,
        19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29, 30,
        31, 32, 33, 34, 35, 36;

    std::cout << "Original matrix:\n" << matrix << std::endl;

    // Extract a block from the original matrix
    Eigen::Block<Eigen::MatrixXi> block = matrix.block(1, 1, 4, 4);
    std::cout << "Extracted block:\n" << block << std::endl;

    // Extract a subblock from the block
    Eigen::Block<Eigen::Block<Eigen::MatrixXi>> subblock = block.block(1, 1, 2, 2);

    std::cout << "Extracted subblock of a block:\n" << subblock << std::endl;

    // Get the original matrix from the block
    Eigen::MatrixXi matrixFromBlock = subblock.matrix();

    std::cout << "original Matrix of subblock:\n" << matrixFromBlock << std::endl;

    return;
}

void eigenPermBlock()
{
     Eigen::SparseMatrix<double> sparseMatrix(4, 4);
    sparseMatrix.insert(0, 0) = 1.0;
    sparseMatrix.insert(1, 2) = 2.0;
    sparseMatrix.insert(2, 1) = 3.0;
    sparseMatrix.insert(3, 3) = 4.0;

    //std::cout << "Original sparse matrix:\n" << sparseMatrix << std::endl;

    // Extract a block from the sparse matrix
    Eigen::SparseMatrix<double> block = sparseMatrix.block(1, 1, 3, 3);

    std::cout << "Sparse matrix block:\n" << block << std::endl;
    // Perform the permutation on the block
    // Generate a permutation vector
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> permutation(block.rows());
    //permutation.setIdentity();
    //permutation.applyTranspositionOnTheRight(0, 1);  // Example permutation: swap rows 0 and 1
    std::vector<int> stdvec = {0,2,1};
    Eigen::Map<Eigen::VectorXi> eigvec(stdvec.data(), stdvec.size());
    permutation.indices() = eigvec;
    // Apply the permutation to the block
    block = permutation.transpose() * block *permutation;

    std::cout << "Sparse matrix with permuted block:\n" << block << std::endl;
    
}



void eigenZeroDense()
{

    Eigen::MatrixXd mat(5,5); // Dense matrix of doubles, initialized with zeros

    std::cout << "Zero matrix:\n" << mat << std::endl;

    Eigen::MatrixXd sub1(2, 2), sub2(2,2), sub3(2,2);
    sub1 << 1, 2,
            3, 4;
    sub2 << 5, 6,
            7, 8;
    sub3 << 9, 10,
            11, 12;
    Eigen::MatrixXd combined(6, 6), zeros(2,2), matrix;
    //combined.block(0, 0, 2, 2) = sub1;

    combined<< sub1, zeros, sub1,
               zeros, sub2, sub1,
               sub1, sub1, sub3;

     std::cout << "combine matrix:\n" << combined <<'\n';
    
    std::cout <<" default \n"<<  matrix.cols()<<'\n';

    return;
}