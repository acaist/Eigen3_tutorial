#include "eigen_examples.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <memory>


void eigenCombineSparse()
{
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::Triplet<double> Triplet;

    // Create four small sparse matrices
    SparseMatrix A(2, 2);
    A.insert(0, 0) = 1.0;
    A.insert(1, 1) = 2.0;
    std::cout<<"A block\n"<<A<<std::endl;

    SparseMatrix B(2, 2);
    B.insert(0, 1) = 3.0;
    B.insert(1, 0) = 4.0;
     std::cout<<"B block\n"<<B<<std::endl;

    SparseMatrix C(2, 2);
    C.insert(0, 0) = 5.0;
    C.insert(1, 1) = 6.0;
     std::cout<<"C block\n"<<C<<std::endl;

    SparseMatrix D(2, 2);
    D.insert(0, 1) = 7.0;
    D.insert(1, 0) = 8.0;
     std::cout<<"D block\n"<<D<<std::endl;

    // Convert smaller matrices to triplets
    std::vector<Triplet> triplets;
    triplets.reserve(A.nonZeros() + B.nonZeros() + C.nonZeros() + D.nonZeros());

    auto addBlockToTriplets = [&triplets](const SparseMatrix& block, int rowOffset, int colOffset) {
        for (int k = 0; k < block.outerSize(); ++k) {
            for (SparseMatrix::InnerIterator it(block, k); it; ++it) {
                triplets.emplace_back(it.row() + rowOffset, it.col() + colOffset, it.value());
            }
        }
    };
    
    int offset_row[3] = {0, A.rows(), A.rows()+B.rows()};
    int offset_col[3] = {0, A.cols(), A.cols()+B.cols()};

    addBlockToTriplets(A, offset_row[0], offset_col[0]);
    addBlockToTriplets(B, offset_row[0], offset_col[1]);
    addBlockToTriplets(C, offset_row[0], offset_col[2]);


    addBlockToTriplets(B, offset_row[1], offset_col[0]);
    addBlockToTriplets(C, offset_row[1], offset_col[1]);
    addBlockToTriplets(D, offset_row[1], offset_col[2]);

    addBlockToTriplets(C, offset_row[2], offset_col[0]);
    addBlockToTriplets(D, offset_row[2], offset_col[1]);
    addBlockToTriplets(A, offset_row[2], offset_col[2]);

    // Build the larger sparse matrix from triplets
    int nrows = A.rows()+B.rows()+C.rows();
    int ncols = A.cols()+B.cols()+C.cols();

    SparseMatrix combined(nrows, ncols);

    combined.setFromTriplets(triplets.begin(), triplets.end());

    SparseMatrix moveSparse((combined));
    moveSparse = std::move(combined);
    

    // Print the combined matrix
    std::cout <<"moved matrix\n"<< combined << std::endl;
     std::cout << " moved to sparse matrix \n"<<moveSparse << std::endl;
    

    return;
}