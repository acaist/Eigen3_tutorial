#include "eigen_examples.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <memory>

void eigenBlockBlockSparse()
{
    Eigen::SparseMatrix<double> sparseMatrix(6, 6);
    sparseMatrix.insert(0, 0) = 1.0;
    sparseMatrix.insert(1, 1) = 2.0;
    sparseMatrix.insert(2, 2) = 3.0;
    sparseMatrix.insert(3, 3) = 4.0;
    sparseMatrix.insert(4, 4) = 5.0;
    sparseMatrix.insert(5, 5) = 6.0;

    std::cout << "Original sparse matrix:\n" << sparseMatrix << std::endl;

    // Extract a block from the sparse matrix
    Eigen::SparseMatrix<double> block = sparseMatrix.block(1, 1, 4, 4);

    // Extract a subblock from the block
    Eigen::SparseMatrix<double> subblock = block.block(1, 1, 2, 2);

    std::cout << "Subblock:\n" << subblock << std::endl;

}

bool eigenCheckSingularity() {
    Eigen::SparseMatrix<double> sparseMatrix(4, 4);
    sparseMatrix.insert(0, 0) = 1.0;
    sparseMatrix.insert(1, 1) = 2.0;
    sparseMatrix.insert(2, 2) = 0.0;
    sparseMatrix.insert(2, 1) = 3.0;
    sparseMatrix.insert(3, 3) = 4.0;

    int colToRemove = 1;
    //Eigen::SparseLU<Eigen::SparseMatrix<double>> lu(sparseMatrix);
    //lu.analyzePattern(sparseMatrix);
    //lu.factorize(sparseMatrix);
    //lu.compute(sparseMatrix);
    //Eigen::SparseMatrix<double> U = lu.matrixU();
    //int rank = U.rows; //U.rows();

    Eigen::MatrixXd matrix = sparseMatrix.toDense();
    std::cout<<"Matrix \n"<<sparseMatrix<<std::endl;

    Eigen::FullPivLU<Eigen::MatrixXd> luA(matrix);
    //int rank = luA.rank();

    bool isSingular = (luA.rank() < matrix.cols());
    std::cout<<"rank "<<luA.rank() <<" dim "<<matrix.cols()<<'\n';
    if (isSingular) {
        std::cout << "Sparse matrix is singular." << std::endl;
    } else {
        std::cout << "Sparse matrix is not singular." << std::endl;
    }

    // Compress the matrix to remove the marked rows
    
    Eigen::MatrixXd removedMatirx;
    removedMatirx = eigenRemoveOneCol(matrix, colToRemove);
    
    std::cout << "removed matrix: col=" << colToRemove<< std::endl << removedMatirx << std::endl;

    Eigen::FullPivLU<Eigen::MatrixXd> luAS(removedMatirx);
    isSingular = (luAS.rank() < removedMatirx.cols());
    std::cout << "removed is sigular ? " <<isSingular<<std::endl;
    return (luAS.rank() < removedMatirx.cols());
}

Eigen::MatrixXd
eigenRemoveOneCol(Eigen::MatrixXd& originalMatrix, const int colToRemove)
{
    /*
    Eigen::MatrixXd originalMatrixXD(4, 4);
    originalMatrixXD << 1, 2, 3, 4,
                      5, 6, 7, 8,
                      9, 10, 11, 12,
                      13, 14, 15, 16;

    Eigen::SparseMatrix<double> sparseMatrix(4, 4);
    sparseMatrix.insert(0, 0) = 1.0;
    sparseMatrix.insert(1, 1) = 2.0;
    sparseMatrix.insert(2, 2) = 3.0;
    sparseMatrix.insert(2, 1) = 3.0;
    sparseMatrix.insert(3, 3) = 4.0;

    Eigen::MatrixXd originalMatrix = sparseMatrix.toDense();
    */
    //auto originalMatrix = sparseMatrix;
    std::cout << "Original matrix:" << std::endl << originalMatrix << std::endl;

    int nrow = originalMatrix.rows();
    int ncol = originalMatrix.cols();
    //int colToRemove = 2;
    int indx1[4]={0, 0, nrow, colToRemove};
    int indx2[4]={0, colToRemove+1, nrow, ncol-(colToRemove+1)};
    // Split the matrix into two submatrices
    Eigen::MatrixXd subMatrix1 = originalMatrix.block(indx1[0],indx1[1],indx1[2],indx1[3]);
    Eigen::MatrixXd subMatrix2 = originalMatrix.block(indx2[0],indx2[1],indx2[2],indx2[3]);

    //std::cout << "Submatrix 1:" << std::endl << subMatrix1 << std::endl;
    //std::cout << "Submatrix 2:" << std::endl << subMatrix2 << std::endl;

    // Merge the two submatrices into a new matrix
    Eigen::MatrixXd mergedMatrix(4, ncol-1);
    mergedMatrix.block(indx1[0],indx1[1],indx1[2],indx1[3]) = subMatrix1;
    mergedMatrix.block(indx2[0],indx2[1]-1,indx2[2],indx2[3]) = subMatrix2;

    std::cout << "Merged matrix:" << std::endl << mergedMatrix << std::endl;
    return mergedMatrix;
}

int eigenDiagnalAdd()
{
    typedef Eigen::SparseMatrix<double> SparseMatrix;

    // Create a sparse matrix
    SparseMatrix sparseMatrix(3, 3);
    sparseMatrix.insert(0, 0) = 1.0;
    sparseMatrix.insert(1, 1) = 2.0;
    sparseMatrix.insert(2, 2) = 3.0;

    // Get the diagonal elements as a sparse vector
    SparseMatrix::DiagonalReturnType diagonal = sparseMatrix.diagonal();
    std::cout<<"original matrix \n"<<sparseMatrix<<'\n';
    // Add a small value to each diagonal element
     diagonal.array() += 0.1;

     // Get the diagonal elements as a DiagonalMatrix
    //Eigen::DiagonalMatrix<double, Eigen::Dynamic> diagonal(sparseMatrix);

    // Add a small value to each diagonal element
    //diagonal.diagonal().array() += 0.1;


    // Output the modified diagonal elements
    for (int i = 0; i < diagonal.size(); ++i) {
        std::cout << diagonal(i) << " ";
    }
    std::cout << std::endl;

    std::cout<<"diagnal added matrix \n"<<sparseMatrix<<'\n';

    return 0;
}


int eigenSVDdecompose() {
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::SparseLU<SparseMatrix> SparseLU;
    typedef Eigen::JacobiSVD<SparseMatrix> SparseSVD;

    // Create a sparse matrix
    SparseMatrix sparseMatrix(3, 3);
    sparseMatrix.insert(0, 0) = 1.0;
    sparseMatrix.insert(1, 1) = 2.0;
    sparseMatrix.insert(2, 2) = 0.0;
    sparseMatrix.insert(2, 1) = 1.0;


    // Perform SVD decomposition
    Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
    //std::cout << "A: \n" << A << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd B = sparseMatrix.toDense();
    std::cout << "original: \n" << B << std::endl;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd2(sparseMatrix.toDense(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    //Eigen::JacobiSVD<SparseMatrix> svd3(sparseMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto U = svd2.matrixU();
    auto V = svd2.matrixV();
    auto D = svd2.singularValues();

    std::cout <<"Diagnal: \n" << D <<std::endl;

    D.array() += 0.1;
    std::cout <<"Dignal added: \n" << D <<std::endl;

    Eigen::MatrixXd reconMat = U*D.asDiagonal()*V.transpose();

    std::cout << " reconstructed matrix \n"<<reconMat<<std::endl;

    Eigen::SparseMatrix<double> spview = reconMat.sparseView();
    SparseMatrix sparseMatrix2(2, 2);
    sparseMatrix2=reconMat.sparseView();
    std::cout<<" Sparse view \n"<<sparseMatrix2<<std::endl;


    return 0;
}


void eigenBlockRef()
{
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    SparseMatrix spmat(4, 4);
    spmat.insert(0, 0) = 1.0;
    spmat.insert(1, 1) = 2.0;
    spmat.insert(2, 2) = 3.0;
    spmat.insert(2, 1) = 1.0;
    spmat.insert(3, 3) = 4.0;

    std::cout<<" original matrix \n" <<spmat<<std::endl;

    eigenGetBlock(spmat);

    std::cout<<" blocked matrix \n" <<spmat<<std::endl;
}

void eigenGetBlock(Eigen::SparseMatrix<double>& matrix)
{
    matrix = matrix.block(0,0,2,2);
}



int eigenInvSpmat()
{
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::Triplet<double> Triplet;

    const int size = 3;
    std::vector<Triplet> coefficients;
    coefficients.reserve(6);
    coefficients.push_back(Triplet(0, 0, 1.0));
    coefficients.push_back(Triplet(0, 1, 2.0));
    coefficients.push_back(Triplet(1, 0, 3.0));
    coefficients.push_back(Triplet(1, 2, 4.0));
    coefficients.push_back(Triplet(2, 1, 5.0)); // 5
    coefficients.push_back(Triplet(2, 2, 6.0)); // 6

    SparseMatrix sparseMatrix(size, size);
    sparseMatrix.setFromTriplets(coefficients.begin(), coefficients.end());

    std::cout << "Original Matrix:" << std::endl << sparseMatrix.toDense() << std::endl;

    //Eigen::SparseLU<SparseMatrix> solver;
    Eigen::SparseLU<SparseMatrix> solver;
    //Eigen::SparseQR<SparseMatrix, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(sparseMatrix);

    if (solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed! May be matrix is singular " << std::endl;
        return 1;
    }

    SparseMatrix I(size, size);
    I.setIdentity();
    //SparseMatrix inverse = solver.solve(I);
    Eigen::MatrixXd inverse = solver.solve(I);

    if (solver.info() != Eigen::Success) {
        std::cerr << "Decomposition failed!" << std::endl;
        return 1;
    }
    
    std::cout << "Inverse Matrix:" << std::endl << inverse << std::endl;
    SparseMatrix  spinv = inverse.sparseView();
    SparseMatrix  valid = sparseMatrix*spinv;

    Eigen::MatrixXd validDen = sparseMatrix.toDense()*inverse;

    std::cout << "A*Ainv sparse Matrix:" << std::endl << valid << std::endl;
    std::cout << "A*Ainv dense Matrix:" << std::endl << validDen << std::endl;

    return 0;
}

int eigenReconstruct()
{
    typedef Eigen::SparseMatrix<double> SparseMatrix;

    SparseMatrix A(4, 4);  // Original sparse matrix
    A.insert(0, 0) = 1.0;
    A.insert(1, 1) = 2.0;
    A.insert(2, 2) = 3.0;
    A.insert(2, 1) = 1.0;
    A.insert(3, 3) = 4.0;

    // Create sub-matrices
    auto A11 = A.block(0, 0, 2, 2);
    SparseMatrix A12 = A.block(0, 2, 2, 2);
    SparseMatrix A21 = A.block(2, 0, 2, 2);
    SparseMatrix A22 = A.block(2, 2, 2, 2);

    A21=A11;

    // Combine sub-matrices into a new dense matrix
    Eigen::MatrixXd B(4, 4);
    B << A11.toDense(), A12.toDense(), A21.toDense(), A22.toDense();
    SparseMatrix Bsp = B.sparseView();

    std::cout << "B:\n" << Bsp << std::endl;

    //copy back
    //A.block(2, 2, 2, 2) = A21;
    
    std::cout << "A  orignal:\n" << A << std::endl;

    return 0;
}
