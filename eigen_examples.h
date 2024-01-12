#pragma once
#include <Eigen/Sparse>
#include <Eigen/Dense>
//#include <Eigen/SVD>


//eigen3 test functions
void eigenPerm();
void eigenBlock();
void eigenMap();
void eigenBlockBlock();
void eigenPermBlock();
void eigenBlockBlockSparse();
bool eigenCheckSingularity();
Eigen::MatrixXd eigenRemoveOneCol(Eigen::MatrixXd&, const int);
int eigenDiagnalAdd();
int eigenSVDdecompose();
void eigenBlockRef();
void eigenGetBlock(Eigen::SparseMatrix<double>& matrix);
int eigenInvSpmat();
int eigenReconstruct();
void eigenZeroDense();
void fillupDiagnal();


//other testing function
void stdVecProd();
void uniquePtrOwnship();

void sharePtrtest();
void stdmove();
std::tuple<std::string, int, int> GetUserAge(std::string);
void returnaTuple();
void testMultipleVec();
int foreachtest();

void opencv2test();
int embedPython();