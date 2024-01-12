#include "eigen_examples.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <memory>

void stdmove()
{
   std::cout << "C++ Version: " << __cplusplus << std::endl;

   std::cout << "Hello World \n";
   std::string str = "hello";
   std::vector<double> vd={1.2,1.4,1.6};

   std::cout << "before move str: " << str << str.size() << std::endl;
   for(auto i : vd){
      std::cout<<i<<std::endl;
   }
  


   std::vector<std::string> vstr;
   vstr.push_back(std::move(str));
   
   std::cout << "after move str: " << str << str.size() << std::endl;
   std::cout << " vector : " << vstr.back() << std::endl;

   std::vector<double> vd_move=std::move(vd);

   std::cout <<"original vector size: "<<vd.size() <<" reciver vector size: "<<vd_move.size() <<'\n';
   for(auto i : vd_move){
       std::cout<<i<<std::endl;
   }
   
}

std::tuple<std::string, int, int> GetUserAge(std::string str="None")
{  
   //return std::make_tuple<std::string, int, int> (std::move(str+" LX"), 18, 0x09);
   return std::make_tuple(std::move(str+" LX"), 18, 0x09);
}

void returnaTuple()
{
   std::tuple<std::string, int, int> result = GetUserAge("June");

	std::string name;
	int age;
	int user_id;

	std::tie(name, age, user_id) = result; // <tuple> provide to resolve a tuple result

	std::cout << "查询结果：" << name << "	" << "年龄：" << age <<"	"<<"用户id:"<<user_id <<std::endl;

	return;
}

int foreachtest()
{
    std::vector<int> numbers = {5, 4, 3, 2, 1};

    // Lambda function to print each element
    auto printElement = [](int num) {
        std::cout << num << " ";
    };

    // Using std::for_each to iterate over the vector
    std::for_each(numbers.begin(), numbers.end(), printElement);

    std::cout << std::endl;

    return 0;

}

std::tuple<std::vector<int>, std::vector<double>, std::vector<std::string>> getMultipleVectors() {
    std::vector<int> intVector = {1, 2, 3, 4, 5};
    std::vector<double> doubleVector = {1.1, 2.2, 3.3, 4.4, 5.5};
    std::vector<std::string> stringVector = {"apple", "banana", "cherry", "date", "elderberry"};

    // Returning the vectors as a tuple using std::move
    return std::make_tuple(std::move(intVector), std::move(doubleVector), std::move(stringVector));
}

void testMultipleVec() {
    // Getting the multiple vectors from the function
    auto result = getMultipleVectors();
    std::vector<int> intVec;
    std::vector<double> doubleVec;
    std::vector<std::string> strVec;

   std::tie(intVec, doubleVec, strVec) = result;
   for(int i : intVec){
      std::cout<<i<<' ';
   }
   for(double i: doubleVec){
      std::cout<<i<<' ';
   }
   for(std::string s : strVec){
      std::cout<<s<<' ';
   }
   return;
}


void
stdVecProd()
{
    std::vector<int> perm = {0, 2, 1, 4, 3};
    std::vector<int> vector = {6, 7, 8, 9, 10};

    std::vector<int> result(vector.size());

    // Perform element-wise multiplication
    std::transform(perm.begin(), perm.end(), 
                   vector.begin(), result.begin(), 
                   [&vector](int a, int b) { return vector[a]; });

    // Print the result
    std::cout << "Result vector: ";
    for (int num : result) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

}




struct MyObject {
    int value;

    MyObject(int val) : value(val) {
        std::cout << "Constructing MyObject with value: " << value << std::endl;
    }

    ~MyObject() {
        std::cout << "Destructing MyObject with value: " << value << std::endl;
    }
};

void uniquePtrOwnship()
{
    std::unique_ptr<MyObject> obj1 = std::make_unique<MyObject>(1);
    std::unique_ptr<MyObject> obj2 = std::make_unique<MyObject>(2);
    //obj2 = std::move(obj1);  // Transfer ownership from obj1 to obj2

    // obj1 is now empty, and obj2 owns the MyObject instance
	std::cout<<"before move"<<'\n';
	obj2 = std::move(obj1);
	
	std::cout<<"before return"<<'\n';

}


