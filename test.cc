#include <iostream>

int main() 
{
    // Print CPP Version:
    #ifdef __GNUC__ 
        std::cout << "GCC version: " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__ << std::endl; 
    #elif defined(_MSC_VER)
        std::cout << "MSVC version: " << _MSC_VER << std::endl; 
    #else
        std::cout << "Unknown compiler" << std::endl; 
    #endif

    if (__cplusplus == 202101L) std::cout << "C++23";
    else if (__cplusplus == 202002L) std::cout << "C++20";
    else if (__cplusplus == 201703L) std::cout << "C++17";
    else if (__cplusplus == 201402L) std::cout << "C++14";
    else if (__cplusplus == 201103L) std::cout << "C++11";
    else if (__cplusplus == 199711L) std::cout << "C++98";
    else std::cout << "pre-standard C++." << __cplusplus;
    std::cout << "\n";
    
}
