#include <iostream>
#include <ctime>

using namespace std;

int main(){
    clock_t x = clock();
    while (true){
        cout << x << endl;
        x = clock();
    }
}