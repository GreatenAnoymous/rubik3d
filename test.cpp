#include <thread>
#include <iostream>
#include <vector>



void foo(int z){
    for(int i=0;i<z;i++) std::cout<<"Thread using function object as callable\n";
}


class thread_obj{
public:
    void operator()(int x){
        for(int i=0;i<x;i++) std::cout<<"Thread using theead object as callable\n";
    }
};

void test(int m,int &k){

    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            k+=1;
        }
    }
}

int main(){
    int m=100,k1=0,k2=1,k3=2;
    std::thread th1(test,m,std::ref(k1));

    std::thread th2(test,m,std::ref(k2));


    std::thread th3(test,m,std::ref(k3));
    th1.join();
    th2.join();
    th3.join();
    std::cout<<k1<<" "<<k2<<" "<<k3<<std::endl;
    return 0;
}