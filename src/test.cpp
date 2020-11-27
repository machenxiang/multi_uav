#include<iostream>
using namespace std;

int main(){

   for(int i=0;i<20;i++ ){
HOME:
    if(i==15){
        i++;
        goto HOME;
    }
    cout<<"i="<<i<<endl;
}

    return 0;
}