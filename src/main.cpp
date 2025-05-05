#include <iostream>
#include "maxon.h"
using namespace std;


int main ()
{
    cout << "cout_debug";
    cout <<"cout_debug2";
    Control bancata1;
    int intero=0;
    intero++;
    cout << intero;
    bool bancata_=false;

    bancata_=bancata1.InitNetwork("USB0","1000000");

    cout << bancata_;
    













    return 0;
    
    
} 
