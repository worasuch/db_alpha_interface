// 
// Created by Carlos on May 2019
//

#include "interface.h"


int main(int argc,char* argv[])
{     
    Interface db_listener_interface(argc,argv);
    while(db_listener_interface.runListener()){}
    return(0);
}