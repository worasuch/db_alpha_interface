//
// Created by Carlos on May 2019
//

#ifndef DB_ALPHA_INTERFACE_H
#define DB_ALPHA_INTERFACE_H

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "listener.h"

using namespace std;

class Listener;

class Interface
{
    private:

        ofstream positions_csv;
        Listener * db_listener;
        int cnt = 0;

    public:

        Interface(int argc,char* argv[]);
        bool runListener();

        void saveDataToFile(vector<float> data, ofstream& file);
};

#endif
