//
// Created by Carlos on May 2019
//

#include "interface.h"


Interface::Interface(int argc,char* argv[])
{
    db_listener = new Listener(argc, argv);
    cnt = 0;
    positions_csv.open("/home/charlie/Workspace/AI/MasterThesis/experiments/learned_positions.csv");
    db_listener->initMsg();
}


void Interface::saveDataToFile(vector<float> data, ofstream& file)
{
    file << data[0];
    for(size_t i=1; i<data.size(); i++)
    {
        file << ", " << data[i];
    }
    file << endl;
}


bool Interface::runListener()
{
    if(ros::ok())
    {
        db_listener->readPostions();

        if(cnt > 100)
        {
            saveDataToFile(db_listener->jointPositions, positions_csv);
        }

        cnt++;
        db_listener->rosSpinOnce();
        return true; 
    }
    else
    {
        positions_csv.close();
        cout << "Data saved to /Workspace/AI/MasterThesis/experiments/learned_positions.csv" << endl; 
        delete db_listener;
        return false;
    }
    
}