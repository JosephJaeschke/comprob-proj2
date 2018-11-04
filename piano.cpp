#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstring>
#include "pqp/include/PQP.h"
#include "quaternion.h"

PQP_Model room;
PQP_Model piano;

/*
 * Read trigles from txt file
 * Build PQP model using trigles
 * Return PQP model
 * */
PQP_Model readmodel(std::string name)
{
    PQP_Model m;
    m.BeginModel();
    // read from model file
    std::ifstream infile;
    std::string file_name = name + ".txt";
    char file_name_char[file_name.size() + 1];
    strcpy(file_name_char, file_name.c_str());
    infile.open(file_name_char);
    if(!infile)
    {
        std::cout<<"Wrong model name for collision checker"<<std::endl;
        exit(EXIT_FAILURE);
    }
    int tri_num;
    infile >> tri_num;
    std::cout<<"number of triangles for "<<name<<": "<<tri_num<<std::endl;
    for(int num=0; num<tri_num; num++)
    {
        PQP_REAL p[3][3];
        for(int r=0; r<3; r++)
        {
            for(int c=0; c<3; c++)
            {
                double v;
                infile>>v;
                p[r][c] = v;
            }
        }
        m.AddTri(p[0], p[1], p[2], num);
    }
    infile.close();
    m.EndModel();
    std::cout<<"model "<<name<<" loaded..."<<std::endl;
    return m;
}

int main()
{
	room=readmodel("room");
	piano=readmodel("piano");
	return 0;
}
