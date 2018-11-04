#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstring>
#include "pqp/include/PQP.h"
#include "quaternion.h"

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
	PQP_Model room=readmodel("room");
	PQP_Model piano=readmodel("piano");
	std::cout<<"load success"<<std::endl;
	PQP_REAL T1[3];
   	// define rotation of model 1
	PQP_REAL R1[3][3];
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            if(i==j)
            {
               	R1[i][j] = 1;
            }
            else
            {
                R1[i][j] = 0;
          	}
        }
        T1[i] = 0;
    }
    // define translation of model 2
    PQP_REAL T2[3] = {3, 4, 2.7};
    // define rotation of model 2
    PQP_REAL R2[3][3];
    for(int i=0; i<3; i++)
    {
       	for(int j=0; j<3; j++)
        {
            	if(i==j)
            	{
              		R2[i][j] = 1;
            	}
            	else
            	{	
                	R2[i][j] = 0;
            	}
       	}
    }
    // Do collision check
    PQP_CollideResult cres;
    PQP_Collide(&cres, R1, T1, &piano, R2, T2, &room);
    int colliding = cres.Colliding();
    int num_tri = cres.NumTriTests();
    std::cout<<"Is collide? "<<colliding<<std::endl;
	return 0;
}
