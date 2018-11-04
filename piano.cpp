#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstring>
#include "pqp/include/PQP.h"
#include "quaternion.hpp"

float MAX_X=10,MAX_Y=10,MAX_Z=10;
float W_T=1, W_R=1;

typedef struct configuration
{
	PQP_REAL x;
	PQP_REAL y;
	PQP_REAL z;
	quat q;
} config;

/*
	sample R^3 uniformly at random and sample a quaternion at random
*/
config sample()
{
	config c;
	c.q=Quat::random();
	c.x=(PQP_REAL)rand()/(PQP_REAL)(RAND_MAX/MAX_X);
	c.y=(PQP_REAL)rand()/(PQP_REAL)(RAND_MAX/MAX_Y);
	c.z=(PQP_REAL)rand()/(PQP_REAL)(RAND_MAX/MAX_Z);
	return c;
}

/*
	distance metric for SE(3)
	weighted sum of translational distance and rotational distance
*/
float distance(config c1, config c2)
{
	float tDist=sqrt((c1.x-c2.x)*(c1.x-c2.x)+(c1.y-c2.y)*(c1.y-c2.y)+(c1.z-c2.z)*(c1.z-c2.z)); 
	float rDist=Quat::distance(c1.q,c2.q);
	return W_T*tDist+W_R*rDist;
}

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
    // create pqp model
    PQP_Model m1 = readmodel("room");
    PQP_Model m2 = readmodel("piano");
    // define translation of model 1
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
	/*
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
	*/
	quat q=Quat::random();
	PQP_REAL R2[3][3];
	PQP_REAL (*pass)[3][3]=&R2;
	Quat::toMatrix(q,pass);
    // Do collision check
    PQP_CollideResult cres;
    PQP_Collide(&cres, R1, T1, &m1, R2, T2, &m2);
    int colliding = cres.Colliding();
    int num_tri = cres.NumTriTests();
    std::cout<<"Is collision? "<<colliding<<std::endl;
    return 0;
}
