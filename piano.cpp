#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
#include <bits/stdc++.h>
#include "pqp/include/PQP.h"
#include "quaternion.hpp"

#define PRM_ITR 200

using namespace std;

float MAX_X=10,MAX_Y=10,MAX_Z=10;
float W_T=1, W_R=1;

typedef struct configuration
{
	PQP_REAL x;
	PQP_REAL y;
	PQP_REAL z;
	quat q;
	double dist;
} state;

typedef struct edgeBetween
{
		state s1;
		state s2;
} edge;

/*
	sample SE(3) uniformly at random and sample a quaternion at random
*/
state sample()
{
	state s;
	s.q=Quat::random();
	s.x=(PQP_REAL)rand()/(PQP_REAL)(RAND_MAX/MAX_X);
	s.y=(PQP_REAL)rand()/(PQP_REAL)(RAND_MAX/MAX_Y);
	s.z=(PQP_REAL)rand()/(PQP_REAL)(RAND_MAX/MAX_Z);
	s.dist=0;
	return s;
}

/*
	distance metric for SE(3)
	weighted sum of translational distance and rotational distance
*/
double stateDistance(state c1, state c2)
{
	float tDist=sqrt((c1.x-c2.x)*(c1.x-c2.x)+(c1.y-c2.y)*(c1.y-c2.y)+(c1.z-c2.z)*(c1.z-c2.z)); 
	float rDist=Quat::distance(c1.q,c2.q);
	return W_T*tDist+W_R*rDist;
}

int collision(PQP_Model* piano, PQP_Model* room,state newState)
{
	PQP_REAL pianoR[3][3];
	PQP_REAL (*pass)[3][3]=&pianoR;
	Quat::toMatrix(newState.q,pass);
    PQP_REAL roomT[3] = {0, 0, 0}; //room does not move
	PQP_REAL pianoT[3] = {newState.x, newState.y, newState.z};
	PQP_REAL roomR[3][3]; //set to identity matrix since room does not rotate
	roomR[0][0]=1;
	roomR[0][1]=0;
	roomR[0][2]=0;
	roomR[1][0]=0;
	roomR[1][1]=1;
	roomR[1][2]=0;
	roomR[2][0]=0;
	roomR[2][1]=0;
	roomR[2][2]=1;
	PQP_CollideResult cres;
    PQP_Collide(&cres, pianoR, pianoT, piano, roomR, roomT, room);
    return cres.Colliding();
}

bool compState(const state& s1, const state& s2)
{
	return s1.dist<s2.dist;
}

void prmk(PQP_Model* piano, PQP_Model* room, int k)
{
	//k is neighbor count
	vector<state> nodes;
	vector<edge> edges;
	for(int i=0;i<PRM_ITR;i++)
	{
		state newSample=sample();
		while(collision(piano,room,newSample)!=0)
		{
			//new state has a collision, so try again
			newSample=sample();
		}
		vector<state> N;
		//find distances to every node wrt sample and order from least to greatest
		for(vector<state>::iterator it=nodes.begin();it!=nodes.end();++it)
		{
			(*it).dist=stateDistance((*it),newSample);
		}
		sort(nodes.begin(),nodes.end(),compState);
		//check paths from new sample to k neighbors
		int edgesAdded=0;
		for(int j=0;j<k;j++)
		{
			int badPath=0;
			double step=0;
			double gap=stateDistance(newSample,nodes[j]);
			while(step<1)
			{
				state checkState;
				checkState.x=step*newSample.x+(1-step)*nodes[j].x;
				checkState.y=step*newSample.y+(1-step)*nodes[j].y;
				checkState.z=step*newSample.z+(1-step)*nodes[j].z;
				checkState.q.w=nodes[j].q.w;
				checkState.q.x=nodes[j].q.x;
				checkState.q.y=nodes[j].q.y;
				checkState.q.z=nodes[j].q.z;
				int c=collision(piano,room,checkState);
				if(c==1)
				{
					badPath=1;
					break;
				}
				step+=0.1;
			}
			if(badPath==0)
			{
				//collision free path between neighbor and new state
				edge newEdge;
				newEdge.s1=nodes[j];
				newEdge.s2=newSample;
				edges.push_back(newEdge);
				edgesAdded++;
			}
		}
		if(edgesAdded>0)
		{
			nodes.push_back(newSample);
		}
	}
}

/*
 * Read trigles from txt file
 * Build PQP model using trigles
 * Return PQP model
 * */
PQP_Model readmodel(string name)
{
    PQP_Model m;
    m.BeginModel();
    // read from model file
    fstream infile;
    string file_name = name + ".txt";
    char file_name_char[file_name.size() + 1];
    strcpy(file_name_char, file_name.c_str());
    infile.open(file_name_char);
    if(!infile)
    {
        cout<<"Wrong model name for collision checker"<<endl;
        exit(EXIT_FAILURE);
    }
    int tri_num;
    infile >> tri_num;
    cout<<"number of triangles for "<<name<<": "<<tri_num<<endl;
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
    cout<<"model "<<name<<" loaded..."<<endl;
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
	quat q=Quat::random();
    return 0;
}
