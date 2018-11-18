#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>
#include <float.h>
#include "pqp/include/PQP.h"
#include "quaternion.hpp"

#define PRM_ITR 200
#define PRM_STAR_CONST 3.17132879986883333333
#define CCH_RADIUS 2.0
#define DELTA 0.0001

using namespace std;

float MAX_X=10,MAX_Y=10,MAX_Z=10;
float W_T=1.0, W_R=1.0;

typedef struct configuration
{
	PQP_REAL x;
	PQP_REAL y;
	PQP_REAL z;
	quat q;
	PQP_REAL px;
	PQP_REAL py;
	PQP_REAL pz;
	quat pq;
	double g;
	double h;
	double dist;
} state;

bool operator==(const state& s1, const state& s2)
{
	if(s1.x!=s2.x||s1.y!=s2.y||s1.z!=s2.z)
	{
		return false;
	}
	if(s1.q.w!=s2.q.w||s1.q.x!=s2.q.x||s1.q.y!=s2.q.y||s1.q.z!=s2.q.z)
	{
		return false;
	}
	return true;
}

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
	srand(time(0));
	s.x=-10+((PQP_REAL)rand()/RAND_MAX)*20;
	s.y=-10+((PQP_REAL)rand()/RAND_MAX)*20;
	s.z=((PQP_REAL)rand()/RAND_MAX)*5;
	s.q=Quat::random();
	s.px=NAN;
	s.py=NAN;
	s.pz=NAN;
	s.pq=Quat::random();
	s.g=DBL_MAX;
	s.h=DBL_MAX;
	s.dist=0;
	return s;
}

/*
	distance metric for SE(3)
	weighted sum of translational distance and rotational distance
*/
double stateDistance(state c1, state c2)
{
	double tDist=sqrt((c1.x-c2.x)*(c1.x-c2.x)+(c1.y-c2.y)*(c1.y-c2.y)+(c1.z-c2.z)*(c1.z-c2.z)); 
	double rDist=Quat::distance(c1.q,c2.q);
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

bool compDist(const state& s1, const state& s2)
{
	return s1.dist<s2.dist;
}

bool a_starComp(const state& s1, const state& s2)
{
	return (s1.g+s1.h)<(s2.g+s2.h);
}

void setParent(state* child, state* parent)
{
	(*child).px=(*parent).x;
	(*child).py=(*parent).y;
	(*child).pz=(*parent).z;
	(*child).pq=(*parent).q;
	return;
}

vector<state> a_star(state start, state goal,vector<state> nodes, vector<edge> edges)
{
	start.g=0;
	setParent(&start,&start);
	vector<state> fringe;
	vector<state> closed;
	fringe.push_back(start);
	while(fringe.size()!=0)
	{
		state s=fringe[1];
		fringe.erase(fringe.begin());
		if(s==goal)
		{
			vector<state> path;
			path.push_back(s);

			return fringe;
		}
		closed.push_back(s);
		for(int i=0;i<edges.size();i++)
		{
			if(edges[i].s1==s)
			{
				//succ is s2
				state succ=edges[i].s2;
				bool inClosed=false;
				for(int j=0;j<closed.size();j++)
				{
					if(succ==closed[i])
					{
						inClosed=true;
					}
					if(!inClosed)
					{
						bool inFringe=false;
						for(int k=0;k<fringe.size();k++)
						{
							if(succ==fringe[i])
							{
								inFringe=true;
								break;
							}
							if(!inFringe)
							{
								succ.g=DBL_MAX;
								succ.px=NAN;
								succ.py=NAN;
								succ.pz=NAN;
							}
							if(s.g+1<succ.g)
							{
								succ.g=s.g+1;
								setParent(&succ,&s);
								if(inFringe)
								{
									fringe.erase(fringe.begin()+k);									
								}
								succ.h=sqrt((s.x-succ.x)*(s.x-succ.x)+(s.y-succ.y)*(s.y-succ.y)+(s.z-succ.z)*(s.z-succ.z)); 
								fringe.push_back(succ);
								sort(fringe.begin(),fringe.end(),a_starComp);
							}
						}
					}
				}
			}
			else if(edges[i].s2==s)
			{
				//succ is s1
				state succ=edges[i].s1;
				bool inClosed=false;
				for(int j=0;j<closed.size();j++)
				{
					if(succ==closed[i])
					{
						inClosed=true;
					}
					if(!inClosed)
					{
						bool inFringe=false;
						for(int k=0;k<fringe.size();k++)
						{
							if(succ==fringe[i])
							{
								inFringe=true;
								break;
							}
							if(!inFringe)
							{
								succ.g=DBL_MAX;
								succ.px=NAN;
								succ.py=NAN;
								succ.pz=NAN;
							}
							if(s.g+1<succ.g)
							{
								succ.g=s.g+1;
								setParent(&succ,&s);
								if(inFringe)
								{
									fringe.erase(fringe.begin()+k);									
								}
								succ.h=sqrt((s.x-succ.x)*(s.x-succ.x)+(s.y-succ.y)*(s.y-succ.y)+(s.z-succ.z)*(s.z-succ.z)); 
								fringe.push_back(succ);
								sort(fringe.begin(),fringe.end(),a_starComp);
							}
						}
					}
				}

			}
		}
	}
	return fringe;
}

/*
	The PRM algorithm using the connected component heursistic
*/
void prmcch(PQP_Model* piano, PQP_Model* room)
{
	vector<state> all_nodes;
	vector<edge> edges;
	state start=sample();
	while(collision(piano,room,start)!=0)
	{
		//new state has a collision, so try again
		start=sample();
	}
	state goal=sample();
	while(collision(piano,room,goal)!=0)
	{
		//new state has a collision, so try again
		goal=sample();
	}
	all_nodes.push_back(start);
	all_nodes.push_back(goal);
	for(int i=0;i<PRM_ITR;i++)
	{
		state newSample=sample();
		while(collision(piano,room,newSample)!=0)
		{
			//new state has a collision, so try again
			newSample=sample();
		}
		all_nodes.push_back(newSample);
		//find distances to every node wrt sample and order from least to greatest
		for(vector<state>::iterator it=all_nodes.begin();it!=all_nodes.end();++it)
		{
			(*it).dist=stateDistance((*it),newSample);
		}
		sort(all_nodes.begin(),all_nodes.end(),compDist);
		//check the neighborhood of the new sample
		int j=0;
		while(all_nodes[j].dist<CCH_RADIUS)
		{
			vector<state> path=a_star(start,all_nodes[j],all_nodes,edges);
			if(path.size()==0)
			{
				bool badPath=false;
				double step=0;
				state checkState;
				//check for collision in rotation
				while(step<1)
				{
					checkState.x=all_nodes[j].x;
					checkState.y=all_nodes[j].y;
					checkState.z=all_nodes[j].z;
					checkState.q=Quat::slerp(all_nodes[j].q,newSample.q,step);
					int c=collision(piano,room,checkState);
					if(c==1)
					{
						badPath=true;
						break;
					}
					step+=DELTA;
				}
				step=0;
				//check for collision in translation
				while(step<1)
				{
					checkState.x=step*newSample.x+(1-step)*all_nodes[j].x;
					checkState.y=step*newSample.y+(1-step)*all_nodes[j].y;
					checkState.z=step*newSample.z+(1-step)*all_nodes[j].z;
					checkState.q=newSample.q;
					int c=collision(piano,room,checkState);
					if(c==1)
					{
						badPath=true;
						break;
					}
					step+=DELTA;
				}
				if(!badPath)
				{
					//collision free path between neighbor and new state
					edge newEdge;
					newEdge.s1=all_nodes[j];
					newEdge.s2=newSample;
					edges.push_back(newEdge);
				}
			}
		}
	}
	return;
}

/*
	The PRM algorithm using k-nearest neighbors
*/
void prmk(PQP_Model* piano, PQP_Model* room, int k)
{
	//k is neighbor count
	vector<state> all_nodes;
	vector<edge> edges;
	for(int i=0;i<PRM_ITR;i++)
	{
		state newSample=sample();
		while(collision(piano,room,newSample)!=0)
		{
			//new state has a collision, so try again
			newSample=sample();
		}
		all_nodes.push_back(newSample);
		//find distances to every node wrt sample and order from least to greatest
		for(vector<state>::iterator it=all_nodes.begin();it!=all_nodes.end();++it)
		{
			(*it).dist=stateDistance((*it),newSample);
		}
		sort(all_nodes.begin(),all_nodes.end(),compDist);
		//check paths from new sample to k neighbors
		for(int j=0;j<k;++j)
		{
			bool badPath=false;
			double step=0;
			state checkState;
			//check for collision in rotation
			while(step<1)
			{
				checkState.x=all_nodes[j].x;
				checkState.y=all_nodes[j].y;
				checkState.z=all_nodes[j].z;
				checkState.q=Quat::slerp(all_nodes[j].q,newSample.q,step);
				int c=collision(piano,room,checkState);
				if(c==1)
				{
					badPath=true;
					break;
				}
				step+=DELTA;
			}
			step=0;
			//check for collision in translation
			while(step<=1)
			{
				/*
				checkState.x=step*newSample.x+(1-step)*all_nodes[j].x;
				checkState.y=step*newSample.y+(1-step)*all_nodes[j].y;
				checkState.z=step*newSample.z+(1-step)*all_nodes[j].z;
				checkState.q=newSample.q;
				int c=collision(piano,room,checkState);
				*/
				checkState.x=newSample.x+step*all_nodes[j].x;
				checkState.y=newSample.y+step*all_nodes[j].y;
				checkState.z=newSample.z+step*all_nodes[j].z;
				checkState.q=newSample.q;
				int c=collision(piano,room,checkState);
				if(c==1)
				{
					badPath=true;
					break;
				}
				step+=DELTA;
			}
			if(!badPath)
			{
				//collision free path between neighbor and new state
				edge newEdge;
				newEdge.s1=all_nodes[j];
				newEdge.s2=newSample;
				edges.push_back(newEdge);
			}
		}
	}
	return;
}

/*
	The PRM algorithm for asymptotic optimallity
*/
void prm_star(PQP_Model* piano, PQP_Model* room)
{
	vector<state> all_nodes;
	vector<edge> edges;
	for(int i=0;i<PRM_ITR;i++)
	{
		state newSample=sample();
		while(collision(piano,room,newSample)!=0)
		{
			//new state has a collision, so try again
			newSample=sample();
		}
		all_nodes.push_back(newSample);
		//find distances to every node wrt sample and order from least to greatest
		for(vector<state>::iterator it=all_nodes.begin();it!=all_nodes.end();++it)
		{
			(*it).dist=stateDistance((*it),newSample);
		}
		sort(all_nodes.begin(),all_nodes.end(),compDist);
		//check paths from new sample to neighbors
		int edgesAdded=0,j=0;
		int cap=ceil(PRM_STAR_CONST*log(all_nodes.size()));
		while(edgesAdded<cap&&j<all_nodes.size())
		{
			bool badPath=false;
			double step=0;
			state checkState;
			//check for collision in rotation
			while(step<1)
			{
					checkState.x=all_nodes[j].x;
					checkState.y=all_nodes[j].y;
					checkState.z=all_nodes[j].z;
					checkState.q=Quat::slerp(all_nodes[j].q,newSample.q,step);
					int c=collision(piano,room,checkState);
					if(c==1)
					{
						badPath=true;
						break;
					}
					step+=DELTA;
			}
			step=0;
			//check for collision in translation
			while(step<1)
			{
	
				checkState.q=newSample.q;
				int c=collision(piano,room,checkState);
				if(c==1)
				{
					badPath=true;
					break;
				}
				step+=DELTA;
			}
			if(!badPath)
			{
				//collision free path between neighbor and new state
				edge newEdge;
				newEdge.s1=all_nodes[j];
				newEdge.s2=newSample;
				edges.push_back(newEdge);
				edgesAdded++;
			}
			j++;
		}
	}
	return;
}

int main()
{
    // create room PQP model
    PQP_Model* room = new PQP_Model;
    room->BeginModel();
    // read from model file
    fstream infile;
    string file_name = "room.txt";
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
        room->AddTri(p[0], p[1], p[2], num);
    }
    infile.close();
    room->EndModel();
    cout<<"model room loaded..."<<endl;
 
 	//create piano PQP model
    PQP_Model* piano = new PQP_Model;
	piano->BeginModel();
    // read from model file
    file_name = "piano.txt";
    file_name_char[file_name.size() + 1];
    strcpy(file_name_char, file_name.c_str());
    infile.open(file_name_char);
    if(!infile)
    {
        cout<<"Wrong model name for collision checker"<<endl;
        exit(EXIT_FAILURE);
    }
    tri_num;
    infile >> tri_num;
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
        piano->AddTri(p[0], p[1], p[2], num);
    }
    infile.close();
    piano->EndModel();
    cout<<"model piano loaded..."<<endl;
	/*
    PQP_REAL roomT[3]; //room does not move
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
	PQP_REAL T[3]={5.0,8.0,0.3};
	PQP_CollideResult cres;
    PQP_Collide(&cres, roomR, T, piano, roomR, roomT, room);
	cout<<cres.Colliding()<<endl;
*/
	int input;
	cout<<"\t(1) PRM connected component heuristic\n\
	(2) PRM k-nearest neighbors\n\
	(3) PRM*\n"<<endl;
	cin>>input;
	switch(input)
	{
		case 1:
			cout<<"Starting PRM with connected components heuristic..."<<endl;
			prmcch(piano,room);
			break;
		case 2:
			int k;
			cout<<"Enter k value: ";
			cin>>k;
			cout<<"Starting PRM with k-nearest neighbors..."<<endl;
			prmk(piano,room,k);
			break;
		case 3:
			cout<<"Starting PRM*..."<<endl;
			prm_star(piano,room);
			break;
		default:
			cout<<"ERROR: Bad input"<<endl;
			return 1;
	}
	delete room;
	delete piano;
	cout<<"RETURNED"<<endl<<flush;
    return 0;
}
