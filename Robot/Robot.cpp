/**
* 
* 	@File Robot.cpp
*	@Author Alex Rich and John Allard @ HMC 2014
*	@Info Implementation of movements for robots
*
**/

#include "Robot.h"

using namespace std;

#define PI 3.14159

Robot::Robot() : state(1)
    {
        vector<float> v;
        v.push_back(0.0);
        v.push_back(0.0);
        v.push_back(0.0);

        this->heading = v;
        this->location = v;
        this->destination = v;
    }

vector<float> Robot::NextMove()
{
	if (this->state == 0) // Point To Point Nav
		return P2P();
	else if (this->state == 1) // User Input
		return UserInput();
}

vector<float> Robot::P2P()
{
	float turn = 0;
	float translate = 0;
	float change = CurrentAngle() - DesiredAngle();
	if (change > PI / 2)
		change = change - PI;
	if (abs(change) > 0.1)
		turn = change;
	float d = Distance();
	if (d > 0.1 && turn < 0.2)
		translate = min((float) 2.0, (float) d);
	vector<float> result;
	result.push_back(turn * 180 / PI);
	result.push_back(translate);
	return result;
}

vector<float> Robot::UserInput()
{
	float translate;
	int turn;

	cout << "Translate: ";
	cin >> translate;
	cin.get();
	cout << endl;

	cout << "Turn: ";
	cin >> turn;
	cin.get();
	cout << endl;
	
	vector<float> result;
	result.push_back(turn);
	result.push_back(translate);
	return result;
}

float Robot::CurrentAngle()
{
	float x = heading[0] - location[0];
	float y = heading[1] - location[1];
	return atan2(x, y);
}

float Robot::DesiredAngle()
{
	float x = destination[0] - location[0];
	float y = destination[1] - location[1];
	return atan2(x, y);
}

float Robot::Distance()
{
	float x = location[0] - destination[0];
	float y = location[1] - destination[0];
	return sqrt(x*x + y*y);
}

vector<float> ToVector(float a, float b, float c)
{
	vector<float> v;
	v.push_back(a);
	v.push_back(b);
	v.push_back(c);
	return v;
}

vector<float> Robot::GetHeading() 			{ return this->heading; }
void Robot::SetHeading(vector<float> v) 	{ this->heading = v; }
vector<float> Robot::GetLocation() 			{ return this->location; }
void Robot::SetLocation(vector<float> v)	{ this->location = v; }
vector<float> Robot::GetDestination() 		{ return this->destination; }
void Robot::SetDestination(vector<float> v) { this->destination = v; }
int Robot::GetState() 						{ return this->state; }
void Robot::SetState(int i) 				{ this->state = i; }

void Robot::SetHeading(float x, float y, float z) 		{ SetHeading(ToVector(x, y, z)); }
void Robot::SetLocation(float x, float y, float z) 		{ SetLocation(ToVector(x, y, z)); }
void Robot::SetDestination(float x, float y, float z) 	{ SetDestination(ToVector(x, y, z)); }