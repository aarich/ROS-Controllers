/**
*	
*	@File Robot.h
*	@Author Alex Rich and John Allard @ HMC 2014
*	@Info AI for robot movements
*
**/

using namespace std;

class Robot
{
private:

	vector<float> heading;
	vector<float> location;
	vector<float> destination;
	int state;

	vector<float> P2P();
	vector<float> UserInput();

public:

	Robot();

	vector<float> 	NextMove(); // Returns the translation and turn components of the next move

	float CurrentAngle();
	float DesiredAngle();
	float Distance();

	vector<float> 	GetHeading();
	void 			SetHeading(vector<float>);
	vector<float>	GetLocation();
	void 			SetLocation(vector<float>);
	vector<float>	GetDestination();
	void			SetDestination(vector<float>);
	int				GetState();
	void			SetState(int);

};