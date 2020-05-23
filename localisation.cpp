#include "localisation.h"
#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <Eigen/Dense>

#define numParticles 100
#define num_Particles 17000
#define OccGridX_value 170
#define OccGridY_value 100
#define GridCell_value 100

using namespace std;
using namespace Eigen;

bool paused;
particle particle_OccGrid[OccGridX_value][OccGridY_value];
float estimatedPositionX;
float estimatedPositionY;
float estimatedOrientationTh;

localisation::localisation()
{
}

localisation::~localisation()
{
}

void localisation::setup(globalRobotPositionData globalRobotPosition, ArRobot& robot, sfmlSim_occupancyMap sfmlOccupancyMap)
{
	std::cout << "localisation setup" << std::endl;

	/* PREVIOUS ROBOT POSITION */
	previous_PositionX = globalRobotPosition.x;
	previous_PositionY = globalRobotPosition.y;
	previous_OrientationTh = globalRobotPosition.th;

	/* OCCUPANCY GRID DIMENSIONS */
	OccGrid_Xmax = sfmlOccupancyMap.maxX;
	OccGrid_Ymax = sfmlOccupancyMap.maxY;
	OccGrid_Xorig = sfmlOccupancyMap.originX;
	OccGrid_Yorig = sfmlOccupancyMap.originY;
	OccGrid_Xdimension = OccGrid_Xmax - OccGrid_Xorig;
	OccGrid_Ydimension = OccGrid_Ymax - OccGrid_Yorig;

	/* INITIALIZE PARTICLES Xt includes {X,Y,Th} */
	for (int x = 0; x < OccGridX_value; x++)
	{
		for (int y = 0; y < OccGridY_value; y++)
		{
			particle p[OccGridX_value][OccGridY_value];
			p[x][y].x = rand() % (OccGrid_Xmax - OccGrid_Xorig) + OccGrid_Xorig;
			p[x][y].y = rand() % (OccGrid_Ymax - OccGrid_Yorig) + OccGrid_Yorig;
			p[x][y].Th = rand() % 360;
			p[x][y].w = 1.0 / (float)(OccGridX_value*OccGridY_value);
			particle_OccGrid[x][y] = p[x][y];
		}
	}
}

void localisation::update(globalRobotPositionData globalRobotPosition, ArRobot& robot, sfmlSim_occupancyMap sfmlOccupancyMap)
{
	std::cout << "localisation update" << std::endl;

	/********** MONTE CARLO LOCALIZATION**********/
	float M_Array[OccGridX_value] = { };
	float max = M_Array[0];
	float M = 0;
	float sum = 0;
	float alpha = 0;

	/* CURRENT POSITION */
	float robot_PositionX = globalRobotPosition.x;
	float robot_PositionY = globalRobotPosition.y;
	float robot_OrientationTh = globalRobotPosition.th;

	/* MOTION DIFFERENCE */
	float dX = robot_PositionX - previous_PositionX;
	float dY = robot_PositionY - previous_PositionY;
	float dTh = robot_OrientationTh - previous_OrientationTh;

	/* ORIENTATION ERROR*/
	if (dTh < 0) 
	{
		dTh = dTh + 360;
	}

	if (robot_OrientationTh < 0) 
	{
		robot_OrientationTh = robot_OrientationTh + 360;
	}

	/* SET PREVIOUS POSITION */
	previous_PositionX = globalRobotPosition.x;
	previous_PositionY = globalRobotPosition.y;
	previous_OrientationTh = globalRobotPosition.th;

	/* RANDOM FLOAT BETWEEN 0 AND 1 */
	float random_Float_Index = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

	if(robot_PositionX != 0 && robot_PositionY != 0)
	{ 
		for (int x = 0; x < OccGridX_value; x++)						// Maximum X Value of Occupancy Grid
		{
			for (int y = 0; y < OccGridY_value; y++)					// Maximum Y Value of Occupancy Grid
			{

				/* OCCUPANY GRID CALCULATIONS */
				float x_Grid = OccGrid_Xorig + (x * GridCell_value);	// Subtract the origin the "zero" robots position in Occupancy Grid
				float y_Grid = OccGrid_Yorig + (y * GridCell_value);	// Divide by the cell resolution 100mm

				/* DISTANCE THREHSOLD FOR PARTICLE UPDATE */
				float BoundingBoxThreshold = 1250;

				/* DISTANCE FROM ROBOT POSITION TO CURRENT CELL WITHIN LOOP */
				float distToCell_1 = sqrt(((x_Grid - robot_PositionX)*(x_Grid - robot_PositionX)) + ((y_Grid - robot_PositionY)*(y_Grid - robot_PositionY)));

				/* DISTANCE FROM PARTICLE POSITION TO CURRENT CELL WITHIN LOOP */
				float distToCell_2 = sqrt(((x_Grid - particle_OccGrid[x][y].x)*(x_Grid - particle_OccGrid[x][y].x)) + 
					((y_Grid - particle_OccGrid[x][y].y)*(y_Grid - particle_OccGrid[x][y].y)));

				if (distToCell_1 < BoundingBoxThreshold && distToCell_2 < BoundingBoxThreshold)
				{
					/* SAMPLED PARTICLE AT RANDOM INDEX J */
					j_Particle = particle_OccGrid[x][y];

					/* PREDICTION PHASE */
					/* Sigma X Component */
					float sigma_L_x = 0.25;											// ADJUST LATER - JUDGE BASED ON SPEED OF ROBOT IN ARIA ??
					float sigma_U_x = 0.75;											// Maximum X movement [mm/s]
					float i_sigma_x = dX - 1;
					float j_sigma_x = dX + 1;										// Maximum X movement [mm/s]

					/* Sigma Y Component */
					float sigma_L_y = 0.25;
					float sigma_U_y = 0.75;											// Maximum Y movement [mm/s]
					float i_sigma_y = dY - 1;
					float j_sigma_y = dY + 1;										// Maximum Y movement [mm/s]

					/* Sigma Theta Component */
					float sigma_L_th = 0.25;
					float sigma_U_th = 0.75;										// Maximum Angle change [Deg./s]
					float i_sigma_th = dTh - 1;
					float j_sigma_th = dTh + 1;										// Maximum Angle change [Deg./s]

					/* USE LINEAR EQUATIONS TO FIND SIGMA_X, Y AND THETA */
					float sigma_piecewise_x = (((sigma_U_x - sigma_L_x) / (i_sigma_x - j_sigma_x)) * dX) + sigma_U_x;			// SIGMA X
					float sigma_piecewise_y = (((sigma_U_y - sigma_L_y) / (i_sigma_y - j_sigma_y)) * dY) + sigma_U_y;			// SIGMA Y
					float sigma_piecewise_Th = (((sigma_U_th - sigma_L_th) / (i_sigma_th - j_sigma_th)) * dTh) + sigma_U_th;	// SIGMA THETA

					/* SAMPLE VALUES OF DX, DY AND DTHETA USING BOX MULLER */
					float dx_t = getNormalRand(dX, sigma_piecewise_x);				// BOX MULLER SAMPLED VALUES
					float dy_t = getNormalRand(dY, sigma_piecewise_y);
					float dTh_t = getNormalRand(dTh, sigma_piecewise_Th);

					float dx_ = dx_t * cos(dTh_t);									// DX', DY' AND DTHETA VALUES
					float dy_ = dy_t * sin(dTh_t);
					float dTh_ = dTh_t;

					/* UPDATE PARTICLE X,Y,Th VALUE */
					j_Particle.x = (dx_ + particle_OccGrid[x][y].x);
					j_Particle.y = (dy_ + particle_OccGrid[x][y].y);
					j_Particle.Th = (dTh_ + particle_OccGrid[x][y].Th);
					j_Particle.w = particle_OccGrid[x][y].w;

					/* CORRECTION PHASE */
					double sensorLeftAngle, sensorRightAngle;
					float sensorLeft = robot.checkRangeDevicesCurrentPolar(0.0, 90.0, &sensorLeftAngle);
					float sensorRight = robot.checkRangeDevicesCurrentPolar(90.0, 0.0, &sensorRightAngle);

					if (sensorLeft > 0 && sensorLeft < 1000)
					{
						/* ACTUAL SONAR READING VALUE */
						float detectX = cos(sensorLeftAngle) * (sensorLeft);
						float detectY = sin(sensorLeftAngle) * (sensorLeft);
						float detectTh = particle_OccGrid[x][y].Th * (M_PI / 180.0);
						float diffX = (detectX * cos(detectTh)) - (detectY * sin(detectTh));
						float diffY = (detectX * sin(detectTh)) + (detectY * cos(detectTh));
						int sensorX = diffX + j_Particle.x;
						int sensorY = diffY + j_Particle.y;

						int OccGridx = particle_OccGrid[x][y].x;
						int OccGridy = particle_OccGrid[x][y].y;

						/* PARTICLE PERCEPTION */
						float particle_Perception = sfmlOccupancyMap.getProbability(OccGridx, OccGridy);

						float actual_Perception = sfmlOccupancyMap.getProbability(sensorX, sensorY);

						/* COMPUTE THE WEIGHT OF THE PARTICLE */
						j_Particle.w /*Wt_*/ = 1 - fabs(particle_Perception - actual_Perception);
					}

					else if (sensorRight > 0 && sensorRight < 1000)
					{
						/* ACTUAL SONAR READING VALUE */
						float detectX = cos(sensorRightAngle) * (sensorRight);
						float detectY = sin(sensorRightAngle) * (sensorRight);
						float detectTh = particle_OccGrid[x][y].Th * (M_PI / 180.0);
						float diffX = (detectX * cos(detectTh)) - (detectY * sin(detectTh));
						float diffY = (detectX * sin(detectTh)) + (detectY * cos(detectTh));
						int sensorX = diffX + j_Particle.x;
						int sensorY = diffY + j_Particle.y;

						int OccGridx = particle_OccGrid[x][y].x;
						int OccGridy = particle_OccGrid[x][y].y;

						/* PARTICLE PERCEPTION */
						float particle_Perception = sfmlOccupancyMap.getProbability(OccGridx, OccGridy);

						float actual_Perception = sfmlOccupancyMap.getProbability(sensorX, sensorY);

						/* COMPUTE THE WEIGHT OF THE PARTICLE */
						j_Particle.w /*Wt_*/ = 1 - fabs(particle_Perception - actual_Perception);
					}

					/* UPDATE NORMALIZATION FACTOR */ 
					alpha = alpha + j_Particle.w;

					/* INSERT SAMPLE INTO SAMPLE SET - St*/
					particle_OccGrid[x][y].x = j_Particle.x;
					particle_OccGrid[x][y].y = j_Particle.y;
					particle_OccGrid[x][y].Th = j_Particle.Th;
					particle_OccGrid[x][y].w = j_Particle.w;

					/* POSITION ESTIMATION */
					printf("Grid X: %d, Grid Y: %d\n", x, y);
					printf("Updated Particle: %.2f, %.2f, %.2f, %.4f\n", j_Particle.x, j_Particle.y, j_Particle.Th, j_Particle.w);
				}
			}
		}

		for (int x = 0; x < OccGridX_value; x++) // Maximum X Value of Occupancy Grid
		{
			for (int y = 0; y < OccGridY_value; y++) // Maximum Y Value of Occupancy Grid
			{
				if (alpha > 0)
				{
					/* NORMALIZE ALL PARTICLES WITHIN SET */
					particle_OccGrid[x][y].w = particle_OccGrid[x][y].w / alpha;
				}
			}
		}
	}
}


void localisation::draw(sf::RenderTarget & target, sf::RenderStates states) const
{

	if (doParticles)
	{
		/* VISUALIZATION OF PARTICLES */
		sf::VertexArray Points(sf::Points, numParticles);
		sf::RectangleShape ParticleArray;
		float threshold = 0.1;
		
		for (int x = 0; x < OccGridX_value; x++)
		{
			for (int y = 0; y < OccGridY_value; y++)
			{
				if (particle_OccGrid[x][y].w > threshold)
				{
					ParticleArray.setSize(sf::Vector2f(150.f, 150.f));		// Occupancy Grid Size
					ParticleArray.setOrigin(sf::Vector2f(0.0, 0.0));
					ParticleArray.setFillColor(sf::Color::Blue);

					ParticleArray.setPosition(sf::Vector2f(particle_OccGrid[x][y].x, particle_OccGrid[x][y].y));
					ParticleArray.setRotation(particle_OccGrid[x][y].Th);

					target.draw(ParticleArray, states);
				}
			}
		}
	}
}

float localisation::getNormalRand(float c, float sigma)
{
	float x1, x2, w;
	x1 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
	x2 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
	while (sqrt(x1*x1 + x2 * x2) > 1.0)
	{
		x1 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
		x2 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
	}
	w = sqrt((-2.0 * log(x1*x1 + x2 * x2)) / (x1*x1 + x2 * x2));

	return c + (x1 * w) * sigma;
}

void localisation::simulateEncoders(globalRobotPositionData globalRobotPosition, ArRobot& robot)
{
	myX = globalRobotPosition.x; // init x pos
	myY = globalRobotPosition.y; // init y pos
	myTh = globalRobotPosition.th; // init heading

	l = 425.0; // Width of a P3 DX in mm
	rw = 95.0; // Wheel radius in mm
	tr = 360.0; // Number of odemetry clicks for a full circle

	long long unsigned int oldt = (long long unsigned int)oldTs;
	long long unsigned int newt = (long long unsigned int)newTs;

	double diff = 1; // 1 Second Increments
	double d1 = (diff * robot.getLeftVel());
	double d2 = (diff * robot.getRightVel());

	t1 = floor((d1 * tr) / (2.0 * M_PI * rw));
	t2 = floor((d2 * tr) / (2.0 * M_PI * rw));
}