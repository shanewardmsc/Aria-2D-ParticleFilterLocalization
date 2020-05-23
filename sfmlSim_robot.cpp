#include "sfmlSim_robot.h"



sfmlSim_robot::sfmlSim_robot(ArRobot& robot, globalRobotPositionData& globalRobotPosition, sf::Vector2f& startingPos)
{
	robot_body.setSize(sf::Vector2f(robot.getRobotLength(), robot.getRobotWidth()));
	robot_body.setOutlineThickness(10);
	robot_body.setOutlineColor(sf::Color::Black);
	robot_body.setFillColor(sf::Color::Red);
	robot_body.setOrigin(sf::Vector2f(robot.getRobotLength() / 2, robot.getRobotWidth() / 2));
	robot.comInt(ArCommands::SIM_STAT, 1);
	robot_body.setPosition(globalRobotPosition.x, globalRobotPosition.y);
	robot_body.setRotation(globalRobotPosition.th);

	for (int i = 0; i < 16; i++)
	{
		sensorLines[i].setPrimitiveType(sf::Lines);
		sensorLines[i].resize(2);

		sensorLines[i][0].position = startingPos;
		sensorLines[i][1].position = startingPos;
	}

}


sfmlSim_robot::~sfmlSim_robot()
{
}


void sfmlSim_robot::update(ArRobot& robot, globalRobotPositionData& globalRobotPosition)
{
	//______________________________________________________________________________________________________________________ UPDATE
	//ROBOT MOVE
	robot.comInt(ArCommands::SIM_STAT, 1);
	robot_body.setPosition(globalRobotPosition.x, globalRobotPosition.y);
	robot_body.setRotation(globalRobotPosition.th);

	// Update sonar sensor lines
	ArSensorReading* sonarReading;
	int radius = robot.getRobotWidth();
	for (int i = 0; i < 16; i++)
	{
		sonarReading = robot.getSonarReading(i);

		if (sonarReading->getRange() == 5000) {
			sensorLines[i][0].color = sf::Color::Red;
			sensorLines[i][1].color = sf::Color::Red;
		}
		else {
			sensorLines[i][0].color = sf::Color::Green;
			sensorLines[i][1].color = sf::Color::Green;
		}

		// Do sonar stuff here
		int reading = sonarReading->getRange();
		int xOffset = sonarReading->getSensorPosition().getX();
		int yOffset = sonarReading->getSensorPosition().getY();
		int offset = sqrt(pow(xOffset, 2) + pow(yOffset, 2));

		int x = robot_body.getPosition().x - cos((sonarReading->getSensorTh() + globalRobotPosition.th - 180) * M_PI / 180)*((reading + offset));
		int y = robot_body.getPosition().y - sin((sonarReading->getSensorTh() + globalRobotPosition.th - 180) * M_PI / 180)*((reading + offset));


		if (_USE_LPF == 1)
		{
			smoothSonarX_array[i] = smoothSonarX_array[i] - (LPF_smooth_beta * (smoothSonarX_array[i] - x));
			smoothSonarY_array[i] = smoothSonarY_array[i] - (LPF_smooth_beta * (smoothSonarY_array[i] - y));

			x = smoothSonarX_array[i];
			y = smoothSonarY_array[i];
		}

		sensorLines[i][1].position = sf::Vector2f(x, y);
		sensorLines[i][0].position = sf::Vector2f(robot_body.getPosition().x, robot_body.getPosition().y);

	}
}

void sfmlSim_robot::draw(sf::RenderTarget& target, sf::RenderStates states) const
{

	// apply the entity's transform -- combine it with the one that was passed by the caller
	//states.transform *= getTransform(); // getTransform() is defined by sf::Transformable

										// apply the texture
	//states.texture = &m_texture;

	// you may also override states.shader or states.blendMode if you want

	// draw the vertex array
	//target.draw(m_vertices, states);


	target.draw(robot_body, states);
	for (int i = 0; i < 16; i++) target.draw(sensorLines[i], states);
}

