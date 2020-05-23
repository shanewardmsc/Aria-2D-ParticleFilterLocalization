#include "sfmlSim_trail.h"



sfmlSim_trail::sfmlSim_trail(double robotLength, double robotWidth)
{
	rLength = robotLength;
	rWidth = robotWidth;

	old_second = -1;
}


sfmlSim_trail::~sfmlSim_trail()
{
}

void sfmlSim_trail::update(globalRobotPositionData& globalRobotPosition)
{
	if (doTrail) {
		current_second = difftime(time(0), start);

		if (current_second > old_second) {
			//std::cout << current_second << std::endl;
			old_second = current_second;


			sf::RectangleShape rect;

			rect.setSize(sf::Vector2f(rLength, rWidth));
			rect.setOutlineThickness(10);
			rect.setOutlineColor(sf::Color::Cyan);
			rect.setOrigin(sf::Vector2f(rLength / 2, rWidth / 2));
			rect.setFillColor(sf::Color::Transparent);

			rect.setPosition(sf::Vector2f(globalRobotPosition.x, globalRobotPosition.y));
			rect.setRotation(globalRobotPosition.th);

			trailPoints.push_back(rect);

		}
	}
	
}

void sfmlSim_trail::clear()
{
	trailPoints.clear();
	if (doTrail == true) {
		doTrail = false;
	}
	else {
		doTrail = true;
	}
}

void sfmlSim_trail::draw(sf::RenderTarget& target, sf::RenderStates states) const
{

	// apply the entity's transform -- combine it with the one that was passed by the caller
	//states.transform *= getTransform(); // getTransform() is defined by sf::Transformable

	// apply the texture
	//states.texture = &m_texture;

	// you may also override states.shader or states.blendMode if you want

	// draw the vertex array
	//target.draw(m_vertices, states);


	
	for (int i = 0; i < trailPoints.size(); i++) target.draw(trailPoints[i], states);
	for (int i = 0; i < trailLines.size();  i++) target.draw(trailLines[i], states);
}

