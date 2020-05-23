#include "sfmlSim_hud.h"



sfmlSim_hud::sfmlSim_hud(sf::Vector2f& startingPos)
{
	// Link and load the font file, fonts will not show without this
	if (!font.loadFromFile("arial.ttf"))
	{
		std::cout << "Font not found." << std::endl;
	}

	
	fps_text.setString("fps: --");
	fps_text.setFont(font);
	fps_text.setCharacterSize(300);
	fps_text.setColor(sf::Color::Green);
	fps_text.setPosition(sf::Vector2f(0, 0));


	globalRobotPos_text.setString("gPos: --");
	globalRobotPos_text.setFont(font);
	globalRobotPos_text.setCharacterSize(300);
	globalRobotPos_text.setColor(sf::Color::Green);
	globalRobotPos_text.setPosition(sf::Vector2f(0, 0));


	mapCenter.setString("+");
	mapCenter.setFont(font);
	mapCenter.setCharacterSize(300);
	mapCenter.setColor(sf::Color::Yellow);
	mapCenter.setPosition(sf::Vector2f(0, 0));
	
	robotHome.setString("+");
	robotHome.setFont(font);
	robotHome.setCharacterSize(300);
	robotHome.setColor(sf::Color::Cyan);
	robotHome.setPosition(startingPos);


}


sfmlSim_hud::~sfmlSim_hud()
{
}

void sfmlSim_hud::update(sf::View& camera, globalRobotPositionData& globalRobotPosition, sf::Vector2f windowCoords)
{
	/*
	// update fps counter
	float currentTime = clock.restart().asSeconds();
	float fps = 1.f / currentTime;
	lastTime = currentTime;
	// create string from fps float and update the fps text
	char c[10];
	sprintf(c, "fps: %.1f", fps);
	std::string string(c);
	sf::String str(string);
	fps_text.setString(str);
	*/
	fps_text.setPosition(windowCoords);
	


	// Update global position of robot in map
	std::string gposString;
	gposString.append("gPos: ");
	gposString.append(std::to_string(globalRobotPosition.x) + "/");
	gposString.append(std::to_string(globalRobotPosition.y) + "/");
	gposString.append(std::to_string(globalRobotPosition.th));
	globalRobotPos_text.setString(gposString);
	globalRobotPos_text.setPosition(sf::Vector2f(windowCoords.x, windowCoords.y + (20* 20)));

	

}


void sfmlSim_hud::draw(sf::RenderTarget& target, sf::RenderStates states) const
{

	// apply the entity's transform -- combine it with the one that was passed by the caller
	states.transform *= getTransform(); // getTransform() is defined by sf::Transformable

										// apply the texture
	states.texture = &m_texture;

	// you may also override states.shader or states.blendMode if you want

	// draw the vertex array
	target.draw(m_vertices, states);

	target.draw(mapCenter);
	target.draw(robotHome);
	target.draw(fps_text);
	target.draw(globalRobotPos_text);

}