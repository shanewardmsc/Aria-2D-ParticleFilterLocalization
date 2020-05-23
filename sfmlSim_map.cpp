#include "sfmlSim_map.h"
#include <ArMap.h>

sfmlSim_map::sfmlSim_map(std::string map_name)
{

	minX = 0;
	minY = 0;
	maxX = 0;
	maxY = 0;

	int input_map_counter = 0;
	std::ifstream in(map_name);
	std::string file_line;

	
	float robotThStartingPosition = 0;

	if (!in) {
		std::cout << "Cannot open input file.\n";
	}

	bool isCoordinates = false;

	while (std::getline(in, file_line)) {
		// output the line
		std::cout << file_line << std::endl;

		int pos = file_line.find("RobotHome");
		if (pos > 1) {

			std::istringstream iss(file_line);
			std::vector < std::string> line;

			while (iss) {
				std::string word;
				iss >> word;
				line.push_back(word);
			}
			std::cout << "X " << std::stof(line[2]) << std::endl;
			std::cout << "Y " << line[3] << std::endl;
			std::cout << "Th " << line[4] << std::endl;

			robotXYStartingPosition.x = std::stof(line[2]);
			robotXYStartingPosition.y = std::stof(line[3]);
			robotThStartingPosition = std::stof(line[4]);
		}
		if (file_line == "DATA")
		{
			isCoordinates = false;
			std::cout << "Cordinates = false" << std::endl;
		}
		if (isCoordinates == true)
		{
			std::istringstream iss(file_line);
			std::vector < std::string> line;
			while (iss) {
				std::string word;
				iss >> word;
				line.push_back(word);
			}

			//MIN MAX for occupancy grid
			//X
			if (std::stof(line[0]) < minX) minX = std::stof(line[0]);
			if (std::stof(line[2]) < minX) minX = std::stof(line[2]);
			if (std::stof(line[0]) > maxX) maxX = std::stof(line[0]);
			if (std::stof(line[2]) > maxX) maxX = std::stof(line[2]);
			//Y
			if (std::stof(line[1]) < minY) minY = std::stof(line[1]);
			if (std::stof(line[3]) < minY) minY = std::stof(line[3]);
			if (std::stof(line[1]) > maxY) maxY = std::stof(line[1]);
			if (std::stof(line[3]) > maxY) maxY = std::stof(line[3]);

			inputMap[input_map_counter].setPrimitiveType(sf::Lines);
			inputMap[input_map_counter].resize(2);
			inputMap[input_map_counter][0].position = sf::Vector2f(std::stof(line[0]), std::stof(line[1]));
			inputMap[input_map_counter][1].position = sf::Vector2f(std::stof(line[2]), std::stof(line[3]));
			inputMap[input_map_counter][0].color = sfml_mapLineColor;
			inputMap[input_map_counter][1].color = sfml_mapLineColor;
			input_map_counter++;
		}
		if (file_line == "LINES")
		{
			isCoordinates = true;
			std::cout << "Cordinates = true" << std::endl;
		}
	}

	std::cout << "Min X: " << minX << " Y: " << minY << std::endl;
	std::cout << "Max X: " << maxX << " Y: " << maxY << std::endl;
}

sfmlSim_map::~sfmlSim_map()
{
}

void sfmlSim_map::update()
{
}

void sfmlSim_map::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
	
	// apply the entity's transform -- combine it with the one that was passed by the caller
	states.transform *= getTransform(); // getTransform() is defined by sf::Transformable

	// apply the texture
	states.texture = &m_texture;

	// you may also override states.shader or states.blendMode if you want

	// draw the vertex array
	target.draw(m_vertices, states);

	for (int i = 0; i < inputMap.size(); i++) target.draw(inputMap[i], states);

	
}