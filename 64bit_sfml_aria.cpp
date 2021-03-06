
// Include SFML First (throws a strange error if not)
#include <SFML\Graphics.hpp>
#include <SFML\Window.hpp>
#include <SFML\System.hpp>
#include <SFML\Main.hpp>

// Include Aria
#include "Aria.h"

// Include our classes
#include "globalRobotPositionData.h"

#include "sfmlSim_robot.h"
#include "sfmlSim_map.h"
#include "sfmlSim_hud.h"
#include "sfmlSim_trail.h"
#include "sfmlSim_camera.h"
#include "sfmlSim_occupancyMap.h"
#include "localisation.h"

std::string MAP = "Mine";
std::string MAP_NAME = "maps/" + MAP + ".map";

// MONTE CARLO LOCALISATION CREATION
localisation mcl;

time_t start = time(0);
double old_second;
double current_second;

/*

	Handle MobileSim global coords of robots position 
	(should not be used in the aria robot controller, only to help create the sfml simulator)

*/

globalRobotPositionData globalRobotPosition;
bool handleSimStatPacket(ArRobotPacket* pkt)
{
	if (pkt->getID() != 0x62) return false; // SIMSTAT has id 0x62
	char a = pkt->bufToByte();  // unused byte
	char b = pkt->bufToByte();  // unused byte
	ArTypes::UByte4 flags = pkt->bufToUByte4();
	int simint = pkt->bufToUByte2();
	int realint = pkt->bufToUByte2();
	int lastint = pkt->bufToUByte2();
	globalRobotPosition.x = pkt->bufToByte4();
	globalRobotPosition.y = pkt->bufToByte4();
	int realZ = pkt->bufToByte4();
	globalRobotPosition.th = pkt->bufToByte4();
	//printf("\tTrue Pose = (%d, %d, %d, %d)\n", realX, realY, realZ, realTh);
	if (flags & ArUtil::BIT1)
	{
		double lat = pkt->bufToByte4() / 10e6;
		double lon = pkt->bufToByte4() / 10e6;
		double alt = pkt->bufToByte4() / 100;
		double qual = pkt->bufToByte() / 100;
	}
	else
	{
	}
	return true;
}

int main(int argc, char **argv)
{
	//______________________________________________________________________________________________________________________
	//______________________________________________________________________________________________________________________ ARIA

	Aria::init();																		//Initialise ARIA library
	ArArgumentParser argParser(&argc, argv);											//Initialise argument parser
	argParser.loadDefaultArguments();													//Load standard arguments
	ArRobot robot;																		//Initialise Robot object
	ArRobotConnector robotConnector(&argParser, &robot);								//Initialise Robot Connector

	if (!robotConnector.connectRobot())													//Check if robot is running
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())					//Log options
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	ArKeyHandler keyHandler;															//Escape Key Handler
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	puts("Press  Escape to exit.");

	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	std::cout << robot.getRobotRadius() << std::endl;

	ArGlobalRetFunctor1<bool, ArRobotPacket*> mySimStatPacketHandler(&handleSimStatPacket); // Packet Handler for true pose data
	robot.addPacketHandler(&mySimStatPacketHandler);

	robot.runAsync(true);
	robot.enableMotors();																//Enable Motors
	robot.setAbsoluteMaxLatVel(200);

	ArActionStallRecover recover;
	ArActionConstantVelocity move;
	ArActionAvoidFront frontavoid;
	ArActionBumpers bumpers;



	robot.addAction(&recover, 100);
	robot.addAction(&bumpers, 75);
	robot.addAction(&move, 60);
	robot.addAction(&frontavoid, 90);
	
	
	//______________________________________________________________________________________________________________________
	//______________________________________________________________________________________________________________________ SFML

	const int window_width = 1000;
	const int window_height = 600;
	//sf::Color sfml_windowBackgroundColor = sf::Color(127.5, 127.5, 127.5);
	sf::Color sfml_windowBackgroundColor = sf::Color::Black;

	sf::RenderWindow window(sf::VideoMode(window_width, window_height), "sfmlSimulator"); // Create the sfml window object
	window.setVerticalSyncEnabled(true);	// call it once, after creating the window
	window.setFramerateLimit(60);	// Set the frame rate to 24fps
	sf::Transform invert_transform;	// Invert camera from Aria coord system to SFML coord system (flipped vertically)
	invert_transform.scale(1, -1);


	// Create objects!
	sfmlSim_camera sfmlCamera;
	sfmlSim_map sfmlMap(MAP_NAME);														// Creates the map
	sfmlSim_robot sfmlRobot(robot, globalRobotPosition, sfmlMap.robotXYStartingPosition);		// Creates a robot
	sfmlSim_hud sfmlHUD(sfmlMap.robotXYStartingPosition);										// Displays data
	sfmlSim_trail sfmlTrail(robot.getRobotLength(), robot.getRobotWidth());						// Creates the trail
	sfmlSim_occupancyMap sfmlOccupancyMap(MAP, sfmlMap.minX, sfmlMap.minY, sfmlMap.maxX, sfmlMap.maxY);													// Creates the occupancy Map 

	// Monte Carlo Localisation Setup
	mcl.setup(globalRobotPosition, robot, sfmlOccupancyMap);
	

	// This runs asynchronously with Aria
	while (window.isOpen()) {

		// PROCESS USER INPUTS ---------------------------------------------------------------------------------------------------------------

		sf::Event event;

		while (window.pollEvent(event)) 
		{
			// Request for closing the window
			if (event.type == sf::Event::Closed) {
				window.close();	// Close the window
				Aria::exit(0);	// Stop Aria
			}
			// catch the resize events
			if (event.type == sf::Event::Resized)
			{
				// update the view to the new size of the window
				sf::FloatRect visibleArea(0.f, 0.f, event.size.width, event.size.height);
				window.setView(sf::View(visibleArea));
			}

			//Mouse move helper
			if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)		
			{
				sfmlMap.robotXYStartingPosition.x = event.mouseButton.x;
				sfmlMap.robotXYStartingPosition.y = event.mouseButton.y;
			}

			if (event.type == sf::Event::EventType::KeyPressed) {

				// Keyboard mouse zoom in
				if (event.key.code == sf::Keyboard::W) sfmlCamera.zoom(-1);
				// Keyboard mouse zoom out
				if (event.key.code == sf::Keyboard::S) sfmlCamera.zoom(1);
				// Reset camera to starting position
				if (event.key.code == sf::Keyboard::R) sfmlCamera.reset();
				// Turn trails on/off
				if (event.key.code == sf::Keyboard::T) sfmlTrail.clear();
				// Centre camera on robot position
				if (event.key.code == sf::Keyboard::F) sfmlCamera.follow();
				// Move camera using keyboard arrow keys 
				if (event.key.code == sf::Keyboard::Left) sfmlCamera.move(-300, 0);
				if (event.key.code == sf::Keyboard::Right) sfmlCamera.move(300, 0);
				if (event.key.code == sf::Keyboard::Up) sfmlCamera.move(0, -300);
				if (event.key.code == sf::Keyboard::Down) sfmlCamera.move(0, 300);
			}
		}
	
		// Mouse click and drag window
		if (event.type == sf::Event::MouseMoved && sf::Mouse::isButtonPressed(sf::Mouse::Left))				
		{
			sfmlCamera.move(-(event.mouseMove.x - sfmlMap.robotXYStartingPosition.x) / 5, -(event.mouseMove.y - sfmlMap.robotXYStartingPosition.y) / 5);
		}

		if (event.type == sf::Event::MouseButtonPressed) {
			if (event.mouseButton.button == sf::Mouse::Left) {
				sf::Vector2f loc = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
				std::cout << sfmlOccupancyMap.getProbability(loc.x, loc.y) << std::endl;
				//std::cout << "global " << loc.x << " " << loc.y << std::endl;
			}
		}
		




		// UPDATE -----------------------------------------------------------------------------------------------------------------------

		sfmlRobot.update(robot, globalRobotPosition);
		sfmlMap.update();
		sfmlHUD.update(sfmlCamera.camera, globalRobotPosition, window.mapPixelToCoords(sf::Vector2i(0, 0)));
		sfmlTrail.update(globalRobotPosition);
		sfmlCamera.update(sfmlRobot);


		// Monte Carlo Localisation Update
		current_second = difftime(time(0), start);
		if (current_second > old_second) {
			std::cout << "seconds since start: " << current_second << std::endl;
			old_second = current_second;

			mcl.update(globalRobotPosition, robot, sfmlOccupancyMap);

		}



		// DRAW / RENDERING -------------------------------------------------------------------------------------------------------------

		window.clear(sfml_windowBackgroundColor); // clear the screen and set background to defined color
	
		// draw everything here...
		// objects are drawn in the order here
		// window.draw(...);
		window.draw(sfmlOccupancyMap, invert_transform);
		window.draw(sfmlMap, invert_transform);
		window.draw(sfmlTrail, invert_transform);
		window.draw(sfmlRobot, invert_transform);
		window.draw(sfmlHUD, invert_transform);
		window.draw(mcl, invert_transform);
	
		// end the current frame
		window.setView(sfmlCamera.camera);
		window.display(); // Display Window
	
	}

	
	//______________________________________________________________________________________________________________________
	//______________________________________________________________________________________________________________________ ARIA EXIT

	robot.waitForRunExit();																//Wait for robot task loop to end before exiting the program				
	Aria::exit(0);																		//Exit Aria
}





