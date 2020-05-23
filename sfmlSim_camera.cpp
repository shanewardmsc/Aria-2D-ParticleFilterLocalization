#include "sfmlSim_camera.h"



sfmlSim_camera::sfmlSim_camera()
{
	//camera(sf::FloatRect(0, 0, window_width, window_height));	// Create the camera object
	camera.setCenter(camera_starting_posX, camera_starting_posY);
	camera.setSize(window_width * window_scaling_factor, window_height * window_scaling_factor);
	bool cameraFollowRobot = false;
}


sfmlSim_camera::~sfmlSim_camera()
{
}

void sfmlSim_camera::update(sfmlSim_robot& robot)
{
	if (cameraFollowRobot == true)
	{
		camera.setCenter(robot.robot_body.getPosition().x, -robot.robot_body.getPosition().y);
	}
}

void sfmlSim_camera::reset()
{
	window_scaling_factor = 10;
	camera.setCenter(0, 0);
	camera.setSize(window_width * window_scaling_factor, window_height * window_scaling_factor);
}

void sfmlSim_camera::setSize(int width, int height)
{
	camera.setSize(width, height);
}

void sfmlSim_camera::setCenter(float x, float y)
{
	camera.setCenter(x, y);
}

void sfmlSim_camera::follow()
{
	if (cameraFollowRobot == false)
	{
		cameraFollowRobot = true;
	}
	else
	{
		cameraFollowRobot = false;
	}
}

void sfmlSim_camera::follow(sfmlSim_robot & robot)
{
}

void sfmlSim_camera::move(int x, int y)
{
	camera.move(x, y);
}

void sfmlSim_camera::zoom(int value)
{
	if (value > 0) {
		window_scaling_factor += window_scaling_delta_value;
		camera.setSize(window_width * window_scaling_factor, window_height * window_scaling_factor);
	}
	else if (value < 0) {
		window_scaling_factor -= window_scaling_delta_value;
		camera.setSize(window_width * window_scaling_factor, window_height * window_scaling_factor);
	}
}
