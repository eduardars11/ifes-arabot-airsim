#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include <stdlib.h>

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <fstream>
#include <math.h>
#include <sstream>
#include <cstring>
#include <string>
#include "Waypoints.h"
#include "LateralControl.h"
#include "LongitudinalControl.h"
#include "pnmfile.h"
#include "uchar.h"

using namespace msr::airlib;
using namespace std;

bool finish(const msr::airlib::Pose &pose)
{
	if (pose.position[0] > -3 && pose.position[0] < -1) {
		if (pose.position[1] > -5 && pose.position[1] < 5) {
			return true;
		}
	}
	return false;

}

bool add(const msr::airlib::Pose &initial, const msr::airlib::Pose &final, float intervalo)
{
	float final_x = final.position[0];
	float final_y = final.position[1];
	float initial_x = initial.position[0];
	float initial_y = initial.position[1];
	float dist = sqrt((pow((final_x - initial_x), 2.0) + (pow((final_y - initial_y), 2.0))));
	if (dist >= intervalo)
		return true;
	return false;
}

void drive(msr::airlib::CarRpcLibClient &client, float &acceleration, float &steering) {
	CarApiBase::CarControls controls;
	if (acceleration >= 0)
		controls.throttle = acceleration;
	else
		controls.brake = -acceleration;
	controls.steering = steering;
	client.setCarControls(controls);
}

bool LeituraMapa(const char* mapaPista, int x, int y) {
	segment::image<segment::uchar>* faixa;
	faixa = segment::loadPBM(mapaPista);
	if (faixa->access[150 + x][150 + y] == 1) {
		return 1;
	}
}

void VerificaFaixa(const char* mapaPista, int x, int y) {
	segment::image<segment::uchar>* pista;
	pista = new segment::image<unsigned char>(300, 300, false);
	pista->init(1);
	pista->access[150 + x][150 + y] = 0;
	if (pista->access[150 + x][150 + y] == 0 && LeituraMapa(mapaPista, x, y)) {
		std::cout << "Fora da faixa ! \n" << std::endl;
	}
}

int main()
{
	Waypoints checkpoints, trajectory;
	LateralControl lateral_control(20, 6, 9);
	LongitudinalControl velocity_control(1.0, 0, 0.01);
	msr::airlib::CarRpcLibClient client;

	int option;
	std::cout << "Escolha uma das opcoes abaixo:\n";
	std::cout << "[1] Para Salvar manualmente os pontos.\n";
	std::cout << "[2] Para o Carro andar automaticamente.\n";
	std::cout << "[3] Para o Carro mapear automaticamente.\n";
	std::cin >> option;

	segment::image<unsigned char> *map = 0;

	try {
		client.confirmConnection();
		client.reset();

		if (option >= 2) {
			checkpoints.LoadWaypoints("Trajetoria.txt");
			client.enableApiControl(true);
		}
		if (option == 3)
		{
			map = new segment::image<unsigned char>(300, 300, false);
			map->init(1);
		}

		msr::airlib::Pose car_pose_previous;
		car_pose_previous.position[0] = 0;
		car_pose_previous.position[1] = 0;
		msr::airlib::Pose car_pose_current;

		do {
			auto car_state = client.getCarState();
			auto car_speed = car_state.speed;

			car_pose_current = car_state.kinematics_estimated.pose;
			if (option == 1) {
				if (add(car_pose_previous, car_pose_current, 3.0)) {
					trajectory.AddWaypoints(car_pose_current.position[0], car_pose_current.position[1], car_speed);
					car_pose_previous = car_pose_current;
					int x = (int)car_pose_current.position[0];
					int y = (int)car_pose_current.position[1];
					VerificaFaixa("Mapa.pbm", x, y);
				}
			}
			if (option >= 2) {
				Vector3r pose(car_pose_current.position[0], car_pose_current.position[1], VectorMath::yawFromQuaternion(car_pose_current.orientation));
				double desired_velocity = checkpoints.GetWaypointVelocity(pose);
				float steering = lateral_control.Update(checkpoints, pose, car_speed);
				float acceleration = velocity_control.Update(car_speed, desired_velocity);
				drive(client, acceleration, steering);
			}
			if (option == 3)
			{
				int x = (int)car_pose_current.position[0];
				int y = (int)car_pose_current.position[1];
				map->access[150+x][150+y] = 0; //Translada as coordenadas para garantir que os indices sejam sempre positivos
				
				// TODO : mapear a pista colocando 0 em toda sua extensao em nao somente na trajetoria do carro como foi feito acima
			}

		} while (!finish(car_pose_current));

		if (option == 1)
		{
			trajectory.SaveWaypoints("Trajetoria.txt");
		}
		if (option == 3)
		{
			segment::savePBM(map, "Mapa.pbm");
			delete map;
		}
	}

	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Verifique os erros enviados pela API do AirSim." << std::endl << msg << std::endl; std::cin.get();
	}
	
	return 0;
}