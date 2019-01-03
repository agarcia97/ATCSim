/*
 * AirController.cpp
 *
 *  Created on: 21/07/2014
 *      Author: paco
 *
 *  Copyright 2014 Francisco Martín
 *
 *  This file is part of ATCSim.
 *
 *  ATCSim is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ATCSim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ATCSim.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AirController.h"
#include "Airport.h"
#include "Flight.h"
#include "Position.h"
#include <list>
#include <fstream>
#include <cmath>
#include "Common.h"

#define SPEED 220.0f //velocidad en los circuitos
#define DIST 900.0f //distancia entre avion y pos2, para aterrizar(600 es seguro)

namespace atcsim{

AirController::AirController() {
	// TODO Auto-generated constructor stub

}

AirController::~AirController() {
	// TODO Auto-generated destructor stub
}

bool
AirController::anyland(std::list<Flight*> flights, std::string id)
{
  Position pos0(3500.0, 0.0, 100.0);
  Position pos1(1500.0, 0.0, 50.0);
  Position pos2(200.0, 0.0, 25.0);
  Position pos1_15(8000.0, 750.0, 1120.0);
  bool landing = false;
  std::list<Flight*>::iterator it = flights.begin();

  float dist1 = pos1_15.distance(pos0); //distancia entre pos0 y salidas de los circuitos

  while (landing!=true && it!=flights.end()){
    Position pos_aux = (*it)->getRoute()->front().pos;
    if((pos_aux.distance(pos0)==0 && (*it)->getPosition().distance(pos0)<=dist1 && (*it)->getId()!=id)
        || (pos_aux.distance(pos1)==0)
        || (pos_aux.distance(pos2)==0 && (*it)->getPosition().distance(pos2)>=DIST)){
      landing = true;
    }
    it++;
  }
  return landing;
}

bool
AirController::freeCP_1(std::list<Flight*> flights, Position pos_14, Position pos_15)
{
  bool free = true;
  std::list<Flight*>::iterator it = flights.begin();
  while (free!=false && it!=flights.end()){
    if((*it)->getRoute()->front().pos.distance(pos_15) == 0
        && (*it)->getPosition().distance(pos_15) >= 3500
        && (*it)->getPosition().distance(pos_15) <= 4300
        && (*it)->getInclination() < toRadians(10.0)){
      free = false;
    }
    if((*it)->getRoute()->front().pos.distance(pos_14) == 0
        && (*it)->getPosition().distance(pos_14) <= 3500){
      free = false;
    }
    ++it;
  }
  return free;
}

bool
AirController::freeCP_2(std::list<Flight*> flights, Position pos_1, Position pos_15)
{
  bool free = true;
  std::list<Flight*>::iterator it = flights.begin();
  while (free!=false && it!=flights.end()){
    if((*it)->getRoute()->front().pos.distance(pos_15)==0
        && (*it)->getPosition().distance(pos_15)<=pos_1.distance(pos_15)+1500
        && (*it)->getPosition().distance(pos_15)>=pos_1.distance(pos_15)-1500
        && (*it)->getInclination() <= toRadians(6.0)){
      free = false;
    }
    it++;
  }
  return free;
}




void
AirController::doWork()
{
  std::list<Flight*> flights = Airport::getInstance()->getFlights();
  std::list<Flight*>::iterator it;

  Position pos0(3500.0, 0.0, 100.0);
  Position pos1(1500.0, 0.0, 50.0);
  Position pos2(200.0, 0.0, 25.0);
  Position pos3(-750.0, 0.0, 25.0);

  //Waypoints circuito1
  Position pos1_0(14140.0, 750.0, 3000.0);
  Position pos1_1(13270.0, 750.0, 3000.0);
  Position pos1_2(6820.0, 750.0, 3000.0);
  Position pos1_3(6820.0, 12640.0, 3000.0);
  Position pos1_4(12140.0, 12640.0, 3000.0);
  Position pos1_5(12140.0, 2250.0, 3000.0);
  Position pos1_6(8320.0, 2250.0, 3000.0);
  Position pos1_7(8320.0, 9640.0, 1500.0);
  Position pos1_8(8320.0, 11140.0, 1500.0);
  Position pos1_9(10640.0, 11140.0, 1500.0);
  Position pos1_10(10640.0, 2250.0, 1500.0);
  Position pos1_11(6820.0, 2250.0, 1500.0);
  Position pos1_12(6820.0, 12640.0, 1500.0);
  Position pos1_13(12140.0, 12640.0, 1500.0);
  Position pos1_14(12140.0, 750.0, 1500.0);
  Position pos1_15(8000.0, 750.0, 1120.0);

  //Waypoints circuito2
  Position pos2_0(5320.0, 14140.0, 3000.0);
  Position pos2_1(5320.0, 11060.0, 3000.0);
  Position pos2_2(5320.0, 1500.0, 3000.0);
  Position pos2_3(0.0, 1500.0, 3000.0);
  Position pos2_4(0.0, 14140.0, 3000.0);
  Position pos2_5(3820.0, 14140.0, 3000.0);
  Position pos2_6(3820.0, 3000.0, 3000.0);
  Position pos2_7(1500.0, 3000.0, 3000.0);
  Position pos2_8(1500.0, 11140.0, 1500.0);
  Position pos2_9(1500.0, 12640.0, 1500.0);
  Position pos2_10(3820.0, 12640.0, 1500.0);
  Position pos2_11(3820.0, 1500.0, 1500.0);
  Position pos2_12(0.0, 1500.0, 1500.0);
  Position pos2_13(0.0, 14140.0, 1500.0);
  Position pos2_14(5320.0, 14140.0, 1500.0);
  Position pos2_15(5320.0, 4130.0, 1060.0);

  //Waypoints circuito3
  Position pos3_0(14140.0, -750.0, 3000.0);
  Position pos3_1(13270.0, -750.0, 3000.0);
  Position pos3_2(6820.0, -750.0, 3000.0);
  Position pos3_3(6820.0, -12640.0, 3000.0);
  Position pos3_4(12140.0, -12640.0, 3000.0);
  Position pos3_5(12140.0, -2250.0, 3000.0);
  Position pos3_6(8320.0, -2250.0, 3000.0);
  Position pos3_7(8320.0, -9640.0, 1500.0);
  Position pos3_8(8320.0, -11140.0, 1500.0);
  Position pos3_9(10640.0, -11140.0, 1500.0);
  Position pos3_10(10640.0, -2250.0, 1500.0);
  Position pos3_11(6820.0, -2250.0, 1500.0);
  Position pos3_12(6820.0, -12640.0, 1500.0);
  Position pos3_13(12140.0, -12640.0, 1500.0);
  Position pos3_14(12140.0, -750.0, 1500.0);
  Position pos3_15(8000.0, -750.0, 1120.0);

  //Waypoints circuito4
  Position pos4_0(5320.0, -14140.0, 3000.0);
  Position pos4_1(5320.0, -11060.0, 3000.0);
  Position pos4_2(5320.0, -1500.0, 3000.0);
  Position pos4_3(0.0, -1500.0, 3000.0);
  Position pos4_4(0.0, -14140.0, 3000.0);
  Position pos4_5(3820.0, -14140.0, 3000.0);
  Position pos4_6(3820.0, -3000.0, 3000.0);
  Position pos4_7(1500.0, -3000.0, 3000.0);
  Position pos4_8(1500.0, -11140.0, 1500.0);
  Position pos4_9(1500.0, -12640.0, 1500.0);
  Position pos4_10(3820.0, -12640.0, 1500.0);
  Position pos4_11(3820.0, -1500.0, 1500.0);
  Position pos4_12(0.0, -1500.0, 1500.0);
  Position pos4_13(0.0, -14140.0, 1500.0);
  Position pos4_14(5320.0, -14140.0, 1500.0);
  Position pos4_15(5320.0, -4130.0, 1060.0);


  Route r0, r1, r2, r3, raux, raux2, raux3, raux4, raux5;
  Route c1_0, c1_1, c1_2, c1_3, c1_4, c1_5, c1_6, c1_7, c1_8, c1_9, c1_10, c1_11, c1_12, c1_13, c1_14, c1_15;
  Route c2_0, c2_1, c2_2, c2_3, c2_4, c2_5, c2_6, c2_7, c2_8, c2_9, c2_10, c2_11, c2_12, c2_13, c2_14, c2_15;
  Route c3_0, c3_1, c3_2, c3_3, c3_4, c3_5, c3_6, c3_7, c3_8, c3_9, c3_10, c3_11, c3_12, c3_13, c3_14, c3_15;
  Route c4_0, c4_1, c4_2, c4_3, c4_4, c4_5, c4_6, c4_7, c4_8, c4_9, c4_10, c4_11, c4_12, c4_13, c4_14, c4_15;

  r0.pos = pos0;
  r0.speed = SPEED;
  r1.pos = pos1;
  r1.speed = 100.0;
  r2.pos = pos2;
  r2.speed = 19.0;
  r3.pos = pos3;
  r3.speed = 15.0;

  //Route circuito1
  c1_0.pos = pos1_0;
  c1_0.speed = 500;
  c1_1.pos = pos1_1;
  c1_1.speed = SPEED;
  c1_2.pos = pos1_2;
  c1_2.speed = SPEED;
  c1_3.pos = pos1_3;
  c1_3.speed = SPEED;
  c1_4.pos = pos1_4;
  c1_4.speed = SPEED;
  c1_5.pos = pos1_5;
  c1_5.speed = SPEED;
  c1_6.pos = pos1_6;
  c1_6.speed = SPEED;
  c1_7.pos = pos1_7;
  c1_7.speed = SPEED;
  c1_8.pos = pos1_8;
  c1_8.speed = SPEED;
  c1_9.pos = pos1_9;
  c1_9.speed = SPEED;
  c1_10.pos = pos1_10;
  c1_10.speed = SPEED;
  c1_11.pos = pos1_11;
  c1_11.speed = SPEED;
  c1_12.pos = pos1_12;
  c1_12.speed = SPEED;
  c1_13.pos = pos1_13;
  c1_13.speed = SPEED;
  c1_14.pos = pos1_14;
  c1_14.speed = SPEED;
  c1_15.pos = pos1_15;
  c1_15.speed = SPEED;

  //Route circuito2
  c2_0.pos = pos2_0;
  c2_0.speed = 500;
  c2_1.pos = pos2_1;
  c2_1.speed = SPEED;
  c2_2.pos = pos2_2;
  c2_2.speed = SPEED;
  c2_3.pos = pos2_3;
  c2_3.speed = SPEED;
  c2_4.pos = pos2_4;
  c2_4.speed = SPEED;
  c2_5.pos = pos2_5;
  c2_5.speed = SPEED;
  c2_6.pos = pos2_6;
  c2_6.speed = SPEED;
  c2_7.pos = pos2_7;
  c2_7.speed = SPEED;
  c2_8.pos = pos2_8;
  c2_8.speed = SPEED;
  c2_9.pos = pos2_9;
  c2_9.speed = SPEED;
  c2_10.pos = pos2_10;
  c2_10.speed = SPEED;
  c2_11.pos = pos2_11;
  c2_11.speed = SPEED;
  c2_12.pos = pos2_12;
  c2_12.speed = SPEED;
  c2_13.pos = pos2_13;
  c2_13.speed = SPEED;
  c2_14.pos = pos2_14;
  c2_14.speed = SPEED;
  c2_15.pos = pos2_15;
  c2_15.speed = SPEED;

  //Route circuito3
  c3_0.pos = pos3_0;
  c3_0.speed = 500;
  c3_1.pos = pos3_1;
  c3_1.speed = SPEED;
  c3_2.pos = pos3_2;
  c3_2.speed = SPEED;
  c3_3.pos = pos3_3;
  c3_3.speed = SPEED;
  c3_4.pos = pos3_4;
  c3_4.speed = SPEED;
  c3_5.pos = pos3_5;
  c3_5.speed = SPEED;
  c3_6.pos = pos3_6;
  c3_6.speed = SPEED;
  c3_7.pos = pos3_7;
  c3_7.speed = SPEED;
  c3_8.pos = pos3_8;
  c3_8.speed = SPEED;
  c3_9.pos = pos3_9;
  c3_9.speed = SPEED;
  c3_10.pos = pos3_10;
  c3_10.speed = SPEED;
  c3_11.pos = pos3_11;
  c3_11.speed = SPEED;
  c3_12.pos = pos3_12;
  c3_12.speed = SPEED;
  c3_13.pos = pos3_13;
  c3_13.speed = SPEED;
  c3_14.pos = pos3_14;
  c3_14.speed = SPEED;
  c3_15.pos = pos3_15;
  c3_15.speed = SPEED;

  //Route circuito4
  c4_0.pos = pos4_0;
  c4_0.speed = 500;
  c4_1.pos = pos4_1;
  c4_1.speed = SPEED;
  c4_2.pos = pos4_2;
  c4_2.speed = SPEED;
  c4_3.pos = pos4_3;
  c4_3.speed = SPEED;
  c4_4.pos = pos4_4;
  c4_4.speed = SPEED;
  c4_5.pos = pos4_5;
  c4_5.speed = SPEED;
  c4_6.pos = pos4_6;
  c4_6.speed = SPEED;
  c4_7.pos = pos4_7;
  c4_7.speed = SPEED;
  c4_8.pos = pos4_8;
  c4_8.speed = SPEED;
  c4_9.pos = pos4_9;
  c4_9.speed = SPEED;
  c4_10.pos = pos4_10;
  c4_10.speed = SPEED;
  c4_11.pos = pos4_11;
  c4_11.speed = SPEED;
  c4_12.pos = pos4_12;
  c4_12.speed = SPEED;
  c4_13.pos = pos4_13;
  c4_13.speed = SPEED;
  c4_14.pos = pos4_14;
  c4_14.speed = SPEED;
  c4_15.pos = pos4_15;
  c4_15.speed = SPEED;



/*
  Position posaux(5000.0, 0.0, 2000.0);
  Position posaux2(5000.0, -3000.0, 2000.0);
  Position posaux3(8500.0, -3000.0, 2000.0);
  Position posaux4(8500.0, 0.0, 2000.0);
  Position posaux5(15000.0, 15000.0, 2000.0);
  raux.pos = posaux;
  raux.speed = 150.0;
  raux2.pos = posaux2;
  raux2.speed = 500.0;
  raux3.pos = posaux3;
  raux3.speed = 15.0;
  raux4.pos = posaux4;
  raux4.speed = 500.0;
  raux5.pos = posaux5;
  raux5.speed = 500.0;
//&& (*it)->getPosition().get_x()>=AIRPORT_DISTANCE_MAX*cos(0.25*M_PI) && (*it)->getPosition().get_y()>=0
*/


  for(it = flights.begin(); it!=flights.end(); ++it){
    if((*it)->getRoute()->empty()){
      (*it)->getRoute()->push_back(r3);
      (*it)->getRoute()->push_front(r2);
      (*it)->getRoute()->push_front(r1);
      (*it)->getRoute()->push_front(r0);

      //sector 1
      if((*it)->getPosition().get_x()>=AIRPORT_DISTANCE_MAX*cos(0.25*M_PI)
          && (*it)->getPosition().get_y()>=0
          && (*it)->getPosition().get_y()<=AIRPORT_DISTANCE_MAX*sin(0.25*M_PI)){
        (*it)->getRoute()->push_front(c1_15);
        (*it)->getRoute()->push_front(c1_14);
        (*it)->getRoute()->push_front(c1_13);
        (*it)->getRoute()->push_front(c1_12);
        (*it)->getRoute()->push_front(c1_11);
        (*it)->getRoute()->push_front(c1_10);
        (*it)->getRoute()->push_front(c1_9);
        (*it)->getRoute()->push_front(c1_8);
        (*it)->getRoute()->push_front(c1_7);
        (*it)->getRoute()->push_front(c1_6);
        (*it)->getRoute()->push_front(c1_5);
        (*it)->getRoute()->push_front(c1_4);
        (*it)->getRoute()->push_front(c1_3);
        (*it)->getRoute()->push_front(c1_2);
        (*it)->getRoute()->push_front(c1_1);
        (*it)->getRoute()->push_front(c1_0);

      //sector 3
      }else if((*it)->getPosition().get_x()>=AIRPORT_DISTANCE_MAX*cos(0.25*M_PI)
          && (*it)->getPosition().get_y()<0
          && (*it)->getPosition().get_y()>=-AIRPORT_DISTANCE_MAX*sin(0.25*M_PI)){
        (*it)->getRoute()->push_front(c3_15);
        (*it)->getRoute()->push_front(c3_14);
        (*it)->getRoute()->push_front(c3_13);
        (*it)->getRoute()->push_front(c3_12);
        (*it)->getRoute()->push_front(c3_11);
        (*it)->getRoute()->push_front(c3_10);
        (*it)->getRoute()->push_front(c3_9);
        (*it)->getRoute()->push_front(c3_8);
        (*it)->getRoute()->push_front(c3_7);
        (*it)->getRoute()->push_front(c3_6);
        (*it)->getRoute()->push_front(c3_5);
        (*it)->getRoute()->push_front(c3_4);
        (*it)->getRoute()->push_front(c3_3);
        (*it)->getRoute()->push_front(c3_2);
        (*it)->getRoute()->push_front(c3_1);
        (*it)->getRoute()->push_front(c3_0);

      //sector 2
      }else if((*it)->getPosition().get_y()>AIRPORT_DISTANCE_MAX*sin(0.25*M_PI)){
        (*it)->getRoute()->push_front(c2_15);
        (*it)->getRoute()->push_front(c2_14);
        (*it)->getRoute()->push_front(c2_13);
        (*it)->getRoute()->push_front(c2_12);
        (*it)->getRoute()->push_front(c2_11);
        (*it)->getRoute()->push_front(c2_10);
        (*it)->getRoute()->push_front(c2_9);
        (*it)->getRoute()->push_front(c2_8);
        (*it)->getRoute()->push_front(c2_7);
        (*it)->getRoute()->push_front(c2_6);
        (*it)->getRoute()->push_front(c2_5);
        (*it)->getRoute()->push_front(c2_4);
        (*it)->getRoute()->push_front(c2_3);
        (*it)->getRoute()->push_front(c2_2);
        (*it)->getRoute()->push_front(c2_1);
        (*it)->getRoute()->push_front(c2_0);

      //sector 4
      }else if((*it)->getPosition().get_y()<=-AIRPORT_DISTANCE_MAX*sin(0.25*M_PI)){
        (*it)->getRoute()->push_front(c4_15);
        (*it)->getRoute()->push_front(c4_14);
        (*it)->getRoute()->push_front(c4_13);
        (*it)->getRoute()->push_front(c4_12);
        (*it)->getRoute()->push_front(c4_11);
        (*it)->getRoute()->push_front(c4_10);
        (*it)->getRoute()->push_front(c4_9);
        (*it)->getRoute()->push_front(c4_8);
        (*it)->getRoute()->push_front(c4_7);
        (*it)->getRoute()->push_front(c4_6);
        (*it)->getRoute()->push_front(c4_5);
        (*it)->getRoute()->push_front(c4_4);
        (*it)->getRoute()->push_front(c4_3);
        (*it)->getRoute()->push_front(c4_2);
        (*it)->getRoute()->push_front(c4_1);
        (*it)->getRoute()->push_front(c4_0);
      }

    }


    Route r_frist = (*it)->getRoute()->front();
    //circuito1 bajar nivel
    if((*it)->getPosition().distance(pos1_1)<=DIST_POINT
        && r_frist.pos.distance(pos1_2)==0 && freeCP_1(flights, pos1_14, pos1_15)){
      int i;
      for(i=0; i<13; i++){
        (*it)->getRoute()->pop_front();
      }
    }
    //circuito1 aterrizando
    if((*it)->getPosition().distance(pos1_15)<=DIST_POINT
        && r_frist.pos.distance(pos0)==0 && anyland(flights, (*it)->getId())){
      (*it)->getRoute()->push_front(c1_15);
      (*it)->getRoute()->push_front(c1_14);
      (*it)->getRoute()->push_front(c1_13);
      (*it)->getRoute()->push_front(c1_12);
      (*it)->getRoute()->push_front(c1_11);
    }

    //circuito3 bajar nivel
    if((*it)->getPosition().distance(pos3_1)<=DIST_POINT
        && r_frist.pos.distance(pos3_2)==0 && freeCP_1(flights, pos3_14, pos3_15)){
      int i;
      for(i=0; i<13; i++){
        (*it)->getRoute()->pop_front();
      }
    }
    //circuito3 aterrizando
    if((*it)->getPosition().distance(pos3_15)<=DIST_POINT
        && r_frist.pos.distance(pos0)==0 && anyland(flights, (*it)->getId())){
      (*it)->getRoute()->push_front(c3_15);
      (*it)->getRoute()->push_front(c3_14);
      (*it)->getRoute()->push_front(c3_13);
      (*it)->getRoute()->push_front(c3_12);
      (*it)->getRoute()->push_front(c3_11);
    }

    //circuito2 bajar nivel
    if((*it)->getPosition().distance(pos2_1)<=DIST_POINT
        && r_frist.pos.distance(pos2_2)==0 && freeCP_2(flights, pos2_1, pos2_15)){
      int i;
      for(i=0; i<13; i++){
        (*it)->getRoute()->pop_front();
      }
    }
    //circuito2 aterrizando
    if((*it)->getPosition().distance(pos2_15)<=DIST_POINT
        && r_frist.pos.distance(pos0)==0 && anyland(flights, (*it)->getId())){
      (*it)->getRoute()->push_front(c2_15);
      (*it)->getRoute()->push_front(c2_14);
      (*it)->getRoute()->push_front(c2_13);
      (*it)->getRoute()->push_front(c2_12);
      (*it)->getRoute()->push_front(c2_11);
    }

    //circuito4 bajar nivel
    if((*it)->getPosition().distance(pos4_1)<=DIST_POINT
        && r_frist.pos.distance(pos4_2)==0 && freeCP_2(flights, pos4_1, pos4_15)){
      int i;
      for(i=0; i<13; i++){
        (*it)->getRoute()->pop_front();
      }
    }
    //circuito4 aterrizando
    if((*it)->getPosition().distance(pos4_15)<=DIST_POINT
        && r_frist.pos.distance(pos0)==0 && anyland(flights, (*it)->getId())){
      (*it)->getRoute()->push_front(c4_15);
      (*it)->getRoute()->push_front(c4_14);
      (*it)->getRoute()->push_front(c4_13);
      (*it)->getRoute()->push_front(c4_12);
      (*it)->getRoute()->push_front(c4_11);
    }


  }
}

}  // namespace atcsim
