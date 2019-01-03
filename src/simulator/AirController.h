/*
 * AirController.h
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

#ifndef AIRCONTROLLER_H_
#define AIRCONTROLLER_H_

#include "Singleton.h"

#include "Airport.h"
#include "Flight.h"
#include "Position.h"
#include <list>
#include <fstream>
#include <cmath>
#include "Common.h"

namespace atcsim{

class AirController: public Singleton<AirController> {
public:
	AirController();
	virtual ~AirController();

	bool anyland(std::list<Flight*> flights, std::string id);
	bool freeCP_1(std::list<Flight*> flights, Position pos_14, Position pos_15); //para circuitos 1 y 3
	bool freeCP_2(std::list<Flight*> flights, Position pos_1, Position pos_15); //para circuitos 2 y 4
	void doWork();
};

};  // namespace atcsim

#endif /* AIRCONTROLLER_H_ */
