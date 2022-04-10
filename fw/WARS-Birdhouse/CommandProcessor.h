/* 
 * LoRa Birdhouse Mesh Network Project
 * Wellesley Amateur Radio Society
 * 
 * Copyright (C) 2022 Bruce MacKinnon
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef _CommandProcessor_h
#define _CommandProcessor_h

int sendPing(int argc, const char** argv);
int info(int argc, const char **argv);
int sleep(int argc, const char **argv);
int setBatteryLimit(int argc, const char **argv);
int setAddr(int argc, const char **argv);
int setCall(int argc, const char **argv);
int doPrint(int argc, const char **argv);
int boot(int argc, const char **argv);
int bootRadio(int argc, const char **argv);
int sendReset(int argc, const char **argv);
int doResetCounters(int argc, const char **argv);
int doRem(int argc, const char **argv);
int sendText(int argc, const char **argv);

int setRoute(int argc, const char **argv);
int clearRoutes(int argc, const char **argv);
int sendSetRoute(int argc, const char **argv);
int sendGetRoute(int argc, const char **argv);


#endif
