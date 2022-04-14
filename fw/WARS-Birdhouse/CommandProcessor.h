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

int sendPing(int argc, char** argv);
int sendGetSed(int argc, char** argv);
int sendReset(int argc, char **argv);
int info(int argc, char **argv);
int sleep(int argc, char **argv);
int setBatteryLimit(int argc, char **argv);
int setAddr(int argc, char **argv);
int setCall(int argc, char **argv);
int doPrint(int argc, char **argv);
int boot(int argc, char **argv);
int bootRadio(int argc, char **argv);
int doResetCounters(int argc, char **argv);
int doRem(int argc, char **argv);
int sendText(int argc, char **argv);

int setRoute(int argc, char **argv);
int clearRoutes(int argc, char **argv);
int sendSetRoute(int argc, char **argv);
int sendGetRoute(int argc, char **argv);

#endif
