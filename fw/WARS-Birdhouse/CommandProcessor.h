#ifndef _CommandProcessor_h
#define _CommandProcessor_h

int sendPing(int argc, const char** argv);
int info(int argc, const char **argv);
int sleep(int argc, const char **argv);
int setBatteryLimit(int argc, const char **argv);
int setRoute(int argc, const char **argv);
int clearRoutes(int argc, const char **argv);

#endif
