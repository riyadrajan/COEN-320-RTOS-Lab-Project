#ifndef DISPLAY_SYSTEM_H
#define DISPLAY_SYSTEM_H

#include <iostream>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include "Msg_structs.h"

class DisplaySystem {
public:
    DisplaySystem();
    ~DisplaySystem();

    void start();
    void join();

private:
    void displayAirspace();      // reads shared memory every 5s, logs every 30s
    void listenForCollisions();  // IPC server on "display" channel

    std::thread displayThread;
    std::thread collisionThread;
    std::atomic<bool> running;
};

#endif // DISPLAY_SYSTEM_H
