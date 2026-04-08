#include "DisplaySystem.h"
#include <cstring>
#include <ctime>
#include <fstream>
#include <chrono>
#include <iomanip>

#define SHM_NAME         "/radar_shared_mem_40290831"
#define SHARED_MEM_SIZE  sizeof(SharedMemory)
#define DISPLAY_INTERVAL_S  5   // print aircraft list every 5 seconds (per spec)
#define LOG_INTERVAL_S      30  // log airspace snapshot every 30 seconds (per spec)

DisplaySystem::DisplaySystem() : running(false) {}

DisplaySystem::~DisplaySystem() {
    running = false;
    join();
}

void DisplaySystem::start() {
    running = true;
    displayThread   = std::thread(&DisplaySystem::displayAirspace,     this);
    collisionThread = std::thread(&DisplaySystem::listenForCollisions,  this);
}

void DisplaySystem::join() {
    if (displayThread.joinable())   displayThread.join();
    if (collisionThread.joinable()) collisionThread.join();
}

// ── Thread 1: read shared memory periodically ─────────────────────────────────
void DisplaySystem::displayAirspace() {
    int    tick        = 0;
    int    logTick     = 0;

    while (running) {
        // Sleep 1 second between polls so we can track 5s / 30s boundaries cheaply
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++tick;
        ++logTick;

        if (tick < DISPLAY_INTERVAL_S) continue;
        tick = 0;

        // Open and map shared memory read-only
        int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
        if (shm_fd == -1) {
            std::cerr << "Display: waiting for shared memory...\n";
            continue;
        }

        SharedMemory* shm = (SharedMemory*)mmap(NULL, SHARED_MEM_SIZE,
                                                 PROT_READ, MAP_SHARED, shm_fd, 0);
        if (shm == MAP_FAILED) {
            perror("Display: mmap failed");
            close(shm_fd);
            continue;
        }

        if (shm->is_empty.load()) {
            std::cout << "\n[Display] Airspace is empty.\n";
            munmap(shm, SHARED_MEM_SIZE);
            close(shm_fd);
            continue;
        }

        // ── Print plan view ──────────────────────────────────────────────────
        std::time_t now = std::time(nullptr);
        std::cout << "\n========== Airspace @ " << std::ctime(&now);
        std::cout << std::left
                  << std::setw(6)  << "ID"
                  << std::setw(12) << "PosX"
                  << std::setw(12) << "PosY"
                  << std::setw(12) << "PosZ"
                  << std::setw(12) << "VelX"
                  << std::setw(12) << "VelY"
                  << std::setw(12) << "VelZ" << "\n";

        int count = shm->count;
        for (int i = 0; i < count; ++i) {
            const msg_plane_info& p = shm->plane_data[i];
            std::cout << std::left
                      << std::setw(6)  << p.id
                      << std::setw(12) << p.PositionX
                      << std::setw(12) << p.PositionY
                      << std::setw(12) << p.PositionZ
                      << std::setw(12) << p.VelocityX
                      << std::setw(12) << p.VelocityY
                      << std::setw(12) << p.VelocityZ << "\n";
        }

        // ── Log snapshot every 30 seconds ────────────────────────────────────
        if (logTick >= LOG_INTERVAL_S) {
            logTick = 0;
            std::ofstream log("/tmp/airspace_history.log", std::ios::app);
            if (log.is_open()) {
                log << "--- Snapshot @ " << std::ctime(&now);
                for (int i = 0; i < count; ++i) {
                    const msg_plane_info& p = shm->plane_data[i];
                    log << "  ID=" << p.id
                        << " pos=(" << p.PositionX << "," << p.PositionY << "," << p.PositionZ << ")"
                        << " vel=(" << p.VelocityX << "," << p.VelocityY << "," << p.VelocityZ << ")\n";
                }
            }
        }

        munmap(shm, SHARED_MEM_SIZE);
        close(shm_fd);
    }
}

// ── Thread 2: IPC server — receive COLLISION_DETECTED from ComputerSystem ─────
void DisplaySystem::listenForCollisions() {
    name_attach_t* channel = name_attach(NULL, "display", 0);
    if (channel == NULL) {
        perror("Display: name_attach on 'display' failed");
        return;
    }

    std::cout << "Display: collision alert server listening on channel 'display'.\n";

    while (running) {
        Message_inter_process msg{};
        int rcvid = MsgReceive(channel->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (running) perror("Display: MsgReceive failed");
            continue;
        }

        int ok = 0;
        MsgReply(rcvid, EOK, &ok, sizeof(ok));

        if (msg.type != MessageType::COLLISION_DETECTED) continue;

        // Deserialize list of colliding pairs
        size_t numPairs = msg.dataSize / sizeof(std::pair<int, int>);
        std::pair<int, int> pairs[50];  // max 50 pairs
        if (numPairs > 0 && msg.dataSize <= sizeof(pairs)) {
            std::memcpy(pairs, msg.data.data(), msg.dataSize);
            for (size_t i = 0; i < numPairs; ++i) {
                std::cout << "\n*** COLLISION WARNING: Aircraft "
                          << pairs[i].first << " and " << pairs[i].second
                          << " ***\n";
            }
        }
    }

    name_detach(channel, 0);
}
