#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <algorithm>    // For std::min
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <sys/dispatch.h>
#include <cstring> // For memcpy

// COEN320 Task 3.1, set the display channel name
#define display_channel_name "display"
#define SHM_NAME "/radar_shared_mem_40290831"
#define SHARED_MEMORY_SIZE sizeof(SharedMemory)



ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();
    cleanupSharedMemory();
}

bool ComputerSystem::initializeSharedMemory() {
	// Open the shared memory object
	while (true) {
        // COEN320 Task 3.2
		// Attempt to open the shared memory object (You need to use the same name as Task 2 in Radar)
        // In case of error, retry until successful
        // e.g. shm_open("/radar_shared_mem", O_RDONLY, 0666);
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
        while(shm_fd == -1){
            perror("shm_open failed. Retrying to open shared memory");
            shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
            if (shm_fd != -1) break;
        }

        // COEN320 Task 3.3
		// Map the shared memory object into the process's address space
        // The shared memory should be mapped to "shared_mem" (check for errors)
        shared_mem = (SharedMemory *)mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ,
		MAP_SHARED, shm_fd, 0);
		if (shared_mem == MAP_FAILED) {
		perror("mmap");
		exit(EXIT_FAILURE);
	}

        std::cout << "Shared memory initialized successfully." << std::endl;
        return true;
	}
}

void ComputerSystem::cleanupSharedMemory() {
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
    }
    if (shm_fd != -1) {
        close(shm_fd);
    }
}

bool ComputerSystem::startMonitoring() {
    if (initializeSharedMemory()) {
        running = true;
        std::cout << "Starting monitoring thread." << std::endl;
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);
        monitorOperatorInput = std::thread(&ComputerSystem::processMessage, this);
        return true;
    } else {
        std::cerr << "Failed to initialize shared memory. Monitoring not started.\n";
        return false;
    }
}

void ComputerSystem::joinThread() {
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
    if (monitorOperatorInput.joinable()) {
        monitorOperatorInput.join();
    }
}

void ComputerSystem::monitorAirspace() {
	//std::cout << "Initial is_empty value: " << shared_mem->is_empty.load() << std::endl;
	ATCTimer timer(1,0);
	// Vector to store plane data
	std::vector<msg_plane_info> plane_data_vector;
	uint64_t timestamp;
    // Keep monitoring indefinitely until `stopMonitoring` is called
	while (shared_mem->is_empty.load()) {
		std::cout << "Waiting for planes in airspace...\n";
		timer.waitTimer();
	}

	while (running) {
		if (shared_mem->is_empty.load()) {
			std::cout << "No planes in airspace. Stopping monitoring.\n";
			running = false;
	        break;
        } else {
        	plane_data_vector.clear();
        	// Print separator and timestamp
            //std::cout << "\n================= Shared Memory Update =================\n";

            // Display the timestamp in a readable format
            timestamp = shared_mem->timestamp;
            //std::cout << "Last Update Timestamp: " << timestamp << "\n";
            //std::cout << "Number of planes in shared memory: " << shared_mem->count << "\n";

            for (int i = 0; i < shared_mem->count; ++i) {
            	const msg_plane_info& plane = shared_mem->plane_data[i];
            	// Store the plane info in the vector
            	plane_data_vector.push_back(plane);
            }
        }
		//**************Call Collision Detector*********************
		if (plane_data_vector.size()>1)
            checkCollision(timestamp, plane_data_vector);
		else
            std::cout << "No collision possible with single plane\n";
        // Sleep for a short interval before the next poll
       timer.waitTimer();
    }
	std::cout << "Exiting monitoring loop." << std::endl;
}

void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    std::cout << "Checking for collisions at time: " << currentTime << std::endl;

    std::vector<std::pair<int, int>> collisionPairs;

    // COEN320 Task 3.4: check every unique pair of planes
    for (size_t i = 0; i < planes.size(); ++i) {
        for (size_t j = i + 1; j < planes.size(); ++j) {
            if (checkAxes(planes[i], planes[j])) {
                std::cout << "*** COLLISION WARNING: Aircraft "
                          << planes[i].id << " and " << planes[j].id << " ***\n";
                collisionPairs.emplace_back(planes[i].id, planes[j].id);
            }
        }
    }

    // COEN320 Task 3.5: send collision alert to Display
    if (!collisionPairs.empty()) {
        Message_inter_process msg_to_send;
        // data array is 256 bytes; each pair<int,int> is 8 bytes → max 32 pairs per message
        constexpr size_t kMaxPairs = 256 / sizeof(std::pair<int, int>);
        size_t numPairs = std::min(collisionPairs.size(), kMaxPairs);
        size_t dataSize = numPairs * sizeof(std::pair<int, int>);

        msg_to_send.header = true;
        msg_to_send.planeID = -1;
        msg_to_send.type = MessageType::COLLISION_DETECTED;
        msg_to_send.dataSize = dataSize;
        std::memcpy(msg_to_send.data.data(), collisionPairs.data(), dataSize);
        sendCollisionToDisplay(msg_to_send);
    }
}

bool ComputerSystem::checkAxes(msg_plane_info plane1, msg_plane_info plane2) {
    // COEN320 Task 3.4
    // A collision is defined as two planes entering the defined airspace constraints within the time constraint
    // You need to implement the logic to check if plane1 and plane2 will collide within the time constraint
    // Return true if they will collide, false otherwise
    // A simple approach is to just check if their positions will be within the defined constraints (e.g., CONSTRAINT_X, CONSTRAINT_Y, CONSTRAINT_Z)
    // A more accurate approach would involve calculating their future positions based on their velocities
    // and checking if those future positions will be within the defined constraints within the time constraint
    // Assuming 0 acceleration, eqn will be s = s0 + vt
    double S1x = plane1.PositionX + plane1.VelocityX * timeConstraintCollisionFreq;
    double S1y = plane1.PositionY + plane1.VelocityY * timeConstraintCollisionFreq;
    double S1z = plane1.PositionZ + plane1.VelocityZ * timeConstraintCollisionFreq;

    double S2x = plane2.PositionX + plane2.VelocityX * timeConstraintCollisionFreq;
    double S2y = plane2.PositionY + plane2.VelocityY * timeConstraintCollisionFreq;
    double S2z = plane2.PositionZ + plane2.VelocityZ * timeConstraintCollisionFreq;

    double deltaX = fabs(S1x - S2x);
    double deltaY = fabs(S1y - S2y);
    double deltaZ = fabs(S1z - S2z);

    double deltaSx, deltaSy, deltaSz;
    deltaSx = fabs(plane1.PositionX - plane2.PositionX);
    deltaSy = fabs(plane1.PositionY - plane2.PositionY);
    deltaSz = fabs(plane1.PositionZ - plane2.PositionZ);

    if (deltaX <= CONSTRAINT_X && deltaY <= CONSTRAINT_Y && deltaZ <= CONSTRAINT_Z || deltaSx <= CONSTRAINT_X && deltaSy <= CONSTRAINT_Y  && deltaSz <= CONSTRAINT_Z) {
        return true;
    } 

    return false; // Placeholder return value; replace with actual collision detection logic
}


void ComputerSystem::sendCollisionToDisplay(const Message_inter_process& msg){
	int display_channel = name_open(display_channel_name, 0);
	if (display_channel == -1) {
		throw std::runtime_error("Computer system: Error occurred while attaching to display");
	}
	int reply;

	int status = MsgSend(display_channel, &msg, sizeof(msg), &reply, sizeof(reply));
	if (status == -1) {
		perror("Computer system: Error occurred while sending message to display channel");
	}
    name_close(display_channel);
}

// COEN320 Task 4: receive commands from OperatorConsole and route to CommunicationsSystem
void ComputerSystem::processMessage() {
    name_attach_t* channel = name_attach(NULL, "COMP_SYS_40290831", 0);
    if (channel == NULL) {
        perror("ComputerSystem: name_attach for operator channel failed");
        return;
    }

    std::cout << "ComputerSystem: operator channel COMP_SYS_40290831 ready.\n";

    while (running) {
        Message_inter_process msg{};
        int rcvid = MsgReceive(channel->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            perror("ComputerSystem: MsgReceive on operator channel failed");
            continue;
        }

        int ok = 0;
        MsgReply(rcvid, EOK, &ok, sizeof(ok));

        if (msg.type == MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS) {
            handleTimeConstraintChange(msg);
        } else {
            // Forward heading/position/altitude commands to CommunicationsSystem
            sendMessagesToComms(msg);
        }
    }

    name_detach(channel, 0);
}

void ComputerSystem::sendMessagesToComms(const Message_inter_process& msg) {
    int ch = name_open("ATC_COMMS_40290831", 0);
    if (ch == -1) {
        perror("ComputerSystem: could not open CommunicationsSystem channel");
        return;
    }
    int reply = 0;
    if (MsgSend(ch, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
        perror("ComputerSystem: MsgSend to CommunicationsSystem failed");
    }
    name_close(ch);
}

void ComputerSystem::handleTimeConstraintChange(const Message_inter_process& msg) {
    if (msg.dataSize < sizeof(int)) {
        std::cerr << "ComputerSystem: invalid time constraint message size\n";
        return;
    }
    int n = 0;
    std::memcpy(&n, msg.data.data(), sizeof(int));
    timeConstraintCollisionFreq = n;
    std::cout << "ComputerSystem: collision time constraint updated to " << n << " seconds.\n";
}
