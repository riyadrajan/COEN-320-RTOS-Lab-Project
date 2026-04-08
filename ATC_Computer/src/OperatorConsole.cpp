#include "OperatorConsole.h"
#include <fstream>
#include <sstream>
#include <cstring>
#include <ctime>

// ComputerSystem receives operator commands on this channel
#define COMP_SYS_CHANNEL "COMP_SYS_40290831"

OperatorConsole::OperatorConsole() {}

void OperatorConsole::start() {
    Operator_Console = std::thread(&OperatorConsole::HandleConsoleInputs, this);
}

OperatorConsole::~OperatorConsole() {
    exit = true;
    if (Operator_Console.joinable()) {
        Operator_Console.join();
    }
}

// Send a Message_inter_process to ComputerSystem's operator channel
static bool sendToComputerSystem(const Message_inter_process& msg) {
    int ch = name_open(COMP_SYS_CHANNEL, 0);
    if (ch == -1) {
        perror("OperatorConsole: name_open to ComputerSystem failed");
        return false;
    }
    int reply = 0;
    if (MsgSend(ch, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
        perror("OperatorConsole: MsgSend failed");
        name_close(ch);
        return false;
    }
    name_close(ch);
    return true;
}

void OperatorConsole::logCommand(const std::string& command) {
    std::ofstream log("/tmp/operator_log.txt", std::ios::app);
    if (!log.is_open()) return;

    // Prefix with a simple timestamp (seconds since epoch)
    std::time_t now = std::time(nullptr);
    log << "[" << now << "] " << command << "\n";
}

void OperatorConsole::HandleConsoleInputs() {
    std::cout << "OperatorConsole ready. Commands:\n"
              << "  <id> heading <vx> <vy> <vz>\n"
              << "  <id> position <x> <y> <z>\n"
              << "  <id> altitude <z>\n"
              << "  time_constraint <seconds>\n"
              << "  exit\n";

    std::string line;
    while (!exit && std::getline(std::cin, line)) {
        if (line.empty()) continue;

        logCommand(line);

        std::istringstream ss(line);
        std::string token;
        ss >> token;

        // --- exit ---
        if (token == "exit") {
            exit = true;
            break;
        }

        // --- time_constraint <n> ---
        if (token == "time_constraint") {
            int n = 0;
            if (!(ss >> n)) {
                std::cerr << "Usage: time_constraint <seconds>\n";
                continue;
            }
            Message_inter_process msg{};
            msg.header = true;
            msg.planeID = -1;
            msg.type = MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS;
            msg.dataSize = sizeof(int);
            std::memcpy(msg.data.data(), &n, sizeof(int));
            sendToComputerSystem(msg);
            continue;
        }

        // All remaining commands start with a plane ID
        int planeID = 0;
        try {
            planeID = std::stoi(token);
        } catch (...) {
            std::cerr << "Unknown command: " << line << "\n";
            continue;
        }

        std::string cmd;
        if (!(ss >> cmd)) {
            std::cerr << "Missing sub-command for plane " << planeID << "\n";
            continue;
        }

        Message_inter_process msg{};
        msg.header = true;
        msg.planeID = planeID;

        // --- heading <vx> <vy> <vz> ---
        if (cmd == "heading") {
            msg_change_heading h{};
            if (!(ss >> h.VelocityX >> h.VelocityY >> h.VelocityZ)) {
                std::cerr << "Usage: <id> heading <vx> <vy> <vz>\n";
                continue;
            }
            h.ID = planeID;
            msg.type = MessageType::REQUEST_CHANGE_OF_HEADING;
            msg.dataSize = sizeof(msg_change_heading);
            std::memcpy(msg.data.data(), &h, sizeof(h));

        // --- position <x> <y> <z> ---
        } else if (cmd == "position") {
            msg_change_position p{};
            if (!(ss >> p.x >> p.y >> p.z)) {
                std::cerr << "Usage: <id> position <x> <y> <z>\n";
                continue;
            }
            msg.type = MessageType::REQUEST_CHANGE_POSITION;
            msg.dataSize = sizeof(msg_change_position);
            std::memcpy(msg.data.data(), &p, sizeof(p));

        // --- altitude <z> ---
        } else if (cmd == "altitude") {
            msg_change_heading a{};
            if (!(ss >> a.altitude)) {
                std::cerr << "Usage: <id> altitude <z>\n";
                continue;
            }
            a.ID = planeID;
            msg.type = MessageType::REQUEST_CHANGE_ALTITUDE;
            msg.dataSize = sizeof(msg_change_heading);
            std::memcpy(msg.data.data(), &a, sizeof(a));

        } else {
            std::cerr << "Unknown sub-command: " << cmd << "\n";
            continue;
        }

        sendToComputerSystem(msg);
    }

    std::cout << "OperatorConsole: exiting.\n";
}
