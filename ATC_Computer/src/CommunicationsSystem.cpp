#include "CommunicationsSystem.h"
#include <cstring>

// Named channel that ComputerSystem uses to send commands to CommunicationsSystem
#define COMMS_CHANNEL "ATC_COMMS_40290831"

CommunicationsSystem::CommunicationsSystem() {}

void CommunicationsSystem::start() {
    Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
}

CommunicationsSystem::~CommunicationsSystem() {
    if (Communications_System.joinable()) {
        Communications_System.join();
    }
}

void CommunicationsSystem::HandleCommunications() {
    name_attach_t* channel = name_attach(NULL, COMMS_CHANNEL, 0);
    if (channel == NULL) {
        perror("CommunicationsSystem: name_attach failed");
        return;
    }

    std::cout << "CommunicationsSystem: listening on channel " << COMMS_CHANNEL << "\n";

    while (true) {
        Message_inter_process msg{};
        int rcvid = MsgReceive(channel->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            perror("CommunicationsSystem: MsgReceive failed");
            continue;
        }

        int ok = 0;
        MsgReply(rcvid, EOK, &ok, sizeof(ok));  // always reply before doing anything else

        if (msg.type == MessageType::EXIT) {
            break;
        }

        messageAircraft(msg);
    }

    name_detach(channel, 0);
    std::cout << "CommunicationsSystem: shutdown.\n";
}

void CommunicationsSystem::messageAircraft(const Message_inter_process& msg) {
    // Aircraft channel name: group prefix + plane ID
    std::string channelName = "40290831" + std::to_string(msg.planeID);

    int plane_ch = name_open(channelName.c_str(), 0);
    if (plane_ch == -1) {
        std::cerr << "CommunicationsSystem: could not open channel for aircraft "
                  << msg.planeID << "\n";
        return;
    }

    int reply = 0;
    if (MsgSend(plane_ch, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
        perror("CommunicationsSystem: MsgSend to aircraft failed");
    }

    name_close(plane_ch);
}
