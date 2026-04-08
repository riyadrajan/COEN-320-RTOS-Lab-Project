#include "ComputerSystem.h"
#include "OperatorConsole.h"
#include "CommunicationsSystem.h"

int main() {
    //1. Start CommunicationsSystem first so its channel ("ATC_COMMS_40290831") is
    //ready before ComputerSystem's processMessage thread tries to forward commands
    CommunicationsSystem comms;
    comms.start();

    //2. Start monitoring (whchi launches monitorAirspace + processMessage threads, & which
    //registers "COMP_SYS_40290831" channel before OperatorConsole sends to it).
    ComputerSystem computerSystem;
    if (!computerSystem.startMonitoring()) {
        std::cerr << "Failed to start monitoring." << std::endl;
        return 1;
    }

    //3. OperatorConsole is started last so the channel it writes would already exist.
    OperatorConsole console;
    console.start();

    computerSystem.joinThread();

    std::cout << "Monitoring stopped. Exiting main." << std::endl;
    return 0;
}
