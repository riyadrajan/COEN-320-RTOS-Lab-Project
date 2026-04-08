#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#include <atomic>
#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

class OperatorConsole {
public:
	OperatorConsole();
    ~OperatorConsole();
    void start();

private:
    void HandleConsoleInputs();
    void logCommand(const std::string& command);
    std::thread Operator_Console;
    std::atomic<bool> exit{false};
};



#endif /* OPERATORCONSOLE_H_ */
