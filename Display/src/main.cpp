#include "DisplaySystem.h"

int main() {
    DisplaySystem display;
    display.start();
    display.join();
    std::cout << "Display: exiting.\n";
    return 0;
}
