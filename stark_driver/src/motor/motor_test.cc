
#include <cstdio>
#include "fsleep.hh"
#include "MotorController.hh"

void test(std::string dev) {
    printf("Testing motor controller @ %s\n", dev.c_str());

    bool enable_aux = false;

    MotorController mc(dev);
    for(int i = 0; i < 10; ++i) {
        mc.set_power(0.0, 0.5);
        mc.set_aux(enable_aux);
        enable_aux = !enable_aux;
        fsleep(0.5);
    }

    mc.set_power(0.0, 0.0);
}

int main(int argc, char* argv[]) {
    if(argc != 2) {
        printf("Usage: ./motor_test /dev/ttyUSBXX\n");
        return 1;
    }

    test(argv[1]);
    return 0;
}

