#ifndef MESSENGER_HPP
#define MESSENGER_HPP

#include "main.h"

// Your code here
class Messenger{
    public:
        enum CYLINDER_STATE{
            OPEN,
            CLOSE
        };
        enum CONVEYOR_STATE{
            SET,
            HOLD,
            SHOOT
        };
        Messenger();
            void cylinder(CYLINDER_STATE state);
            void small_conveyor(CONVEYOR_STATE state);
            void big_conveyor(CONVEYOR_STATE state);
        
    private:
        // Add any private members or methods if needed
        uint8_t packet[3];
        void send();
};

#endif // MESSENGER_HPP