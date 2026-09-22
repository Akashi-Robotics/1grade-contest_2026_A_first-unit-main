#ifndef MESSENGER_HPP
#define MESSENGER_HPP

#include "main.h"

// Your code here
class Messenger{
    public:
        enum CYLINDER_OPERATION_MODE{
            RIGHT,
            LEFT
        };
        enum CYLINDER_STATE{
            OPEN,
            CLOSE,
            REVERSE
        };
        enum CONVEYOR_OPERATION_MODE{
            TALL,
            RIGHT,
            LEFT
        };
        enum CONVEYOR_STATE{
            SET,
            HOLD,
            SHOOT,
            PLUS,
            MINUS
        };
        Messenger();
            void cylinder(CYLINDER_OPERATION_MODE mode, CYLINDER_STATE state);
            void conveyor(CONVEYOR_OPERATION_MODE mode, CONVEYOR_STATE state);
        
    private:
        // Add any private members or methods if needed
        uint8_t packet[5];
        void send();
};

#endif // MESSENGER_HPP