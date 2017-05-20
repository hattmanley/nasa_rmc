#pragma once

#include "IntervalTimer.h"
#include "core_pins.h"

namespace AltEncoder
{
    class Controller;

    //--------------------------------------------------------------------------------------------

    class Encoder
    {
    public:
        Encoder(int pinA, int pinB, int mode = INPUT) : counter(0), _pinA(pinA), _pinB(pinB)
        {
            pinMode(_pinA, INPUT);
            pinMode(_pinB, INPUT);
        }
        volatile int counter;

    private:
        const int _pinA, _pinB;
        unsigned last;

        friend Controller;
    };

    //--------------------------------------------------------------------------------------------

    class Controller
    {
    public:

        static void begin(Encoder** _encList, int t)
        {
            encList = _encList;
            timer.begin(isrEncoderRead, t);
        }

        static void isrEncoderRead(void)
        {
            int current;
            int diff;
            Encoder** encoder = encList;

            while (*encoder != nullptr)
            {
                current = 0;
                if (digitalReadFast((*encoder)->_pinA)) current = 3;
                if (digitalReadFast((*encoder)->_pinB)) current ^= 1;
                diff = (*encoder)->last - current;
                if (diff & 1)
                {
                    (*encoder)->last = current;
                    (*encoder)->counter += (diff & 2) - 1;
                }
                encoder++;
            }          
        }

    private:
        static Encoder** encList;
        static IntervalTimer timer;
    };
}
