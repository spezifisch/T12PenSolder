/*
 * Copyright 2023 spezifisch <spezifisch23@proton.me> https://github.com/spezifisch
 *
 * This file is part of T12PenSolder.
 *
 * T12PenSolder is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 of the
 * License.
 *
 * T12PenSolder is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Foobar. If not, see
 * <https://www.gnu.org/licenses/>.
 */

#ifndef DEBOUNCE_H
#define DEBOUNCE_H

template <unsigned DEBOUNCE_COUNT>
class Debounce
{
public:
    // call every 100ms or so with button pin reading,
    // returns true if pressed
    bool measure(bool level)
    {
        if (level)
        {
            if (pressCount < DEBOUNCE_COUNT)
            {
                pressCount++;

                // button must be pressed for at least this many cycles
                if (pressCount == DEBOUNCE_COUNT)
                {
                    // count button as pressed, you can call reset() if you want repeated while button is held
                    return true;
                }
            }
        }
        else
        {
            pressCount = 0;
        }

        return false;
    }
    
    void reset()
    {
        pressCount = 0;
    }

protected:
    unsigned pressCount = 0;
};

#endif /* DEBOUNCE_H */
