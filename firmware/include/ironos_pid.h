/*
 * Copyright 2023 spezifisch <spezifisch23@proton.me> https://github.com/spezifisch
 * Copyright 2020 Ralim, IronOS https://github.com/Ralim/IronOS
 *
 * Mostly copied from IronOS v2.21, GPLv3: https://github.com/Ralim/IronOS/blob/v2.21/LICENSE
 * Original source files:
 * - https://github.com/Ralim/IronOS/blob/v2.21/source/Core/Threads/PIDThread.cpp
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

#ifndef IRONOS_PID_H
#define IRONOS_PID_H

namespace IronOS
{
    template <class T = int32_t>
    struct Integrator
    {
        T sum;

        T update(const T val, const int32_t inertia, const int32_t gain, const int32_t rate, const int32_t limit)
        {
            // Decay the old value. This is a simplified formula that still works with decent results
            // Ideally we would have used an exponential decay but the computational effort required
            // by exp function is just not justified here in respect to the outcome
            sum = (sum * (100 - (inertia / rate))) / 100;
            // Add the new value x integration interval ( 1 / rate)
            sum += (gain * val) / rate;

            // limit the output
            if (sum > limit)
                sum = limit;
            else if (sum < -limit)
                sum = -limit;

            return sum;
        }

        void set(T const val) { sum = val; }

        T get(bool positiveOnly = true) const { return (positiveOnly) ? ((sum > 0) ? sum : 0) : sum; }
    };
}

#endif /* IRONOS_PID_H */
