#ifndef IRONOS_PID_H
#define IRONOS_PID_H

// from IronOS, GPL3: https://github.com/Ralim/IronOS/blob/dev/LICENSE
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
