#ifndef DEBOUNCE_H
#define DEBOUNCE_H

template <unsigned DEBOUNCE_COUNT>
class Debounce
{
public:
    bool measure(bool level)
    {
        if (level)
        {
            if (pressCount < DEBOUNCE_COUNT)
            {
                pressCount++;

                if (pressCount == DEBOUNCE_COUNT)
                {
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
