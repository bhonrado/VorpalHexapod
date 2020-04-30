#ifndef BUZZER_H
#define BUZZER_H
#include <Arduino.h>   

#define BF_ERROR                100         // deep beep for error situations
#define BD_MED                  50          // medium long beep duration
#define DEFAULT_BEEP_DURATION   250

class Buzzer
{
 private:
    uint8_t pin;
 public:
    Buzzer(uint8_t pin);
    void Init();

    /**
     * @brief Generates a square wave of the specified frequency 
     *        (and 50% duty cycle) on a pin.
     * 
     * @param freq      the frequency of the tone in hertz
     * @param duration  the duration of the tone in milliseconds 
     * @note            if no second param is given we'll default to 250 milliseconds for the beep
     */
    void Beep(int freq, int duration = DEFAULT_BEEP_DURATION);

    void BeepError();
};

extern Buzzer buzzer;

#endif // BUZZER_H