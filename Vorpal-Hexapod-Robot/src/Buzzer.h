#ifndef BUZZER_H
#define BUZZER_H

#define BeeperPin 4           // digital 4 used for beeper

#define BF_ERROR  100         // deep beep for error situations
#define BD_MED    50          // medium long beep duration

void beep(int f, int t);
void beep(int f);

#endif