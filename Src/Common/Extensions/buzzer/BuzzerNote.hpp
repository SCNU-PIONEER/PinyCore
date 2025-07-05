#pragma once

#include <stdint.h>

namespace BUZZER {

struct Note {
    uint16_t frequency;
    uint16_t duration;
};

#define NOTE_B0   31
#define NOTE_C1   33
#define NOTE_CS1  35
#define NOTE_D1   37
#define NOTE_DS1  39
#define NOTE_E1   41
#define NOTE_F1   44
#define NOTE_FS1  46
#define NOTE_G1   49
#define NOTE_GS1  52
#define NOTE_A1   55
#define NOTE_AS1  58
#define NOTE_B1   62
#define NOTE_C2   65
#define NOTE_CS2  69
#define NOTE_D2   73
#define NOTE_DS2  78
#define NOTE_E2   82
#define NOTE_F2   87
#define NOTE_FS2  93
#define NOTE_G2   98
#define NOTE_GS2  104
#define NOTE_A2   110
#define NOTE_AS2  117
#define NOTE_B2   123
#define NOTE_C3   131
#define NOTE_CS3  139
#define NOTE_D3   147
#define NOTE_DS3  156
#define NOTE_E3   165
#define NOTE_F3   175
#define NOTE_FS3  185
#define NOTE_G3   196
#define NOTE_GS3  208
#define NOTE_A3   220
#define NOTE_AS3  233
#define NOTE_B3   247
#define NOTE_C4   262
#define NOTE_CS4  277
#define NOTE_D4   294
#define NOTE_DS4  311
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370
#define NOTE_G4   392
#define NOTE_GS4  415
#define NOTE_A4   440
#define NOTE_AS4  466
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_CS5  554
#define NOTE_D5   587
#define NOTE_DS5  622
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_FS5  740
#define NOTE_G5   784
#define NOTE_GS5  831
#define NOTE_A5   880
#define NOTE_AS5  932
#define NOTE_B5   988
#define NOTE_C6   1047
#define NOTE_CS6  1109
#define NOTE_D6   1175
#define NOTE_DS6  1245
#define NOTE_E6   1319
#define NOTE_F6   1397
#define NOTE_FS6  1480
#define NOTE_G6   1568
#define NOTE_GS6  1661
#define NOTE_A6   1760
#define NOTE_AS6  1865
#define NOTE_B6   1976
#define NOTE_C7   2093
#define NOTE_CS7  2217
#define NOTE_D7   2349
#define NOTE_DS7  2489
#define NOTE_E7   2637
#define NOTE_F7   2794
#define NOTE_FS7  2960
#define NOTE_G7   3136
#define NOTE_GS7  3322
#define NOTE_A7   3520
#define NOTE_AS7  3729
#define NOTE_B7   3951
#define NOTE_C8   4186
#define NOTE_CS8  4435
#define NOTE_D8   4699
#define NOTE_DS8  4978
#define NOTE_REST 0

const Note dji[] = {
    { NOTE_B7, 250 },  { NOTE_FS7, 250 }, { NOTE_C7, 250 },
    { NOTE_REST, 50 }, { NOTE_FS7, 50 },  { NOTE_C7, 50 },
};

const Note PinyCore[]{ { NOTE_C5, 150 },
                       { NOTE_E5, 150 },
                       { NOTE_G5, 150 },
                       { NOTE_C6, 300 },
                       { 0, 50 } };

const Note allNote[]{
    { NOTE_B0, 250 },  { NOTE_C1, 250 },  { NOTE_CS1, 250 }, { NOTE_D1, 250 },
    { NOTE_DS1, 250 }, { NOTE_E1, 250 },  { NOTE_F1, 250 },  { NOTE_FS1, 250 },
    { NOTE_G1, 250 },  { NOTE_GS1, 250 }, { NOTE_A1, 250 },  { NOTE_AS1, 250 },
    { NOTE_B1, 250 },  { NOTE_C2, 250 },  { NOTE_CS2, 250 }, { NOTE_D2, 250 },
    { NOTE_DS2, 250 }, { NOTE_E2, 250 },  { NOTE_F2, 250 },  { NOTE_FS2, 250 },
    { NOTE_G2, 250 },  { NOTE_GS2, 250 }, { NOTE_A2, 250 },  { NOTE_AS2, 250 },
    { NOTE_B2, 250 },  { NOTE_C3, 250 },  { NOTE_CS3, 250 }, { NOTE_D3, 250 },
    { NOTE_DS3, 250 }, { NOTE_E3, 250 },  { NOTE_F3, 250 },  { NOTE_FS3, 250 },
    { NOTE_G3, 250 },  { NOTE_GS3, 250 }, { NOTE_A3, 250 },  { NOTE_AS3, 250 },
    { NOTE_B3, 250 },  { NOTE_C4, 250 },  { NOTE_CS4, 250 }, { NOTE_D4, 250 },
    { NOTE_DS4, 250 }, { NOTE_E4, 250 },  { NOTE_F4, 250 },  { NOTE_FS4, 250 },
    { NOTE_G4, 250 },  { NOTE_GS4, 250 }, { NOTE_A4, 250 },  { NOTE_AS4, 250 },
    { NOTE_B4, 250 },  { NOTE_C5, 250 },  { NOTE_CS5, 250 }, { NOTE_D5, 250 },
    { NOTE_DS5, 250 }, { NOTE_E5, 250 },  { NOTE_F5, 250 },  { NOTE_FS5, 250 },
    { NOTE_G5, 250 },  { NOTE_GS5, 250 }, { NOTE_A5, 250 },  { NOTE_AS5, 250 },
    { NOTE_B5, 250 },  { NOTE_C6, 250 },  { NOTE_CS6, 250 }, { NOTE_D6, 250 },
    { NOTE_DS6, 250 }, { NOTE_E6, 250 },  { NOTE_F6, 250 },  { NOTE_FS6, 250 },
    { NOTE_G6, 250 },  { NOTE_GS6, 250 }, { NOTE_A6, 250 },  { NOTE_AS6, 250 },
    { NOTE_B6, 250 },  { NOTE_C7, 250 },  { NOTE_CS7, 250 }, { NOTE_D7, 250 },
    { NOTE_DS7, 250 }, { NOTE_E7, 250 },  { NOTE_F7, 250 },  { NOTE_FS7, 250 },
    { NOTE_G7, 250 },  { NOTE_GS7, 250 }, { NOTE_A7, 250 },  { NOTE_AS7, 250 },
    { NOTE_B7, 250 },  { NOTE_C8, 250 },  { NOTE_CS8, 250 }, { NOTE_D8, 250 },
    { NOTE_DS8, 250 }
};
}
