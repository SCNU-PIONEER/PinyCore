#pragma once

#include <cstdint>

namespace BUZZER {

enum class Tone_e : uint16_t {
    B0 = 31,
    C1 = 33,
    CS1 = 35,
    D1 = 37,
    DS1 = 39,
    E1 = 41,
    F1 = 44,
    FS1 = 46,
    G1 = 49,
    GS1 = 52,
    A1 = 55,
    AS1 = 58,
    B1 = 62,
    C2 = 65,
    CS2 = 69,
    D2 = 73,
    DS2 = 78,
    E2 = 82,
    F2 = 87,
    FS2 = 93,
    G2 = 98,
    GS2 = 104,
    A2 = 110,
    AS2 = 117,
    B2 = 123,
    C3 = 131,
    CS3 = 139,
    D3 = 147,
    DS3 = 156,
    E3 = 165,
    F3 = 175,
    FS3 = 185,
    G3 = 196,
    GS3 = 208,
    A3 = 220,
    AS3 = 233,
    B3 = 247,
    C4 = 262,
    CS4 = 277,
    D4 = 294,
    DS4 = 311,
    E4 = 330,
    F4 = 349,
    FS4 = 370,
    G4 = 392,
    GS4 = 415,
    A4 = 440,
    AS4 = 466,
    B4 = 494,
    C5 = 523,
    CS5 = 554,
    D5 = 587,
    DS5 = 622,
    E5 = 659,
    F5 = 698,
    FS5 = 740,
    G5 = 784,
    GS5 = 831,
    A5 = 880,
    AS5 = 932,
    B5 = 988,
    C6 = 1047,
    CS6 = 1109,
    D6 = 1175,
    DS6 = 1245,
    E6 = 1319,
    F6 = 1397,
    FS6 = 1480,
    G6 = 1568,
    GS6 = 1661,
    A6 = 1760,
    AS6 = 1865,
    B6 = 1976,
    C7 = 2093,
    CS7 = 2217,
    D7 = 2349,
    DS7 = 2489,
    E7 = 2637,
    F7 = 2794,
    FS7 = 2960,
    G7 = 3136,
    GS7 = 3322,
    A7 = 3520,
    AS7 = 3729,
    B7 = 3951,
    C8 = 4186,
    CS8 = 4435,
    D8 = 4699,
    DS8 = 4978,
    REST = 0
};

struct Note_s {
    Tone_e tone;
    uint16_t duration;
    Note_s(Tone_e _tone = Tone_e::REST, uint16_t _duration = 0) : tone(_tone), duration(_duration) {}
};

// NOLINTBEGIN(readability-identifier-naming)
const Note_s Dji[] = {
    { Tone_e::B7, 250 },  { Tone_e::FS7, 250 }, { Tone_e::C7, 250 },
    { Tone_e::REST, 50 }, { Tone_e::FS7, 50 },  { Tone_e::C7, 50 },
};

const Note_s Wechat[] = { { Tone_e::E5, 80 }, { Tone_e::G5, 20 }, { Tone_e::REST, 25 },
                          { Tone_e::C5, 80 }, { Tone_e::E5, 20 }, { Tone_e::REST, 50 } };

const Note_s WindowsXP[] = { { Tone_e::A4, 80 }, { Tone_e::CS5, 80 }, { Tone_e::E5, 80 }, { Tone_e::A5, 160 } };

const Note_s Victory[] = { { Tone_e::G4, 100 }, { Tone_e::C5, 100 }, { Tone_e::E5, 100 },
                           { Tone_e::G5, 200 }, { Tone_e::E5, 100 }, { Tone_e::G5, 400 } };

const Note_s Press[] = { { Tone_e::G4, 30 },
                         { Tone_e::G5, 30 },
                         { Tone_e::REST, 30 },
                         { Tone_e::G4, 30 },
                         { Tone_e::G5, 30 } };

const Note_s PinyCore[]{ { Tone_e::C5, 150 },
                         { Tone_e::E5, 150 },
                         { Tone_e::G5, 150 },
                         { Tone_e::C6, 300 },
                         { Tone_e::REST, 50 } };

const Note_s AllNote[]{
    { Tone_e::B0, 250 },  { Tone_e::C1, 250 },  { Tone_e::CS1, 250 }, { Tone_e::D1, 250 },  { Tone_e::DS1, 250 },
    { Tone_e::E1, 250 },  { Tone_e::F1, 250 },  { Tone_e::FS1, 250 }, { Tone_e::G1, 250 },  { Tone_e::GS1, 250 },
    { Tone_e::A1, 250 },  { Tone_e::AS1, 250 }, { Tone_e::B1, 250 },  { Tone_e::C2, 250 },  { Tone_e::CS2, 250 },
    { Tone_e::D2, 250 },  { Tone_e::DS2, 250 }, { Tone_e::E2, 250 },  { Tone_e::F2, 250 },  { Tone_e::FS2, 250 },
    { Tone_e::G2, 250 },  { Tone_e::GS2, 250 }, { Tone_e::A2, 250 },  { Tone_e::AS2, 250 }, { Tone_e::B2, 250 },
    { Tone_e::C3, 250 },  { Tone_e::CS3, 250 }, { Tone_e::D3, 250 },  { Tone_e::DS3, 250 }, { Tone_e::E3, 250 },
    { Tone_e::F3, 250 },  { Tone_e::FS3, 250 }, { Tone_e::G3, 250 },  { Tone_e::GS3, 250 }, { Tone_e::A3, 250 },
    { Tone_e::AS3, 250 }, { Tone_e::B3, 250 },  { Tone_e::C4, 250 },  { Tone_e::CS4, 250 }, { Tone_e::D4, 250 },
    { Tone_e::DS4, 250 }, { Tone_e::E4, 250 },  { Tone_e::F4, 250 },  { Tone_e::FS4, 250 }, { Tone_e::G4, 250 },
    { Tone_e::GS4, 250 }, { Tone_e::A4, 250 },  { Tone_e::AS4, 250 }, { Tone_e::B4, 250 },  { Tone_e::C5, 250 },
    { Tone_e::CS5, 250 }, { Tone_e::D5, 250 },  { Tone_e::DS5, 250 }, { Tone_e::E5, 250 },  { Tone_e::F5, 250 },
    { Tone_e::FS5, 250 }, { Tone_e::G5, 250 },  { Tone_e::GS5, 250 }, { Tone_e::A5, 250 },  { Tone_e::AS5, 250 },
    { Tone_e::B5, 250 },  { Tone_e::C6, 250 },  { Tone_e::CS6, 250 }, { Tone_e::D6, 250 },  { Tone_e::DS6, 250 },
    { Tone_e::E6, 250 },  { Tone_e::F6, 250 },  { Tone_e::FS6, 250 }, { Tone_e::G6, 250 },  { Tone_e::GS6, 250 },
    { Tone_e::A6, 250 },  { Tone_e::AS6, 250 }, { Tone_e::B6, 250 },  { Tone_e::C7, 250 },  { Tone_e::CS7, 250 },
    { Tone_e::D7, 250 },  { Tone_e::DS7, 250 }, { Tone_e::E7, 250 },  { Tone_e::F7, 250 },  { Tone_e::FS7, 250 },
    { Tone_e::G7, 250 },  { Tone_e::GS7, 250 }, { Tone_e::A7, 250 },  { Tone_e::AS7, 250 }, { Tone_e::B7, 250 },
    { Tone_e::C8, 250 },  { Tone_e::CS8, 250 }, { Tone_e::D8, 250 },  { Tone_e::DS8, 250 }
};
// NOLINTEND(readability-identifier-naming)

} // namespace BUZZER