#ifndef SRC_MUSIC_H
#define SRC_MUSIC_H

#include <stdint.h>

/* Peppa Pig, supplied score page 1, opening theme (bars 1-12).
 * Monophonic adaptation: accompaniment and dense ornaments are simplified.
 * C major, quarter note = 96 BPM. One unit is a sixteenth note.
 * Frequencies are Hz; zero means rest. No sampled audio is required.
 */
typedef struct {
    uint16_t frequency_hz;
    uint8_t sixteenths;
} music_note_t;

#define MUSIC_BPM 96U
static const music_note_t pepa_pig[] = {
    /* 1-2: 5 3 1 2 5 | 5 7 2 4 3 1 */
    {784,2}, {659,1}, {523,1}, {587,2}, {784,2},
    {784,1}, {988,1}, {1175,1}, {1397,1}, {1319,2}, {1047,2},
    /* 3-4: short offbeat figures; lower voice omitted. */
    {0,1}, {784,1}, {1319,1}, {0,1}, {784,1}, {1319,1}, {0,1}, {1319,1},
    {0,1}, {784,1}, {1397,1}, {0,1}, {784,1}, {1397,1}, {0,1}, {1397,1},
    /* 5-6: opening phrase repeated. */
    {784,2}, {659,1}, {523,1}, {587,2}, {784,2},
    {784,1}, {988,1}, {1175,1}, {1397,1}, {1319,2}, {1047,2},
    /* 7: 3/4 chromatic run, ornament reduced to sixteenths. */
    {740,1}, {1175,1}, {1397,1}, {1397,1}, {1568,1}, {1319,1},
    {1047,1}, {1047,1}, {1175,1}, {1397,1}, {1397,1}, {1175,1},
    /* 8: return to 2/4. */
    {784,2}, {659,1}, {523,1}, {587,2}, {784,2},
    /* 9-10: rising phrase and chromatic answer. */
    {784,1}, {988,1}, {1175,1}, {1397,1}, {1319,2}, {1047,2},
    {1109,1}, {1175,1}, {1397,1}, {1397,1}, {1480,1}, {1568,1}, {1047,1}, {1047,1},
    /* 11-12: closing phrase, octave grace note omitted. */
    {784,2}, {659,1}, {523,1}, {587,2}, {784,2},
    {784,1}, {988,1}, {1175,1}, {1397,1}, {1047,2}, {0,2}
};
/* User-supplied melody. C5 = 1, C6 = 1' (high octave).
 * Each note is 1/4 beat = one sixteenth; '-' extends the preceding note.
 * Keep the previous Peppa Pig adaptation above as pepa_pig.
 */
static const music_note_t mode3_music[] = {
    /* 6 6 6 6 6 5 6 */
    {880,1}, {880,1}, {880,1}, {880,1}, {880,1}, {784,1}, {880,1},
    /* 1' 1' 1' 1' 1' 7 1' */
    {1047,1}, {1047,1}, {1047,1}, {1047,1}, {1047,1}, {988,1}, {1047,1},
    /* 5 5 5 5 5 4 5 */
    {784,1}, {784,1}, {784,1}, {784,1}, {784,1}, {698,1}, {784,1},
    /* 7 7 7 7 7 6 7 */
    {988,1}, {988,1}, {988,1}, {988,1}, {988,1}, {880,1}, {988,1},
    /* 6 6 6 6 6 5 6 */
    {880,1}, {880,1}, {880,1}, {880,1}, {880,1}, {784,1}, {880,1},
    /* 1' 1' 1' 1' 1' 7 1' */
    {1047,1}, {1047,1}, {1047,1}, {1047,1}, {1047,1}, {988,1}, {1047,1},
    /* 5 5 5 5 5 4 5 */
    {784,1}, {784,1}, {784,1}, {784,1}, {784,1}, {698,1}, {784,1},
    /* 7 - - - : sustain one pitch for four units, without retriggering. */
    {988,4}
};

#define MODE3_MUSIC mode3_music
#define MODE3_MUSIC_COUNT (sizeof(MODE3_MUSIC) / sizeof(MODE3_MUSIC[0]))

#endif
