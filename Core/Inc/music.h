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

/* 音符对照表：十二平均律，A4=440 Hz；频率四舍五入为整数 Hz。
 * 本文件以 1=C5 为基准：低八度为 C4，高八度 1'=C6，再高八度为 C7。
 * 简谱     音名       第4组Hz   第5组Hz   第6组Hz   第7组Hz
 * ----------------------------------------------------------------
 * 1      C            262       523      1047      2093
 * #1     C#           277       554      1109      2217
 * 2      D            294       587      1175      2349
 * #2     D#           311       622      1245      2489
 * 3      E            330       659      1319      2637
 * 4      F            349       698      1397      2794
 * #4     F#           370       740      1480      2960
 * 5      G            392       784      1568      3136
 * #5     G#           415       831      1661      3322
 * 6      A            440       880      1760      3520
 * #6     A#           466       932      1865      3729
 * 7      B            494       988      1976      3951
 *
 * 0 / _：休止，frequency_hz=0；不发声，但保留该项时值。
 * 升八度：音名相同、组数加1、频率约翻倍；整数取整可能相差1 Hz。
 * 数组格式：{frequency_hz, sixteenths}；sixteenths 的单位是1/4拍。
 * 时值对照：1=1/4拍，2=1/2拍，4=1拍，8=2拍，16=4拍。
 * 96 BPM时：每拍625ms；延音增加同一项时值，不拆成重复触发的音。
 * 当前valorant：D6(1175)、D6(1175)、休止、D6(1175)、G6(1568)。
 */

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

/* Valorant: 2 2 _ 2 5-, raised one octave (D6/G6), 96 BPM.
 * Durations use sixteenths and retain the current selected rhythm.
 * '_' is silence; final 5 is one uninterrupted sustained note.
 */
static const music_note_t valorant[] = {
    {661,1}, {661,1}, {0,1}, {661,1}, {832,2}
};

/* Select one startup song: pepa_pig, mode3_music, or valorant. */
#define MODE3_MUSIC valorant
#define MODE3_MUSIC_COUNT (sizeof(MODE3_MUSIC) / sizeof(MODE3_MUSIC[0]))

#endif
