#ifndef __PSXPORT_H__
#define __PSXPORT_H__

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <sys/types.h>
#include <assert.h>

#ifndef PATH_MAX
#define PATH_MAX 2048
#endif

/* The definition sees libc's PATH_MAX while some users of this header use the
 * conservative fallback above. The bound is irrelevant to callers and making
 * it part of the extern type causes an LTO type-mismatch warning. */
extern char cheatsdir[];

struct ps1_controller {
    uint8_t id;
    uint8_t joy_right_ax0;
    uint8_t joy_right_ax1;
    uint8_t joy_left_ax0;
    uint8_t joy_left_ax1;
    uint8_t Vib[2];
    uint8_t VibF[2];
    uint8_t pad_mode;
    uint8_t pad_controllertype;
    uint8_t configmode;
};

extern struct ps1_controller player_controller[2];

#define CONFIG_VERSION 2   /* 2: PlayStation face-button layout */

unsigned get_ticks(void);
void wait_ticks(unsigned s);
void pad_update(void);
uint16_t pad_read(int num);

void video_flip(void);
void video_clear(void);
void port_printf(int x, int y, const char *text);

extern unsigned short *SCREEN;
extern int SCREEN_WIDTH, SCREEN_HEIGHT;

int state_load(int slot);
int state_save(int slot);

int SelectGame(void);
int GameMenu(void);

#endif
