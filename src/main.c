/* issues

- preset save blip with internal timer vs ext trig

*/

#include <stdio.h>

// asf
#include "delay.h"
#include "compiler.h"
#include "flashc.h"
#include "preprocessor.h"
#include "print_funcs.h"
#include "intc.h"
#include "pm.h"
#include "gpio.h"
#include "spi.h"
#include "sysclk.h"

// skeleton
#include "types.h"
#include "events.h"
#include "i2c.h"
#include "init_trilogy.h"
#include "init_common.h"
#include "monome.h"
#include "timers.h"
#include "adc.h"
#include "util.h"
#include "ftdi.h"

// this
#include "conf_board.h"
#include "ii.h"
	

#define FIRSTRUN_KEY 0x22


const u16 SCALES[24][16] = {

{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},													// ZERO
{0, 68, 136, 170, 238, 306, 375, 409, 477, 545, 579, 647, 715, 784, 818,	886},					// ionian [2, 2, 1, 2, 2, 2, 1]
{0, 68, 102, 170, 238, 306, 340, 409, 477, 511, 579, 647, 715, 750, 818,	886},					// dorian [2, 1, 2, 2, 2, 1, 2]
{0, 34, 102, 170, 238, 272, 340, 409, 443, 511, 579, 647, 681, 750, 818,	852},					// phrygian [1, 2, 2, 2, 1, 2, 2]
{0, 68, 136, 204, 238, 306, 375, 409, 477, 545, 613, 647, 715, 784, 818,	886},					// lydian [2, 2, 2, 1, 2, 2, 1]
{0, 68, 136, 170, 238, 306, 340, 409, 477, 545, 579, 647, 715, 750, 818,	886},					// mixolydian [2, 2, 1, 2, 2, 1, 2]
{0, 68, 136, 170, 238, 306, 340, 409, 477, 545, 579, 647, 715, 750, 818,	886},					// aeolian [2, 1, 2, 2, 1, 2, 2]
{0, 34, 102, 170, 204, 272, 340, 409, 443, 511, 579, 613, 681, 750, 818,	852},					// locrian [1, 2, 2, 1, 2, 2, 2]

{0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 341, 375, 409, 443, 477, 511},						// chromatic
{0, 68, 136, 204, 272, 341, 409, 477, 545, 613, 682, 750, 818, 886, 954,	1022},					// whole
{0, 170, 340, 511, 681, 852, 1022, 1193, 1363, 1534, 1704, 1875, 2045, 2215, 2386, 2556},			// fourths
{0, 238, 477, 715, 954, 1193, 1431, 1670, 1909, 2147, 2386, 2625, 2863, 3102, 3340, 3579},		// fifths
{0, 17, 34, 51, 68, 85, 102, 119, 136, 153, 170, 187, 204, 221, 238,	255},						// quarter
{0, 8, 17, 25, 34, 42, 51, 59, 68, 76, 85, 93, 102, 110, 119, 127},								// eighth
{0, 61, 122, 163, 245, 327, 429, 491, 552, 613, 654, 736, 818, 920, 982, 1105}, 					// just
{0, 61, 130, 163, 245, 337, 441, 491, 552, 621, 654, 736, 828, 932, 982, 1105},					// pythagorean

{0, 272, 545, 818, 1090, 1363, 1636, 1909, 2181, 2454, 2727, 3000, 3272, 3545, 3818, 4091},		// equal 10v
{0, 136, 272, 409, 545, 682, 818, 955, 1091, 1228, 1364, 1501, 1637, 1774, 1910, 2047},			// equal 5v
{0, 68, 136, 204, 272, 341, 409, 477, 545, 613, 682, 750, 818, 886, 954, 1023},					// equal 2.5v
{0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 340, 374, 408, 442, 476, 511},						// equal 1.25v
{0, 53, 118, 196, 291, 405, 542, 708, 908, 1149, 1441, 1792, 2216, 2728, 3345, 4091},				// log-ish 10v
{0, 136, 272, 409, 545, 682, 818, 955, 1091, 1228, 1364, 1501, 1637, 1774, 1910, 2047},			// log-ish 5v
{0, 745, 1362, 1874, 2298, 2649, 2941, 3182, 3382, 3548, 3685, 3799, 3894, 3972, 4037, 4091},		// exp-ish 10v
{0, 372, 681, 937, 1150, 1325, 1471, 1592, 1692, 1775, 1844, 1901, 1948, 1987, 2020, 2047}		// exp-ish 5v

};

typedef enum {
	mTrig, mMap, mSeries
} edit_modes;

typedef enum {
	mForward=0, mReverse, mDrunk, mRandom, mPing, mPingRep
} step_modes;

typedef enum {
    mPingFwd = 1,
    mPingRev = -1
} ping_direction;

typedef struct {
	u8 loop_start, loop_end, loop_len, loop_dir;
	u16 step_choice;
	u8 cv_mode[2];
	u8 tr_mode;
	step_modes step_mode;
    ping_direction ping_dir;
	u8 steps[16];
	u8 step_probs[16];
	u16 cv_values[16];
	u16 cv_steps[2][16];
	u16 cv_curves[2][16];
	u8 cv_probs[2][16];
} whale_pattern;

typedef struct {
	whale_pattern wp[16];
	u16 series_list[64];
	u8 series_start, series_end;
	u8 tr_mute[4];
	u8 cv_mute[2];
} whale_set;

typedef const struct {
	u8 fresh;
	edit_modes edit_mode;
	u8 preset_select;
	u8 glyph[8][8];
	whale_set w[8];
} nvram_data_t;

whale_set w;

u8 preset_mode, preset_select, front_timer;
u8 glyph[8];

edit_modes edit_mode;
u8 edit_cv_step, edit_cv_ch;
s8 edit_cv_value;
u8 edit_prob, live_in, scale_select;
u8 pattern, next_pattern, pattern_jump, ping_pattern_jump;

u8 series_pos, series_next, series_jump, series_playing, scroll_pos;

u8 key_alt, key_meta, center;
u8 held_keys[32], key_count, key_times[256];
u8 keyfirst_pos, keysecond_pos;
s8 keycount_pos, keycount_series, keycount_cv;

s8 pos, cut_pos, next_pos, drunk_step, triggered;
u8 cv_chosen[2];
u16 cv0, cv1;

u8 param_accept, *param_dest8;
u16 clip;
u16 *param_dest;
u8 quantize_in;

u8 clock_phase;
u16 clock_time, clock_temp;
u8 series_step;

u16 adc[4];
u8 SIZE, LENGTH, VARI;

typedef void(*re_t)(void);
re_t re;


// NVRAM data structure located in the flash array.
__attribute__((__section__(".flash_nvram")))
static nvram_data_t flashy;




////////////////////////////////////////////////////////////////////////////////
// prototypes

static void refresh(void);
static void refresh_mono(void);
static void refresh_preset(void);
static void clock(u8 phase);

// start/stop monome polling/refresh timers
extern void timers_set_monome(void);
extern void timers_unset_monome(void);

// check the event queue
static void check_events(void);

// handler protos
static void handler_None(s32 data) { ;; }
static void handler_KeyTimer(s32 data);
static void handler_Front(s32 data);
static void handler_ClockNormal(s32 data);
static void handler_ClockExt(s32 data);

static void ww_process_ii(uint8_t *data, uint8_t l);

u8 flash_is_fresh(void);
void flash_unfresh(void);
void flash_write(void);
void flash_read(void);




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application clock code

void clock(u8 phase) {
	static u8 i1, count;
	static u16 found[16];

	if(phase) {
		gpio_set_gpio_pin(B10);


		if(pattern_jump) {
			pattern = next_pattern;
			next_pos = w.wp[pattern].loop_start;
			pattern_jump = 0;
		}
		// for series mode and delayed pattern change
		if(series_jump) {
			series_pos = series_next;
			if(series_pos == w.series_end)
				series_next = w.series_start;
			else {
				series_next++;
				if(series_next>63)
					series_next = w.series_start;
			}

			// print_dbg("\r\nSERIES next ");
			// print_dbg_ulong(series_next);
			// print_dbg(" pos ");
			// print_dbg_ulong(series_pos);

			count = 0;
			for(i1=0;i1<16;i1++) {
				if((w.series_list[series_pos] >> i1) & 1) {
					found[count] = i1;
					count++;
				}
			}

			if(count == 1)
				next_pattern = found[0];
			else {

				next_pattern = found[rnd()%count];
			}

			pattern = next_pattern;
			series_playing = pattern;
			if(w.wp[pattern].step_mode == mReverse)
				next_pos = w.wp[pattern].loop_end;
			else {
				next_pos = w.wp[pattern].loop_start;
                w.wp[pattern].ping_dir = mPingFwd;
            }

			series_jump = 0;
			series_step = 0;
		}

		pos = next_pos;

		// live param record
		if(param_accept && live_in) {
			param_dest = &w.wp[pattern].cv_curves[edit_cv_ch][pos];
			w.wp[pattern].cv_curves[edit_cv_ch][pos] = adc[1];
		}

		// calc next step
		if(w.wp[pattern].step_mode == mForward) { 		// FORWARD
			if(pos == w.wp[pattern].loop_end) 
                next_pos = w.wp[pattern].loop_start;
			else if(pos >= LENGTH) next_pos = 0;
			else next_pos++;
			cut_pos = 0;
		}
		else if(w.wp[pattern].step_mode == mReverse) {	// REVERSE
			if(pos == w.wp[pattern].loop_start)
				next_pos = w.wp[pattern].loop_end;
			else if(pos <= 0)
				next_pos = LENGTH;
			else next_pos--;
			cut_pos = 0;
		}
		else if(w.wp[pattern].step_mode == mDrunk) {	// DRUNK
			drunk_step += (rnd() % 3) - 1; // -1 to 1
			if(drunk_step < -1) drunk_step = -1;
			else if(drunk_step > 1) drunk_step = 1;

			next_pos += drunk_step;
			if(next_pos < 0) 
				next_pos = LENGTH;
			else if(next_pos > LENGTH) 
				next_pos = 0;
			else if(w.wp[pattern].loop_dir == 1 && next_pos < w.wp[pattern].loop_start)
				next_pos = w.wp[pattern].loop_end;
			else if(w.wp[pattern].loop_dir == 1 && next_pos > w.wp[pattern].loop_end)
				next_pos = w.wp[pattern].loop_start;
			else if(w.wp[pattern].loop_dir == 2 && next_pos < w.wp[pattern].loop_start && next_pos > w.wp[pattern].loop_end) {
				if(drunk_step == 1)
					next_pos = w.wp[pattern].loop_start;
				else
					next_pos = w.wp[pattern].loop_end;
			}

			cut_pos = 1;
 		}
		else if(w.wp[pattern].step_mode == mRandom) {	// RANDOM
			next_pos = (rnd() % (w.wp[pattern].loop_len + 1)) + w.wp[pattern].loop_start;
			// print_dbg("\r\nnext pos:");
			// print_dbg_ulong(next_pos);
			if(next_pos > LENGTH) next_pos -= LENGTH + 1;
			cut_pos = 1;
		}
        else if(w.wp[pattern].step_mode == mPing) {     // PING, 12343212
            if(pos == w.wp[pattern].loop_end && mPingFwd == w.wp[pattern].ping_dir) {
                w.wp[pattern].ping_dir = mPingRev;
                next_pos += w.wp[pattern].ping_dir;
            }
            else if(pos == w.wp[pattern].loop_start && mPingRev == w.wp[pattern].ping_dir) {
                w.wp[pattern].ping_dir = mPingFwd;
                // set this here because this step changes the state needed to identify the switch
                ping_pattern_jump = 1;
                next_pos += w.wp[pattern].ping_dir;
            }
            else if(pos >= LENGTH) next_pos = 0;
            else next_pos += w.wp[pattern].ping_dir;
            cut_pos = 0;
        }
        else if(w.wp[pattern].step_mode == mPingRep) {     // PINGREP, 1234432112
            if(pos == w.wp[pattern].loop_end && mPingFwd == w.wp[pattern].ping_dir) {
                w.wp[pattern].ping_dir = mPingRev;
            }
            else if(pos == w.wp[pattern].loop_end && mPingRev == w.wp[pattern].ping_dir) {
                next_pos += w.wp[pattern].ping_dir;
            }
            else if(pos == w.wp[pattern].loop_start && mPingRev == w.wp[pattern].ping_dir) {
                w.wp[pattern].ping_dir = mPingFwd;
                // set this here because this step changes the state needed to identify the switch
                ping_pattern_jump = 1;
            }
            else if(pos == w.wp[pattern].loop_start && mPingFwd == w.wp[pattern].ping_dir) {
                next_pos += w.wp[pattern].ping_dir;
            }
            else if(pos >= LENGTH) next_pos = 0;
            else next_pos += w.wp[pattern].ping_dir;
            cut_pos = 0;
        }

        // guard against -ve next_pos from skip back
        if (next_pos < 0) {
            next_pos = 0;
        }

		// next pattern?
		if(pos == w.wp[pattern].loop_end && w.wp[pattern].step_mode == mForward) {
			if(edit_mode == mSeries) 
				series_jump++;
			else if(next_pattern != pattern)
				pattern_jump++;
		}
		else if(pos == w.wp[pattern].loop_start && w.wp[pattern].step_mode == mReverse) {
			if(edit_mode == mSeries) 
				series_jump++;
			else if(next_pattern != pattern)
				pattern_jump++;
		}
		else if(ping_pattern_jump == 1 && (w.wp[pattern].step_mode == mPing || w.wp[pattern].step_mode == mPingRep)) {
             if(edit_mode == mSeries) 
                series_jump++;
             else if(next_pattern != pattern)
                pattern_jump++;
        }
		else if(series_step == w.wp[pattern].loop_len && w.wp[pattern].step_mode != mPing && w.wp[pattern].step_mode != mPingRep) {
			series_jump++;
		}
        // reset this
        ping_pattern_jump = 0;

		if(edit_mode == mSeries)
			series_step++;


		// TRIGGER
		triggered = 0;
		if((rnd() % 255) < w.wp[pattern].step_probs[pos]) {
			
			if(w.wp[pattern].step_choice & 1<<pos) {
				count = 0;
				for(i1=0;i1<4;i1++)
					if(w.wp[pattern].steps[pos] >> i1 & 1) {
						found[count] = i1;
						count++;
					}

				if(count == 0)
					triggered = 0;
				else if(count == 1)
					triggered = 1<<found[0];
				else
					triggered = 1<<found[rnd()%count];
			}	
			else {
				triggered = w.wp[pattern].steps[pos];
			}
			
			if(w.wp[pattern].tr_mode == 0) {
				if(triggered & 0x1 && w.tr_mute[0]) gpio_set_gpio_pin(B00);
				if(triggered & 0x2 && w.tr_mute[1]) gpio_set_gpio_pin(B01);
				if(triggered & 0x4 && w.tr_mute[2]) gpio_set_gpio_pin(B02);
				if(triggered & 0x8 && w.tr_mute[3]) gpio_set_gpio_pin(B03);
			} else {
				if(w.tr_mute[0]) {
					if(triggered & 0x1) gpio_set_gpio_pin(B00);
					else gpio_clr_gpio_pin(B00);
				}
				if(w.tr_mute[1]) {
					if(triggered & 0x2) gpio_set_gpio_pin(B01);
					else gpio_clr_gpio_pin(B01);
				}
				if(w.tr_mute[2]) {
					if(triggered & 0x4) gpio_set_gpio_pin(B02);
					else gpio_clr_gpio_pin(B02);
				}
				if(w.tr_mute[3]) {
					if(triggered & 0x8) gpio_set_gpio_pin(B03);
					else gpio_clr_gpio_pin(B03);
				}

			}
		}

		monomeFrameDirty++;


		// PARAM 0
		if((rnd() % 255) < w.wp[pattern].cv_probs[0][pos] && w.cv_mute[0]) {
			if(w.wp[pattern].cv_mode[0] == 0) {
				cv0 = w.wp[pattern].cv_curves[0][pos];
			}
			else {
				count = 0;
				for(i1=0;i1<16;i1++)
					if(w.wp[pattern].cv_steps[0][pos] & (1<<i1)) {
						found[count] = i1;
						count++;
					}
				if(count == 1) 
					cv_chosen[0] = found[0];
				else
					cv_chosen[0] = found[rnd() % count];
				cv0 = w.wp[pattern].cv_values[cv_chosen[0]];			
			}
		}

		// PARAM 1
		if((rnd() % 255) < w.wp[pattern].cv_probs[1][pos] && w.cv_mute[1]) {
			if(w.wp[pattern].cv_mode[1] == 0) {
				cv1 = w.wp[pattern].cv_curves[1][pos];
			}
			else {
				count = 0;
				for(i1=0;i1<16;i1++)
					if(w.wp[pattern].cv_steps[1][pos] & (1<<i1)) {
						found[count] = i1;
						count++;
					}
				if(count == 1) 
					cv_chosen[1] = found[0];
				else
					cv_chosen[1] = found[rnd() % count];

				cv1 = w.wp[pattern].cv_values[cv_chosen[1]];			
			}
		}


		// write to DAC
    spi_selectChip(DAC_SPI,DAC_SPI_NPCS);
		 // spi_write(SPI,0x39);	// update both
    spi_write(DAC_SPI,0x31);	// update A
		// spi_write(SPI,0x38);	// update B
		// spi_write(SPI,pos*15);	// send position
 		// spi_write(SPI,0);
    spi_write(DAC_SPI,cv0>>4);
    spi_write(DAC_SPI,cv0<<4);
    spi_unselectChip(DAC_SPI,DAC_SPI_NPCS);

    spi_selectChip(DAC_SPI,DAC_SPI_NPCS);
    spi_write(DAC_SPI,0x38);	// update B
    spi_write(DAC_SPI,cv1>>4);
    spi_write(DAC_SPI,cv1<<4);
    spi_unselectChip(SPI,DAC_SPI_NPCS);
	}
	else {
		gpio_clr_gpio_pin(B10);

		if(w.wp[pattern].tr_mode == 0) {
			gpio_clr_gpio_pin(B00);
			gpio_clr_gpio_pin(B01);
			gpio_clr_gpio_pin(B02);
			gpio_clr_gpio_pin(B03);
		}
 	}

	// print_dbg("\r\n pos: ");
	// print_dbg_ulong(pos);
}



////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t adcTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomePollTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomeRefreshTimer  = { .next = NULL, .prev = NULL };



static void clockTimer_callback(void* o) {  
	// static event_t e;
	// e.type = kEventTimer;
	// e.data = 0;
	// event_post(&e);
	if(clock_external == 0) {
		// print_dbg("\r\ntimer.");

		clock_phase++;
		if(clock_phase>1) clock_phase=0;
		(*clock_pulse)(clock_phase);
	}
}

static void keyTimer_callback(void* o) {  
	static event_t e;
	e.type = kEventKeyTimer;
	e.data = 0;
	event_post(&e);
}

static void adcTimer_callback(void* o) {  
	static event_t e;
	e.type = kEventPollADC;
	e.data = 0;
	event_post(&e);
}


// monome polling callback
static void monome_poll_timer_callback(void* obj) {
  // asynchronous, non-blocking read
  // UHC callback spawns appropriate events
	ftdi_read();
}

// monome refresh callback
static void monome_refresh_timer_callback(void* obj) {
	if(monomeFrameDirty > 0) {
		static event_t e;
		e.type = kEventMonomeRefresh;
		event_post(&e);
	}
}

// monome: start polling
void timers_set_monome(void) {
	// print_dbg("\r\n setting monome timers");
	timer_add(&monomePollTimer, 20, &monome_poll_timer_callback, NULL );
	timer_add(&monomeRefreshTimer, 30, &monome_refresh_timer_callback, NULL );
}

// monome stop polling
void timers_unset_monome(void) {
	// print_dbg("\r\n unsetting monome timers");
	timer_remove( &monomePollTimer );
	timer_remove( &monomeRefreshTimer ); 
}



////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_FtdiConnect(s32 data) { ftdi_setup(); }
static void handler_FtdiDisconnect(s32 data) { 
	timers_unset_monome();
	// event_t e = { .type = kEventMonomeDisconnect };
	// event_post(&e);
}

static void handler_MonomeConnect(s32 data) {
	u8 i1;
	// print_dbg("\r\n// monome connect /////////////////"); 
	keycount_pos = 0;
	key_count = 0;
	SIZE = monome_size_x();
	LENGTH = SIZE - 1;
	// print_dbg("\r monome size: ");
	// print_dbg_ulong(SIZE);
	VARI = monome_is_vari();
	// print_dbg("\r monome vari: ");
	// print_dbg_ulong(VARI);

	if(VARI) re = &refresh;
	else re = &refresh_mono;

	for(i1=0;i1<16;i1++)
		if(w.wp[i1].loop_end > LENGTH)
			w.wp[i1].loop_end = LENGTH;
	

	// monome_set_quadrant_flag(0);
	// monome_set_quadrant_flag(1);
	timers_set_monome();
}

static void handler_MonomePoll(s32 data) { monome_read_serial(); }
static void handler_MonomeRefresh(s32 data) {
	if(monomeFrameDirty) {
		if(preset_mode == 0) (*re)(); //refresh_mono();
		else refresh_preset();

		(*monome_refresh)();
	}
}


static void handler_Front(s32 data) {
	print_dbg("\r\n FRONT HOLD");

	if(data == 0) {
		front_timer = 15;
		if(preset_mode) preset_mode = 0;
		else preset_mode = 1;
	}
	else {
		front_timer = 0;
	}

	monomeFrameDirty++;
}

static void handler_PollADC(s32 data) {
	u16 i;
	adc_convert(&adc);

	// CLOCK POT INPUT
	i = adc[0];
	i = i>>2;
	if(i != clock_temp) {
		// 1000ms - 24ms
		clock_time = 25000 / (i + 25);
		// print_dbg("\r\nnew clock (ms): ");
		// print_dbg_ulong(clock_time);

		timer_set(&clockTimer, clock_time);
	}
	clock_temp = i;

	// PARAM POT INPUT
	if(param_accept && edit_prob) {
		*param_dest8 = adc[1] >> 4; // scale to 0-255;
		// print_dbg("\r\nnew prob: ");
		// print_dbg_ulong(*param_dest8);
		// print_dbg("\t" );
		// print_dbg_ulong(adc[1]);
	}
	else if(param_accept) {
		if(quantize_in)
			*param_dest = (adc[1] / 34) * 34;
		else
			*param_dest = adc[1];
		monomeFrameDirty++;
	}
	else if(key_meta) {
		i = adc[1]>>6;
		if(i > 58)
			i = 58;
		if(i != scroll_pos) {
			scroll_pos = i;
			monomeFrameDirty++;
			// print_dbg("\r scroll pos: ");
			// print_dbg_ulong(scroll_pos);
		}
	}
}

static void handler_SaveFlash(s32 data) {
	flash_write();
}

static void handler_KeyTimer(s32 data) {
	static u16 i1,x,n1;

	if(front_timer) {
		if(front_timer == 1) {
			static event_t e;
			e.type = kEventSaveFlash;
			event_post(&e);

			preset_mode = 0;
			front_timer--;
		}
		else front_timer--;
	}

	for(i1=0;i1<key_count;i1++) {
		if(key_times[held_keys[i1]])
		if(--key_times[held_keys[i1]]==0) {
			if(edit_mode != mSeries && preset_mode == 0) {
				// preset copy
				if(held_keys[i1] / 16 == 2) {
					x = held_keys[i1] % 16;
					for(n1=0;n1<16;n1++) {
						w.wp[x].steps[n1] = w.wp[pattern].steps[n1];
						w.wp[x].step_probs[n1] = w.wp[pattern].step_probs[n1];
						w.wp[x].cv_values[n1] = w.wp[pattern].cv_values[n1];
						w.wp[x].cv_steps[0][n1] = w.wp[pattern].cv_steps[0][n1];
						w.wp[x].cv_curves[0][n1] = w.wp[pattern].cv_curves[0][n1];
						w.wp[x].cv_probs[0][n1] = w.wp[pattern].cv_probs[0][n1];
						w.wp[x].cv_steps[1][n1] = w.wp[pattern].cv_steps[1][n1];
						w.wp[x].cv_curves[1][n1] = w.wp[pattern].cv_curves[1][n1];
						w.wp[x].cv_probs[1][n1] = w.wp[pattern].cv_probs[1][n1];
					}

					w.wp[x].cv_mode[0] = w.wp[pattern].cv_mode[0];
					w.wp[x].cv_mode[1] = w.wp[pattern].cv_mode[1];

					w.wp[x].loop_start = w.wp[pattern].loop_start;
					w.wp[x].loop_end = w.wp[pattern].loop_end;
					w.wp[x].loop_len = w.wp[pattern].loop_len;
					w.wp[x].loop_dir = w.wp[pattern].loop_dir;

					w.wp[x].tr_mode = w.wp[pattern].tr_mode;
					w.wp[x].step_mode = w.wp[pattern].step_mode;
					w.wp[x].ping_dir = w.wp[pattern].ping_dir;

					pattern = x;
					next_pattern = x;
					monomeFrameDirty++;

					// print_dbg("\r\n saved pattern: ");
					// print_dbg_ulong(x);
				}
			}
			else if(preset_mode == 1) {
				if(held_keys[i1] % 16 == 0) {
					preset_select = held_keys[i1] / 16;
					// flash_write();
					static event_t e;
					e.type = kEventSaveFlash;
					event_post(&e);
					preset_mode = 0;
				}
			}

			// print_dbg("\rlong press: "); 
			// print_dbg_ulong(held_keys[i1]);
		}
	}
}

static void handler_ClockNormal(s32 data) {
	clock_external = !gpio_get_pin_value(B09); 
}

static void handler_ClockExt(s32 data) {
	clock(data); 
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application grid code

static void handler_MonomeGridKey(s32 data) { 
	u8 x, y, z, index, i1, found, count;
	s16 delta;
	monome_grid_key_parse_event_data(data, &x, &y, &z);
	// print_dbg("\r\n monome event; x: "); 
	// print_dbg_hex(x); 
	// print_dbg("; y: 0x"); 
	// print_dbg_hex(y); 
	// print_dbg("; z: 0x"); 
	// print_dbg_hex(z);

	//// TRACK LONG PRESSES
	index = y*16 + x;
	if(z) {
		held_keys[key_count] = index;
		key_count++;
		key_times[index] = 10;		//// THRESHOLD key hold time
	} else {
		found = 0; // "found"
		for(i1 = 0; i1<key_count; i1++) {
			if(held_keys[i1] == index) 
				found++;
			if(found) 
				held_keys[i1] = held_keys[i1+1];
		}
		key_count--;

		// FAST PRESS
		if(key_times[index] > 0) {
			if(edit_mode != mSeries && preset_mode == 0) {
				if(index/16 == 2) {
					i1 = index % 16;
					if(key_alt)
						next_pattern = i1;
					else {
						pattern = i1;
						next_pattern = i1;
					}
				}
			}
			// PRESET MODE FAST PRESS DETECT
			else if(preset_mode == 1) {
				if(x == 0 && y != preset_select) {
					preset_select = y;
					for(i1=0;i1<8;i1++)
						glyph[i1] = flashy.glyph[preset_select][i1];
				}
 				else if(x==0 && y == preset_select) {
					flash_read();

					preset_mode = 0;
				}
			}
			// print_dbg("\r\nfast press: ");
			// print_dbg_ulong(index);
			// print_dbg(": ");
			// print_dbg_ulong(key_times[index]);
		}
	}

	// PRESET SCREEN
	if(preset_mode) {
		// glyph magic
		if(z && x>7) {
			glyph[y] ^= 1<<(x-8);
			monomeFrameDirty++;	
		}
	}
	// NOT PRESET
	else {

		// OPTIMIZE: order this if-branch by common priority/use
		//// SORT

		// cut position
		if(y == 1) {
			keycount_pos += z * 2 - 1;
			if(keycount_pos < 0) keycount_pos = 0;
			// print_dbg("\r\nkeycount: "); 
			// print_dbg_ulong(keycount_pos);

			if(keycount_pos == 1 && z) {
				if(key_alt == 0) {
					if(key_meta != 1) {
						next_pos = x;
						cut_pos++;
						monomeFrameDirty++;
					}
					keyfirst_pos = x;
				}
				else if(key_alt == 1) {
                    if ((LENGTH > 8  && (LENGTH - x) <= mPingRep) || ((LENGTH - x) <= mPing)) {
                        // Step modes, mPingRep not available on 8x8 grid
                        w.wp[pattern].step_mode = LENGTH-x;
                        w.wp[pattern].ping_dir = mPingFwd;
                    }
                    // FIXME
                    else if(x == 0) {
                        if(pos == w.wp[pattern].loop_start)
                            next_pos = w.wp[pattern].loop_end;
                        else if(pos == 0)
                            next_pos = LENGTH;
                        else next_pos--;
                        cut_pos = 1;
                        monomeFrameDirty++;
                    }
                    // FIXME
                    else if(x == 1) {
                        if(pos == w.wp[pattern].loop_end) next_pos = w.wp[pattern].loop_start;
                        else if(pos == LENGTH) next_pos = 0;
                        else next_pos++;
                        cut_pos = 1;
                        monomeFrameDirty++;
                    }
                    else if(x == 2 ) {
                        next_pos = (rnd() % (w.wp[pattern].loop_len + 1)) + w.wp[pattern].loop_start;
                        cut_pos = 1;
                        monomeFrameDirty++;					
                    }
				}
			}
			else if(keycount_pos == 2 && z) {
				w.wp[pattern].loop_start = keyfirst_pos;
				w.wp[pattern].loop_end = x;
	 			monomeFrameDirty++;
	 			if(w.wp[pattern].loop_start > w.wp[pattern].loop_end) w.wp[pattern].loop_dir = 2;
	 			else if(w.wp[pattern].loop_start == 0 && w.wp[pattern].loop_end == LENGTH) w.wp[pattern].loop_dir = 0;
	 			else w.wp[pattern].loop_dir = 1;

	 			w.wp[pattern].loop_len = w.wp[pattern].loop_end - w.wp[pattern].loop_start;

	 			if(w.wp[pattern].loop_dir == 2)
	 				w.wp[pattern].loop_len = (LENGTH - w.wp[pattern].loop_start) + w.wp[pattern].loop_end + 1;

				// print_dbg("\r\nloop_len: "); 
				// print_dbg_ulong(w.wp[pattern].loop_len);
			}
		}

		// top row
		else if(y == 0) {
			if(x == LENGTH) {
				key_alt = z;
				if(z == 0) {
					param_accept = 0;
					live_in = 0;
				}
				monomeFrameDirty++;
			}
			else if(x < 4 && z) {
				if(key_alt)
					w.wp[pattern].tr_mode ^= 1;
				else if(key_meta)
					w.tr_mute[x] ^= 1;
				else 
					edit_mode = mTrig;
				edit_prob = 0;
				param_accept = 0;
				monomeFrameDirty++;
			}
			else if(SIZE==16 && x > 3 && x < 12 && z) {
				param_accept = 0;
				edit_cv_ch = (x-4)/4;
				edit_prob = 0;

				if(key_alt)
					w.wp[pattern].cv_mode[edit_cv_ch] ^= 1;
				else if(key_meta)
					w.cv_mute[edit_cv_ch] ^= 1;
				else
					edit_mode = mMap;

				monomeFrameDirty++;
			}
			else if(SIZE==8 && (x == 4 || x == 5) && z) {
				param_accept = 0;
				edit_cv_ch = x-4;
				edit_mode = mMap;
				edit_prob = 0;

				if(key_alt)
					w.wp[pattern].cv_mode[edit_cv_ch] ^= 1;
				else if(key_meta)
					w.cv_mute[edit_cv_ch] ^= 1;

				monomeFrameDirty++;
			}
			else if(x == LENGTH-1 && z && key_alt) {
				edit_mode = mSeries;
				monomeFrameDirty++;
			}
			else if(x == LENGTH-1)
				key_meta = z;
		}


		// toggle steps and prob control
		else if(edit_mode == mTrig) {
			if(z && y>3 && edit_prob == 0) {
				if(key_alt)
					w.wp[pattern].steps[pos] |=  1 << (y-4);
				else if(key_meta) {
					w.wp[pattern].step_choice ^= (1<<x);
				}
				else
					w.wp[pattern].steps[x] ^= (1<<(y-4));
				monomeFrameDirty++;
			}
			// step probs
			else if(z && y==3) {
				if(key_alt)
					edit_prob = 1;
				else {
					if(w.wp[pattern].step_probs[x] == 255) w.wp[pattern].step_probs[x] = 0;
					else w.wp[pattern].step_probs[x] = 255;
				}	
				monomeFrameDirty++;
			}
			else if(edit_prob == 1) {
				if(z) {
					if(y == 4) w.wp[pattern].step_probs[x] = 192;
					else if(y == 5) w.wp[pattern].step_probs[x] = 128;
					else if(y == 6) w.wp[pattern].step_probs[x] = 64;
					else w.wp[pattern].step_probs[x] = 0;
				}
			}
		}	
		
		// edit map and probs
		else if(edit_mode == mMap) {
			// step probs
			if(z && y==3) {
				if(key_alt)
					edit_prob = 1;
				else  {
					if(w.wp[pattern].cv_probs[edit_cv_ch][x] == 255) w.wp[pattern].cv_probs[edit_cv_ch][x] = 0;
					else w.wp[pattern].cv_probs[edit_cv_ch][x] = 255;
				}
					
				monomeFrameDirty++;
			}
			// edit data
			else if(edit_prob == 0) {
				// CURVES
				if(w.wp[pattern].cv_mode[edit_cv_ch] == 0) {
					if(y == 4 && z) {
						if(center) 
							delta = 3;
						else if(key_alt)
							delta = 409;
						else						
							delta = 34;

						if(key_meta == 0) {
							// saturate
							if(w.wp[pattern].cv_curves[edit_cv_ch][x] + delta < 4092)
								w.wp[pattern].cv_curves[edit_cv_ch][x] += delta;
							else
								w.wp[pattern].cv_curves[edit_cv_ch][x] = 4092;
						}
						else {
							for(i1=0;i1<16;i1++) {
								// saturate
								if(w.wp[pattern].cv_curves[edit_cv_ch][i1] + delta < 4092)
									w.wp[pattern].cv_curves[edit_cv_ch][i1] += delta;
								else
									w.wp[pattern].cv_curves[edit_cv_ch][i1] = 4092;
							}
						}
					}
					else if(y == 6 && z) {
						if(center)
							delta = 3;
						else if(key_alt)
							delta = 409;
						else
							delta = 34;

						if(key_meta == 0) {
							// saturate
							if(w.wp[pattern].cv_curves[edit_cv_ch][x] > delta)
								w.wp[pattern].cv_curves[edit_cv_ch][x] -= delta;
							else
								w.wp[pattern].cv_curves[edit_cv_ch][x] = 0;
						}
						else {
							for(i1=0;i1<16;i1++) {
								// saturate
								if(w.wp[pattern].cv_curves[edit_cv_ch][i1] > delta)
									w.wp[pattern].cv_curves[edit_cv_ch][i1] -= delta;
								else
									w.wp[pattern].cv_curves[edit_cv_ch][i1] = 0;
							}
						}

					}
					else if(y == 5) {
						if(z == 1) {
	 						center = 1;
	 						if(quantize_in)
	 							quantize_in = 0;
	 						else if(key_alt)
								w.wp[pattern].cv_curves[edit_cv_ch][x] = clip;
							else
								clip = w.wp[pattern].cv_curves[edit_cv_ch][x];
						}
						else
							center = 0;
					}
					else if(y == 7) {
						if(key_alt && z) {
							param_dest = &w.wp[pattern].cv_curves[edit_cv_ch][pos];
							w.wp[pattern].cv_curves[edit_cv_ch][pos] = (adc[1] / 34) * 34;
							quantize_in = 1;
							param_accept = 1;
							live_in = 1;
						}
						else if(center && z) {
							if(key_meta == 0) 
								w.wp[pattern].cv_curves[edit_cv_ch][x] = rand() % ((adc[1] / 34) * 34 + 1);
							else {
								for(i1=0;i1<16;i1++) {
									w.wp[pattern].cv_curves[edit_cv_ch][i1] = rand() % ((adc[1] / 34) * 34 + 1);
								}
							}
						}
						else {
							param_accept = z;
							param_dest = &w.wp[pattern].cv_curves[edit_cv_ch][x];
							if(z) {
								w.wp[pattern].cv_curves[edit_cv_ch][x] = (adc[1] / 34) * 34;
								quantize_in = 1;
							}
							else
								quantize_in = 0;
						}
						monomeFrameDirty++;
					}
				}
				// MAP
				else {
					if(scale_select && z) {
						// index -= 64;
						index = (y-4) * 8 + x;
						if(index < 24 && y<8) {
							for(i1=0;i1<16;i1++)
								w.wp[pattern].cv_values[i1] = SCALES[index][i1];
							print_dbg("\rNEW SCALE ");
							print_dbg_ulong(index);
						}

						scale_select = 0;
						monomeFrameDirty++;
					}
					else {
						if(z && y==4) {
							edit_cv_step = x;
							count = 0;
							for(i1=0;i1<16;i1++)
									if((w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] >> i1) & 1) {
										count++;
										edit_cv_value = i1;
									}
							if(count>1)
								edit_cv_value = -1;

							keycount_cv = 0;

							monomeFrameDirty++;
						}
						// load scale
						else if(key_alt && y==7 && x == 0 && z) {
							scale_select++;
							monomeFrameDirty++;
						}
						// read pot					
						else if(y==7 && key_alt && edit_cv_value != -1 && x==LENGTH) {
							param_accept = z;
							param_dest = &(w.wp[pattern].cv_values[edit_cv_value]);
							// print_dbg("\r\nparam: ");
							// print_dbg_ulong(*param_dest);
						}
						else if((y == 5 || y == 6) && z && x<4 && edit_cv_step != -1) {
							delta = 0;
	 						if(x == 0)
								delta = 409;
							else if(x == 1)
								delta = 239;
							else if(x == 2)
								delta = 34;
							else if(x == 3)
								delta = 3;

							if(y == 6)
								delta *= -1;
							
							if(key_alt) {
								for(i1=0;i1<16;i1++) {
									if(w.wp[pattern].cv_values[i1] + delta > 4092)
										w.wp[pattern].cv_values[i1] = 4092;
									else if(delta < 0 && w.wp[pattern].cv_values[i1] < -1*delta)
										w.wp[pattern].cv_values[i1] = 0;
									else
										w.wp[pattern].cv_values[i1] += delta;
								}
							}
							else {
								if(w.wp[pattern].cv_values[edit_cv_value] + delta > 4092)
									w.wp[pattern].cv_values[edit_cv_value] = 4092;
								else if(delta < 0 && w.wp[pattern].cv_values[edit_cv_value] < -1*delta)
									w.wp[pattern].cv_values[edit_cv_value] = 0;
								else
									w.wp[pattern].cv_values[edit_cv_value] += delta;
							}

							monomeFrameDirty++;
						}
						// choose values
						else if(y==7) {
							keycount_cv += z*2-1;
							if(keycount_cv < 0)
								keycount_cv = 0;

							if(z) {
								count = 0;
								for(i1=0;i1<16;i1++)
									if((w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] >> i1) & 1)
										count++;

								// single press toggle
								if(keycount_cv == 1 && count < 2) {
									w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] = (1<<x);
									edit_cv_value = x;
								}
								// multiselect
								else if(keycount_cv > 1 || count > 1) {
									w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] ^= (1<<x);

									if(!w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step])
										w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] = (1<<x);

									count = 0;
									for(i1=0;i1<16;i1++)
										if((w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] >> i1) & 1) {
											count++;
											edit_cv_value = i1;
										}

									if(count > 1)
										edit_cv_value = -1;
								}

								monomeFrameDirty++;
							}
						}
					}
				}
			}
			else if(edit_prob == 1) {
				if(z) {
					if(y == 4) w.wp[pattern].cv_probs[edit_cv_ch][x] = 192;
					else if(y == 5) w.wp[pattern].cv_probs[edit_cv_ch][x] = 128;
					else if(y == 6) w.wp[pattern].cv_probs[edit_cv_ch][x] = 64;
					else w.wp[pattern].cv_probs[edit_cv_ch][x] = 0;
				}
			}
		}

		// series mode
		else if(edit_mode == mSeries) {
			if(z && key_alt) {
				if(x == 0)
					series_next = y-2+scroll_pos;
				else if(x == LENGTH-1)
					w.series_start = y-2+scroll_pos;
				else if(x == LENGTH)
					w.series_end = y-2+scroll_pos;

				if(w.series_end < w.series_start)
					w.series_end = w.series_start;
			}
			else {
				keycount_series += z*2-1;
				if(keycount_series < 0)
					keycount_series = 0;

				if(z) {
					count = 0;
					for(i1=0;i1<16;i1++)
						count += (w.series_list[y-2+scroll_pos] >> i1) & 1;

					// single press toggle
					if(keycount_series == 1 && count < 2) {
						w.series_list[y-2+scroll_pos] = (1<<x);
					}
					// multi-select
					else if(keycount_series > 1 || count > 1) {
						w.series_list[y-2+scroll_pos] ^= (1<<x);

						// ensure not fully clear
						if(!w.series_list[y-2+scroll_pos])
							w.series_list[y-2+scroll_pos] = (1<<x);
					}
				}
			}

			monomeFrameDirty++;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// application grid redraw
static void refresh() {
	u8 i1,i2;


	// clear top, cut, pattern, prob
	for(i1=0;i1<16;i1++) {
		monomeLedBuffer[i1] = 0;
		monomeLedBuffer[16+i1] = 0;
		monomeLedBuffer[32+i1] = 4;
		monomeLedBuffer[48+i1] = 0;
	}

	// dim mode
	if(edit_mode == mTrig) {
		monomeLedBuffer[0] = 4;
		monomeLedBuffer[1] = 4;
		monomeLedBuffer[2] = 4;
		monomeLedBuffer[3] = 4;
	}
	else if(edit_mode == mMap) {
		if(SIZE==16) {
			monomeLedBuffer[4+(edit_cv_ch*4)] = 4;
			monomeLedBuffer[5+(edit_cv_ch*4)] = 4;
			monomeLedBuffer[6+(edit_cv_ch*4)] = 4;
			monomeLedBuffer[7+(edit_cv_ch*4)] = 4;
		}
		else
			monomeLedBuffer[4+edit_cv_ch] = 4;
	}
	else if(edit_mode == mSeries) {
		monomeLedBuffer[LENGTH-1] = 7;
	}

	// alt
	monomeLedBuffer[LENGTH] = 4;
	if(key_alt) monomeLedBuffer[LENGTH] = 11;

	// show mutes or on steps
	if(key_meta) {
		if(w.tr_mute[0]) monomeLedBuffer[0] = 11;
		if(w.tr_mute[1]) monomeLedBuffer[1] = 11;
		if(w.tr_mute[2]) monomeLedBuffer[2] = 11;
		if(w.tr_mute[3]) monomeLedBuffer[3] = 11;
	}
	else if(triggered) {
		if(triggered & 0x1 && w.tr_mute[0]) monomeLedBuffer[0] = 11 - 4 * w.wp[pattern].tr_mode;
		if(triggered & 0x2 && w.tr_mute[1]) monomeLedBuffer[1] = 11 - 4 * w.wp[pattern].tr_mode;
		if(triggered & 0x4 && w.tr_mute[2]) monomeLedBuffer[2] = 11 - 4 * w.wp[pattern].tr_mode;
		if(triggered & 0x8 && w.tr_mute[3]) monomeLedBuffer[3] = 11 - 4 * w.wp[pattern].tr_mode;
	}

	// cv indication
	if(key_meta) {
		if(SIZE==16) {
			if(w.cv_mute[0]) {
				monomeLedBuffer[4] = 11;
				monomeLedBuffer[5] = 11;
				monomeLedBuffer[6] = 11;
				monomeLedBuffer[7] = 11;
			}
			if(w.cv_mute[1]) {
				monomeLedBuffer[8] = 11;
				monomeLedBuffer[9] = 11;
				monomeLedBuffer[10] = 11;
				monomeLedBuffer[11] = 11;
			}
		}
		else {
			if(w.cv_mute[0])
				monomeLedBuffer[4] = 11;
			if(w.cv_mute[1])
				monomeLedBuffer[5] = 11;
		}

	}
	else if(SIZE==16) {
		monomeLedBuffer[cv0 / 1024 + 4] = 11;
		monomeLedBuffer[cv1 / 1024 + 8] = 11;
	}

	// show pos loop dim
	if(w.wp[pattern].loop_dir) {	
		for(i1=0;i1<SIZE;i1++) {
			if(w.wp[pattern].loop_dir == 1 && i1 >= w.wp[pattern].loop_start && i1 <= w.wp[pattern].loop_end)
				monomeLedBuffer[16+i1] = 4;
			else if(w.wp[pattern].loop_dir == 2 && (i1 <= w.wp[pattern].loop_end || i1 >= w.wp[pattern].loop_start)) 
				monomeLedBuffer[16+i1] = 4;
		}
	}

	// show position and next cut
	if(cut_pos) monomeLedBuffer[16+next_pos] = 7;
	monomeLedBuffer[16+pos] = 15;

	// show pattern
	monomeLedBuffer[32+pattern] = 11;
	if(pattern != next_pattern) monomeLedBuffer[32+next_pattern] = 7;

	// show step data
	if(edit_mode == mTrig) {
		if(edit_prob == 0) {
			for(i1=0;i1<SIZE;i1++) {
	 			for(i2=0;i2<4;i2++) {
					if((w.wp[pattern].steps[i1] & (1<<i2)) && i1 == pos && (triggered & 1<<i2) && w.tr_mute[i2]) monomeLedBuffer[(i2+4)*16+i1] = 11;
					else if(w.wp[pattern].steps[i1] & (1<<i2) && (w.wp[pattern].step_choice & 1<<i1)) monomeLedBuffer[(i2+4)*16+i1] = 4;
					else if(w.wp[pattern].steps[i1] & (1<<i2)) monomeLedBuffer[(i2+4)*16+i1] = 7;
					else if(i1 == pos) monomeLedBuffer[(i2+4)*16+i1] = 4;
					else monomeLedBuffer[(i2+4)*16+i1] = 0;
				}

				// probs
				if(w.wp[pattern].step_probs[i1] == 255) monomeLedBuffer[48+i1] = 11;
				else if(w.wp[pattern].step_probs[i1] > 0) monomeLedBuffer[48+i1] = 4;
			}
		}
		else if(edit_prob == 1) {
			for(i1=0;i1<SIZE;i1++) {
				monomeLedBuffer[64+i1] = 4;
				monomeLedBuffer[80+i1] = 4;
				monomeLedBuffer[96+i1] = 4;
				monomeLedBuffer[112+i1] = 4;

				if(w.wp[pattern].step_probs[i1] == 255)
					monomeLedBuffer[48+i1] = 11;
				else if(w.wp[pattern].step_probs[i1] == 0) {
					monomeLedBuffer[48+i1] = 0;
					monomeLedBuffer[112+i1] = 7;
				}
				else if(w.wp[pattern].step_probs[i1]) {
					monomeLedBuffer[48+i1] = 4;
					monomeLedBuffer[64+16*(3-(w.wp[pattern].step_probs[i1]>>6))+i1] = 7;
				}
			}
		}
	}

	// show map
	else if(edit_mode == mMap) {
		if(edit_prob == 0) {
			// CURVES
			if(w.wp[pattern].cv_mode[edit_cv_ch] == 0) {
				for(i1=0;i1<SIZE;i1++) {
					// probs
					if(w.wp[pattern].cv_probs[edit_cv_ch][i1] == 255) monomeLedBuffer[48+i1] = 11;
					else if(w.wp[pattern].cv_probs[edit_cv_ch][i1] > 0) monomeLedBuffer[48+i1] = 7;

					monomeLedBuffer[112+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1] > 1023) * 7;
					monomeLedBuffer[96+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1] > 2047) * 7;
					monomeLedBuffer[80+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1] > 3071) * 7;
					monomeLedBuffer[64+i1] = 0;
					monomeLedBuffer[64+16*(3-(w.wp[pattern].cv_curves[edit_cv_ch][i1]>>10))+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1]>>7) & 0x7;
				}

				// play step highlight
				monomeLedBuffer[64+pos] += 4;
				monomeLedBuffer[80+pos] += 4;
				monomeLedBuffer[96+pos] += 4;
				monomeLedBuffer[112+pos] += 4;
			}
			// MAP
			else {
				if(!scale_select) {
					for(i1=0;i1<SIZE;i1++) {
						// probs
						if(w.wp[pattern].cv_probs[edit_cv_ch][i1] == 255) monomeLedBuffer[48+i1] = 11;
						else if(w.wp[pattern].cv_probs[edit_cv_ch][i1] > 0) monomeLedBuffer[48+i1] = 7;

						// clear edit select line
						monomeLedBuffer[64+i1] = 4;

						// show current edit value, selected
						if(edit_cv_value != -1) {
							if((w.wp[pattern].cv_values[edit_cv_value] >> 8) >= i1)
								monomeLedBuffer[80+i1] = 7;
							else
								monomeLedBuffer[80+i1] = 0;

							if(((w.wp[pattern].cv_values[edit_cv_value] >> 4) & 0xf) >= i1)
								monomeLedBuffer[96+i1] = 4;
							else
								monomeLedBuffer[96+i1] = 0;
						}
						else {
							monomeLedBuffer[80+i1] = 0;
							monomeLedBuffer[96+i1] = 0;
						}

						// show steps
						if(w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] & (1<<i1)) monomeLedBuffer[112+i1] = 7;
						else monomeLedBuffer[112+i1] = 0;
					}

					// show play position
					monomeLedBuffer[64+pos] = 7;
					// show edit position
					monomeLedBuffer[64+edit_cv_step] = 11;
					// show playing note
					monomeLedBuffer[112+cv_chosen[edit_cv_ch]] = 11;
				}
				else {
					for(i1=0;i1<SIZE;i1++) {
						// probs
						if(w.wp[pattern].cv_probs[edit_cv_ch][i1] == 255) monomeLedBuffer[48+i1] = 11;
						else if(w.wp[pattern].cv_probs[edit_cv_ch][i1] > 0) monomeLedBuffer[48+i1] = 7;

						monomeLedBuffer[64+i1] = (i1<8) * 4;						
						monomeLedBuffer[80+i1] = (i1<8) * 4;						
						monomeLedBuffer[96+i1] = (i1<8) * 4;						
						monomeLedBuffer[112+i1] = 0;
					}

					monomeLedBuffer[112] = 7;
				}

			}
		}
		else if(edit_prob == 1) {
			for(i1=0;i1<SIZE;i1++) {
				monomeLedBuffer[64+i1] = 4;
				monomeLedBuffer[80+i1] = 4;
				monomeLedBuffer[96+i1] = 4;
				monomeLedBuffer[112+i1] = 4;

				if(w.wp[pattern].cv_probs[edit_cv_ch][i1] == 255)
					monomeLedBuffer[48+i1] = 11;
				else if(w.wp[pattern].cv_probs[edit_cv_ch][i1] == 0) {
					monomeLedBuffer[48+i1] = 0;
					monomeLedBuffer[112+i1] = 7;
				}
				else if(w.wp[pattern].cv_probs[edit_cv_ch][i1]) {
					monomeLedBuffer[48+i1] = 4;
					monomeLedBuffer[64+16*(3-(w.wp[pattern].cv_probs[edit_cv_ch][i1]>>6))+i1] = 7;
				}
			}
		}

	}

	// series
	else if(edit_mode == mSeries) {
		for(i1 = 0;i1<6;i1++) {
			for(i2=0;i2<SIZE;i2++) {
				// start/end bars, clear
				if(i1+scroll_pos == w.series_start || i1+scroll_pos == w.series_end) monomeLedBuffer[32+i1*16+i2] = 4;
				else monomeLedBuffer[32+i1*16+i2] = 0;
			}

			// scroll position helper
			monomeLedBuffer[32+i1*16+((scroll_pos+i1)/(64/SIZE))] = 4;
			
			// sidebar selection indicators
			if(i1+scroll_pos > w.series_start && i1+scroll_pos < w.series_end) {
				monomeLedBuffer[32+i1*16] = 4;
				monomeLedBuffer[32+i1*16+LENGTH] = 4;
			}

			for(i2=0;i2<SIZE;i2++) {
				// show possible states
				if((w.series_list[i1+scroll_pos] >> i2) & 1)
					monomeLedBuffer[32+(i1*16)+i2] = 7;
			}

		}

		// highlight playhead
		if(series_pos >= scroll_pos && series_pos < scroll_pos+6) {
			monomeLedBuffer[32+(series_pos-scroll_pos)*16+series_playing] = 11;
		}
	}

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}


// application grid redraw without varibright
static void refresh_mono() {
	u8 i1,i2;

	// clear top, cut, pattern, prob
	for(i1=0;i1<16;i1++) {
		monomeLedBuffer[i1] = 0;
		monomeLedBuffer[16+i1] = 0;
		monomeLedBuffer[32+i1] = 0;
		monomeLedBuffer[48+i1] = 0;
	}

	// show mode
	if(edit_mode == mTrig) {
		monomeLedBuffer[0] = 11;
		monomeLedBuffer[1] = 11;
		monomeLedBuffer[2] = 11;
		monomeLedBuffer[3] = 11;
	}
	else if(edit_mode == mMap) {
		if(SIZE==16) {
			monomeLedBuffer[4+(edit_cv_ch*4)] = 11;
			monomeLedBuffer[5+(edit_cv_ch*4)] = 11;
			monomeLedBuffer[6+(edit_cv_ch*4)] = 11;
			monomeLedBuffer[7+(edit_cv_ch*4)] = 11;
		}
		else
			monomeLedBuffer[4+edit_cv_ch] = 11;
	}
	else if(edit_mode == mSeries) {
		monomeLedBuffer[LENGTH-1] = 11;
	}

	if(key_meta) {
		monomeLedBuffer[0] = 11 * w.tr_mute[0];
		monomeLedBuffer[1] = 11 * w.tr_mute[1];
		monomeLedBuffer[2] = 11 * w.tr_mute[2];
		monomeLedBuffer[3] = 11 * w.tr_mute[3];

		if(SIZE == 16) {
			monomeLedBuffer[4] = 11 * w.cv_mute[0];
			monomeLedBuffer[5] = 11 * w.cv_mute[0];
			monomeLedBuffer[6] = 11 * w.cv_mute[0];
			monomeLedBuffer[7] = 11 * w.cv_mute[0];
			monomeLedBuffer[8] = 11 * w.cv_mute[1];
			monomeLedBuffer[9] = 11 * w.cv_mute[1];
			monomeLedBuffer[10] = 11 * w.cv_mute[1];
			monomeLedBuffer[11] = 11 * w.cv_mute[1];
		} else {
			monomeLedBuffer[4] = 11 * w.cv_mute[0];
			monomeLedBuffer[5] = 11 * w.cv_mute[1];
		}


	}

	// alt
	if(key_alt) monomeLedBuffer[LENGTH] = 11;

	// show position
	monomeLedBuffer[16+pos] = 15;

	// show pattern
	monomeLedBuffer[32+pattern] = 11;

	// show step data
	if(edit_mode == mTrig) {
		if(edit_prob == 0) {
			for(i1=0;i1<SIZE;i1++) {
	 			for(i2=0;i2<4;i2++) {
					if(w.wp[pattern].steps[i1] & (1<<i2)) monomeLedBuffer[(i2+4)*16+i1] = 11;
					else monomeLedBuffer[(i2+4)*16+i1] = 0;
				}

				// probs
				if(w.wp[pattern].step_probs[i1] > 0) monomeLedBuffer[48+i1] = 11;
			}
		}
		else if(edit_prob == 1) {
			for(i1=0;i1<SIZE;i1++) {
				monomeLedBuffer[64+i1] = 0;
				monomeLedBuffer[80+i1] = 0;
				monomeLedBuffer[96+i1] = 0;
				monomeLedBuffer[112+i1] = 0;

				if(w.wp[pattern].step_probs[i1] == 255)
					monomeLedBuffer[48+i1] = 11;
				else if(w.wp[pattern].step_probs[i1] == 0) {
					monomeLedBuffer[48+i1] = 0;
					monomeLedBuffer[112+i1] = 11;
				}
				else if(w.wp[pattern].step_probs[i1]) {
					monomeLedBuffer[48+i1] = 11;
					monomeLedBuffer[64+16*(3-(w.wp[pattern].step_probs[i1]>>6))+i1] = 11;
				}
			}
		}
	}

	// show map
	else if(edit_mode == mMap) {
		if(edit_prob == 0) {
			// CURVES
			if(w.wp[pattern].cv_mode[edit_cv_ch] == 0) {
				for(i1=0;i1<SIZE;i1++) {
					// probs
					if(w.wp[pattern].cv_probs[edit_cv_ch][i1] > 0) monomeLedBuffer[48+i1] = 11;

					monomeLedBuffer[112+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1] > 511) * 11;
					monomeLedBuffer[96+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1] > 1535) * 11;
					monomeLedBuffer[80+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1] > 2559) * 11;
					monomeLedBuffer[64+i1] = (w.wp[pattern].cv_curves[edit_cv_ch][i1] > 3583) * 11;
				}
			}
			// MAP
			else {
				if(!scale_select) {
					for(i1=0;i1<SIZE;i1++) {
						// probs
						if(w.wp[pattern].cv_probs[edit_cv_ch][i1] > 0) monomeLedBuffer[48+i1] = 11;

						// clear edit row
						monomeLedBuffer[64+i1] = 0;

						// show current edit value, selected
						if(edit_cv_value != -1) {
							if((w.wp[pattern].cv_values[edit_cv_value] >> 8) >= i1)
								monomeLedBuffer[80+i1] = 11;
							else
								monomeLedBuffer[80+i1] = 0;

							if(((w.wp[pattern].cv_values[edit_cv_value] >> 4) & 0xf) >= i1)
								monomeLedBuffer[96+i1] = 11;
							else
								monomeLedBuffer[96+i1] = 0;
						}
						else {
							monomeLedBuffer[80+i1] = 0;
							monomeLedBuffer[96+i1] = 0;
						}

						// show steps
						if(w.wp[pattern].cv_steps[edit_cv_ch][edit_cv_step] & (1<<i1)) monomeLedBuffer[112+i1] = 11;
						else monomeLedBuffer[112+i1] = 0;
					}

					// show edit position
					monomeLedBuffer[64+edit_cv_step] = 11;
					// show playing note
					monomeLedBuffer[112+cv_chosen[edit_cv_ch]] = 11;
				}
				else {
					for(i1=0;i1<SIZE;i1++) {
						// probs
						if(w.wp[pattern].cv_probs[edit_cv_ch][i1] > 0) monomeLedBuffer[48+i1] = 11;

						monomeLedBuffer[64+i1] = 0;						
						monomeLedBuffer[80+i1] = 0;						
						monomeLedBuffer[96+i1] = 0;						
						monomeLedBuffer[112+i1] = 0;
					}

					monomeLedBuffer[112] = 11;
				}

			}
		}
		else if(edit_prob == 1) {
			for(i1=0;i1<SIZE;i1++) {
				monomeLedBuffer[64+i1] = 0;
				monomeLedBuffer[80+i1] = 0;
				monomeLedBuffer[96+i1] = 0;
				monomeLedBuffer[112+i1] = 0;

				if(w.wp[pattern].cv_probs[edit_cv_ch][i1] == 255)
					monomeLedBuffer[48+i1] = 11;
				else if(w.wp[pattern].cv_probs[edit_cv_ch][i1] == 0) {
					monomeLedBuffer[48+i1] = 0;
					monomeLedBuffer[112+i1] = 11;
				}
				else if(w.wp[pattern].cv_probs[edit_cv_ch][i1]) {
					monomeLedBuffer[48+i1] = 11;
					monomeLedBuffer[64+16*(3-(w.wp[pattern].cv_probs[edit_cv_ch][i1]>>6))+i1] = 11;
				}
			}
		}

	}

	// series
	else if(edit_mode == mSeries) {
		for(i1 = 0;i1<6;i1++) {
			for(i2=0;i2<SIZE;i2++) {
				// start/end bars, clear
				if((key_meta || key_alt) && (i1+scroll_pos == w.series_start || i1+scroll_pos == w.series_end)) monomeLedBuffer[32+i1*16+i2] = 11;
				else monomeLedBuffer[32+i1*16+i2] = 0;
			}

			// scroll position helper
			// monomeLedBuffer[32+i1*16+((scroll_pos+i1)/(64/SIZE))] = 4;
			
			// sidebar selection indicators
			if((key_meta || key_alt) && i1+scroll_pos > w.series_start && i1+scroll_pos < w.series_end) {
				monomeLedBuffer[32+i1*16] = 11;
				monomeLedBuffer[32+i1*16+LENGTH] = 11;
			}

			for(i2=0;i2<SIZE;i2++) {
				// show possible states
				if((w.series_list[i1+scroll_pos] >> i2) & 1)
					monomeLedBuffer[32+(i1*16)+i2] = 11;
			}

		}

		// highlight playhead
		if(series_pos >= scroll_pos && series_pos < scroll_pos+6 && (pos & 1)) {
			monomeLedBuffer[32+(series_pos-scroll_pos)*16+series_playing] = 0;
		}
	}

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}


static void refresh_preset() {
	u8 i1,i2;

	for(i1=0;i1<128;i1++)
		monomeLedBuffer[i1] = 0;

	monomeLedBuffer[preset_select * 16] = 11;

	for(i1=0;i1<8;i1++)
		for(i2=0;i2<8;i2++)
			if(glyph[i1] & (1<<i2))
				monomeLedBuffer[i1*16+i2+8] = 11;

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}


/*
	{"WW.MUTE1",WW_MUTE1},
	{"WW.MUTE2",WW_MUTE2},
	{"WW.MUTE3",WW_MUTE3},
	{"WW.MUTE4",WW_MUTE4},
	{"WW.MUTEA",WW_MUTEA},
	{"WW.MUTEB",WW_MUTEB},
	*/
static void ww_process_ii(uint8_t *data, uint8_t l) {
	uint8_t i;
	int d;

	i = data[0];
	d = (data[1] << 8) + data[2];

	switch(i) {
		case WW_PRESET:
			if(d<0 || d>7)
				break;
			preset_select = d;
			flash_read();
			break;
		case WW_POS:
			if(d<0 || d>15)
				break;
			next_pos = d;
			cut_pos++;
			monomeFrameDirty++;
			break;
		case WW_SYNC:
			if(d<0 || d>15)
				break;
			next_pos = d;
			cut_pos++;
			timer_set(&clockTimer,clock_time);
			clock_phase = 1;
			(*clock_pulse)(clock_phase);
			break;
		case WW_START:
			if(d<0 || d>15)
				break;
			w.wp[pattern].loop_start = d;
 			if(w.wp[pattern].loop_start > w.wp[pattern].loop_end) w.wp[pattern].loop_dir = 2;
 			else if(w.wp[pattern].loop_start == 0 && w.wp[pattern].loop_end == LENGTH) w.wp[pattern].loop_dir = 0;
 			else w.wp[pattern].loop_dir = 1;

 			w.wp[pattern].loop_len = w.wp[pattern].loop_end - w.wp[pattern].loop_start;

 			if(w.wp[pattern].loop_dir == 2)
 				w.wp[pattern].loop_len = (LENGTH - w.wp[pattern].loop_start) + w.wp[pattern].loop_end + 1;
 			monomeFrameDirty++;
			break;
		case WW_END:
			if(d<0 || d>15)
				break;
			w.wp[pattern].loop_end = d;
 			if(w.wp[pattern].loop_start > w.wp[pattern].loop_end) w.wp[pattern].loop_dir = 2;
 			else if(w.wp[pattern].loop_start == 0 && w.wp[pattern].loop_end == LENGTH) w.wp[pattern].loop_dir = 0;
 			else w.wp[pattern].loop_dir = 1;

 			w.wp[pattern].loop_len = w.wp[pattern].loop_end - w.wp[pattern].loop_start;

 			if(w.wp[pattern].loop_dir == 2)
 				w.wp[pattern].loop_len = (LENGTH - w.wp[pattern].loop_start) + w.wp[pattern].loop_end + 1;
 			monomeFrameDirty++;
 			break;
 		case WW_PMODE:
	 		if(d<mForward || d>mPingRep)
				break;
 			w.wp[pattern].step_mode = d;
 			break;
 		case WW_PATTERN:
 			if(d<0 || d>15)
				break;
 			pattern = d;
			next_pattern = d;
			monomeFrameDirty++;
 			break;
 		case WW_QPATTERN:
	 		if(d<0 || d>15)
				break;
 			next_pattern = d;
 			monomeFrameDirty++;
 			break;
 		case WW_MUTE1:
 			if(d) w.tr_mute[0] = 1;
 			else w.tr_mute[0] = 0;
 			break;
 		case WW_MUTE2:
 			if(d) w.tr_mute[1] = 1;
 			else w.tr_mute[1] = 0;
 			break;
 		case WW_MUTE3:
 			if(d) w.tr_mute[2] = 1;
 			else w.tr_mute[2] = 0;
 			break;
 		case WW_MUTE4:
 			if(d) w.tr_mute[3] = 1;
 			else w.tr_mute[3] = 0;
 			break;
 		case WW_MUTEA:
 			if(d) w.cv_mute[0] = 1;
 			else w.cv_mute[0] = 0;
 			break;
 		case WW_MUTEB:
 			if(d) w.cv_mute[1] = 1;
 			else w.cv_mute[1] = 0;
 			break;
		default:
			break;
	}
  // print_dbg("\r\nmp: ");
  // print_dbg_ulong(i);
  // print_dbg(" ");
  // print_dbg_ulong(d);
}




// assign event handlers
static inline void assign_main_event_handlers(void) {
	app_event_handlers[ kEventFront ]	= &handler_Front;
	// app_event_handlers[ kEventTimer ]	= &handler_Timer;
	app_event_handlers[ kEventPollADC ]	= &handler_PollADC;
	app_event_handlers[ kEventKeyTimer ] = &handler_KeyTimer;
	app_event_handlers[ kEventSaveFlash ] = &handler_SaveFlash;
	app_event_handlers[ kEventClockNormal ] = &handler_ClockNormal;
	app_event_handlers[ kEventClockExt ] = &handler_ClockExt;
	app_event_handlers[ kEventFtdiConnect ]	= &handler_FtdiConnect ;
	app_event_handlers[ kEventFtdiDisconnect ]	= &handler_FtdiDisconnect ;
	app_event_handlers[ kEventMonomeConnect ]	= &handler_MonomeConnect ;
	app_event_handlers[ kEventMonomeDisconnect ]	= &handler_None ;
	app_event_handlers[ kEventMonomePoll ]	= &handler_MonomePoll ;
	app_event_handlers[ kEventMonomeRefresh ]	= &handler_MonomeRefresh ;
	app_event_handlers[ kEventMonomeGridKey ]	= &handler_MonomeGridKey ;
}

// app event loop
void check_events(void) {
	static event_t e;
	if( event_next(&e) ) {
		(app_event_handlers)[e.type](e.data);
	}
}

// flash commands
u8 flash_is_fresh(void) {
  return (flashy.fresh != FIRSTRUN_KEY);
  // flashc_memcpy((void *)&flashy.fresh, &i, sizeof(flashy.fresh),   true);
  // flashc_memset32((void*)&(flashy.fresh), fresh_MAGIC, 4, true);
  // flashc_memset((void *)nvram_data, 0x00, 8, sizeof(*nvram_data), true);
}

// write fresh status
void flash_unfresh(void) {
  flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 4, true);
}

void flash_write(void) {
	// print_dbg("\r write preset ");
	// print_dbg_ulong(preset_select);
	flashc_memcpy((void *)&flashy.w[preset_select], &w, sizeof(w), true);
	flashc_memcpy((void *)&flashy.glyph[preset_select], &glyph, sizeof(glyph), true);
	flashc_memset8((void*)&(flashy.preset_select), preset_select, 1, true);
	flashc_memset32((void*)&(flashy.edit_mode), edit_mode, 4, true);
}

void flash_read(void) {
	u8 i1, i2;

	print_dbg("\r\n read preset ");
	print_dbg_ulong(preset_select);

	for(i1=0;i1<16;i1++) {
		for(i2=0;i2<16;i2++) {
			w.wp[i1].steps[i2] = flashy.w[preset_select].wp[i1].steps[i2];
			w.wp[i1].step_probs[i2] = flashy.w[preset_select].wp[i1].step_probs[i2];
			w.wp[i1].cv_probs[0][i2] = flashy.w[preset_select].wp[i1].cv_probs[0][i2];
			w.wp[i1].cv_probs[1][i2] = flashy.w[preset_select].wp[i1].cv_probs[1][i2];
			w.wp[i1].cv_curves[0][i2] = flashy.w[preset_select].wp[i1].cv_curves[0][i2];
			w.wp[i1].cv_curves[1][i2] = flashy.w[preset_select].wp[i1].cv_curves[1][i2];
			w.wp[i1].cv_steps[0][i2] = flashy.w[preset_select].wp[i1].cv_steps[0][i2];
			w.wp[i1].cv_steps[1][i2] = flashy.w[preset_select].wp[i1].cv_steps[1][i2];
			w.wp[i1].cv_values[i2] = flashy.w[preset_select].wp[i1].cv_values[i2];
		}

		w.wp[i1].step_choice = flashy.w[preset_select].wp[i1].step_choice;
		w.wp[i1].loop_end = flashy.w[preset_select].wp[i1].loop_end;
		w.wp[i1].loop_len = flashy.w[preset_select].wp[i1].loop_len;
		w.wp[i1].loop_start = flashy.w[preset_select].wp[i1].loop_start;
		w.wp[i1].loop_dir = flashy.w[preset_select].wp[i1].loop_dir;
		w.wp[i1].step_mode = flashy.w[preset_select].wp[i1].step_mode;
		w.wp[i1].ping_dir = flashy.w[preset_select].wp[i1].ping_dir;
		w.wp[i1].cv_mode[0] = flashy.w[preset_select].wp[i1].cv_mode[0];
		w.wp[i1].cv_mode[1] = flashy.w[preset_select].wp[i1].cv_mode[1];
		w.wp[i1].tr_mode = flashy.w[preset_select].wp[i1].tr_mode;
	}

	w.series_start = flashy.w[preset_select].series_start;
	w.series_end = flashy.w[preset_select].series_end;

	w.tr_mute[0] = flashy.w[preset_select].tr_mute[0];
	w.tr_mute[1] = flashy.w[preset_select].tr_mute[1];
	w.tr_mute[2] = flashy.w[preset_select].tr_mute[2];
	w.tr_mute[3] = flashy.w[preset_select].tr_mute[3];
	w.cv_mute[0] = flashy.w[preset_select].cv_mute[0];
	w.cv_mute[1] = flashy.w[preset_select].cv_mute[1];

	for(i1=0;i1<64;i1++)
		w.series_list[i1] = flashy.w[preset_select].series_list[i1];
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// main

int main(void)
{
	u8 i1,i2;

	sysclk_init();

	init_dbg_rs232(FMCK_HZ);

	init_gpio();
	assign_main_event_handlers();
	init_events();
	init_tc();
	init_spi();
	init_adc();

	irq_initialize_vectors();
	register_interrupts();
	cpu_irq_enable();

	init_usb_host();
	init_monome();

	init_i2c_slave(0x10);


	print_dbg("\r\n\n// white whale //////////////////////////////// ");
	print_dbg_ulong(sizeof(flashy));

	print_dbg(" ");
	print_dbg_ulong(sizeof(w));

	print_dbg(" ");
	print_dbg_ulong(sizeof(glyph));

	if(flash_is_fresh()) {
		print_dbg("\r\nfirst run.");
		flash_unfresh();
		flashc_memset8((void*)&(flashy.edit_mode), mTrig, 4, true);
		flashc_memset32((void*)&(flashy.preset_select), 0, 4, true);


		// clear out some reasonable defaults
		for(i1=0;i1<16;i1++) {
			for(i2=0;i2<16;i2++) {
				w.wp[i1].steps[i2] = 0;
				w.wp[i1].step_probs[i2] = 255;
				w.wp[i1].cv_probs[0][i2] = 255;
				w.wp[i1].cv_probs[1][i2] = 255;
				w.wp[i1].cv_curves[0][i2] = 0;
				w.wp[i1].cv_curves[1][i2] = 0;
				w.wp[i1].cv_values[i2] = SCALES[2][i2];
				w.wp[i1].cv_steps[0][i2] = 1<<i2;
				w.wp[i1].cv_steps[1][i2] = 1<<i2;
			}
			w.wp[i1].step_choice = 0;
			w.wp[i1].loop_end = 15;
			w.wp[i1].loop_len = 15;
			w.wp[i1].loop_start = 0;
			w.wp[i1].loop_dir = 0;
			w.wp[i1].step_mode = mForward;
			w.wp[i1].ping_dir = mPingFwd;
			w.wp[i1].cv_mode[0] = 0;
			w.wp[i1].cv_mode[1] = 0;
			w.wp[i1].tr_mode = 0;
		}

		w.series_start = 0;
		w.series_end = 3;

		w.tr_mute[0] = 1;
		w.tr_mute[1] = 1;
		w.tr_mute[2] = 1;
		w.tr_mute[3] = 1;
		w.cv_mute[0] = 1;
		w.cv_mute[1] = 1;

		for(i1=0;i1<64;i1++)
			w.series_list[i1] = 1;

		// save all presets, clear glyphs
		for(i1=0;i1<8;i1++) {
			flashc_memcpy((void *)&flashy.w[i1], &w, sizeof(w), true);
			glyph[i1] = (1<<i1);
			flashc_memcpy((void *)&flashy.glyph[i1], &glyph, sizeof(glyph), true);
		}
	}
	else {
		// load from flash at startup
		preset_select = flashy.preset_select;
		edit_mode = flashy.edit_mode;
		flash_read();
		for(i1=0;i1<8;i1++)
			glyph[i1] = flashy.glyph[preset_select][i1];
	}

	LENGTH = 15;
	SIZE = 16;

	re = &refresh;

	process_ii = &ww_process_ii;

	clock_pulse = &clock;
	clock_external = !gpio_get_pin_value(B09);

	timer_add(&clockTimer,120,&clockTimer_callback, NULL);
	timer_add(&keyTimer,50,&keyTimer_callback, NULL);
	timer_add(&adcTimer,100,&adcTimer_callback, NULL);
	clock_temp = 10000; // out of ADC range to force tempo

	// setup daisy chain for two dacs
	// spi_selectChip(SPI,DAC_SPI);
	// spi_write(SPI,0x80);
	// spi_write(SPI,0xff);
	// spi_write(SPI,0xff);
	// spi_unselectChip(SPI,DAC_SPI);

	while (true) {
		check_events();
	}
}
