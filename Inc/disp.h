// 7-segment demo common defs
// Timing
#define NDIGITS	4
#define MPX_FREQ	(NDIGITS * 400)
#define MPX_STEPS	100
#define MPX_BRIGHT	(MPX_STEPS - 2)
#define MPX_DIM		(MPX_STEPS / 10)

#if SYSCLK_FREQ % MPX_FREQ
#warning	Imprecise time base!
#endif
extern uint32_t display[NDIGITS];
extern uint32_t display_fade[NDIGITS];
extern uint32_t cpxctrl[NDIGITS];

void common_setup(void);
void run_every_10ms(void);
void disp_init(void);

void blank_digit(uint8_t pos);
void set_digit(uint8_t pos, uint8_t val);

static inline void set_decdigit(uint8_t pos, uint8_t val)
{
	set_digit(pos, val % 10);
}

static inline void set_hexdigit(uint8_t pos, uint8_t val)
{
	set_digit(pos, val & 0xf);
}
