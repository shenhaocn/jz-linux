#ifndef __AK4183_H__
#define __AK4183_H__

#define AK4183_NAME	"ak4183"

#define TS_PIN  GPIO_PEN_IRQ
#define TS_IRQ  (IRQ_GPIO_0 + TS_PIN)

#define TSMINX 6
#define TSMAXX 249
#define TSMINY 9
#define TSMAXY 240

#define SCREEN_X	480
#define SCREEN_Y	272
#define PRESS_Z		256

#define SAMPLES 5

struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
	u16	pressure;
	int	ignore;
};

struct ak4183 {
	struct	input_dev	*input;
	char			phys[32];

	struct	ts_event	event;
	spinlock_t		lock;
	struct	delayed_work	work;
	struct	timer_list	timer;
	struct	pm_dev		*pmdev;
	u16			x_plate_ohms;
	int			irq_enabled;
};

#endif /* __AK4183_H__ */
