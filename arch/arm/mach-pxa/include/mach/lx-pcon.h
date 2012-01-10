/*
 * PCON addresses
 */
#define PCON_KEYBOARD_CMD_ADDR         (0x00)
#define PCON_KEYBOARD_CMD_VAL_READY    (0)
#define PCON_KEYBOARD_CMD_VAL_BUSY     (1)
#define PCON_POWER_CF_ADDR             (0x02)
#define PCON_POWER_PCMCIA_ADDR         (0x03)
#define PCON_POWER_CFG_ADDR            (0x04)
#define PCON_PWRSTATE_ADDR             (0x05)
#define PCON_CMD_ADDR                  (0x0c)  /* 1 + 8 args */

typedef enum pcon_pwrstate_e {
	PCON_PWRSTATE_HWReset	= 0,
	PCON_PWRSTATE_SWReset	= 1,
	PCON_PWRSTATE_Off	= 2,
	PCON_PWRSTATE_Suspend	= 3,
	PCON_PWRSTATE_Sleep	= 4,
	PCON_PWRSTATE_Booting	= 5,
	PCON_PWRSTATE_LoadingOS	= 6,
	PCON_PWRSTATE_Idle	= 7,
	PCON_PWRSTATE_Run	= 8,
	PCON_PWRSTATE_Comatose 	= 9,
	PCON_PWRSTATE_PowerFail	= 10
} pcon_pwrstate_t;

extern int lx_pwr_setstate(pcon_pwrstate_t);
extern pcon_pwrstate_t lx_pwr_getstate(void);
extern int pcon_pwr_setstate(pcon_pwrstate_t);

typedef enum {
	PCON_CMD_Ready		= 0,
	PCON_CMD_ADCRead	= 9,
	PCON_CMD_GetVersion	= 10,
	PCON_CMD_SetMMCPower	= 11,
	PCON_CMD_SetModemPower	= 12,
	PCON_CMD_SetUSBPower	= 13,
	PCON_CMD_SetRedLED	= 14,
	PCON_CMD_SetSerialPower	= 15,
	PCON_CMD_GetSerialCfg	= 17,
	PCON_CMD_BatteryMonitor	= 18,
	PCON_CMD_GetMMCPower	= 19,
	PCON_CMD_GetVModemPower	= 20,
	PCON_CMD_GetUSBPower	= 21,
	PCON_CMD_SetWakeupAlarm	= 22,
} pcon_cmd_t;

/*
 * Host EEPROM emulation addresses
 */
#define PHST_KEYBOARD_START	(0x00)
#define PHST_KEYBOARD_SIZE	(1)
#define PHST_SERIAL_START	(0x02)
#define PHST_SERIAL_SIZE	(1)
#define PHST_POWER_START	(0x05)
#define PHST_POWER_SIZE		(1)
#define PHST_CMDRESP_START	(0x0d)
#define PHST_CMDRESP_SIZE	(8)

struct lx_i2cslave_watcher {
	u16 start;
	u16 size;
	void (*write)(void *, unsigned int addr, unsigned char newval);
};

struct lx_i2cslave_watch {
	struct list_head node;
	u16 start;
	u16 end;
	struct lx_i2cslave_watcher *ops;
	void *data;
};

#define I2C_EEPROM_EMU_SIZE (256)

struct lx_eeprom_emu {
	u16 size;
	u16 ptr;
	unsigned int seen_start;
	struct list_head watch;

	unsigned char bytes[I2C_EEPROM_EMU_SIZE];
};

extern int lx_i2cslave_addwatcher(struct lx_i2cslave_watcher *, void *data);
extern void lx_i2cslave_delwatcher(struct lx_i2cslave_watcher *, void *data);


#define PCON_FLG_ASYNC          (1)

/* exported functions */

extern int pcon_simple_cmd(unsigned int, unsigned int, unsigned int);
#define pcon_set_usbhost_power(on)     \
	pcon_simple_cmd(PCON_CMD_SetUSBPower, !!(on), PCON_FLG_ASYNC)
#define pcon_set_mmc_power(on)         \
	pcon_simple_cmd(PCON_CMD_SetMMCPower, !!(on), 0)
#define pcon_set_modem_power(on)       \
	pcon_simple_cmd(PCON_CMD_SetModemPower, !!(on), 0)
#define pcon_set_serial_power(on)      \
	pcon_simple_cmd(PCON_CMD_SetSerialPower, !!(on), PCON_FLG_ASYNC)

#define pcon_monitorbattery(on, flag)  \
	pcon_simple_cmd(PCON_CMD_BatteryMonitor, !!(on), flag)
#define pcon_setredled(on)             \
	pcon_simple_cmd(PCON_CMD_SetRedLED, !!(on), 0)

extern int pcon_simple_write(unsigned int, unsigned int, unsigned int);
#define pcon_kbd_ack()                 \
	pcon_simple_write(PCON_KEYBOARD_CMD_ADDR, PCON_KEYBOARD_CMD_VAL_READY, PCON_FLG_ASYNC)
#define pcon_set_pcmcia_power(on)      \
	pcon_simple_write(PCON_POWER_PCMCIA_ADDR, !!(on), 0)
#define pcon_set_cf_power(on)          \
	pcon_simple_write(PCON_POWER_CF_ADDR, !!(on), 0)

struct rtc_time;
extern int pcon_rtc_wakeup(struct rtc_time *tm);
