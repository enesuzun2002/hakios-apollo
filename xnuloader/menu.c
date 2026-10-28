/*
 * The boot menu of the Xiaomi Mi 10T (apollo), included by loader.c just before efi_main.
 *
 * On the Mi 10T the loader lives in the phone's `recovery` partition: power + volume up starts it,
 * with nobody at a PC. menu_run() then shows a full-screen menu drawn here (no artwork of anyone's:
 * a ring with a dot, drawn procedurally, and the word "HakiOS" in Montserrat, see menu_font.h):
 *
 *   iOS          the loader's USB command loop, waiting for xl on a PC (what it always did)
 *   Android      a cold reset: with no key held the phone boots Android
 *   Boot log     the loader's log, full screen; any key returns
 *   Power off    EfiResetShutdown
 *
 * Volume up/down move, power selects (ST->con_in, fed by the firmware's ButtonsDxe; the first keys
 * are written to the log and the last one is shown at the bottom, so that their codes can be read
 * off the phone). Holding volume down for a second selects too, if ButtonsDxe repeats a held key.
 *
 * The PC keeps priority, so that xl works unattended:
 *   - for MENU_GRACE_MS after usb_start nothing is drawn; a host that is there already (the cable
 *     state becomes "connected", or a command arrives) means the loader was started by `fastboot
 *     boot` (xl enter) or is wanted by a PC: no menu at all, the command loop as before;
 *   - while the menu shows, USB is serviced as usb_poll_command() does: the first command a host
 *     sends ends the menu and is handled (nothing is lost), the command loop takes over;
 *   - a host that connects while the menu shows makes iOS the default, chosen after
 *     MENU_HOST_MS unless a key is pressed; otherwise the default is Android after MENU_IDLE_MS
 *     without a key.
 *
 * Any other phone (and QEMU) never gets here: loader.c calls menu_run() for platform_apollo only.
 * Nothing here exits boot services or changes what the command loop finds: the canvas is UEFI pool,
 * freed before menu_run() returns; the battery is read with the charger protocols' read calls
 * only, without charger_start(), so charger_enabled and charger_shared are left as they were.
 */

struct menu_glyph {
	uint8_t w, h;           /* the glyph's box */
	int8_t x, y;            /* its top left, from the pen position on the baseline */
	uint8_t adv;
	uint32_t off;           /* in nibbles: 4-bit coverage, row after row */
};

struct menu_font {
	const struct menu_glyph *g;     /* for 32..127; 127 is a middle dot */
	const uint8_t *bits;
	int ascent, descent;
};

#include "menu_font.h"

#define MENU_GRACE_MS   2000    /* before the menu: is a host there already? */
#define MENU_IDLE_MS    30000   /* no key for this long: Android */
#define MENU_HOST_MS    5000    /* a host connected meanwhile, no key yet: iOS */
#define MENU_KEYS_LOGGED 24

#define RGB(r, g, b)    (((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b))
#define C_TOP           RGB(0x0b, 0x10, 0x20)
#define C_BOTTOM        RGB(0x1b, 0x23, 0x40)
#define C_ACCENT        RGB(0x4f, 0x7c, 0xff)
#define C_ACCENT_EDGE   RGB(0x9d, 0xb6, 0xff)
#define C_VIOLET        RGB(0x9b, 0x6b, 0xff)
#define C_CYAN          RGB(0x7d, 0xd3, 0xfc)
#define C_WHITE         RGB(0xff, 0xff, 0xff)
#define C_TEXT2         RGB(0xa9, 0xb4, 0xd6)
#define C_TEXT3         RGB(0x76, 0x82, 0xa8)

/* the layout, for the 1080 x 2400 screen */
#define MARK_X          540
#define MARK_Y          470
#define MARK_R          100
#define MARK_W          26
#define CARD_X0         64
#define CARD_X1         1016
#define CARD_Y          1060
#define CARD_H          200
#define CARD_GAP        32
#define CARD_R          28
#define CARDS_Y0        (CARD_Y - 12)
#define CARDS_Y1        (CARD_Y + 4 * CARD_H + 3 * CARD_GAP + 12)
#define STATUS_Y0       2060
#define STATUS_Y1       2400

enum { M_IOS, M_ANDROID, M_LOG, M_OFF, M_COUNT };
static const char *const menu_title[M_COUNT] = { "iOS", "Android", "Boot log", "Power off" };
static const char *const menu_sub[M_COUNT] = {
	"Connect to a PC and run xl", "Restart into Android", "What the loader has done so far", "Turn the phone off",
};

typedef efi_status (EFIAPI *menu_blt_fn)(efi_gop *self, void *buffer, uint32_t op, uint64_t sx, uint64_t sy,
    uint64_t dx, uint64_t dy, uint64_t w, uint64_t h, uint64_t delta);

static struct {
	uint32_t *px;           /* the canvas, 0x00RRGGBB = the GOP's BGRA in memory */
	int w, h;
	int usb_ok;
	int sel;
	int keys;               /* keys read so far */
	int key_pressed;        /* since the menu showed */
	uint16_t last_scan;
	char16 last_char;
	int soc, soc_estimated, charging;       /* soc -1: unknown */
	const char *note;       /* replaces the selected entry's line under its title */
} M;

static uint64_t
menu_ms(void)
{
	return READ_SYSREG(CNTVCT_EL0) * 1000 / READ_SYSREG(CNTFRQ_EL0);
}

static uint64_t
menu_isqrt(uint64_t n)
{
	uint64_t r = 0, b = 1ULL << 62;

	while (b > n) {
		b >>= 2;
	}
	while (b) {
		if (n >= r + b) {
			n -= r + b;
			r = (r >> 1) + b;
		} else {
			r >>= 1;
		}
		b >>= 2;
	}
	return r;
}

/* d over s, by a of 256 */
static inline uint32_t
menu_mix(uint32_t d, uint32_t s, unsigned a)
{
	int dr = (d >> 16) & 0xff, dg = (d >> 8) & 0xff, db = d & 0xff;
	int sr = (s >> 16) & 0xff, sg = (s >> 8) & 0xff, sb = s & 0xff;

	if (a >= 256) {
		return s;
	}
	dr += ((sr - dr) * (int)a) >> 8;
	dg += ((sg - dg) * (int)a) >> 8;
	db += ((sb - db) * (int)a) >> 8;
	return RGB(dr, dg, db);
}

static inline void
menu_blend(int x, int y, uint32_t rgb, unsigned a)
{
	if (a != 0 && x >= 0 && y >= 0 && x < M.w && y < M.h) {
		uint32_t *p = &M.px[(uint64_t)y * M.w + x];
		*p = menu_mix(*p, rgb, a);
	}
}

static unsigned
menu_clamp256(int64_t v)
{
	return v <= 0 ? 0 : v >= 256 ? 256 : (unsigned)v;
}

/* the background of rows y0..y1: the vertical gradient, and a soft glow of the accent behind the mark */
static void
menu_background(int y0, int y1)
{
	const int64_t glow_r = 600, glow_r2 = glow_r * glow_r;
	int tr = (C_TOP >> 16) & 0xff, tg = (C_TOP >> 8) & 0xff, tb = C_TOP & 0xff;
	int br = (C_BOTTOM >> 16) & 0xff, bg = (C_BOTTOM >> 8) & 0xff, bb = C_BOTTOM & 0xff;

	for (int y = y0 < 0 ? 0 : y0; y < y1 && y < M.h; y++) {
		uint32_t c = RGB(tr + (br - tr) * y / M.h, tg + (bg - tg) * y / M.h, tb + (bb - tb) * y / M.h);
		uint32_t *row = &M.px[(uint64_t)y * M.w];
		int64_t dy = y - MARK_Y;

		for (int x = 0; x < M.w; x++) {
			row[x] = c;
		}
		if (dy * dy >= glow_r2) {
			continue;
		}
		for (int x = 0; x < M.w; x++) {
			int64_t dx = x - MARK_X, d2 = dx * dx + dy * dy;

			if (d2 < glow_r2) {
				int64_t t = 256 - d2 * 256 / glow_r2;

				row[x] = menu_mix(c, C_ACCENT, (unsigned)(t * t * 46 >> 16));
			}
		}
	}
}

/* coverage (of 256) of pixel (x, y) by the rectangle x0..x1, y0..y1 with corners of radius r */
static unsigned
menu_rr_cov(int x, int y, int x0, int y0, int x1, int y1, int r)
{
	int64_t px = ((int64_t)x << 8) + 128, py = ((int64_t)y << 8) + 128, ex = 0, ey = 0, d;
	int64_t l = (int64_t)(x0 + r) << 8, rr = (int64_t)(x1 - r) << 8;
	int64_t t = (int64_t)(y0 + r) << 8, b = (int64_t)(y1 - r) << 8;

	if (x < x0 || x >= x1 || y < y0 || y >= y1) {
		return 0;
	}
	if (px < l) {
		ex = l - px;
	} else if (px > rr) {
		ex = px - rr;
	}
	if (py < t) {
		ey = t - py;
	} else if (py > b) {
		ey = py - b;
	}
	if (ex == 0 && ey == 0) {
		return 256;
	}
	d = ex == 0 ? ey : ey == 0 ? ex : (int64_t)menu_isqrt((uint64_t)(ex * ex + ey * ey));
	return menu_clamp256(128 - (d - ((int64_t)r << 8)));
}

/* a rounded rectangle filled with `fill` (by fa of 256) and edged by bw pixels of `edge` (by ea) */
static void
menu_rrect(int x0, int y0, int x1, int y1, int r, uint32_t fill, unsigned fa, uint32_t edge, unsigned ea, int bw)
{
	for (int y = y0; y < y1; y++) {
		for (int x = x0; x < x1; x++) {
			unsigned co = menu_rr_cov(x, y, x0, y0, x1, y1, r), ci;

			if (co == 0) {
				continue;
			}
			menu_blend(x, y, fill, fa * co >> 8);
			if (bw > 0 && ea > 0) {
				ci = menu_rr_cov(x, y, x0 + bw, y0 + bw, x1 - bw, y1 - bw, r - bw);
				if (co > ci) {
					menu_blend(x, y, edge, ea * (co - ci) >> 8);
				}
			}
		}
	}
}

/* coverage (of 256) of pixel (x, y) by the disc of radius r around (cx, cy) */
static unsigned
menu_disc_cov(int x, int y, int cx, int cy, int r)
{
	int64_t dx = ((int64_t)(x - cx) << 8) + 128, dy = ((int64_t)(y - cy) << 8) + 128;

	return menu_clamp256(128 + ((int64_t)r << 8) - (int64_t)menu_isqrt((uint64_t)(dx * dx + dy * dy)));
}

static void
menu_disc(int cx, int cy, int r, uint32_t rgb, unsigned a)
{
	for (int y = cy - r - 1; y <= cy + r + 1; y++) {
		for (int x = cx - r - 1; x <= cx + r + 1; x++) {
			menu_blend(x, y, rgb, a * menu_disc_cov(x, y, cx, cy, r) >> 8);
		}
	}
}

/*
 * The mark: an open ring in a blue-to-violet gradient with a small cyan "moon" on it, top right,
 * the ring parted around it. Geometry only.
 */
static void
menu_mark(void)
{
	const int mid = MARK_R - MARK_W / 2;            /* the ring's centre line */
	const int mx = MARK_X + mid * 707 / 1000, my = MARK_Y - mid * 707 / 1000;
	const int moon = 24, gap = 40;

	for (int y = MARK_Y - MARK_R - 1; y <= MARK_Y + MARK_R + 1; y++) {
		for (int x = MARK_X - MARK_R - 1; x <= MARK_X + MARK_R + 1; x++) {
			unsigned a = menu_disc_cov(x, y, MARK_X, MARK_Y, MARK_R), in, cut;
			int t;

			if (a == 0) {
				continue;
			}
			in = menu_disc_cov(x, y, MARK_X, MARK_Y, MARK_R - MARK_W);
			cut = menu_disc_cov(x, y, mx, my, gap);
			a = a > in ? a - in : 0;
			a = a * (256 - cut) >> 8;
			/* along the diagonal from top left (blue) to bottom right (violet) */
			t = ((x - MARK_X) + (y - MARK_Y) + 2 * MARK_R) * 256 / (4 * MARK_R);
			menu_blend(x, y, menu_mix(C_ACCENT, C_VIOLET, menu_clamp256(t)), a);
		}
	}
	menu_disc(mx, my, moon, C_CYAN, 256);
}

static const struct menu_glyph *
menu_glyph(const struct menu_font *f, char c)
{
	unsigned u = (unsigned char)c;

	return &f->g[(u >= 32 && u < 128 ? u : '?') - 32];
}

static int
menu_text_width(const struct menu_font *f, const char *s)
{
	int w = 0;

	for (; *s; s++) {
		w += menu_glyph(f, *s)->adv;
	}
	return w;
}

/* returns where the pen ends */
static int
menu_text(const struct menu_font *f, int x, int base, const char *s, uint32_t rgb, unsigned a)
{
	for (; *s; s++) {
		const struct menu_glyph *g = menu_glyph(f, *s);
		uint32_t n = g->off;

		for (int row = 0; row < g->h; row++) {
			for (int col = 0; col < g->w; col++, n++) {
				unsigned v = (f->bits[n >> 1] >> ((n & 1) * 4)) & 15;

				if (v != 0) {
					menu_blend(x + g->x + col, base + g->y + row, rgb, a * v / 15);
				}
			}
		}
		x += g->adv;
	}
	return x;
}

static void
menu_text_centered(const struct menu_font *f, int base, const char *s, uint32_t rgb, unsigned a)
{
	menu_text(f, (M.w - menu_text_width(f, s)) / 2, base, s, rgb, a);
}

/* a tiny string builder for the status line */
static char *
menu_cat(char *p, const char *s)
{
	while (*s) {
		*p++ = *s++;
	}
	*p = 0;
	return p;
}

static char *
menu_num(char *p, uint64_t v, int hex_digits)
{
	char tmp[24];
	int i = 0;

	if (hex_digits) {
		p = menu_cat(p, "0x");
		while (i < hex_digits) {
			tmp[i++] = "0123456789abcdef"[v & 15];
			v >>= 4;
		}
	} else {
		do {
			tmp[i++] = (char)('0' + v % 10);
			v /= 10;
		} while (v);
	}
	while (i) {
		*p++ = tmp[--i];
	}
	*p = 0;
	return p;
}

/* Copy rows y0..y1 of the canvas to the screen: GOP's Blt, else the framebuffer itself (BGR only). */
static void
menu_show(int y0, int y1)
{
	menu_blt_fn blt = (menu_blt_fn)GOP->blt;

	if (y0 < 0) {
		y0 = 0;
	}
	if (y1 > M.h) {
		y1 = M.h;
	}
	if (y1 <= y0) {
		return;
	}
	if (blt != NULL && !EFI_ERROR(blt(GOP, M.px, 2 /* EfiBltBufferToVideo */, 0, (uint64_t)y0, 0, (uint64_t)y0,
	    (uint64_t)M.w, (uint64_t)(y1 - y0), (uint64_t)M.w * 4))) {
		return;
	}
	if (GOP->mode->info->pixel_format != 1 /* PixelBlueGreenRedReserved8BitPerColor */) {
		return;
	}
	for (int y = y0; y < y1; y++) {
		memcpy((void *)(uintptr_t)(SCREEN.base + (uint64_t)y * SCREEN.stride), &M.px[(uint64_t)y * M.w], (size_t)M.w * 4);
	}
	clean_dcache(SCREEN.base + (uint64_t)y0 * SCREEN.stride, SCREEN.base + (uint64_t)y1 * SCREEN.stride);
}

static void
menu_header(void)
{
	menu_mark();
	menu_text_centered(&menu_font_title, 770, "HakiOS", C_WHITE, 256);
	menu_text_centered(&menu_font_small, 850, "Xiaomi Mi 10T", C_TEXT2, 256);
	menu_text(&menu_font_small, CARD_X0 + 8, CARD_Y - 44, "Choose a system", C_TEXT3, 256);
}

/* one entry's card, at slot `slot` */
static void
menu_card(int slot, int item, int selected, const char *sub)
{
	int y0 = CARD_Y + slot * (CARD_H + CARD_GAP), y1 = y0 + CARD_H;
	int cx = CARD_X0 + 40 + 52, cy = y0 + CARD_H / 2;
	int disabled = item == M_IOS && !M.usb_ok;
	char letter[2] = { menu_title[item][0], 0 };
	const struct menu_glyph *g = menu_glyph(&menu_font_body, letter[0]);

	if (selected) {
		menu_rrect(CARD_X0, y0, CARD_X1, y1, CARD_R, C_ACCENT, 256, C_ACCENT_EDGE, 210, 3);
		menu_disc(cx, cy, 52, C_WHITE, 256);
		menu_text(&menu_font_body, cx - g->adv / 2, cy + 20, letter, C_ACCENT, 256);
	} else {
		menu_rrect(CARD_X0, y0, CARD_X1, y1, CARD_R, C_WHITE, 15, C_WHITE, 30, 2);
		menu_disc(cx, cy, 52, C_WHITE, 26);
		menu_text(&menu_font_body, cx - g->adv / 2, cy + 20, letter, C_WHITE, disabled ? 110 : 230);
	}
	menu_text(&menu_font_body, CARD_X0 + 40 + 104 + 40, y0 + 92, menu_title[item], C_WHITE, disabled ? 120 : 256);
	menu_text(&menu_font_small, CARD_X0 + 40 + 104 + 40, y0 + 150, sub,
	    selected ? C_WHITE : C_TEXT3, selected ? 215 : disabled ? 140 : 256);
}

static void
menu_cards(void)
{
	for (int i = 0; i < M_COUNT; i++) {
		const char *sub = menu_sub[i];

		if (i == M_IOS && !M.usb_ok) {
			sub = "USB is not available";
		}
		if (i == M.sel && M.note != NULL) {
			sub = M.note;
		}
		menu_card(i, i, i == M.sel, sub);
	}
}

static void
menu_status(int default_item, uint64_t left_ms)
{
	char line[128], *p;
	int w;

	/* a hairline, then: battery left, USB right; the countdown and the keys under them */
	for (int x = CARD_X0; x < CARD_X1; x++) {
		menu_blend(x, STATUS_Y0 + 20, C_WHITE, 26);
	}
	p = menu_cat(line, "Battery ");
	if (M.soc < 0) {
		p = menu_cat(p, "--");
	} else {
		p = menu_cat(p, M.soc_estimated ? "~" : "");
		p = menu_num(p, (uint64_t)M.soc, 0);
		p = menu_cat(p, "%");
	}
	if (M.charging) {
		menu_cat(p, " \x7f charging");
	}
	menu_text(&menu_font_small, CARD_X0 + 8, STATUS_Y0 + 92, line, C_TEXT2, 256);
	menu_cat(line, !M.usb_ok ? "USB: unavailable" : usb_connected ? "USB: connected" : "USB: not connected");
	w = menu_text_width(&menu_font_small, line);
	menu_text(&menu_font_small, CARD_X1 - 8 - w, STATUS_Y0 + 92, line, usb_connected ? C_CYAN : C_TEXT2, 256);

	p = menu_cat(line, menu_title[default_item]);
	p = menu_cat(p, " in ");
	p = menu_num(p, (left_ms + 999) / 1000, 0);
	menu_cat(p, " s  \x7f  volume moves, power selects");
	menu_text_centered(&menu_font_small, STATUS_Y0 + 158, line, C_TEXT2, 256);

	if (M.keys > 0) {
		p = menu_cat(line, "last key: scan ");
		p = menu_num(p, M.last_scan, 4);
		p = menu_cat(p, "  char ");
		menu_num(p, M.last_char, 4);
		menu_text_centered(&menu_font_log, STATUS_Y0 + 222, line, C_TEXT3, 256);
	}
}

static void
menu_draw_all(int default_item, uint64_t left_ms)
{
	menu_background(0, M.h);
	menu_header();
	menu_cards();
	menu_status(default_item, left_ms);
	menu_show(0, M.h);
}

static void
menu_draw_cards(void)
{
	menu_background(CARDS_Y0, CARDS_Y1);
	menu_cards();
	menu_show(CARDS_Y0, CARDS_Y1);
}

static void
menu_draw_status(int default_item, uint64_t left_ms)
{
	menu_background(STATUS_Y0, STATUS_Y1);
	menu_status(default_item, left_ms);
	menu_show(STATUS_Y0, STATUS_Y1);
}

/*
 * The battery, read the way charger_poll() reads it but into M only: ChargerEx's presence and
 * voltage, QcomCharger's GetChargingAction (display 0) for the state of charge. No initialize(),
 * no TakeAction, charger_enabled untouched.
 */
static void
menu_battery(void)
{
	static const efi_guid ex_guid = QCOM_CHARGER_EX_GUID, qc_guid = QCOM_CHARGER_GUID;
	static efi_charger_ex *ex;
	static efi_qcom_charger *qc;
	static int looked;
	struct qcom_battery_status status;
	uint32_t mv = 0, action = 0;
	uint8_t plugged = 0;
	int soc = -1, estimated = 0;

	if (!looked) {
		looked = 1;
		if (EFI_ERROR(BS->locate_protocol(&ex_guid, NULL, (void **)&ex))) {
			ex = NULL;
		}
		if (EFI_ERROR(BS->locate_protocol(&qc_guid, NULL, (void **)&qc))) {
			qc = NULL;
		}
		print("menu: battery through ChargerEx %s, QcomCharger %s\n", ex != NULL ? "yes" : "no",
		    qc != NULL ? "yes" : "no");
	}
	if (ex != NULL) {
		if (EFI_ERROR(ex->get_battery_voltage(&mv)) || mv < 2500 || mv > 5000) {
			mv = 0;
		}
		if (EFI_ERROR(ex->get_charger_presence(&plugged))) {
			plugged = 0;
		}
	}
	if (qc != NULL && qc->get_charging_action != NULL) {
		memset(&status, 0, sizeof(status));
		if (!EFI_ERROR(qc->get_charging_action(&action, &status, 0))) {
			if (mv == 0 && status.battery_voltage_mv >= 2500 && status.battery_voltage_mv <= 5000) {
				mv = (uint32_t)status.battery_voltage_mv;
			}
			/* (as charger_poll(): a near-empty gauge on a cell well above empty is not settled yet) */
			if (status.state_of_charge >= 0 && status.state_of_charge <= 100 &&
			    !(status.state_of_charge < 5 && mv > 3600)) {
				soc = status.state_of_charge;
			}
		}
	}
	if (soc < 0 && mv != 0) {
		soc = soc_from_voltage(mv);
		estimated = 1;
	}
	M.soc = soc;
	M.soc_estimated = estimated;
	M.charging = plugged && soc >= 0 && soc < 100;
}

/*
 * One look at the USB controller, as usb_poll_command() does it: a receive for the next command
 * is left with the controller while a host is connected. Returns 1 with a command in cmd.
 */
static int
menu_usb_poll(char *cmd, uint64_t size)
{
	uint32_t msg = UsbDeviceEventNoEvent;
	uint64_t len = 0, n;
	usb_device_event_data data;

	if (!M.usb_ok) {
		return 0;
	}
	if (!usb_rx_queued && usb_connected && !EFI_ERROR(USB->send(USB_EP_OUT, 511, usb_rx_buf))) {
		usb_rx_queued = 1;
	}
	USB->handle_event(&msg, &len, &data);
	if (msg == UsbDeviceEventDeviceStateChange) {
		usb_connected = (data.device_state == UsbDeviceStateConnected);
		usb_rx_queued = 0;
		print("menu: USB %s\n", usb_connected ? "connected" : "disconnected");
		return 0;
	}
	if (msg != UsbDeviceEventTransferNotification || data.transfer.endpoint_index != USB_EP_OUT) {
		return 0;
	}
	usb_rx_queued = 0;
	if (data.transfer.status != UsbDeviceTransferStatusCompleteOK || data.transfer.bytes_completed == 0) {
		return 0;
	}
	n = data.transfer.bytes_completed < size - 1 ? data.transfer.bytes_completed : size - 1;
	memcpy(cmd, usb_rx_buf, (size_t)n);
	cmd[n] = 0;
	return 1;
}

enum { K_NONE, K_UP, K_DOWN, K_SELECT, K_OTHER };

/* the next key, if there is one: logged the first times, and kept for the status line */
static int
menu_key(void)
{
	efi_simple_text_input *in = (efi_simple_text_input *)ST->con_in;
	efi_input_key key;

	if (in == NULL || in->read_key_stroke == NULL || EFI_ERROR(in->read_key_stroke(in, &key))) {
		return K_NONE;
	}
	M.keys++;
	M.last_scan = key.scan_code;
	M.last_char = key.unicode_char;
	if (M.keys <= MENU_KEYS_LOGGED) {
		print("menu: key %d: scan %x unicode %x\n", (uint64_t)M.keys, (uint64_t)key.scan_code,
		    (uint64_t)key.unicode_char);
	}
	/* SCAN_UP/DOWN (and PAGE_UP/DOWN) move; CR, LF, space, SCAN_RIGHT and SCAN_SUSPEND select */
	switch (key.scan_code) {
	case 0x01: case 0x09:
		return K_UP;
	case 0x02: case 0x0a:
		return K_DOWN;
	case 0x03: case 0x102:
		return K_SELECT;
	}
	if (key.unicode_char == '\r' || key.unicode_char == '\n' || key.unicode_char == ' ') {
		return K_SELECT;
	}
	return K_OTHER;
}

/* Keep the watchdog fed, as usb_wait() does while it waits. */
static void
menu_pet(void)
{
	static uint64_t last;

	if (menu_ms() - last >= 1000) {
		last = menu_ms();
		wdt_pet();
	}
}

/*
 * The log, full screen, its last lines, wrapped to the width; until a key (1) or a host's command
 * (2, in cmd).
 */
static int
menu_log_lines(int draw, int skip, int max)
{
	const struct menu_font *f = &menu_font_log;
	const int x0 = 48, x1 = 1032, top = 240, step = 36;
	int lines = 0;
	uint32_t i = 0;
	char line[256];

	while (i < log_len) {
		int n = 0, w = 0;

		while (i < log_len && log_buf[i] != '\n' && n < (int)sizeof(line) - 1) {
			char c = log_buf[i] == '\t' ? ' ' : log_buf[i];
			int cw = menu_glyph(f, c)->adv;

			if (w + cw > x1 - x0) {
				break;
			}
			if ((unsigned char)c >= 32 && (unsigned char)c < 127) {
				line[n++] = c;
				w += cw;
			}
			i++;
		}
		if (i < log_len && log_buf[i] == '\n') {
			i++;
		}
		line[n] = 0;
		if (draw && lines >= skip && lines - skip < max) {
			menu_text(f, x0, top + (lines - skip) * step, line, RGB(0xd5, 0xdc, 0xf0), 240);
		}
		lines++;
	}
	return lines;
}

static int
menu_log_view(char *cmd, uint64_t size)
{
	const int max = 54;
	int lines;

	menu_background(0, M.h);
	menu_text(&menu_font_body, 48, 150, "Boot log", C_WHITE, 256);
	lines = menu_log_lines(0, 0, 0);
	menu_log_lines(1, lines > max ? lines - max : 0, max);
	menu_text_centered(&menu_font_small, 2330, "Press any key to return", C_TEXT2, 256);
	menu_show(0, M.h);
	for (;;) {
		menu_pet();
		if (menu_usb_poll(cmd, size)) {
			return 2;
		}
		if (menu_key() != K_NONE) {
			return 1;
		}
		BS->stall(5000);
	}
}

/* a last picture before the menu goes: the chosen card, and what happens now */
static void
menu_farewell(int item, const char *what)
{
	M.sel = item;
	M.note = what;
	menu_draw_cards();
}

static void
menu_free(void)
{
	if (M.px != NULL) {
		BS->free_pool(M.px);
		M.px = NULL;
	}
}

/*
 * Returns when the command loop is to take over: iOS was chosen, or a host is there (its first
 * command, if one came, already handled). Does not return for Android and power off.
 */
static void
menu_run(int usb_ok)
{
	char cmd[128];
	uint64_t start, last_key, last_tick = 0, last_batt = 0, down_first = 0, down_last = 0, host_since = 0;
	int down_from = 0, down_fired = 0, was_connected;

	if (SCREEN.base == 0 || GOP == NULL || GOP->mode == NULL || GOP->mode->info == NULL) {
		return;
	}
	memset(&M, 0, sizeof(M));
	M.usb_ok = usb_ok;

	/* started by `fastboot boot`, or wanted by a PC: no menu (see the top of this file) */
	start = menu_ms();
	while (usb_ok && menu_ms() - start < MENU_GRACE_MS) {
		menu_pet();
		if (menu_usb_poll(cmd, sizeof(cmd))) {
			print("menu: a host spoke first: no menu\n");
			screen_quiet = 1;
			fb_command(cmd);
			return;
		}
		if (usb_connected) {
			print("menu: a host is connected: no menu\n");
			return;
		}
		BS->stall(2000);
	}

	M.w = (int)SCREEN.width;
	M.h = (int)SCREEN.height;
	if (EFI_ERROR(BS->allocate_pool(EfiLoaderData, (uint64_t)M.w * M.h * 4, (void **)&M.px)) || M.px == NULL) {
		print("menu: no memory for the canvas\n");
		M.px = NULL;
		return;
	}
	print("menu: shown (%d x %d, Blt %s)\n", (uint64_t)M.w, (uint64_t)M.h, GOP->blt != NULL ? "yes" : "no");
	screen_quiet = 1;               /* the console is not to write over the menu */
	{
		/* keys pressed before the menu (volume up, to get here) are not choices */
		int stale = 0;

		while (menu_key() != K_NONE && stale < 32) {
			stale++;
		}
		M.keys = 0;
	}
	menu_battery();
	M.sel = M_IOS;
	last_key = menu_ms();
	was_connected = usb_connected;
	menu_draw_all(M_ANDROID, MENU_IDLE_MS);

	for (;;) {
		uint64_t now = menu_ms(), left;
		int item = -1, k, deflt;

		menu_pet();
		if (menu_usb_poll(cmd, sizeof(cmd))) {
			print("menu: a host spoke: iOS\n");
			menu_farewell(M_IOS, "A PC is talking to the phone");
			menu_free();
			screen_quiet = 1;
			fb_command(cmd);
			return;
		}
		if (usb_connected != was_connected) {
			was_connected = usb_connected;
			host_since = usb_connected ? now : 0;
			if (usb_connected && !M.key_pressed) {
				M.sel = M_IOS;
				menu_draw_cards();
			}
			last_tick = 0;
		}

		/* the default, and how long until it is taken */
		if (host_since != 0 && !M.key_pressed) {
			deflt = M_IOS;
			left = now - host_since >= MENU_HOST_MS ? 0 : MENU_HOST_MS - (now - host_since);
		} else {
			deflt = M_ANDROID;
			left = now - last_key >= MENU_IDLE_MS ? 0 : MENU_IDLE_MS - (now - last_key);
		}
		if (left == 0) {
			print("menu: no key: %s\n", menu_title[deflt]);
			item = deflt;
		}

		k = item < 0 ? menu_key() : K_NONE;
		if (k != K_NONE) {
			M.key_pressed = 1;
			last_key = now;
			last_tick = 0;
			M.note = NULL;
		}
		if (k == K_UP) {
			M.sel = (M.sel + M_COUNT - 1) % M_COUNT;
			menu_draw_cards();
		} else if (k == K_DOWN) {
			/* held: if ButtonsDxe repeats a held key, a second of it selects where it began */
			if (now - down_last > 300) {
				down_first = now;
				down_from = M.sel;
				down_fired = 0;
			}
			down_last = now;
			if (!down_fired && now - down_first >= 1000) {
				down_fired = 1;
				M.sel = down_from;
				item = M.sel;
				print("menu: volume down held: select\n");
			} else {
				M.sel = (M.sel + 1) % M_COUNT;
				menu_draw_cards();
			}
		} else if (k == K_SELECT) {
			item = M.sel;
		}

		if (item == M_IOS) {
			if (!M.usb_ok) {
				M.note = "USB is not available: choose another";
				menu_draw_cards();
			} else {
				print("menu: iOS\n");
				menu_farewell(M_IOS, usb_connected ? "Connected: run xl on the PC" : "Waiting for a PC: run xl there");
				menu_free();
				return;
			}
		} else if (item == M_ANDROID) {
			print("menu: Android\n");
			menu_farewell(M_ANDROID, "Restarting into Android...");
			reboot();
		} else if (item == M_OFF) {
			print("menu: power off\n");
			menu_farewell(M_OFF, "Turning off...");
			hc_take_back("before power off");
			flush_log();
			ST->rt->reset_system(2 /* EfiResetShutdown */, 0, 0, NULL);
			for (;;) {
			}
		} else if (item == M_LOG) {
			if (menu_log_view(cmd, sizeof(cmd)) == 2) {
				print("menu: a host spoke: iOS\n");
				menu_free();
				screen_quiet = 1;
				fb_command(cmd);
				return;
			}
			last_key = menu_ms();
			menu_draw_all(M_ANDROID, MENU_IDLE_MS);
			continue;
		}

		if (now - last_batt >= 10000) {
			last_batt = now;
			menu_battery();
			last_tick = 0;
		}
		if (last_tick == 0 || now - last_tick >= 1000 || k != K_NONE) {
			last_tick = now;
			menu_draw_status(deflt, left);
		}
		BS->stall(5000);
	}
}
