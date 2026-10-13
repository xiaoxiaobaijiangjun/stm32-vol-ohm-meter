/*
 * mid_calib.c - calibration storage (last flash page), UART console and the
 * 1 Hz BLE measurement report. Pure polled code, main-loop friendly.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "mid_calib.h"
#include "hw_uart1.h"
#include "stm32f1xx_hal.h"

/* ---------------- storage layout (44 bytes, halfword-aligned) -------------- */
#define CAL_FLASH_PAGE   0x0800FC00U        /* page 63: last 1 KB of the 64 KB C8 */
#define CAL_MAGIC        0x314C4143U        /* "CAL1" */
#define CAL_VERSION      1U

typedef struct
{
	uint32_t magic;
	uint16_t version;
	uint16_t range_count;
	float    k[CAL_RANGE_COUNT];
	float    b[CAL_RANGE_COUNT];
	uint32_t crc;                /* CRC32 over the 40 bytes above */
} cal_store_t;

typedef char cal_size_check[(sizeof(cal_store_t) == 44U) ? 1 : -1];

static float    g_k[CAL_RANGE_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
static float    g_b[CAL_RANGE_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t  g_src_flash = 0U;            /* 1 = loaded from flash        */

/* published raw values (pre-calibration) from the measurement pages */
static float    g_x[CAL_RANGE_COUNT];
static uint32_t g_x_tick[CAL_RANGE_COUNT];
static uint8_t  g_last_idx = 0U;

/* console session state */
static uint8_t  g_session = 0U;              /* console/cal session active   */
static uint32_t g_session_tick = 0U;
static uint8_t  g_report = 0U;               /* BLE 1 Hz report              */
static uint32_t g_report_tick = 0U;
static char     g_line[64];
static uint8_t  g_line_len = 0U;

/* ---------------- software CRC32 (reflected, zip polynomial) --------------- */
static uint32_t cal_crc32(const uint8_t *d, uint32_t n)
{
	uint32_t crc = 0xFFFFFFFFU;
	uint32_t i;
	uint8_t  bit;

	for(i = 0U; i < n; i++)
	{
		crc ^= d[i];
		for(bit = 0U; bit < 8U; bit++)
		{
			crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
		}
	}
	return crc ^ 0xFFFFFFFFU;
}

/* ---------------- flash ---------------------------------------------------- */
static uint8_t cal_store_save(const float *k, const float *b)
{
	cal_store_t s;
	FLASH_EraseInitTypeDef erase;
	uint32_t page_error = 0U;
	uint32_t i;
	uint8_t ok = 1U;

	s.magic = CAL_MAGIC;
	s.version = CAL_VERSION;
	s.range_count = CAL_RANGE_COUNT;
	memcpy(s.k, k, sizeof(s.k));
	memcpy(s.b, b, sizeof(s.b));
	s.crc = cal_crc32((const uint8_t *)&s, 40U);

	HAL_FLASH_Unlock();
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = CAL_FLASH_PAGE;
	erase.NbPages = 1U;
	if(HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
	{
		ok = 0U;
	}
	for(i = 0U; (i < sizeof(cal_store_t)) && ok; i += 2U)
	{
		uint16_t hw = (uint16_t)(((const uint8_t *)&s)[i] |
		                         ((uint32_t)((const uint8_t *)&s)[i + 1U] << 8));
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CAL_FLASH_PAGE + i, hw) != HAL_OK)
		{
			ok = 0U;
		}
	}
	HAL_FLASH_Lock();

	if(ok)
	{
		/* verify by reading back */
		const cal_store_t *p = (const cal_store_t *)CAL_FLASH_PAGE;
		ok = (p->magic == CAL_MAGIC) && (p->crc == s.crc) &&
		     (cal_crc32((const uint8_t *)p, 40U) == p->crc);
	}
	return ok;
}

static uint8_t cal_store_load(void)
{
	const cal_store_t *p = (const cal_store_t *)CAL_FLASH_PAGE;
	uint8_t i;

	if(p->magic != CAL_MAGIC || p->version != CAL_VERSION ||
	   p->range_count != CAL_RANGE_COUNT)
	{
		return 0U;
	}
	if(cal_crc32((const uint8_t *)p, 40U) != p->crc)
	{
		return 0U;
	}
	for(i = 0U; i < CAL_RANGE_COUNT; i++)
	{
		/* sanity: a k/b pair corrupted into a huge/NaN value must fall back
		 * to defaults instead of poisoning the display */
		if(!(p->k[i] > 0.05f && p->k[i] < 20.0f && p->b[i] > -1000000.0f && p->b[i] < 1000000.0f))
		{
			return 0U;
		}
	}
	memcpy(g_k, p->k, sizeof(g_k));
	memcpy(g_b, p->b, sizeof(g_b));
	return 1U;
}

/* ---------------- public helpers ------------------------------------------- */
float CAL_Apply(uint8_t idx, float x)
{
	if(idx >= CAL_RANGE_COUNT)
	{
		return x;
	}
	return g_k[idx] * x + g_b[idx];
}

void CAL_Publish_Raw(uint8_t idx, float x)
{
	if(idx >= CAL_RANGE_COUNT)
	{
		return;
	}
	g_x[idx] = x;
	g_x_tick[idx] = HAL_GetTick();
	g_last_idx = idx;
}

uint8_t CAL_Is_Active(void)
{
	return g_session;
}

uint8_t CAL_Report_Enabled(void)
{
	return g_report;
}

/* ---------------- console -------------------------------------------------- */
/* Note on number formatting: newlib-nano's printf has no %f by default, so
 * every float leaves the chip as fixed point via snprintf %ld. */
static void reply(const char *s)
{
	HW_UART1_Send(s);
	HW_UART1_Send("\r\n");
}

static uint8_t parse_idx(const char *p, uint8_t *idx)
{
	if((p[0] >= '0') && (p[0] <= '3') && (p[1] == ' ' || p[1] == '\0'))
	{
		*idx = (uint8_t)(p[0] - '0');
		return 1U;
	}
	return 0U;
}

static void cmd_dispatch(char *line)
{
	char *arg = line;

	/* split the first token */
	while((*arg != ' ') && (*arg != '\0'))
	{
		arg++;
	}
	if(*arg == ' ')
	{
		*arg++ = '\0';
	}

	if(strcmp(line, "STAT") == 0)
	{
		char buf[96];
		/* STAT v1 src=DEF|FLASH R0:k,b ... (values in 1e-4 fixed point) */
		strcpy(buf, "STAT v1 src=");
		strcat(buf, g_src_flash ? "FLASH" : "DEF");
		for(uint8_t i = 0U; i < CAL_RANGE_COUNT; i++)
		{
			char one[24];
			long kf = (long)(g_k[i] * 10000.0f + (g_k[i] >= 0 ? 0.5f : -0.5f));
			long bf = (long)(g_b[i] * 10000.0f + (g_b[i] >= 0 ? 0.5f : -0.5f));
			(void)snprintf(one, sizeof(one), " R%u:%ld,%ld", (unsigned)i, kf, bf);
			strcat(buf, one);
		}
		reply(buf);
	}
	else if(strcmp(line, "CAP") == 0)
	{
		uint8_t idx;
		if(!parse_idx(arg, &idx))
		{
			reply("ERR usage: CAP <0-3>");
			return;
		}
		if((HAL_GetTick() - g_x_tick[idx]) > 2000U)
		{
			reply("CAP ERR MODE");   /* that range's page is not running */
			return;
		}
		char buf[32];
		long xf = (long)(g_x[idx] * 100.0f + (g_x[idx] >= 0 ? 0.5f : -0.5f));
		(void)snprintf(buf, sizeof(buf), "CAP %u %s%ld.%02ld",
		               (unsigned)idx, (xf < 0) ? "" : "+", xf / 100L, labs(xf) % 100L);
		reply(buf);
	}
	else if(strcmp(line, "SET") == 0)
	{
		uint8_t idx;
		char *p1, *p2;
		if(!parse_idx(arg, &idx))
		{
			reply("ERR usage: SET <0-3> <k> <b>");
			return;
		}
		p1 = arg + 2U;
		p2 = p1;
		while((*p2 != ' ') && (*p2 != '\0'))
		{
			p2++;
		}
		if(*p2 != ' ')
		{
			reply("ERR usage: SET <0-3> <k> <b>");
			return;
		}
		*p2++ = '\0';
		g_k[idx] = strtof(p1, NULL);
		g_b[idx] = strtof(p2, NULL);
		reply("SET OK");
	}
	else if(strcmp(line, "SAVE") == 0)
	{
		reply(cal_store_save(g_k, g_b) ? "SAVE OK" : "SAVE ERR FLASH");
	}
	else if(strcmp(line, "DEFAULT") == 0)
	{
		for(uint8_t i = 0U; i < CAL_RANGE_COUNT; i++)
		{
			g_k[i] = 1.0f;
			g_b[i] = 0.0f;
		}
		reply("DEFAULT OK (SAVE to persist)");
	}
	else if(strcmp(line, "EXIT") == 0)
	{
		g_session = 0U;
		reply("BYE");
	}
	else if((strcmp(line, "BLE") == 0) && (strncmp(arg, "ON", 2) == 0))
	{
		g_report = 1U;
		reply("BLE ON");
	}
	else if((strcmp(line, "BLE") == 0) && (strncmp(arg, "OFF", 3) == 0))
	{
		g_report = 0U;
		reply("BLE OFF");
	}
	else if(strcmp(line, "HELP") == 0)
	{
		reply("STAT | CAP <0-3> | SET <0-3> <k> <b> | SAVE | DEFAULT | EXIT | BLE ON|OFF");
	}
	else
	{
		reply("ERR ?HELP");
	}
}

/* ---------------- 1 Hz report ----------------------------------------------- */
static void report_send(void)
{
	uint8_t idx = g_last_idx;
	float y = CAL_Apply(idx, g_x[idx]);
	char buf[48];
	long scaled;
	char unit[4];
	char range[8];

	if((HAL_GetTick() - g_x_tick[idx]) > 2000U)
	{
		return;   /* no fresh measurement: stay silent rather than lie */
	}

	switch(idx)
	{
		case CAL_IDX_VOL_0_5:  scaled = (long)(y + (y >= 0 ? 0.5f : -0.5f)); strcpy(unit, "V"); strcpy(range, "0-5V");
			break;
		case CAL_IDX_VOL_5_15: scaled = (long)(y + (y >= 0 ? 0.5f : -0.5f)); strcpy(unit, "V"); strcpy(range, "5-15V");
			break;
		case CAL_IDX_RES_0_1:  scaled = (long)(y + 0.5f);  strcpy(unit, "R"); strcpy(range, "0-1K");
			break;
		default:               scaled = (long)(y + 0.5f);  strcpy(unit, "R"); strcpy(range, "1-100K");
			break;
	}
	(void)snprintf(buf, sizeof(buf), "$M,%s%ld,%s,%s", (y < 0) ? "-" : "", labs((long)scaled), unit, range);
	HW_UART1_Send(buf);
	HW_UART1_Send("\r\n");
}

/* ---------------- main service --------------------------------------------- */
void CAL_Service(void)
{
	uint8_t byte;

	/* drain the console */
	while(HW_UART1_GetByte(&byte))
	{
		if((byte == '\r') || (byte == '\n'))
		{
			if(g_line_len > 0U)
			{
				g_line[g_line_len] = '\0';
				g_session = 1U;   /* any valid line wakes the console session */
				g_session_tick = HAL_GetTick();
				cmd_dispatch(g_line);
				g_line_len = 0U;
			}
		}
		else if((byte >= ' ') && (byte < 0x7F) && (g_line_len < (sizeof(g_line) - 1U)))
		{
			g_line[g_line_len++] = (char)byte;
			if(g_line_len == 1U)
			{
				g_session_tick = HAL_GetTick();
			}
		}
		else
		{
			g_line_len = 0U;   /* control char / overflow: drop the line */
		}
	}

	/* idle session auto-exit (banner only, harmless if left on) */
	if(g_session && ((HAL_GetTick() - g_session_tick) > 120000U))
	{
		g_session = 0U;
	}

	/* 1 Hz BLE report */
	if(g_report && ((HAL_GetTick() - g_report_tick) >= 1000U))
	{
		g_report_tick = HAL_GetTick();
		report_send();
	}
}

void CAL_Init(void)
{
	HW_UART1_Init();
	g_src_flash = cal_store_load();
}
