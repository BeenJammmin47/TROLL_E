/*
 * troll_e.c -- TROLL-E following cart (ECE 445)
 * Target: STM32G474RE (NUCLEO-G474RE now, STM32G474RET3 on the custom PCB)
 *
 * Sensors:  ultrasonic (RCWL-1601, 1 now / up to 3 later)
 *           RPLIDAR C1 (360 deg scanner, UART 460800, standard SCAN mode)
 *           4 half-bridge load sensors -> combinator -> HX711
 *           (after demo) 2x MaUWB DW3000 for locating the user's tag
 * Actuator: 4 ESCs driven by 50 Hz servo-style PWM (1000-2000 us), all same speed for now
 *
 * ---------------------------------------------------------------------------
 * CubeMX setup. Pins and labels match the TROLL-E PCB (trollemcuREV6, final),
 * so the same code runs on the Nucleo and the custom board.
 *
 * NOW (progress demo -- one of each sensor):
 *   TIM2      internal clock, Prescaler 169, Period 4294967295 -> 1 MHz microsecond counter
 *   TIM3      CH1 PA6, CH2 PA7, CH3 PB0, CH4 PB1, all PWM Generation,
 *             Prescaler 169, Period 19999                      -> ESCs (M1..M4_PWM_A)
 *   USART3    PC10 TX / PC11 RX, 460800 8N1, RX DMA Circular,
 *             USART3 global interrupt ON                       -> lidar (PCB header J12)
 *   LPUART1   PA2 / PA3, 115200 8N1                            -> debug terminal (ST-LINK USB)
 *   GPIO labels (= PCB net names):
 *     US_TRIG_C  PC1 output        US_ECHO_C  PA1 input        (center ultrasonic, J21)
 *     HX711_SCK  PA4 output        HX711_DOUT PC3 input, pull-up (J23)
 *
 * AFTER THE DEMO:
 *     US_TRIG_L  PC0 output        US_ECHO_L  PA0 input        (left ultrasonic, J20)
 *     US_TRIG_R  PC2 output        US_ECHO_R  PD2 input        (right ultrasonic, J22)
 *   USART1    PA9 TX / PA10 RX                                 -> UWB 1 (J10)
 *   USART2    PCB: PA2 / PA3 (J11)                             -> UWB 2
 *             Nucleo: PB3 / PB4 instead, because PA2/PA3 are the ST-LINK terminal there
 *   On the PCB, LPUART1 is not available for debug (PA2/PA3 are UWB 2) -- plan on
 *   a USB virtual COM port on PA11/PA12 and point DEBUG_UART/uprintf at it.
 *
 *   Project setting: enable "Use float with printf from newlib-nano (-u _printf_float)"
 *   or every %f below prints blank.
 * ---------------------------------------------------------------------------
 *
 * Serial commands (type in the terminal, press Enter):
 *   t          tare -- cart empty
 *   c 25.0     calibrate with a known weight in lb on the platform (number optional)
 *   s          save tare + calibration to flash (survives power-off)
 *   n          noise test: 20 readings, prints spread in counts and lb
 *   v          toggle printing every raw HX711 reading
 *   p          print current calibration
 *   ?          help
 */

#include "main.h"
#include "troll_e.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart3;

/* Which UART does what -- change here if the wiring moves */
#define DEBUG_UART  hlpuart1   /* terminal for prints and t/c/s commands */
#define LIDAR_UART  huart3     /* RPLIDAR C1 on J12 */

/* ======================= Feature switches ======================= */
#define MOTOR_ENABLED        0   /* 0 = compute + print the speed decision, but keep the ESC at STOP */
#define LIDAR_ENABLED        1
#define LOAD_LIMITS_ENABLED  1   /* load cell caps speed / stops on overload */
#define BOOT_TARE            1   /* 1 = tare at power-up (cart must be empty); 0 = use offset saved in flash */

/* ======================= Distance zones (cm) ======================= */
#define STOP_DIST_CM     40.0f
#define SLOW_DIST_CM     90.0f
#define HYST_CM           5.0f   /* must exceed a threshold by this much to speed back up */

/* ======================= ESC pulse widths (us) -- TUNE ======================= */
#define STOP_US          1000    /* ESC minimum = motor off */
#define START_US         1125    /* motor starts spinning here; ramps begin here, not at 1000 */
#define SLOW_US          1150    /* ~1 mph */
#define FAST_US          1300    /* ~2 mph */
#define NO_TARGET_US     STOP_US /* nothing seen by any sensor -> stop (was FAST in the old code) */
#define RAMP_US_PER_S    100     /* acceleration limit, empty cart */
#define RAMP_US_PER_S_HEAVY 50   /* acceleration limit above LOAD_HEAVY_LB */

/* ======================= Ultrasonic ======================= */
#define US_TIMEOUT_US     25000  /* ~4.3 m */
#define US_PING_PERIOD_MS 60     /* gap between pings so echoes die out (round-robin across sensors) */
#define US_STALE_MS       300    /* reading older than this is ignored */

/* ======================= Lidar (RPLIDAR C1) ======================= */
#define LIDAR_FWD_DEG       0.0f   /* lidar angle that points straight ahead of the cart (0 = lidar's own front) */
#define LIDAR_HALF_FOV_DEG 15.0f   /* look +/- this many degrees around forward */
#define LIDAR_MIN_MM        80     /* ignore closer returns (cart's own frame, blind zone is 50 mm) */
#define LIDAR_MAX_MM        12000
#define LIDAR_MIN_QUALITY   0      /* raise if you see junk points */
#define LIDAR_STALE_MS      500    /* no complete revolution for this long -> lidar ignored */
#define LIDAR_RESTART_MS    3000   /* no data for this long -> RESET and restart scan */
#define LIDAR_RX_SIZE       2048   /* DMA ring buffer; ~80 ms of data at 5000 samples/s */

/* ======================= Load cell ======================= */
#define DEFAULT_COUNTS_PER_LB 33.1f  /* used only until a calibration is saved to flash */
#define KNOWN_WEIGHT_LB       1.36f  /* default for "c" with no number typed */
#define CAL_SAMPLES           20     /* readings averaged for tare / calibrate (~2 s at 10 SPS) */
#define LOAD_FILTER_N         5      /* moving average for the live weight */
#define LOAD_HEAVY_LB         40.0f  /* TUNE: above this -> cap at SLOW and accelerate gently */
#define LOAD_MAX_LB           80.0f  /* TUNE: above this -> refuse to drive */
#define LOAD_STALE_MS         1000   /* no HX711 reading this long -> treat load as unknown */
#define HX_SAMPLE_TIMEOUT_MS  300    /* HX711 at 10 SPS should be ready within ~100 ms */

/* ======================= Timing ======================= */
#define CONTROL_PERIOD_MS  20        /* 50 Hz, matches the ESC frame rate */
#define STATUS_PERIOD_MS   250

/* ======================= Flash storage for calibration ======================= */
/* STM32G474RE (RET6 and RET3 alike) has 512 KB flash. We use the very last page:
 *   dual-bank mode (factory default): bank 2, page 127, 2 KB -> 0x0807F800
 *   single-bank mode:                  page 127,        4 KB -> 0x0807F000
 * Shrink FLASH LENGTH in the linker script to 508K so code can never land there. */
#define FLASH_END_ADDR   0x08080000UL
#define CAL_MAGIC        0x454C5254UL  /* "TRLE" */

/* ====================================================================== */

static char msg[160];

static void uprintf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}

/* ---------- Timing ---------- */
static inline uint32_t micros(void) { return __HAL_TIM_GET_COUNTER(&htim2); }

static void delay_us(uint32_t us)
{
  uint32_t start = micros();
  while ((micros() - start) < us) {}
}

/* ---------- ESC ---------- */
static int32_t currentUs = STOP_US;
static int32_t rampAcc = 0;

/* All four ESCs get the same command for now (straight-line following, no steering) */
static const uint32_t escChannels[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };

static void escWriteUs(int32_t us)
{
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
#if !MOTOR_ENABLED
  us = STOP_US;
#endif
  for (int i = 0; i < 4; i++) {
    __HAL_TIM_SET_COMPARE(&htim3, escChannels[i], (uint32_t)us);
  }
}

static void motorStopNow(void)
{
  currentUs = STOP_US;
  rampAcc = 0;
  escWriteUs(STOP_US);
}

/* ======================================================================
 * Ultrasonic
 * ====================================================================== */
typedef struct {
  GPIO_TypeDef *trigPort; uint16_t trigPin;
  GPIO_TypeDef *echoPort; uint16_t echoPin;
  const char   *name;
} UsSensor;

static const UsSensor usSensors[] = {
  { US_TRIG_C_GPIO_Port, US_TRIG_C_Pin, US_ECHO_C_GPIO_Port, US_ECHO_C_Pin, "US_C" },
  /* After the demo: add the labels in CubeMX, regenerate, then uncomment:
  { US_TRIG_L_GPIO_Port, US_TRIG_L_Pin, US_ECHO_L_GPIO_Port, US_ECHO_L_Pin, "US_L" },
  { US_TRIG_R_GPIO_Port, US_TRIG_R_Pin, US_ECHO_R_GPIO_Port, US_ECHO_R_Pin, "US_R" }, */
};
#define US_COUNT ((int)(sizeof(usSensors) / sizeof(usSensors[0])))

static float    usCm[US_COUNT];   /* last reading, -1 = no echo */
static uint32_t usMs[US_COUNT];   /* HAL tick of last reading   */
static int      usNext = 0;

/* Length of the HIGH pulse in us, or 0 on timeout */
static uint32_t pulseInHigh(GPIO_TypeDef *port, uint16_t pin, uint32_t timeout_us)
{
  uint32_t start = micros();
  while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)   { if (micros() - start > timeout_us) return 0; }
  while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) { if (micros() - start > timeout_us) return 0; }
  uint32_t pulseStart = micros();
  while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)   { if (micros() - start > timeout_us) return 0; }
  return micros() - pulseStart;
}

static float usReadCm(const UsSensor *s)
{
  HAL_GPIO_WritePin(s->trigPort, s->trigPin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(s->trigPort, s->trigPin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(s->trigPort, s->trigPin, GPIO_PIN_RESET);

  uint32_t duration = pulseInHigh(s->echoPort, s->echoPin, US_TIMEOUT_US);
  if (duration == 0) return -1.0f;
  return (float)duration / 58.0f;
}

/* Fire ONE sensor per call, round-robin, so two sensors never hear each other */
static void usService(uint32_t now)
{
  static uint32_t lastPing = 0;
  if (now - lastPing < US_PING_PERIOD_MS) return;
  lastPing = now;

  usCm[usNext] = usReadCm(&usSensors[usNext]);
  usMs[usNext] = HAL_GetTick();
  usNext = (usNext + 1) % US_COUNT;
}

/* ======================================================================
 * RPLIDAR C1 -- standard SCAN mode, parsed in the UART DMA callback
 *   request  A5 20, descriptor A5 5A 05 00 00 40 81, then 5-byte samples:
 *   b0 = quality<<2 | !S<<1 | S     b1 = angle_q6[6:0]<<1 | C(=1)
 *   b2 = angle_q6[14:7]             b3,b4 = distance_q2 (little endian)
 *   angle = angle_q6/64 deg (clockwise), distance = distance_q2/4 mm
 * ====================================================================== */
#define LIDAR_FWD_Q6       ((int32_t)(LIDAR_FWD_DEG * 64.0f))
#define LIDAR_HALF_FOV_Q6  ((int32_t)(LIDAR_HALF_FOV_DEG * 64.0f))

static uint8_t           lidarRx[LIDAR_RX_SIZE];
static uint16_t          lidarRxOld = 0;
static volatile uint16_t lidarFrontMm = 0;   /* closest point in the forward sector last revolution, 0 = none */
static volatile uint32_t lidarRevMs = 0;     /* tick of last completed revolution */
static volatile uint32_t lidarRevCount = 0;
static uint32_t          lidarStartMs = 0;
static uint8_t           lidarResetPending = 0;
static uint32_t          lidarResetMs = 0;

/* parser state -- touched only by the ISR except during (re)start */
static uint8_t  lpState = 0;           /* 0 = waiting for descriptor, 1 = receiving samples */
static uint8_t  lpBuf[7];
static uint8_t  lpLen = 0;
static uint16_t lpMinMm = 0xFFFF;
static const uint8_t LIDAR_DESC[7] = { 0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81 };

static void lidarByte(uint8_t b)
{
  if (lpState == 0) {                                 /* hunt for the response descriptor */
    if (b == LIDAR_DESC[lpLen]) {
      if (++lpLen == sizeof(LIDAR_DESC)) { lpState = 1; lpLen = 0; }
    } else {
      lpLen = (b == LIDAR_DESC[0]) ? 1 : 0;
    }
    return;
  }

  lpBuf[lpLen++] = b;
  if (lpLen < 5) return;

  uint8_t s    = lpBuf[0] & 0x01;
  uint8_t sInv = (lpBuf[0] >> 1) & 0x01;
  uint8_t c    = lpBuf[1] & 0x01;
  if (s == sInv || c != 1) {                          /* misaligned: slide one byte and retry */
    memmove(lpBuf, lpBuf + 1, 4);
    lpLen = 4;
    return;
  }
  lpLen = 0;

  if (s) {                                            /* start of a new 360 deg revolution */
    lidarFrontMm = (lpMinMm == 0xFFFF) ? 0 : lpMinMm;
    lidarRevMs = HAL_GetTick();
    lidarRevCount++;
    lpMinMm = 0xFFFF;
  }

  uint8_t  quality = lpBuf[0] >> 2;
  uint16_t angleQ6 = (uint16_t)((lpBuf[1] >> 1) | ((uint16_t)lpBuf[2] << 7));
  uint16_t distQ2  = (uint16_t)(lpBuf[3] | ((uint16_t)lpBuf[4] << 8));
  if (distQ2 == 0) return;                                    /* 0 = invalid measurement */
#if LIDAR_MIN_QUALITY > 0
  if (quality < LIDAR_MIN_QUALITY) return;
#else
  (void)quality;
#endif

  uint16_t mm = distQ2 >> 2;
  if (mm < LIDAR_MIN_MM || mm > LIDAR_MAX_MM) return;

  int32_t rel = (int32_t)angleQ6 - LIDAR_FWD_Q6;
  while (rel >=  180 * 64) rel -= 360 * 64;
  while (rel <  -180 * 64) rel += 360 * 64;
  if (rel < 0) rel = -rel;
  if (rel <= LIDAR_HALF_FOV_Q6 && mm < lpMinMm) lpMinMm = mm;
}

static void lidarParse(const uint8_t *p, uint16_t n)
{
  while (n--) lidarByte(*p++);
}

static void lidarRxStart(void)
{
  HAL_UART_AbortReceive(&LIDAR_UART);
  lidarRxOld = 0;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&LIDAR_UART, lidarRx, LIDAR_RX_SIZE) != HAL_OK) {
    uprintf("LIDAR: could not start UART DMA (check USART3 RX DMA = Circular)\r\n");
  }
}

static void lidarSend(uint8_t cmd)
{
  uint8_t pkt[2] = { 0xA5, cmd };
  HAL_UART_Transmit(&LIDAR_UART, pkt, 2, 10);
}

static void lidarStartScan(void)
{
  lidarSend(0x25);                 /* STOP -- harmless if already idle */
  HAL_Delay(10);                   /* protocol: wait >= 10 ms after STOP */
  __disable_irq();
  lpState = 0; lpLen = 0; lpMinMm = 0xFFFF;
  __enable_irq();
  lidarSend(0x20);                 /* SCAN -- motor spins up, data starts once speed is stable */
  lidarStartMs = HAL_GetTick();
}

/* Watchdog: if no revolutions arrive, RESET the lidar (non-blocking) and start again */
static void lidarService(uint32_t now)
{
#if LIDAR_ENABLED
  if (lidarResetPending) {
    if (now - lidarResetMs >= 600) {          /* protocol: wait >= 500 ms after RESET */
      lidarResetPending = 0;
      lidarStartScan();
    }
    return;
  }
  if (now - lidarStartMs > LIDAR_RESTART_MS && now - lidarRevMs > LIDAR_RESTART_MS) {
    uprintf("LIDAR: no data -- resetting (check 5 V supply >= 800 mA and TX/RX crossover)\r\n");
    lidarSend(0x40);                          /* RESET */
    lidarResetPending = 1;
    lidarResetMs = now;
  }
#else
  (void)now;
#endif
}

/* Distance to nearest thing in the forward sector, cm; -1 = nothing seen / no data */
static float lidarFrontCm(uint32_t now, int *alive)
{
  *alive = (lidarRevCount > 0) && (now - lidarRevMs < LIDAR_STALE_MS);
  if (!*alive || lidarFrontMm == 0) return -1.0f;
  return lidarFrontMm / 10.0f;
}

/* ---- HAL callbacks (override the weak versions) ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart != &LIDAR_UART) return;
  if (Size != lidarRxOld) {
    if (Size > lidarRxOld) {
      lidarParse(&lidarRx[lidarRxOld], Size - lidarRxOld);
    } else {                                   /* wrapped around the ring */
      lidarParse(&lidarRx[lidarRxOld], LIDAR_RX_SIZE - lidarRxOld);
      lidarParse(&lidarRx[0], Size);
    }
    lidarRxOld = Size;
  }
  if (lidarRxOld >= LIDAR_RX_SIZE) lidarRxOld = 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &LIDAR_UART) lidarRxStart();       /* overrun/noise: restart reception, parser resyncs */
}

/* ======================================================================
 * HX711 load cell
 * ====================================================================== */
static int32_t  hxOffset = 0;
static float    hxCountsPerLb = DEFAULT_COUNTS_PER_LB;
static float    lbHist[LOAD_FILTER_N];
static uint8_t  lbIdx = 0, lbFill = 0;
static float    loadLb = 0.0f;
static uint32_t loadMs = 0;
static int32_t  lastRaw = 0;
static uint8_t  hxVerbose = 0;

static int hxReady(void)
{
  return HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_RESET;
}

/* Clock out one 24-bit reading. Interrupts are held off for ~110 us because a
 * clock-high period > 60 us puts the HX711 to sleep (and the lidar ISR could
 * otherwise land in the middle). */
static int32_t hxReadRaw(void)
{
  uint32_t value = 0;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (int i = 0; i < 24; i++) {
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
    delay_us(2);
    value = (value << 1) | (HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_Pin) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
    delay_us(2);
  }
  /* 25th pulse: channel A, gain 128 for the next conversion */
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
  delay_us(2);
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);

  if (!primask) __enable_irq();

  if (value & 0x800000UL) value |= 0xFF000000UL;   /* sign-extend 24 -> 32 bits */
  return (int32_t)value;
}

/* Wait (with timeout) for one fresh reading. Returns 1 on success. */
static int hxWaitRead(int32_t *out)
{
  uint32_t t0 = HAL_GetTick();
  while (!hxReady()) {
    if (HAL_GetTick() - t0 > HX_SAMPLE_TIMEOUT_MS) return 0;
  }
  *out = hxReadRaw();
  return 1;
}

/* Average n readings. Returns 1 on success, 0 if the HX711 stopped answering. */
static int hxAverage(int n, int32_t *out)
{
  int64_t sum = 0;
  for (int i = 0; i < n; i++) {
    int32_t r;
    if (!hxWaitRead(&r)) return 0;
    sum += r;
  }
  *out = (int32_t)(sum / n);
  return 1;
}

static void loadFilterReset(void) { lbIdx = 0; lbFill = 0; }

static void loadService(uint32_t now)
{
  if (!hxReady()) return;
  lastRaw = hxReadRaw();
  float lb = (float)(lastRaw - hxOffset) / hxCountsPerLb;

  lbHist[lbIdx] = lb;
  lbIdx = (uint8_t)((lbIdx + 1) % LOAD_FILTER_N);
  if (lbFill < LOAD_FILTER_N) lbFill++;
  float sum = 0.0f;
  for (int i = 0; i < lbFill; i++) sum += lbHist[i];
  loadLb = sum / lbFill;
  loadMs = now;

  if (hxVerbose) {
    uprintf("HX raw %ld  net %ld  %.2f lb\r\n", (long)lastRaw, (long)(lastRaw - hxOffset), lb);
  }
}

/* ---------- Calibration stored in flash ---------- */
typedef struct {
  uint32_t magic;
  int32_t  offset;
  float    countsPerLb;
  uint32_t check;
} CalData;   /* 16 bytes = two 64-bit flash words */

static uint32_t calCheck(const CalData *c)
{
  uint32_t f;
  memcpy(&f, &c->countsPerLb, sizeof f);
  return c->magic ^ (uint32_t)c->offset ^ f ^ 0xA5A5A5A5UL;
}

static int calDualBank(void)
{
#if defined(FLASH_OPTR_DBANK)
  return (FLASH->OPTR & FLASH_OPTR_DBANK) != 0U;
#else
  return 0;
#endif
}

static uint32_t calAddr(void)
{
  return calDualBank() ? (FLASH_END_ADDR - 0x800UL) : (FLASH_END_ADDR - 0x1000UL);
}

static int calLoad(CalData *c)
{
  memcpy(c, (const void *)calAddr(), sizeof *c);
  return c->magic == CAL_MAGIC && c->check == calCheck(c) &&
         isfinite(c->countsPerLb) && fabsf(c->countsPerLb) > 0.001f;
}

static int calSave(void)
{
  CalData c = { CAL_MAGIC, hxOffset, hxCountsPerLb, 0 };
  c.check = calCheck(&c);
  uint64_t dw[2];
  memcpy(dw, &c, sizeof dw);

  FLASH_EraseInitTypeDef er = {0};
  uint32_t pageErr = 0;
  er.TypeErase = FLASH_TYPEERASE_PAGES;
  er.Banks     = calDualBank() ? FLASH_BANK_2 : FLASH_BANK_1;
  er.Page      = 127;
  er.NbPages   = 1;

  uint32_t addr = calAddr();
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &pageErr);
  for (int i = 0; i < 2 && st == HAL_OK; i++) {
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8U * (uint32_t)i, dw[i]);
  }
  HAL_FLASH_Lock();

  CalData back;
  return st == HAL_OK && calLoad(&back) && back.offset == hxOffset;
}

/* ---------- Calibration commands ---------- */
static void cmdTare(void)
{
  motorStopNow();
  uprintf("Taring... keep the cart EMPTY and still\r\n");
  int32_t avg;
  if (!hxAverage(CAL_SAMPLES, &avg)) { uprintf("HX711 not responding -- check CLK/DATA wiring and power\r\n"); return; }
  hxOffset = avg;
  loadFilterReset();
  uprintf("Offset = %ld   (type s to save)\r\n", (long)hxOffset);
}

static void cmdCalibrate(float knownLb)
{
  if (!(knownLb > 0.0f)) knownLb = KNOWN_WEIGHT_LB;
  motorStopNow();
  uprintf("Calibrating with %.2f lb -- weight centered, keep still\r\n", knownLb);
  int32_t avg;
  if (!hxAverage(CAL_SAMPLES, &avg)) { uprintf("HX711 not responding\r\n"); return; }
  int32_t net = avg - hxOffset;
  if (labs((long)net) < 20) {
    uprintf("Only %ld counts of change -- is the weight on? Did you tare first?\r\n", (long)net);
    return;
  }
  hxCountsPerLb = (float)net / knownLb;
  loadFilterReset();
  uprintf("COUNTS_PER_LB = %.2f   (type s to save)\r\n", hxCountsPerLb);
  if (labs((long)net) < 2000) {
    uprintf("Warning: only %ld counts of change. Use a heavier weight (10-50 lb)\r\n"
            "and run 'n' -- if noise is close to this, the factor will be poor.\r\n", (long)net);
  }
}

static void cmdNoise(void)
{
  motorStopNow();
  int32_t r[20];
  for (int i = 0; i < 20; i++) {
    if (!hxWaitRead(&r[i])) { uprintf("HX711 not responding\r\n"); return; }
  }
  double mean = 0, var = 0;
  int32_t mn = r[0], mx = r[0];
  for (int i = 0; i < 20; i++) { mean += r[i]; if (r[i] < mn) mn = r[i]; if (r[i] > mx) mx = r[i]; }
  mean /= 20;
  for (int i = 0; i < 20; i++) var += (r[i] - mean) * (r[i] - mean);
  float sd = (float)sqrt(var / 19);
  uprintf("Noise: std %.1f counts (%.3f lb), p-p %ld counts (%.3f lb)\r\n",
          sd, sd / fabsf(hxCountsPerLb), (long)(mx - mn), (float)(mx - mn) / fabsf(hxCountsPerLb));
}

static void printHelp(void)
{
  uprintf("Commands: t=tare  c <lb>=calibrate  s=save  n=noise  v=raw on/off  p=print  ?=help\r\n");
}

static void handleLine(char *line)
{
  while (*line == ' ') line++;
  char cmd = *line;
  if (cmd >= 'A' && cmd <= 'Z') cmd = (char)(cmd + 32);

  switch (cmd) {
    case 't': cmdTare(); break;
    case 'c': cmdCalibrate(strtof(line + 1, NULL)); break;
    case 's':
      motorStopNow();
      uprintf(calSave() ? "Saved to flash.\r\n" : "Flash save FAILED.\r\n");
      break;
    case 'n': cmdNoise(); break;
    case 'v': hxVerbose = !hxVerbose; uprintf("Raw printing %s\r\n", hxVerbose ? "ON" : "OFF"); break;
    case 'p': uprintf("Offset %ld   COUNTS_PER_LB %.2f\r\n", (long)hxOffset, hxCountsPerLb); break;
    case '?': printHelp(); break;
    case 0: break;
    default: uprintf("Unknown command\r\n"); printHelp(); break;
  }
}

/* Non-blocking line reader on the debug UART */
static void serialService(void)
{
  static char line[32];
  static uint8_t len = 0;

  if (__HAL_UART_GET_FLAG(&DEBUG_UART, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&DEBUG_UART);
  while (__HAL_UART_GET_FLAG(&DEBUG_UART, UART_FLAG_RXNE)) {
    uint8_t ch = (uint8_t)(DEBUG_UART.Instance->RDR & 0xFFU);
    if (ch == '\r' || ch == '\n') {
      if (len > 0) {
        uprintf("\r\n");
        line[len] = 0;
        len = 0;
        handleLine(line);
      }
    } else if ((ch == 8 || ch == 127) && len > 0) {   /* backspace */
      len--;
    } else if (len < sizeof(line) - 1 && ch >= 32) {
      line[len++] = (char)ch;
      HAL_UART_Transmit(&DEBUG_UART, &ch, 1, 5);       /* echo */
    }
  }
}

/* ======================================================================
 * Speed decision
 * ====================================================================== */
static int         zone = 0;            /* 0 stop, 1 slow, 2 fast */
static float       fusedCm = -1.0f;
static int32_t     targetUs = STOP_US;
static const char *limitReason = "";

/* Closest valid reading from every forward-facing sensor; -1 if none sees anything */
static float fuseDistance(uint32_t now)
{
  float best = -1.0f;
  for (int i = 0; i < US_COUNT; i++) {
    if (now - usMs[i] < US_STALE_MS && usCm[i] > 0.0f) {
      if (best < 0.0f || usCm[i] < best) best = usCm[i];
    }
  }
#if LIDAR_ENABLED
  int alive;
  float l = lidarFrontCm(now, &alive);
  if (l > 0.0f && (best < 0.0f || l < best)) best = l;
#endif
  return best;
}

static void controlService(uint32_t now)
{
  static uint32_t last = 0;
  uint32_t dt = now - last;
  if (dt < CONTROL_PERIOD_MS) return;
  last = now;
  if (dt > 200) dt = 200;   /* after a blocking command, don't take one giant ramp step */

  /* 1. Distance -> zone, with hysteresis so it doesn't chatter at a boundary */
  fusedCm = fuseDistance(now);
  if (fusedCm < 0.0f) {
    zone = 0;
  } else {
    switch (zone) {
      case 0:
        if (fusedCm >= SLOW_DIST_CM + HYST_CM)      zone = 2;
        else if (fusedCm >= STOP_DIST_CM + HYST_CM) zone = 1;
        break;
      case 1:
        if (fusedCm < STOP_DIST_CM)                 zone = 0;
        else if (fusedCm >= SLOW_DIST_CM + HYST_CM) zone = 2;
        break;
      default:
        if (fusedCm < STOP_DIST_CM)                 zone = 0;
        else if (fusedCm < SLOW_DIST_CM)            zone = 1;
        break;
    }
  }
  targetUs = (fusedCm < 0.0f) ? NO_TARGET_US : (zone == 2 ? FAST_US : zone == 1 ? SLOW_US : STOP_US);
  limitReason = (fusedCm < 0.0f) ? "no target" : "";

  /* 2. Load cell caps the speed */
  int32_t ramp = RAMP_US_PER_S;
#if LOAD_LIMITS_ENABLED
  if (now - loadMs > LOAD_STALE_MS) {
    if (targetUs > SLOW_US) { targetUs = SLOW_US; limitReason = "load unknown"; }
  } else if (loadLb > LOAD_MAX_LB) {
    targetUs = STOP_US; limitReason = "OVERLOAD";
  } else if (loadLb > LOAD_HEAVY_LB) {
    if (targetUs > SLOW_US) { targetUs = SLOW_US; limitReason = "heavy"; }
    ramp = RAMP_US_PER_S_HEAVY;
  }
#endif

  /* 3. Slow down / stop immediately, speed up gradually */
  if (targetUs <= currentUs) {
    currentUs = targetUs;
    rampAcc = 0;
  } else {
    if (currentUs < START_US) currentUs = (targetUs < START_US) ? targetUs : START_US;  /* skip dead band */
    rampAcc += ramp * (int32_t)dt;
    currentUs += rampAcc / 1000;
    rampAcc %= 1000;
    if (currentUs > targetUs) currentUs = targetUs;
  }
  escWriteUs(currentUs);
}

/* ======================================================================
 * Status line
 * ====================================================================== */
static void statusService(uint32_t now)
{
  static uint32_t last = 0;
  if (now - last < STATUS_PERIOD_MS || hxVerbose) return;
  last = now;

  char part[96];
  int n = 0;
  for (int i = 0; i < US_COUNT; i++) {
    if (now - usMs[i] >= US_STALE_MS)  n += snprintf(part + n, sizeof(part) - n, "%s stale  ", usSensors[i].name);
    else if (usCm[i] < 0.0f)           n += snprintf(part + n, sizeof(part) - n, "%s --  ", usSensors[i].name);
    else                               n += snprintf(part + n, sizeof(part) - n, "%s %.0f  ", usSensors[i].name, usCm[i]);
  }
#if LIDAR_ENABLED
  int alive;
  float l = lidarFrontCm(now, &alive);
  if (!alive)        n += snprintf(part + n, sizeof(part) - n, "LID off  ");
  else if (l < 0.0f) n += snprintf(part + n, sizeof(part) - n, "LID --  ");
  else               n += snprintf(part + n, sizeof(part) - n, "LID %.0f  ", l);
#endif
  if (fusedCm < 0.0f) snprintf(part + n, sizeof(part) - n, "-> none");
  else                snprintf(part + n, sizeof(part) - n, "-> %.0f cm", fusedCm);

  uprintf("%s | %.1f lb | ESC %ld%s %s\r\n",
          part, loadLb, (long)currentUs,
          MOTOR_ENABLED ? "" : "(off)", limitReason);
}

/* ======================================================================
 * Public entry points
 * ====================================================================== */
void TrollE_Init(void)
{
  HAL_TIM_Base_Start(&htim2);                     /* 1 MHz microsecond counter */
  for (int i = 0; i < 4; i++) {
    HAL_TIM_PWM_Start(&htim3, escChannels[i]);    /* 50 Hz ESC signals */
  }
  motorStopNow();                                 /* arm the ESC at zero throttle */
  uint32_t armStart = HAL_GetTick();

  for (int i = 0; i < US_COUNT; i++) {
    HAL_GPIO_WritePin(usSensors[i].trigPort, usSensors[i].trigPin, GPIO_PIN_RESET);
    usCm[i] = -1.0f;
    usMs[i] = 0;
  }
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);   /* CLK low keeps HX711 awake */

  uprintf("\r\n=== TROLL-E ===  motor %s, lidar %s\r\n",
          MOTOR_ENABLED ? "ENABLED" : "disabled", LIDAR_ENABLED ? "on" : "off");

  /* Load cell: calibration from flash, then tare */
  CalData cal;
  if (calLoad(&cal)) {
    hxCountsPerLb = cal.countsPerLb;
    hxOffset = cal.offset;
    uprintf("Loaded calibration: COUNTS_PER_LB %.2f, offset %ld\r\n", hxCountsPerLb, (long)hxOffset);
  } else {
    uprintf("No saved calibration -- using default %.2f (run t, c <lb>, s)\r\n", hxCountsPerLb);
  }
  int32_t dummy;
  hxWaitRead(&dummy); hxWaitRead(&dummy);        /* first readings after power-up are unsettled */
#if BOOT_TARE
  cmdTare();
#endif

#if LIDAR_ENABLED
  lidarRxStart();
  lidarStartScan();
#endif

  while (HAL_GetTick() - armStart < 3000) {}      /* give the ESC time to arm */
  printHelp();
}

void TrollE_Loop(void)
{
  serialService();
  loadService(HAL_GetTick());
  usService(HAL_GetTick());
  lidarService(HAL_GetTick());
  controlService(HAL_GetTick());
  statusService(HAL_GetTick());
}
