/*
 * troll_e.c -- TROLL-E following cart (ECE 445)
 * Target: STM32G474RE (NUCLEO-G474RE now, STM32G474RET3 on the custom PCB)
 *
 * Sensors:  ultrasonic (RCWL-1601, 1 now / up to 3 later)
 *           RPLIDAR C1 (360 deg scanner, UART 460800, standard SCAN mode)
 *           4 half-bridge load sensors -> combinator -> HX711
 *           2x MaUWB DW3000 tags on the cart (left J10, right J11), user carries the anchor
 * Actuator: 4x E-S Motor 36PG-3650BL 24 V BLDC gearmotors with BUILT-IN drivers.
 *           Speed = ANALOG voltage on the blue wire (measured), direction = white wire level.
 *           PWM mode: left side (M1, M3) and right side (M2, M4) get separate speeds -> steering.
 *           DAC mode: one shared speed -> straight-line following only.
 *
 * Following / steering (see controlService):
 *   UWB  : user distance = average of the two tags, direction = left minus right ("diff",
 *          positive = user to the right). Both tags silent > 1 s -> emergency stop, resumes
 *          after both report for UWB_RESUME_MS. One tag silent -> straight, slow, ERROR LED.
 *   Front: closest of the 3 ultrasonics (all face forward) and the lidar front sector.
 *          Close + not the user (UWB distance differs by > USER_MATCH_CM) -> turn in place
 *          toward the more open side (side ultrasonic + lidar side sector); tie -> toward the user.
 *   User close and off to one side -> turn in place to face them.
 *
 * ---------------------------------------------------------------------------
 * CubeMX setup. Pins and labels match the TROLL-E PCB (trollemcuREV6, final),
 * so the same code runs on the Nucleo and the custom board.
 *
 * NOW (progress demo -- one of each sensor):
 *   TIM2      internal clock, Prescaler 169, Period 4294967295 -> 1 MHz microsecond counter
 *   Motor speed, one of two ways (picked automatically: DAC enabled in CubeMX -> DAC mode):
 *   DAC mode: DAC2 OUT1 on PA6, external pin, buffer ON -> one voltage for all four blue wires
 *   PWM mode: TIM3 CH1..CH4 on PA6 / PA7 / PB0 / PB1, PWM Generation, Prescaler 0, Period 8499
 *             -> each motor's blue wire gets its own pin (J40 pins 8 / 6 / 4 / 2)
 *                                                              -> motor speed voltage (all blue wires)
 *   TIM3      PWM mode only
 *   GPIO outputs M1_DIR_W..M4_DIR_W (or M1_DIR / M1_PWM_B)     -> motor direction (white wires)
 *             PCB: PC6..PC9.  Nucleo demo: M1_PWM_B moved to PB5 (D4), M2_PWM_B on PC7 (D9).
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
 *   USART1    PA9 TX / PA10 RX, 115200 8N1, RX DMA Circular,
 *             USART1 global interrupt ON                       -> UWB 1 (J10): cart UWB board (tag) sends "D,<cm>"
 *   USART2    PCB: PA2 TX / PA3 RX (J11), same settings as USART1  -> UWB 2 (J11): second cart tag
 *             Nucleo: PB3 / PB4 instead, because PA2/PA3 are the ST-LINK terminal there
 *   On the PCB, LPUART1 is not available (PA2/PA3 are UWB 2): the terminal runs over USB-C
 *   instead (USB Device, CDC class). The code switches automatically -- see DEBUG_OVER_USB.
 *
 * PCB (trollemcuREV6) extras, used automatically when the labels exist in CubeMX:
 *   ESTOP_SENSE PB2 (input), MOTOR_KILL_CMD PC4 (output), LED_STATUS_MCU PB10, LED_ERROR_MCU PB11.
 *   Motor direction on the PCB: M1..M4_PWM_B = PC6..PC9 (J40 pins 7,5,3,1); speed = PA6 (J40 pin 8).
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
 *   m 20       manual motor test, both sides (m -20 = reverse); a = back to automatic
 *   w          toggle printing the raw UWB lines
 *   u          UWB centre: stand straight ahead of the cart (~2 m), then type u
 *   ?          help
 */

#include "main.h"
#include "troll_e.h"

/* ---- Debug terminal: USB virtual COM port on the PCB, ST-LINK (LPUART1) on the Nucleo ----
 * Picked automatically: if CubeMX generated the USB CDC files, the terminal goes over USB-C. */
#ifndef DEBUG_OVER_USB
#  if defined(__has_include)
#    if __has_include("usbd_cdc_if.h")
#      define DEBUG_OVER_USB 1
#    endif
#  endif
#endif
#ifndef DEBUG_OVER_USB
#  define DEBUG_OVER_USB 0
#endif
#if DEBUG_OVER_USB
#  include "usbd_cdc_if.h"
#  include "usbd_cdc.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

extern TIM_HandleTypeDef  htim2;
/* Motor speed output: DAC (one shared voltage) or PWM (one pin per motor).
 * Automatic: if the DAC driver is enabled in CubeMX use the DAC, otherwise TIM3 PWM. */
#ifndef MOTOR_SPEED_PWM
#  ifdef HAL_DAC_MODULE_ENABLED
#    define MOTOR_SPEED_PWM 0
#  else
#    define MOTOR_SPEED_PWM 1
#  endif
#endif
#if MOTOR_SPEED_PWM
extern TIM_HandleTypeDef  htim3;
#else
extern DAC_HandleTypeDef  hdac2;
#endif
#if !DEBUG_OVER_USB
extern UART_HandleTypeDef hlpuart1;
#endif
extern UART_HandleTypeDef huart3;

/* Which UART does what -- change here if the wiring moves */
#define DEBUG_UART  hlpuart1   /* Nucleo terminal (only used when DEBUG_OVER_USB is 0) */
#define LIDAR_UART  huart3     /* RPLIDAR C1 on J12 */

/* ======================= Feature switches ======================= */
#define MOTOR_ENABLED        1   /* 0 = compute + print the speed decision, but keep the motors stopped */
#define LIDAR_ENABLED        1   /* RPLIDAR C1 on J12 (works on the PCB) */
#define UWB_ENABLED          1   /* cart UWB boards (tags) on J10 / USART1 and J11 / USART2 */
#define UWB_COUNT            2   /* 2 = tag 0 on J10 + tag 1 on J11; 1 = J10 only (USART2 not needed) */
#define STEER_ENABLED        1   /* steer + turn in place (needs PWM mode and 2 UWB tags) */
#define LOAD_LIMITS_ENABLED  1   /* load cell caps speed / stops on overload */
#define BOOT_TARE            1   /* 1 = tare at power-up (cart must be empty); 0 = use offset saved in flash */

/* ======================= Distance zones (cm) ======================= */
#define STOP_DIST_CM     60.0f   /* was 40 */
#define SLOW_DIST_CM    125.0f   /* was 90 */
#define HYST_CM           5.0f   /* must exceed a threshold by this much to speed back up */

/* ======================= Steering (UWB) -- TUNE ======================= */
/* diff = left tag distance - right tag distance - offset. With the tags 42 cm apart:
 * 8 cm ~ 11 deg, 15 cm ~ 21 deg, 20 cm ~ 28 deg. Positive diff = user to the right. */
#define UWB_DIFF_OFFSET_CM     0.0f   /* diff with the user dead ahead; measure with 'u', paste here */
#define UWB_FILTER_MS          1000   /* averaging time for diff (steering); was 500, diff jumped around */
#define UWB_MEDIAN_N           5      /* each tag: median of the last 5 readings, throws out single spikes */
#define UWB_RANGE_FILTER_MS    150    /* averaging time for the user distance (short: it must keep up with walking) */
#define STEER_DEADBAND_CM      8.0f   /* |diff| below this -> drive straight */
#define STEER_GAIN_PCT_PER_CM  1.0f   /* steering % per cm of |diff| beyond the deadband */
#define STEER_MAX_PCT          20     /* cap on the steering term */
#define FACE_START_CM          15.0f  /* |diff| above this: close user -> turn to face; driving -> slow down */
#define FACE_STOP_CM           8.0f   /* turning to face stops once |diff| drops below this */
#define USER_MATCH_CM          40.0f  /* a reading within this of the UWB distance = the user, not an obstacle */
#define SIDE_CLEAR_CM          50.0f  /* don't steer toward a side (side ultrasonic / lidar side sector) with
                                         something closer than this -- unless that something is the user */
#define PIVOT_FWD_PCT          15     /* turn in place: side driving forward  (~1.57 V) */
#define PIVOT_REV_PCT          25     /* turn in place: side driving backward (~1.77 V) -- stronger so the
                                         forward side can't drag it along; tune both on the ground */
#define PIVOT_CLEAR_CM         30.0f  /* obstacle turn ends once the front reads >= STOP_DIST_CM + this */
#define PIVOT_TIE_CM           15.0f  /* left/right clearance within this of each other = tie -> toward user */
#define PIVOT_MAX_MS           5000   /* obstacle turn gives up after this; waits until the front clears */
#define UWB_RESUME_MS          2000   /* after a UWB emergency stop: all tags must report this long */

/* ======================= Motors -- TUNE ======================= */
/* E-S Motor 36PG-3650BL-51, 24 V, built-in driver.
 *   red = 24 V, black = GND, blue = speed, white = CW/CCW, yellow = FG (open collector, 6 pulses/rev)
 *   MEASURED on our motors (the datasheet table is wrong for them): blue is an ANALOG speed input.
 *     0 V = stop, ~1.1 V = very slow, ~2.2 V = slower, 3.3 V / unconnected = fast.
 *   So D12 (PA6) drives the blue wires with a real voltage from DAC2, not PWM.
 *   Speed % maps linearly from MOTOR_V_MIN (just turning) to MOTOR_V_MAX; 0 % = 0 V. */
#define MOTOR_V_MIN           1.26f  /* volts where a stopped motor starts turning (measured on the PCB:
                                        old m 15; once turning it keeps going down to ~1.19 V) */
#define MOTOR_V_MAX           3.3f   /* volts at 100 % (DAC can't exceed 3.3 V) */
#define MOTOR_VREF            3.3f   /* DAC reference / PWM high level */
#define MOTOR_PWM_HZ          20000  /* PWM mode: 20 kHz. Average voltage = 3.3 V x duty */
#define START_PCT             1      /* ramps start here (MOTOR_V_MIN already skips the dead band) */
#define SLOW_PCT              10     /* ~1.46 V (was 12) -- tune with wheels on the ground */
#define FAST_PCT              30     /* ~1.87 V (was 41 = 2.1 V, felt too fast following) -- tune */
#define MAX_PCT               70     /* hard cap on any command, including manual 'm' */
#define RAMP_PCT_PER_S        40     /* acceleration limit, empty cart */
#define RAMP_PCT_PER_S_HEAVY  20     /* acceleration limit above LOAD_HEAVY_LB */
#define REVERSE_DWELL_MS      600    /* time at 0 % before the direction pin may change */

/* ======================= Ultrasonic ======================= */
#define US_TIMEOUT_US     25000  /* ~4.3 m */
#define US_PING_PERIOD_MS 60     /* gap between pings so echoes die out (round-robin across sensors) */
#define US_STALE_MS       300    /* reading older than this is ignored */

/* ======================= Lidar (RPLIDAR C1) ======================= */
#define LIDAR_FWD_DEG       0.0f   /* lidar angle that points straight ahead of the cart (0 = lidar's own front) */
#define LIDAR_FRONT_HALF_DEG 15.0f /* front sector +/- this: obstacle / stop check */
#define LIDAR_SIDE_MAX_DEG 60.0f   /* side sectors 15..60 deg left and right (120 deg total) */
#define LIDAR_CW           1       /* 1 = angles increase clockwise seen from above (RPLIDAR). Flip if L/R swap */
#define LIDAR_MIN_MM        80     /* ignore closer returns (cart's own frame, blind zone is 50 mm) */
#define LIDAR_MAX_MM        12000
#define LIDAR_MIN_QUALITY   0      /* raise if you see junk points */
#define LIDAR_STALE_MS      500    /* no complete revolution for this long -> lidar ignored */
#define LIDAR_RESTART_MS    3000   /* no data for this long -> RESET and restart scan */
#define LIDAR_RX_SIZE       2048   /* DMA ring buffer; ~80 ms of data at 5000 samples/s */

/* ======================= UWB (MaUWB tag on the cart, user holds anchor 0) ======================= */
#define UWB_STALE_MS        1000   /* no reading for this long -> UWB ignored */
#define UWB_MAX_CM          3000   /* ignore readings above this */
#define UWB_RX_SIZE         256

/* ======================= Load cell ======================= */
#define DEFAULT_COUNTS_PER_LB 33.1f  /* used only until a calibration is saved to flash */
#define KNOWN_WEIGHT_LB       1.36f  /* default for "c" with no number typed */
#define CAL_SAMPLES           20     /* readings averaged for tare / calibrate (~2 s at 10 SPS) */
#define LOAD_FILTER_N         5      /* moving average for the live weight */
#define LOAD_HEAVY_LB         25.0f  /* same as LOAD_MAX_LB = no separate "heavy" mode for now */
#define LOAD_MAX_LB           25.0f  /* above this -> STOP (raise later) */
#define LOAD_STALE_MS         1000   /* no HX711 reading this long -> treat load as unknown */
#define HX_SAMPLE_TIMEOUT_MS  300    /* HX711 at 10 SPS should be ready within ~100 ms */

/* ======================= Timing ======================= */
#define CONTROL_PERIOD_MS  20        /* speed update rate, 50 Hz */
#define STATUS_PERIOD_MS   500       /* status line paused while you type a command */

/* ======================= Flash storage for calibration ======================= */
/* STM32G474RE (RET6 and RET3 alike) has 512 KB flash. We use the very last page:
 *   dual-bank mode (factory default): bank 2, page 127, 2 KB -> 0x0807F800
 *   single-bank mode:                  page 127,        4 KB -> 0x0807F000
 * Shrink FLASH LENGTH in the linker script to 508K so code can never land there. */
#define FLASH_END_ADDR   0x08080000UL
#define CAL_MAGIC        0x454C5254UL  /* "TRLE" */

/* ====================================================================== */

static char msg[256];

#if DEBUG_OVER_USB
/* Bytes typed in the USB terminal, filled from the USB interrupt (TrollE_UsbRx) */
static volatile uint8_t usbRx[128];
static volatile uint8_t usbRxHead = 0, usbRxTail = 0;

void TrollE_UsbRx(const uint8_t *buf, uint32_t len)
{
  while (len--) {
    uint8_t next = (uint8_t)((usbRxHead + 1) % sizeof(usbRx));
    if (next == usbRxTail) return;               /* full: drop */
    usbRx[usbRxHead] = *buf++;
    usbRxHead = next;
  }
}

static int usbGetChar(uint8_t *ch)
{
  if (usbRxTail == usbRxHead) return 0;
  *ch = usbRx[usbRxTail];
  usbRxTail = (uint8_t)((usbRxTail + 1) % sizeof(usbRx));
  return 1;
}

/* Send over USB; waits (max ~20 ms) for the previous packet. Dropped if no terminal is open. */
static void debugWrite(const uint8_t *data, uint16_t len)
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  static uint8_t tx[sizeof(msg)];
  if (len > sizeof(tx)) len = sizeof(tx);
  uint32_t t0 = HAL_GetTick();
  USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  while (cdc != NULL && cdc->TxState != 0U) { if (HAL_GetTick() - t0 > 20) return; }
  memcpy(tx, data, len);
  while (CDC_Transmit_FS(tx, len) == USBD_BUSY) { if (HAL_GetTick() - t0 > 20) return; }
}
#else
static void debugWrite(const uint8_t *data, uint16_t len)
{
  HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)data, len, HAL_MAX_DELAY);
}
#endif

static void uprintf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  debugWrite((const uint8_t *)msg, (uint16_t)strlen(msg));
}

/* ---------- Timing ---------- */
static inline uint32_t micros(void) { return __HAL_TIM_GET_COUNTER(&htim2); }

static void delay_us(uint32_t us)
{
  uint32_t start = micros();
  while ((micros() - start) < us) {}
}

/* ---------- Motors ---------- */
/* Order on the PCB: M1, M2, M3, M4 -- assumed front-left, front-right, rear-left, rear-right.
 * Direction-pin (white wire) level that drives each motor FORWARD: SET = CCW, RESET = CW.
 * Motors on opposite sides are mirrored, so left and right need opposite levels.
 * Which side is which depends on mounting -- fix these after the wheel test. */
static const GPIO_PinState MOTOR_FWD_LEVEL[4] = { GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET };

/* Direction pin labels (white wire): M1_DIR_W..M4_DIR_W, M1_DIR..M4_DIR, or the PCB net names M1_PWM_B..M4_PWM_B */
#if defined(M1_DIR_W_Pin)
#  define MDIR_PORT(n) M##n##_DIR_W_GPIO_Port
#  define MDIR_PIN(n)  M##n##_DIR_W_Pin
#elif defined(M1_DIR_Pin)
#  define MDIR_PORT(n) M##n##_DIR_GPIO_Port
#  define MDIR_PIN(n)  M##n##_DIR_Pin
#else
#  define MDIR_PORT(n) M##n##_PWM_B_GPIO_Port
#  define MDIR_PIN(n)  M##n##_PWM_B_Pin
#endif
static GPIO_TypeDef * const motorDirPort[4] = { MDIR_PORT(1), MDIR_PORT(2), MDIR_PORT(3), MDIR_PORT(4) };
static const uint16_t       motorDirPin[4]  = { MDIR_PIN(1), MDIR_PIN(2), MDIR_PIN(3), MDIR_PIN(4) };

/* Sides: 0 = left (M1, M3), 1 = right (M2, M4) */
static const int MOTOR_SIDE[4] = { 0, 1, 0, 1 };
static int32_t  sideCur[2]      = { 0, 0 };   /* speed now per side, 0.1 % (0..1000) */
static int32_t  sideRampAcc[2]  = { 0, 0 };
static int      sideRev[2]      = { 0, 0 };   /* 0 = forward, 1 = reverse */
static uint32_t sideMovingMs[2] = { 0, 0 };   /* last time the side was turning */

static void motorSetDir(int side, int rev)
{
  sideRev[side] = rev;
  for (int i = 0; i < 4; i++) {
    if (MOTOR_SIDE[i] != side) continue;
    GPIO_PinState fwd = MOTOR_FWD_LEVEL[i];
    GPIO_PinState lvl = rev ? (fwd == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET) : fwd;
    HAL_GPIO_WritePin(motorDirPort[i], motorDirPin[i], lvl);
  }
}

#if MOTOR_SPEED_PWM
static const uint32_t motorChannels[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
static uint32_t motorPeriod = 8500;   /* timer ticks per PWM cycle (ARR + 1) */

/* Program TIM3 for MOTOR_PWM_HZ regardless of what CubeMX was set to */
static void motorTimerSetup(void)
{
  uint32_t clk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1) clk *= 2;   /* timers run at 2x PCLK if APB1 is divided */
  motorPeriod = clk / MOTOR_PWM_HZ;
  __HAL_TIM_SET_PRESCALER(&htim3, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim3, motorPeriod - 1);
}
#endif

/* pm (0.1 %) -> blue-wire voltage. 0 = stop (0 V). */
static float motorVolts(int32_t pm)
{
  if (pm <= 0) return 0.0f;
  if (pm > MAX_PCT * 10) pm = MAX_PCT * 10;
#if !MOTOR_ENABLED
  return 0.0f;
#endif
  return MOTOR_V_MIN + (MOTOR_V_MAX - MOTOR_V_MIN) * (float)pm / 1000.0f;
}

/* Speeds per side, 0.1 % (direction is set separately with motorSetDir) */
static void motorWrite(int32_t pmLeft, int32_t pmRight)
{
#if MOTOR_SPEED_PWM
  for (int i = 0; i < 4; i++) {
    float v = motorVolts(MOTOR_SIDE[i] == 0 ? pmLeft : pmRight);
    uint32_t ccr = (uint32_t)(v / MOTOR_VREF * (float)motorPeriod + 0.5f);   /* HIGH time = v / 3.3 V */
    if (ccr > motorPeriod) ccr = motorPeriod;
    __HAL_TIM_SET_COMPARE(&htim3, motorChannels[i], ccr);
  }
#else
  /* DAC mode: one voltage for all four -- use the slower side (the controller keeps them equal) */
  float v = motorVolts(pmLeft < pmRight ? pmLeft : pmRight);
  uint32_t code = (uint32_t)(v / MOTOR_VREF * 4095.0f + 0.5f);
  if (code > 4095U) code = 4095U;
  HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);
#endif
}

static void motorStopNow(void)
{
  for (int i = 0; i < 2; i++) { sideCur[i] = 0; sideRampAcc[i] = 0; }
  motorWrite(0, 0);
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
  /* Left / right are included automatically once their labels exist in CubeMX */
#if defined(US_TRIG_L_Pin) && defined(US_ECHO_L_Pin)
  { US_TRIG_L_GPIO_Port, US_TRIG_L_Pin, US_ECHO_L_GPIO_Port, US_ECHO_L_Pin, "US_L" },
#endif
#if defined(US_TRIG_R_Pin) && defined(US_ECHO_R_Pin)
  { US_TRIG_R_GPIO_Port, US_TRIG_R_Pin, US_ECHO_R_GPIO_Port, US_ECHO_R_Pin, "US_R" },
#endif
};
#define US_COUNT ((int)(sizeof(usSensors) / sizeof(usSensors[0])))

static float    usCm[US_COUNT];   /* last reading, -1 = no echo */
static uint32_t usMs[US_COUNT];   /* HAL tick of last reading   */
static int      usNext = 0;
static int      usIdxL = -1, usIdxC = -1, usIdxR = -1;   /* positions in usSensors[], -1 = not fitted */

/* Fresh reading of sensor idx in cm, -1 = nothing seen / stale / not fitted */
static float usGetCm(int idx, uint32_t now)
{
  if (idx < 0 || now - usMs[idx] >= US_STALE_MS || usCm[idx] <= 0.0f) return -1.0f;
  return usCm[idx];
}

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
#define LIDAR_FRONT_Q6     ((int32_t)(LIDAR_FRONT_HALF_DEG * 64.0f))
#define LIDAR_SIDE_Q6      ((int32_t)(LIDAR_SIDE_MAX_DEG * 64.0f))
enum { LID_FRONT = 0, LID_LEFT = 1, LID_RIGHT = 2 };

static uint8_t           lidarRx[LIDAR_RX_SIZE];
static uint16_t          lidarRxOld = 0;
static volatile uint16_t lidarSecMm[3] = { 0, 0, 0 };   /* closest point per sector (front/left/right) last revolution, 0 = none */
static volatile uint32_t lidarRevMs = 0;     /* tick of last completed revolution */
static volatile uint32_t lidarRevCount = 0;
static uint32_t          lidarStartMs = 0;
static uint8_t           lidarResetPending = 0;
static uint32_t          lidarResetMs = 0;

/* parser state -- touched only by the ISR except during (re)start */
static uint8_t  lpState = 0;           /* 0 = waiting for descriptor, 1 = receiving samples */
static uint8_t  lpBuf[7];
static uint8_t  lpLen = 0;
static uint16_t lpMinMm[3] = { 0xFFFF, 0xFFFF, 0xFFFF };
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
    for (int k = 0; k < 3; k++) {
      lidarSecMm[k] = (lpMinMm[k] == 0xFFFF) ? 0 : lpMinMm[k];
      lpMinMm[k] = 0xFFFF;
    }
    lidarRevMs = HAL_GetTick();
    lidarRevCount++;
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

  int32_t rel = (int32_t)angleQ6 - LIDAR_FWD_Q6;     /* + = to the right when LIDAR_CW is 1 */
  while (rel >=  180 * 64) rel -= 360 * 64;
  while (rel <  -180 * 64) rel += 360 * 64;
#if !LIDAR_CW
  rel = -rel;
#endif
  int32_t mag = rel < 0 ? -rel : rel;
  int sec;
  if (mag <= LIDAR_FRONT_Q6)     sec = LID_FRONT;
  else if (mag <= LIDAR_SIDE_Q6) sec = (rel > 0) ? LID_RIGHT : LID_LEFT;
  else return;                                         /* outside the 120 deg view */
  if (mm < lpMinMm[sec]) lpMinMm[sec] = mm;
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
  lpState = 0; lpLen = 0;
  for (int k = 0; k < 3; k++) lpMinMm[k] = 0xFFFF;
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

/* Distance to the nearest thing in a sector (LID_FRONT / LID_LEFT / LID_RIGHT), cm; -1 = nothing / no data */
static float lidarSectorCm(int sec, uint32_t now, int *alive)
{
  *alive = (lidarRevCount > 0) && (now - lidarRevMs < LIDAR_STALE_MS);
  if (!*alive || lidarSecMm[sec] == 0) return -1.0f;
  return lidarSecMm[sec] / 10.0f;
}

/* ======================================================================
 * UWB -- each cart UWB board (a UWB tag, sketch troll_e_uwb.ino) sends one text
 * line per measurement: "D,<cm>" = its distance to the user's anchor.
 * Any other line is ignored.  UWB 0 = tag 0 on J10 (USART1), UWB 1 = tag 1 on J11 (USART2).
 * ====================================================================== */
#if UWB_ENABLED
extern UART_HandleTypeDef huart1;
#if UWB_COUNT > 1
extern UART_HandleTypeDef huart2;
#endif

typedef struct {
  UART_HandleTypeDef *huart;
  const char        *name;               /* for messages */
  uint8_t            rx[UWB_RX_SIZE];    /* DMA ring buffer */
  uint16_t           rxOld;
  char               line[48];
  uint8_t            len;
  volatile int32_t   cm;
  volatile uint32_t  ms;
  volatile uint32_t  count;
  char               last[48];           /* last complete line, printed by the 'w' command */
  volatile uint8_t   lastNew;
  int32_t            hist[UWB_MEDIAN_N]; /* recent readings for the median */
  uint8_t            histN, histI;
} UwbChan;

/* Median of the last UWB_MEDIAN_N readings (fewer right after start) */
static int32_t uwbMedian(UwbChan *u, int32_t cm)
{
  u->hist[u->histI] = cm;
  u->histI = (uint8_t)((u->histI + 1) % UWB_MEDIAN_N);
  if (u->histN < UWB_MEDIAN_N) u->histN++;
  int32_t v[UWB_MEDIAN_N];
  int n = u->histN;
  for (int i = 0; i < n; i++) v[i] = u->hist[i];
  for (int i = 1; i < n; i++) {                       /* insertion sort, n <= 5 */
    int32_t x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return v[n / 2];
}

static UwbChan uwb[UWB_COUNT] = {
  { .huart = &huart1, .name = "USART1 (J10)", .cm = -1 },
#if UWB_COUNT > 1
  { .huart = &huart2, .name = "USART2 (J11)", .cm = -1 },
#endif
};
static uint8_t uwbVerbose = 0;

static void uwbParse(UwbChan *u, const uint8_t *p, uint16_t n)
{
  while (n--) {
    char c = (char)*p++;
    if (c == '\r') continue;
    if (c != '\n') {
      if (u->len < sizeof(u->line) - 1) u->line[u->len++] = c;
      continue;
    }
    u->line[u->len] = 0;
    if (u->len >= 3 && u->line[0] == 'D' && u->line[1] == ',') {
      long cm = strtol(u->line + 2, NULL, 10);
      if (cm > 0 && cm <= UWB_MAX_CM) {
        if (HAL_GetTick() - u->ms > UWB_STALE_MS) u->histN = 0;   /* after a gap, start the median fresh */
        u->cm = uwbMedian(u, (int32_t)cm);
        u->ms = HAL_GetTick();
        u->count++;
      }
    }
    if (!u->lastNew) { memcpy(u->last, u->line, u->len + 1); u->lastNew = 1; }
    u->len = 0;
  }
}

/* DMA ring buffer: parse the bytes between the old and new write position */
static void uwbRing(UwbChan *u, uint16_t pos)
{
  if (pos != u->rxOld) {
    if (pos > u->rxOld) {
      uwbParse(u, &u->rx[u->rxOld], pos - u->rxOld);
    } else {                                   /* wrapped around the ring */
      uwbParse(u, &u->rx[u->rxOld], UWB_RX_SIZE - u->rxOld);
      uwbParse(u, &u->rx[0], pos);
    }
    u->rxOld = pos;
  }
  if (u->rxOld >= UWB_RX_SIZE) u->rxOld = 0;
}

static void uwbRxStart(UwbChan *u)
{
  HAL_UART_AbortReceive(u->huart);
  u->rxOld = 0;
  u->len = 0;
  if (HAL_UARTEx_ReceiveToIdle_DMA(u->huart, u->rx, UWB_RX_SIZE) != HAL_OK) {
    uprintf("UWB: could not start %s DMA (check RX DMA = Circular)\r\n", u->name);
  }
}

/* Distance from cart tag i to the user's anchor, cm; -1 = no recent reading */
static float uwbDistanceCm(int i, uint32_t now)
{
  if (uwb[i].count == 0 || now - uwb[i].ms >= UWB_STALE_MS) return -1.0f;
  return (float)uwb[i].cm;
}

/* Main loop: print the raw lines while 'w' is on */
static void uwbService(void)
{
  for (int i = 0; i < UWB_COUNT; i++) {
    if (uwb[i].lastNew) {
      if (uwbVerbose) uprintf("UWB%d> %s\r\n", i, uwb[i].last);
      uwb[i].lastNew = 0;
    }
  }
}
#endif

/* ---- HAL callbacks (override the weak versions) ---- */
/* DMA ring buffer: hand the bytes between the old and new write position to the parser */
static void ringRx(const uint8_t *ring, uint16_t ringSize, uint16_t *old, uint16_t pos,
                   void (*parse)(const uint8_t *, uint16_t))
{
  if (pos != *old) {
    if (pos > *old) {
      parse(&ring[*old], pos - *old);
    } else {                                   /* wrapped around the ring */
      parse(&ring[*old], ringSize - *old);
      parse(&ring[0], pos);
    }
    *old = pos;
  }
  if (*old >= ringSize) *old = 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &LIDAR_UART) ringRx(lidarRx, LIDAR_RX_SIZE, &lidarRxOld, Size, lidarParse);
#if UWB_ENABLED
  else {
    for (int i = 0; i < UWB_COUNT; i++)
      if (huart == uwb[i].huart) uwbRing(&uwb[i], Size);
  }
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* overrun/noise: restart reception, parser resyncs */
  if (huart == &LIDAR_UART) lidarRxStart();
#if UWB_ENABLED
  else {
    for (int i = 0; i < UWB_COUNT; i++)
      if (huart == uwb[i].huart) uwbRxStart(&uwb[i]);
  }
#endif
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
  uprintf("Commands: t=tare  c <lb>=calibrate  s=save  n=noise  v=raw on/off  p=print\r\n"
          "          m <pct>=manual motor test (m -20 = reverse)  a=back to automatic  ?=help\r\n"
#if UWB_ENABLED
          "          w=UWB raw lines on/off  u=UWB centre (stand straight ahead ~2 m, then type u)\r\n"
#endif
          );
}

#if UWB_ENABLED
static void cmdUwbCentre(void);   /* below, next to the UWB filter */
#endif

static int manualMode = 0;     /* 1 = 'm' command is driving the motors, sensors ignored */
static int manualPct  = 0;     /* signed: negative = reverse */

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
    case 'm': {
      char *end;
      long v = strtol(line + 1, &end, 10);
      if (end == line + 1) { uprintf("Usage: m <percent>, e.g. m 15 or m -15\r\n"); break; }
      if (v >  MAX_PCT) v =  MAX_PCT;
      if (v < -MAX_PCT) v = -MAX_PCT;
      manualMode = 1;
      manualPct = (int)v;
      uprintf("MANUAL motors %d%% -- sensors ignored, type a to return to automatic%s\r\n",
              manualPct, MOTOR_ENABLED ? "" : "  (MOTOR_ENABLED is 0: outputs stay stopped)");
      break;
    }
#if UWB_ENABLED
    case 'w': uwbVerbose = !uwbVerbose; uprintf("UWB raw lines %s\r\n", uwbVerbose ? "ON" : "OFF"); break;
    case 'u': cmdUwbCentre(); break;
#endif
    case 'a': manualMode = 0; manualPct = 0; uprintf("Automatic control\r\n"); break;
    case '?': printHelp(); break;
    case 0: break;
    default: uprintf("Unknown command\r\n"); printHelp(); break;
  }
}

/* Non-blocking line reader on the debug UART */
static uint8_t typingLen = 0;   /* characters typed so far; status line pauses while > 0 */

static void serialService(void)
{
  static char line[32];
  uint8_t len = typingLen;

  uint8_t ch;
#if DEBUG_OVER_USB
  while (usbGetChar(&ch)) {
#else
  if (__HAL_UART_GET_FLAG(&DEBUG_UART, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&DEBUG_UART);
  while (__HAL_UART_GET_FLAG(&DEBUG_UART, UART_FLAG_RXNE)) {
    ch = (uint8_t)(DEBUG_UART.Instance->RDR & 0xFFU);
#endif
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
      debugWrite(&ch, 1);                              /* echo */
    }
  }
  typingLen = len;
}

/* ======================================================================
 * PCB-only signals (each compiled in only if its CubeMX label exists)
 *   ESTOP_SENSE    PB2  input, R5 pull-up. J4 = normally-closed e-stop to GND:
 *                       LOW = OK, HIGH = pressed OR J4 unplugged -> STOP
 *   MOTOR_KILL_CMD PC4  output to J5, HIGH = kill (motors stopped), LOW = motors may run
 *   LED_STATUS_MCU PB10 green-ish "alive" LED, blinks once a second
 *   LED_ERROR_MCU  PB11 on when: e-stop, HX711 not answering, or overload
 * ====================================================================== */
static int estopActive(void)
{
#ifdef ESTOP_SENSE_Pin
  return HAL_GPIO_ReadPin(ESTOP_SENSE_GPIO_Port, ESTOP_SENSE_Pin) == GPIO_PIN_SET;
#else
  return 0;
#endif
}

static void killOutput(int kill)
{
#ifdef MOTOR_KILL_CMD_Pin
  HAL_GPIO_WritePin(MOTOR_KILL_CMD_GPIO_Port, MOTOR_KILL_CMD_Pin, kill ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  (void)kill;
#endif
}

static void ledService(uint32_t now, int error)
{
#ifdef LED_STATUS_MCU_Pin
  HAL_GPIO_WritePin(LED_STATUS_MCU_GPIO_Port, LED_STATUS_MCU_Pin,
                    ((now / 500U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
#ifdef LED_ERROR_MCU_Pin
  HAL_GPIO_WritePin(LED_ERROR_MCU_GPIO_Port, LED_ERROR_MCU_Pin, error ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  (void)error;
#endif
  (void)now;
}

/* ======================================================================
 * Speed + steering decision
 * ====================================================================== */
typedef enum {
  MODE_FOLLOW, MODE_WAIT, MODE_PIVOT, MODE_FACE, MODE_BLOCKED,
  MODE_ONETAG, MODE_UWBSTOP, MODE_MANUAL, MODE_ESTOP, MODE_OVERLOAD
} Mode;
static const char *const MODE_NAME[] = {
  "FOLLOW", "WAIT", "PIVOT", "FACE", "BLOCKED", "ONE-TAG", "UWB-STOP", "MANUAL", "E-STOP", "OVERLOAD"
};

static Mode        mode = MODE_UWBSTOP;
static int         zone = 0;              /* 0 stop, 1 slow, 2 fast */
static float       fusedCm = -1.0f;       /* distance the speed zone is based on */
static float       frontCm = -1.0f, leftCm = -1.0f, rightCm = -1.0f, userCm = -1.0f;
static const char *limitReason = "";
static int         pivotDir = 0;          /* +1 = turning right (clockwise), -1 = left */
static uint32_t    pivotStartMs = 0;

static float closer(float a, float b)    /* smaller valid distance; -1 = neither valid */
{
  if (a <= 0.0f) return b;
  if (b <= 0.0f) return a;
  return a < b ? a : b;
}

/* ---- UWB: filtered distance and left/right difference, lost-signal latch ---- */
#if UWB_ENABLED
static float    uwbDiffF = 0.0f, uwbRangeF = -1.0f;
static int      uwbDiffOk = 0, uwbRangeOk = 0;
static float    uwbDiffOffset = UWB_DIFF_OFFSET_CM;
static int      uwbFresh = 0;             /* tags with a recent reading */
static int      uwbStopLatched = 1;       /* emergency stop until all tags report (also at power-up) */
static int      uwbAllRun = 0;
static uint32_t uwbAllSince = 0;

static void uwbUpdate(uint32_t now, uint32_t dt)
{
  float d0 = uwbDistanceCm(0, now);
  float d1 = (UWB_COUNT > 1) ? uwbDistanceCm(1, now) : -1.0f;
  uwbFresh = (d0 > 0.0f) + (d1 > 0.0f);
  float k  = (float)dt / (float)(UWB_FILTER_MS + dt);         /* exponential averages */
  float kr = (float)dt / (float)(UWB_RANGE_FILTER_MS + dt);

  float rng = (uwbFresh == 2) ? 0.5f * (d0 + d1) : (d0 > 0.0f ? d0 : d1);
  if (uwbFresh == 0)    uwbRangeOk = 0;
  else if (!uwbRangeOk) { uwbRangeF = rng; uwbRangeOk = 1; }
  else                  uwbRangeF += kr * (rng - uwbRangeF);

  if (uwbFresh == 2) {
    float diff = d0 - d1 - uwbDiffOffset;              /* + = user to the right */
    if (!uwbDiffOk) { uwbDiffF = diff; uwbDiffOk = 1; }
    else            uwbDiffF += k * (diff - uwbDiffF);
  } else {
    uwbDiffOk = 0;
  }

  /* All tags silent -> emergency stop. Resume only after all report for UWB_RESUME_MS. */
  if (uwbFresh == 0 && !uwbStopLatched) {
    uwbStopLatched = 1;
    uprintf("UWB: no signal -- EMERGENCY STOP\r\n");
  }
  if (uwbFresh == UWB_COUNT) {
    if (!uwbAllRun) { uwbAllRun = 1; uwbAllSince = now; }
  } else {
    uwbAllRun = 0;
  }
  if (uwbStopLatched && uwbAllRun && now - uwbAllSince >= UWB_RESUME_MS) {
    uwbStopLatched = 0;
    uprintf("UWB: signal back -- following\r\n");
  }
}
#endif

#if UWB_ENABLED
/* 'u': user stands straight ahead -> the current left/right difference becomes the zero point */
static void cmdUwbCentre(void)
{
  if (!uwbDiffOk) { uprintf("UWB centre: need both tags reporting\r\n"); return; }
  uwbDiffOffset += uwbDiffF;
  uwbDiffF = 0.0f;
  uprintf("UWB centre offset %.1f cm (until reset -- put it in UWB_DIFF_OFFSET_CM to keep it)\r\n", uwbDiffOffset);
}
#endif

/* Turn in place: dir +1 = right (left side forward, right side back), -1 = left */
static void pivotSpeeds(int dir, int32_t *tl, int32_t *tr)
{
  *tl = (dir > 0) ?  PIVOT_FWD_PCT * 10 : -PIVOT_REV_PCT * 10;
  *tr = (dir > 0) ? -PIVOT_REV_PCT * 10 :  PIVOT_FWD_PCT * 10;
}

/* Speed for a distance, with hysteresis so it doesn't chatter at a boundary */
static int32_t zoneSpeed(float cm)
{
  if (cm < 0.0f) { zone = 0; return 0; }
  switch (zone) {
    case 0:
      if (cm >= SLOW_DIST_CM + HYST_CM)      zone = 2;
      else if (cm >= STOP_DIST_CM + HYST_CM) zone = 1;
      break;
    case 1:
      if (cm < STOP_DIST_CM)                 zone = 0;
      else if (cm >= SLOW_DIST_CM + HYST_CM) zone = 2;
      break;
    default:
      if (cm < STOP_DIST_CM)                 zone = 0;
      else if (cm < SLOW_DIST_CM)            zone = 1;
      break;
  }
  return zone == 2 ? FAST_PCT * 10 : zone == 1 ? SLOW_PCT * 10 : 0;
}

static void controlService(uint32_t now)
{
  static uint32_t last = 0;
  uint32_t dt = now - last;
  if (dt < CONTROL_PERIOD_MS) return;
  last = now;
  if (dt > 200) dt = 200;   /* after a blocking command, don't take one giant ramp step */

  /* ---- 1. Sensors ---- */
  int alive = 0;
  float lidF = -1.0f, lidL = -1.0f, lidR = -1.0f;
#if LIDAR_ENABLED
  lidF = lidarSectorCm(LID_FRONT, now, &alive);
  lidL = lidarSectorCm(LID_LEFT,  now, &alive);
  lidR = lidarSectorCm(LID_RIGHT, now, &alive);
#endif
  (void)alive;
  float usL = usGetCm(usIdxL, now), usC = usGetCm(usIdxC, now), usR = usGetCm(usIdxR, now);
  frontCm = closer(closer(usL, usC), closer(usR, lidF));   /* anything in the cart's path */
  leftCm  = closer(usL, lidL);                             /* left lane + lidar left sector */
  rightCm = closer(usR, lidR);

  int canSteer = 0;
  float diff = 0.0f;
#if UWB_ENABLED
  uwbUpdate(now, dt);
  userCm = uwbRangeOk ? uwbRangeF : -1.0f;
  diff = uwbDiffF;
  canSteer = STEER_ENABLED && MOTOR_SPEED_PWM && (UWB_COUNT > 1) && uwbDiffOk;
  fusedCm = (userCm > 0.0f) ? closer(userCm, frontCm) : -1.0f;   /* UWB = the user; front sensors can only slow it */
#else
  userCm = -1.0f;
  fusedCm = frontCm;                                         /* no UWB: forward sensors decide, straight only */
#endif
  int32_t base = zoneSpeed(fusedCm);

  int obstacle = (frontCm > 0.0f && frontCm < STOP_DIST_CM);
  int isUser   = obstacle && userCm > 0.0f && fabsf(frontCm - userCm) <= USER_MATCH_CM;
  int clear    = (frontCm < 0.0f || frontCm >= STOP_DIST_CM + PIVOT_CLEAR_CM);
  float adiff  = fabsf(diff);

  /* ---- 2. Decide: signed target per side, 0.1 % (negative = reverse) ---- */
  int32_t tl = 0, tr = 0;
  limitReason = "";

  /* An obstacle turn keeps going (same direction) until the front clears or it times out */
  if (mode == MODE_PIVOT) {
    if (clear || isUser || !canSteer)           mode = MODE_FOLLOW;   /* re-decide below */
    else if (now - pivotStartMs > PIVOT_MAX_MS) mode = MODE_BLOCKED;
  }
  if (mode == MODE_BLOCKED && clear) mode = MODE_FOLLOW;

#if UWB_ENABLED
  if (uwbStopLatched) {
    mode = MODE_UWBSTOP;
    limitReason = "no UWB signal";
  } else if (uwbFresh < UWB_COUNT || UWB_COUNT < 2) {
    mode = MODE_ONETAG;                         /* distance only: straight, slow at most */
    tl = tr = (base > SLOW_PCT * 10) ? SLOW_PCT * 10 : base;
    if (obstacle) limitReason = isUser ? "user close" : "obstacle";
  } else
#endif
  if (mode == MODE_PIVOT) {
    pivotSpeeds(pivotDir, &tl, &tr);
  } else if (mode == MODE_BLOCKED) {
    limitReason = "turn timed out";
  } else if (obstacle && !isUser && canSteer) {
    /* Something in the way that isn't the user: turn toward the more open side */
    float l = (leftCm  > 0.0f) ? leftCm  : 9999.0f;
    float r = (rightCm > 0.0f) ? rightCm : 9999.0f;
    int tie = (l < STOP_DIST_CM && r < STOP_DIST_CM) || fabsf(l - r) < PIVOT_TIE_CM;
    pivotDir = tie ? (diff >= 0.0f ? +1 : -1) : (r > l ? +1 : -1);
    pivotStartMs = now;
    mode = MODE_PIVOT;
    pivotSpeeds(pivotDir, &tl, &tr);
  } else if (obstacle) {
    mode = MODE_WAIT;
    limitReason = isUser ? "user close" : "obstacle";
  } else if (userCm > 0.0f && userCm < STOP_DIST_CM) {
    /* User close: face them if they're off to one side, otherwise wait */
    if (canSteer && (adiff > FACE_START_CM || (mode == MODE_FACE && adiff > FACE_STOP_CM))) {
      mode = MODE_FACE;
      int dir = diff > 0.0f ? +1 : -1;
      pivotSpeeds(dir, &tl, &tr);
    } else {
      mode = MODE_WAIT;
      limitReason = "user close";
    }
  } else {
    mode = MODE_FOLLOW;
    if (fusedCm < 0.0f) limitReason = "no target";
    int32_t steer = 0;
    if (canSteer && base > 0 && adiff > STEER_DEADBAND_CM) {
      steer = (int32_t)(STEER_GAIN_PCT_PER_CM * (adiff - STEER_DEADBAND_CM) * 10.0f);
      if (steer > STEER_MAX_PCT * 10) steer = STEER_MAX_PCT * 10;
      if (adiff > FACE_START_CM && base > SLOW_PCT * 10) base = SLOW_PCT * 10;   /* sharp turn: slow down */
      int rBlocked = rightCm > 0.0f && rightCm < SIDE_CLEAR_CM && fabsf(rightCm - userCm) > USER_MATCH_CM;
      int lBlocked = leftCm  > 0.0f && leftCm  < SIDE_CLEAR_CM && fabsf(leftCm  - userCm) > USER_MATCH_CM;
      if (diff > 0.0f && rBlocked) { steer = 0; limitReason = "right side close"; }
      if (diff < 0.0f && lBlocked) { steer = 0; limitReason = "left side close"; }
      if (diff < 0.0f) steer = -steer;           /* user to the left -> right side faster */
    }
    tl = base + steer;  if (tl < 0) tl = 0;
    tr = base - steer;  if (tr < 0) tr = 0;
  }

  /* ---- 3. Load cell caps the speed ---- */
  int32_t ramp = RAMP_PCT_PER_S;
  int32_t cap = MAX_PCT * 10;
#if LOAD_LIMITS_ENABLED
  if (now - loadMs > LOAD_STALE_MS) {
    cap = SLOW_PCT * 10; if (!*limitReason) limitReason = "load unknown";
  } else if (loadLb > LOAD_MAX_LB) {
    cap = 0; mode = MODE_OVERLOAD; limitReason = "OVERLOAD";
  } else if (loadLb > LOAD_HEAVY_LB) {
    cap = SLOW_PCT * 10; ramp = RAMP_PCT_PER_S_HEAVY; if (!*limitReason) limitReason = "heavy";
  }
#endif

  /* ---- 4. Manual test command overrides everything above (except e-stop) ---- */
  if (manualMode) {
    tl = tr = manualPct * 10;
    cap = MAX_PCT * 10;
    mode = MODE_MANUAL;
    limitReason = "MANUAL";
  }
  if (estopActive()) { tl = tr = 0; mode = MODE_ESTOP; limitReason = "E-STOP"; }

  int32_t tgt[2] = { tl, tr };
  for (int sd = 0; sd < 2; sd++) {
    if (tgt[sd] >  cap) tgt[sd] =  cap;
    if (tgt[sd] < -cap) tgt[sd] = -cap;
  }

  /* ---- 5. Direction changes: hold BOTH sides at 0 until each side that must reverse has
   *         been stopped for REVERSE_DWELL_MS, then flip it (so a turn never starts as a lurch) ---- */
  int needFlip = 0;
  for (int sd = 0; sd < 2; sd++)
    if (tgt[sd] != 0 && (tgt[sd] < 0) != sideRev[sd]) needFlip = 1;
  if (needFlip) {
    for (int sd = 0; sd < 2; sd++) {
      int want = tgt[sd] < 0;
      if (tgt[sd] != 0 && want != sideRev[sd] && sideCur[sd] == 0 && now - sideMovingMs[sd] >= REVERSE_DWELL_MS)
        motorSetDir(sd, want);
      tgt[sd] = 0;
    }
  }

  /* ---- 6. Per side: slow down / stop immediately, speed up gradually ---- */
  for (int sd = 0; sd < 2; sd++) {
    int32_t want = tgt[sd] < 0 ? -tgt[sd] : tgt[sd];
    if (want <= sideCur[sd]) {
      sideCur[sd] = want;
      sideRampAcc[sd] = 0;
    } else {
      if (sideCur[sd] < START_PCT * 10) sideCur[sd] = (want < START_PCT * 10) ? want : START_PCT * 10;
      sideRampAcc[sd] += ramp * 10 * (int32_t)dt;          /* 0.1 % per second * ms */
      sideCur[sd] += sideRampAcc[sd] / 1000;
      sideRampAcc[sd] %= 1000;
      if (sideCur[sd] > want) sideCur[sd] = want;
    }
    if (sideCur[sd] != 0) sideMovingMs[sd] = now;
  }
  motorWrite(sideCur[0], sideCur[1]);
  killOutput(sideCur[0] == 0 && sideCur[1] == 0);
}

/* ERROR LED reasons beyond the load cell / e-stop: lost UWB (all or one tag) */
static int uwbAlarm(void)
{
#if UWB_ENABLED
  return uwbStopLatched || uwbFresh < UWB_COUNT;
#else
  return 0;
#endif
}

/* ======================================================================
 * Status line
 * ====================================================================== */
static void statusService(uint32_t now)
{
  static uint32_t last = 0;
  if (now - last < STATUS_PERIOD_MS || hxVerbose || typingLen > 0) return;
  last = now;

  char part[200];
  int n = 0;
  for (int i = 0; i < US_COUNT; i++) {
    if (now - usMs[i] >= US_STALE_MS)  n += snprintf(part + n, sizeof(part) - n, "%s stale  ", usSensors[i].name);
    else if (usCm[i] < 0.0f)           n += snprintf(part + n, sizeof(part) - n, "%s --  ", usSensors[i].name);
    else                               n += snprintf(part + n, sizeof(part) - n, "%s %.0f  ", usSensors[i].name, usCm[i]);
  }
#if LIDAR_ENABLED
  int alive;
  float lf = lidarSectorCm(LID_FRONT, now, &alive);
  float ll = lidarSectorCm(LID_LEFT,  now, &alive);
  float lr = lidarSectorCm(LID_RIGHT, now, &alive);
  if (!alive) n += snprintf(part + n, sizeof(part) - n, "LID off  ");
  else        n += snprintf(part + n, sizeof(part) - n, "LID L%.0f F%.0f R%.0f  ",
                            ll < 0 ? 0.0f : ll, lf < 0 ? 0.0f : lf, lr < 0 ? 0.0f : lr);   /* 0 = nothing */
#endif
#if UWB_ENABLED
  for (int i = 0; i < UWB_COUNT; i++) {
    float u = uwbDistanceCm(i, now);
    if (u < 0.0f) n += snprintf(part + n, sizeof(part) - n, "UWB%d --  ", i);
    else          n += snprintf(part + n, sizeof(part) - n, "UWB%d %.0f  ", i, u);
  }
  if (uwbDiffOk) n += snprintf(part + n, sizeof(part) - n, "diff %+.0f  ", uwbDiffF);
#endif
  if (fusedCm < 0.0f) snprintf(part + n, sizeof(part) - n, "-> none");
  else                snprintf(part + n, sizeof(part) - n, "-> %.0f cm", fusedCm);

  uprintf("%s | %.1f lb | %s L%s%ld R%s%ld%s %s\r\n",
          part, loadLb, MODE_NAME[mode],
          sideRev[0] ? "-" : "", (long)(sideCur[0] / 10),
          sideRev[1] ? "-" : "", (long)(sideCur[1] / 10),
          MOTOR_ENABLED ? "" : "(off)", limitReason);
}

/* ======================================================================
 * Public entry points
 * ====================================================================== */
void TrollE_Init(void)
{
  HAL_TIM_Base_Start(&htim2);                     /* 1 MHz microsecond counter */
  killOutput(1);                                  /* external kill asserted until we drive */
  motorSetDir(0, 0);                              /* both sides forward */
  motorSetDir(1, 0);
#if MOTOR_SPEED_PWM
  motorTimerSetup();                              /* 20 kHz, one channel per motor */
  for (int i = 0; i < 4; i++) __HAL_TIM_SET_COMPARE(&htim3, motorChannels[i], 0);   /* 0 % = stop */
  htim3.Instance->EGR = TIM_EGR_UG;
  for (int i = 0; i < 4; i++) HAL_TIM_PWM_Start(&htim3, motorChannels[i]);
#else
  HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);           /* speed voltage on D12, starts at 0 V = stop */
#endif
  motorStopNow();

  for (int i = 0; i < US_COUNT; i++) {
    HAL_GPIO_WritePin(usSensors[i].trigPort, usSensors[i].trigPin, GPIO_PIN_RESET);
    usCm[i] = -1.0f;
    usMs[i] = 0;
    if (strcmp(usSensors[i].name, "US_L") == 0) usIdxL = i;
    if (strcmp(usSensors[i].name, "US_C") == 0) usIdxC = i;
    if (strcmp(usSensors[i].name, "US_R") == 0) usIdxR = i;
  }
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);   /* CLK low keeps HX711 awake */

  uprintf("\r\n=== TROLL-E ===  motor %s (%s speed), lidar %s, uwb %s, steering %s, terminal over %s\r\n",
          MOTOR_ENABLED ? "ENABLED" : "disabled", MOTOR_SPEED_PWM ? "PWM" : "DAC",
          LIDAR_ENABLED ? "on" : "off", UWB_ENABLED ? "on" : "off",
          (STEER_ENABLED && MOTOR_SPEED_PWM && UWB_ENABLED && UWB_COUNT > 1) ? "on" : "off (needs PWM + 2 UWB)",
          DEBUG_OVER_USB ? "USB" : "ST-LINK");

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
#if UWB_ENABLED
  for (int i = 0; i < UWB_COUNT; i++) uwbRxStart(&uwb[i]);
#endif

  printHelp();
}

void TrollE_Loop(void)
{
  serialService();
  loadService(HAL_GetTick());
  usService(HAL_GetTick());
  lidarService(HAL_GetTick());
#if UWB_ENABLED
  uwbService();
#endif
  controlService(HAL_GetTick());
  statusService(HAL_GetTick());
  {
    uint32_t now = HAL_GetTick();
    int hxDead = (now - loadMs > LOAD_STALE_MS);
    ledService(now, estopActive() || hxDead || loadLb > LOAD_MAX_LB || uwbAlarm());
  }
}
