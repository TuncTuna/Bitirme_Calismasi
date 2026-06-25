/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @date           : 09.06.2026
  * @author         : tunak
  * @AI             : Claude AI Sonnet 4.6
  ******************************************************************************
  *
  * DEĞİŞİKLİKLER (09.06.2026):
  *  - Austin ivme algoritması düzeltildi (steps_to_go > n koşulu korundu,
  *    delay güncelleme formülü 2*c / (4n+1) olarak standardize edildi)
  *  - Update_Motors(), Stepper_Update() üzerine refactor edildi (DRY)
  *  - Set_All_Motors_Synced() içinde c0 artık orantılanmıyor (sabit base_c0)
  *  - Set_Motor_Angle() içinde n sıfırlama ve current_delay = c0 garantilendi
  *  - Stepper_Update() yavaşlama bölgesinde steps_to_go == 0 guard eklendi
  *  - İlk pozisyon hareketi Set_Motor_Angle() → Set_All_Motors_Synced() ile
  *    değiştirildi; motorlar artık başlangıç pozisyonuna da senkron gidiyor
  *
  * HIZ AYARLARI:
  *  - Tepe hız          → min_delay  (düşürünce hızlanır, örn. 80~150 µs)
  *  - Kalkış yumuşaklığı→ c0         (artırınca yumuşar,  örn. 2000~4000 µs)
  *  - İvme agresifliği  → ACCEL_FACTOR define (artırınca sert ivmelenir)
  *  - Gripper hızı      → Stepper_Init içindeki min_delay / c0
  *  - Genel ayar noktası→ Set_All_Motors_Speed(min_delay, c0) çağrısı
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// -----------------------------------------------------------------------
// PAKET YAPISI
// -----------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  start_byte;    // 0xFF        1 byte
    uint8_t  Mode;          //             1 byte
    float    Dist1;         //             4 byte
    float    Theta2;        //             4 byte
    float    Theta3;        //             4 byte
    float    Theta4;        //             4 byte
    float    Gripper_cmd;   //             4 byte
    uint16_t crc;           //             2 byte
    uint8_t  stop_byte;     // 0xFE        1 byte
} MatlabPacket_t;           // TOPLAM:    25 byte

typedef struct {
    float Dist1;
    float Theta2;
    float Theta3;
    float Theta4;
    float Gripper_cmd;
} RobotTarget_t;

typedef enum {
    Mode_MANUAL   = 0x01,
    Mode_TEACHPAD = 0x02,
    Mode_AUTO     = 0x03
} RobotMode;

// -----------------------------------------------------------------------
// STEP MOTOR STRUCT
// -----------------------------------------------------------------------
typedef struct {
    GPIO_TypeDef* step_port;
    uint16_t      step_pin;
    GPIO_TypeDef* dir_port;
    uint16_t      dir_pin;
    GPIO_TypeDef* limit_port;
    uint16_t      limit_pin;

    long     current_pos;
    long     target_pos;

    uint32_t last_step_time;
    float    current_delay;
    float    min_delay;

    long     n;
    float    c0;

    uint8_t  is_moving;
} Stepper_t;

// -----------------------------------------------------------------------
// GRİPPER STRUCT
// -----------------------------------------------------------------------
typedef enum {
    GRIPPER_IDLE,
    GRIPPER_GRIPPING,    // Parmaklar kapanıyor           (cmd=1)
    GRIPPER_ROTATING,    // +180° döner, saptan ayırır    (cmd=1)
    GRIPPER_RELEASING,   // Parmaklar açılıyor            (cmd=2)
    GRIPPER_UNROTATING   // -180° geri döner              (cmd=2)
} GripperState_t;

typedef struct {
    Stepper_t      finger;  // Parmak motoru
    Stepper_t      rotate;  // Rotasyon motoru
    GripperState_t state;
} Gripper_t;

// -----------------------------------------------------------------------
// CMD STATE (ana hareket için)
// -----------------------------------------------------------------------
typedef enum {
    CMD_IDLE,
    CMD_MOVING,
    CMD_GRIPPING
} CmdState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESCALER           83
#define STEPS_PER_REV       3200.0f

// -----------------------------------------------------------------------
// HIZ / İVME AYARI — BURADAN DEĞİŞTİR
// -----------------------------------------------------------------------
// ACCEL_FACTOR: İvme agresifliği. Artırınca daha sert ivmelenir/yavaşlar.
// Önerilen aralık: 1.2 ~ 2.5
#define ACCEL_FACTOR        1.2f

#define RX_BUFFER_SIZE      25

// -----------------------------------------------------------------------
// GRİPPER SABİTLERİ
// -----------------------------------------------------------------------
#define GRIPPER_GRIP_STEPS    5000      // Parmak kapanma adımı
#define GRIPPER_ROTATE_DEG    270.0f    // Catch rotasyon açısı +180°
#define GRIPPER_GEAR_RATIO    1.0f      // Rotasyon motoru dişli oranı
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Stepper_t    motors[4];
Gripper_t    gripper;

RobotTarget_t robotTarget;
uint8_t       rxBuffer[RX_BUFFER_SIZE];
RobotMode     current_mode      = Mode_MANUAL;

volatile uint8_t  new_packet_received = 0;
volatile uint8_t  pending_gripper     = 0;
volatile uint8_t  test_gripper_cmd    = 0;  // Debug
volatile CmdState cmd_state           = CMD_IDLE;
volatile uint32_t uart_irq_counter    = 0;

// -----------------------------------------------------------------------
// DEBUG — Live Expressions'a ekle:
//   debug_delay[0..3]       → anlık gecikme (µs), düşük = hızlı
//   debug_n[0..3]           → ivme adım sayacı
//   debug_steps_to_go[0..3] → hedefe kalan adım
// -----------------------------------------------------------------------
volatile float debug_delay[4]       = {0};
volatile long  debug_n[4]           = {0};
volatile long  debug_steps_to_go[4] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// -----------------------------------------------------------------------
// YARDIMCI: mikrosaniye sayacı
// -----------------------------------------------------------------------
extern TIM_HandleTypeDef htim1;

uint16_t get_micros(void) {
    return __HAL_TIM_GET_COUNTER(&htim1);
}

// -----------------------------------------------------------------------
// YARDIMCI: homing delay
// -----------------------------------------------------------------------
void delay_us_home(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

// -----------------------------------------------------------------------
// YARDIMCI: DONE paketi gönder
// -----------------------------------------------------------------------
void Send_Done(void) {
    const char msg[] = "DONE\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, 5, 100);
}

// -----------------------------------------------------------------------
// CRC
// -----------------------------------------------------------------------
uint16_t Calculate_CRC16_Modbus(uint8_t *data, uint16_t length) {
    uint16_t crc  = 0xFFFF;
    uint16_t poly = 0xA001;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ poly;
            else         crc >>= 1;
        }
    }
    return crc;
}

// -----------------------------------------------------------------------
// PAKET İŞLEYİCİ
// -----------------------------------------------------------------------
uint8_t Process_Incoming_Packet(uint8_t* buffer, RobotTarget_t* target, RobotMode* mode) {
    MatlabPacket_t* packet = (MatlabPacket_t*)buffer;

    if (packet->start_byte != 0xFF || packet->stop_byte != 0xFE)
        return 0;

    // CRC: Mode'dan Gripper_cmd'ye kadar = 1 + 5*4 = 21 byte
    uint16_t calculated_crc = Calculate_CRC16_Modbus((uint8_t*)&packet->Mode, 21);
    if (calculated_crc != packet->crc)
        return 0;

    *mode                = (RobotMode)packet->Mode;
    target->Dist1        = packet->Dist1;
    target->Theta2       = packet->Theta2;
    target->Theta3       = packet->Theta3;
    target->Theta4       = packet->Theta4;
    target->Gripper_cmd  = packet->Gripper_cmd;

    return 1;
}

// -----------------------------------------------------------------------
// UART CALLBACK
// -----------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uart_irq_counter++;
        if (Process_Incoming_Packet(rxBuffer, &robotTarget, &current_mode)) {
            new_packet_received = 1;
        }
        HAL_UART_Receive_IT(huart, rxBuffer, RX_BUFFER_SIZE);
    }
}

// -----------------------------------------------------------------------
// STEP MOTOR — TEMEL INIT
// -----------------------------------------------------------------------
void Stepper_Init(Stepper_t* m,
    GPIO_TypeDef* sp, uint16_t s_pin,
    GPIO_TypeDef* dp, uint16_t d_pin,
    GPIO_TypeDef* lp, uint16_t l_pin)
{
    m->step_port  = sp;  m->step_pin  = s_pin;
    m->dir_port   = dp;  m->dir_pin   = d_pin;
    m->limit_port = lp;  m->limit_pin = l_pin;

    m->current_pos   = 0;
    m->target_pos    = 0;
    m->is_moving     = 0;

    // ---------------------------------------------------------------
    // HIZ AYARLARI — Gripper motorları için varsayılan
    // Ana motorlar Set_All_Motors_Speed() ile eziyor, burası gripper için
    // min_delay: tepe hız (µs) — düşürünce hızlanır (önerilen: 60~150)
    // c0        : başlangıç gecikmesi — artırınca yumuşar  (önerilen: 1500~3000)
    // ---------------------------------------------------------------
    m->min_delay     = 150.0f;
    m->c0            = 2000.0f;
    m->current_delay = 2000.0f;
    m->n             = 0;
}

// motors[] dizisi için kolaylık wrapper
void Motor_Init(int index,
    GPIO_TypeDef* sp, uint16_t s_pin,
    GPIO_TypeDef* dp, uint16_t d_pin,
    GPIO_TypeDef* lp, uint16_t l_pin)
{
    Stepper_Init(&motors[index], sp, s_pin, dp, d_pin, lp, l_pin);
}

// -----------------------------------------------------------------------
// STEP MOTOR — AÇI AYARLA
// -----------------------------------------------------------------------
void Set_Motor_Angle(int index, float angle) {
    long target_steps = (long)((angle / 1.8f) * 16.0f);

    if (target_steps == motors[index].target_pos) return;

    motors[index].target_pos    = target_steps;
    motors[index].is_moving     = 1;
    motors[index].n             = 0;                         // ivme sayacını sıfırla
    motors[index].current_delay = motors[index].c0;          // c0'dan başla

    if (motors[index].target_pos > motors[index].current_pos)
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_SET);
}

// -----------------------------------------------------------------------
// STEP MOTOR — SENKRON HAREKET
// Tüm motorlar aynı anda başlar ve biter.
// min_delay orantılanır (yavaş motor daha yavaş tepe hıza çıkar),
// c0 sabit tutulur (hepsi aynı kalkış yumuşaklığıyla başlar).
// -----------------------------------------------------------------------
void Set_All_Motors_Synced(float a0, float a1, float a2, float a3) {
    long target0 = (long)((a0 / 1.8f) * 16.0f);
    long target1 = (long)((a1 / 1.8f) * 16.0f);
    long target2 = (long)((a2 / 1.8f) * 16.0f);
    long target3 = (long)((a3 / 1.8f) * 16.0f);

    long steps0 = labs(target0 - motors[0].current_pos);
    long steps1 = labs(target1 - motors[1].current_pos);
    long steps2 = labs(target2 - motors[2].current_pos);
    long steps3 = labs(target3 - motors[3].current_pos);

    long max_steps = steps0;
    if (steps1 > max_steps) max_steps = steps1;
    if (steps2 > max_steps) max_steps = steps2;
    if (steps3 > max_steps) max_steps = steps3;

    if (max_steps == 0) return;

    // ---------------------------------------------------------------
    // HIZ AYAR NOKTALARI — Set_All_Motors_Synced
    // base_min_delay: en hızlı (en çok adım atan) motorun tepe hızı
    // base_c0       : tüm motorların kalkış yumuşaklığı (sabit)
    // max_min_delay : motor3'e uygulanan tepe hız üst sınırı (kayış kayması önlemi)
    // ---------------------------------------------------------------
    float base_min_delay = 120.0f;
    float base_c0        = 2500.0f;
    float max_min_delay  = 800.0f;

    // min_delay orantılanır: az adım atan motor daha yavaş gider → senkron bitiş
    motors[0].min_delay = (steps0 > 0) ? base_min_delay * ((float)max_steps / (float)steps0) : 99999.0f;
    motors[1].min_delay = (steps1 > 0) ? base_min_delay * ((float)max_steps / (float)steps1) : 99999.0f;
    motors[2].min_delay = (steps2 > 0) ? base_min_delay * ((float)max_steps / (float)steps2) : 99999.0f;
    motors[3].min_delay = (steps3 > 0) ? fminf(base_min_delay * ((float)max_steps / (float)steps3), max_min_delay) : 99999.0f;

    // c0 sabit: hepsi aynı kalkış hızıyla başlar (tutarlı ivme profili)
    motors[0].c0 = base_c0;
    motors[1].c0 = base_c0;
    motors[2].c0 = base_c0;
    motors[3].c0 = base_c0;

    Set_Motor_Angle(0, a0);
    Set_Motor_Angle(1, a1);
    Set_Motor_Angle(2, a2);
    Set_Motor_Angle(3, a3);
}

// -----------------------------------------------------------------------
// STEP MOTOR — HIZ AYARLA (tüm motors[])
// Kullanım: Set_All_Motors_Speed(100.0f, 2000.0f)
//   - min_delay küçük  → hızlı
//   - c0 büyük         → yumuşak kalkış
// -----------------------------------------------------------------------
void Set_All_Motors_Speed(float min_delay, float c0) {
    for (int i = 0; i < 4; i++) {
        motors[i].min_delay     = min_delay;
        motors[i].c0            = c0;
        motors[i].current_delay = c0;
    }
}

// -----------------------------------------------------------------------
// STEP MOTOR — GÜNCELLE (tek Stepper_t* için)
// Austin (2004) trapezoidal hız profili.
// İvmelenme: steps_to_go > n  → delay azalt
// Yavaşlama: steps_to_go <= n → delay artır
// -----------------------------------------------------------------------
void Stepper_Update(Stepper_t* m) {
    if (!m->is_moving) return;

    if (m->current_pos == m->target_pos) {
        m->is_moving = 0;
        m->n = 0;
        return;
    }

    uint16_t now = get_micros();
    if ((uint16_t)(now - m->last_step_time) < (uint16_t)m->current_delay) return;

    // Adım at
    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_SET);
    for (volatile int k = 0; k < 5; k++);
    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
    m->last_step_time = now;

    if (m->target_pos > m->current_pos) m->current_pos++;
    else                                m->current_pos--;

    long steps_to_go = labs(m->target_pos - m->current_pos);

    if (steps_to_go > m->n) {
        // İvmelenme bölgesi
        if (m->current_delay > m->min_delay) {
            m->n++;
            float new_delay = m->current_delay
                - (2.0f * m->current_delay) / (4.0f * (float)m->n + 1.0f);
            m->current_delay = (new_delay < m->min_delay) ? m->min_delay : new_delay;
        }
    } else {
        // Yavaşlama bölgesi
        if (steps_to_go > 0 && m->n > 0) {
            float new_delay = m->current_delay
                + (2.0f * m->current_delay) / (4.0f * (float)m->n + 1.0f);
            m->current_delay = new_delay;
            m->n--;
        }
    }
}

// -----------------------------------------------------------------------
// STEP MOTOR — ANA DÖNGÜ GÜNCELLEYİCİ (motors[0..3])
// Stepper_Update üzerine refactor edildi — tek kaynak, tutarlı davranış
// -----------------------------------------------------------------------
void Update_Motors(void) {
    for (int i = 0; i < 4; i++) {
        Stepper_Update(&motors[i]);

        // Debug snapshot — Live Expressions her iterasyonda buradan okur
        debug_delay[i]      = motors[i].current_delay;
        debug_n[i]          = motors[i].n;
        debug_steps_to_go[i]= labs(motors[i].target_pos - motors[i].current_pos);
    }
}

// -----------------------------------------------------------------------
// HOMING
// -----------------------------------------------------------------------
void Go_Home(int index) {
    // FAZ 1: Switch'e koş
    if (index == 0)
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_SET);
    else if (index == 2)
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_SET);

    while (HAL_GPIO_ReadPin(motors[index].limit_port, motors[index].limit_pin) == GPIO_PIN_RESET) {
        HAL_GPIO_WritePin(motors[index].step_port, motors[index].step_pin, GPIO_PIN_SET);
        delay_us_home(150);
        HAL_GPIO_WritePin(motors[index].step_port, motors[index].step_pin, GPIO_PIN_RESET);
        delay_us_home(150);
    }

    // FAZ 2: Switch'ten kurtul
    if (index == 0)
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_RESET);
    else if (index == 2)
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(motors[index].dir_port, motors[index].dir_pin, GPIO_PIN_RESET);

    while (HAL_GPIO_ReadPin(motors[index].limit_port, motors[index].limit_pin) == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(motors[index].step_port, motors[index].step_pin, GPIO_PIN_SET);
        delay_us_home(500);
        HAL_GPIO_WritePin(motors[index].step_port, motors[index].step_pin, GPIO_PIN_RESET);
        delay_us_home(500);
    }

    motors[index].current_pos = 0;
    motors[index].target_pos  = 0;
}

// -----------------------------------------------------------------------
// GRİPPER — İÇ YARDIMCI FONKSİYONLAR
// -----------------------------------------------------------------------

// Parmak motorunu hedef adıma gönder
// dir: 1 = kapat (GPIO_PIN_SET), 0 = aç (GPIO_PIN_RESET)
static void Gripper_SetFinger(long target_steps, uint8_t dir) {
    Stepper_t* m     = &gripper.finger;
    m->target_pos    = target_steps;
    m->is_moving     = 1;
    m->n             = 0;
    m->current_delay = m->c0;
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
        dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Rotasyon motorunu açıya gönder (göreceli hareket)
static void Gripper_SetRotate(float angle_deg) {
    Stepper_t* m = &gripper.rotate;

    long steps = (long)(fabsf(angle_deg) / 1.8f * 16.0f * GRIPPER_GEAR_RATIO);

    if (angle_deg >= 0) {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_SET);
        m->target_pos = m->current_pos + steps;
    } else {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
        m->target_pos = m->current_pos - steps;
    }

    m->is_moving     = 1;
    m->n             = 0;
    m->current_delay = m->c0;
}

// -----------------------------------------------------------------------
// GRİPPER — STATE MACHINE (while içinde çağrılacak)
// -----------------------------------------------------------------------
// Operasyon sıraları:
//   cmd=1 → GRIPPING → ROTATING → IDLE
//   cmd=2 → RELEASING → UNROTATING → IDLE
// -----------------------------------------------------------------------
void Gripper_Update(void) {
    Stepper_Update(&gripper.finger);
    Stepper_Update(&gripper.rotate);

    switch (gripper.state) {
        case GRIPPER_IDLE:
            break;

        case GRIPPER_GRIPPING:
            if (!gripper.finger.is_moving) {
                Gripper_SetRotate(GRIPPER_ROTATE_DEG);
                gripper.state = GRIPPER_ROTATING;
            }
            break;

        case GRIPPER_ROTATING:
            if (!gripper.rotate.is_moving) {
                gripper.state = GRIPPER_IDLE;
            }
            break;

        case GRIPPER_RELEASING:
            if (!gripper.finger.is_moving) {
                Gripper_SetRotate(-GRIPPER_ROTATE_DEG);
                gripper.state = GRIPPER_UNROTATING;
            }
            break;

        case GRIPPER_UNROTATING:
            if (!gripper.rotate.is_moving) {
                gripper.state = GRIPPER_IDLE;
            }
            break;
    }
}

// -----------------------------------------------------------------------
// GRİPPER — KOMUT GİRİŞİ
// -----------------------------------------------------------------------
void Gripper_Command(uint8_t cmd) {
    switch (cmd) {
        case 1:
            // Tam sekans: parmakları kapat → döndür → IDLE
            Gripper_SetFinger(GRIPPER_GRIP_STEPS, 1);
            gripper.state = GRIPPER_GRIPPING;
            break;

        case 2:
            // Bırak: parmakları aç → geri döndür → IDLE
            Gripper_SetFinger(0, 0);
            gripper.state = GRIPPER_RELEASING;
            break;

        default:
            break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

    // UART interrupt başlat
    HAL_UART_Receive_IT(&huart1, rxBuffer, RX_BUFFER_SIZE);

    // Timer ayarları (Prescaler: 83 → 1MHz @ 84MHz APB2)
    __HAL_TIM_SET_PRESCALER(&htim1, 83);
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start(&htim1);

    // ---------------------------------------------------------------
    // Motor tanımlamaları
    // ---------------------------------------------------------------
    Motor_Init(0, GPIOC, GPIO_PIN_9,  GPIOC, GPIO_PIN_8,  GPIOA, GPIO_PIN_7);
    // Motor0_STEP:PC9 | Motor0_DIR:PC8  | Motor0_LSWITCH:PA7

    Motor_Init(1, GPIOB, GPIO_PIN_5,  GPIOB, GPIO_PIN_4,  GPIOB, GPIO_PIN_6);
    // Motor1_STEP:PB5 | Motor1_DIR:PB4  | Motor1_LSWITCH:PB6

    Motor_Init(2, GPIOA, GPIO_PIN_6,  GPIOA, GPIO_PIN_5,  GPIOC, GPIO_PIN_7);
    // Motor2_STEP:PA6 | Motor2_DIR:PA5  | Motor2_LSWITCH:PC7

    Motor_Init(3, GPIOC, GPIO_PIN_5,  GPIOC, GPIO_PIN_6,  GPIOB, GPIO_PIN_15);
    // Motor3_STEP:PC5 | Motor3_DIR:PC6  | Motor3_LSWITCH:PB15

    // ---------------------------------------------------------------
    // Gripper motor tanımlamaları
    // *** PİNLERİ KENDİ DONANIM BAĞLANTINA GÖRE DEĞİŞTİR ***
    // ---------------------------------------------------------------
    Stepper_Init(&gripper.finger,
        GPIOC, GPIO_PIN_3,   // finger STEP
        GPIOC, GPIO_PIN_2,   // finger DIR
        GPIOC, GPIO_PIN_0);  // finger LIMIT

    Stepper_Init(&gripper.rotate,
        GPIOC, GPIO_PIN_10,  // rotate STEP
        GPIOC, GPIO_PIN_11,  // rotate DIR
        GPIOC, GPIO_PIN_12); // rotate LIMIT

    gripper.state = GRIPPER_IDLE;

    // ---------------------------------------------------------------
    // Homing
    // ---------------------------------------------------------------
    Go_Home(0);
    Go_Home(3);
    Go_Home(1);
    Go_Home(2);

    // ---------------------------------------------------------------
    // HIZ AYARI — ANA NOKTA
    // İlk argüman: min_delay (µs) — düşürünce hızlanır
    // İkinci argüman: c0 (µs)     — artırınca yumuşak kalkış
    // ---------------------------------------------------------------
    Set_All_Motors_Speed(200.0f, 2000.0f);

    // Enable sinyali
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

    // ---------------------------------------------------------------
    // İlk pozisyonlar — sıfırdan senkron hareket
    // Tüm motorlar aynı anda başlar ve biter.
    // ---------------------------------------------------------------
    Set_All_Motors_Synced(
         180.0f * 235.0f,
          90.0f *  32.86f,
         -90.0f *  33.86f,
          90.0f *  -6.77f
    );

    while (motors[0].is_moving || motors[1].is_moving ||
           motors[2].is_moving || motors[3].is_moving) {
        Update_Motors();
    }

    // Pozisyon sıfırlama — burası mekanik sıfır noktası
    motors[0].current_pos = 0;  motors[0].target_pos = 0;
    motors[1].current_pos = 0;  motors[1].target_pos = 0;
    motors[2].current_pos = 0;  motors[2].target_pos = 0;
    motors[3].current_pos = 0;  motors[3].target_pos = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // ---------------------------------------------------------------
      // DEBUG: test_gripper_cmd STM32CubeIDE Live Watch ile set edilir
      // ---------------------------------------------------------------
      if (test_gripper_cmd != 0) {
          Gripper_Command(test_gripper_cmd);
          test_gripper_cmd = 0;
      }

      //new_packet_received = 1;

      Update_Motors();
      Gripper_Update();

      // ---------------------------------------------------------------
      // 1) Yeni paket geldi → hareketi başlat
      // ---------------------------------------------------------------
      if (new_packet_received == 1 && cmd_state == CMD_IDLE) {
          new_packet_received = 0;

          switch (current_mode) {
              case Mode_MANUAL:
              case Mode_TEACHPAD:
              case Mode_AUTO:
                  Set_All_Motors_Synced(
                      robotTarget.Dist1  * (-200.0f),
                      180.0f / 3.14159274f * robotTarget.Theta2 * -29.86f,
                      180.0f / 3.14159274f * (robotTarget.Theta3 + robotTarget.Theta2) * 15.55f,
                      180.0f / 3.14159274f * robotTarget.Theta4 * -6.0f
                  );
                  break;
          }

          pending_gripper = (uint8_t)robotTarget.Gripper_cmd;
          cmd_state = CMD_MOVING;
      }

      // ---------------------------------------------------------------
      // 2) Motorlar durdu mu?
      // ---------------------------------------------------------------
      if (cmd_state == CMD_MOVING &&
          !motors[0].is_moving &&
          !motors[1].is_moving &&
          !motors[2].is_moving &&
          !motors[3].is_moving)
      {
          if (pending_gripper != 0) {
              Gripper_Command(pending_gripper);
              pending_gripper = 0;
              cmd_state = CMD_GRIPPING;
          } else {
              Send_Done();
              cmd_state = CMD_IDLE;
          }
      }

      // ---------------------------------------------------------------
      // 3) Gripper işlemi bitti mi?
      // ---------------------------------------------------------------
      if (cmd_state == CMD_GRIPPING && gripper.state == GRIPPER_IDLE) {
          Send_Done();
          cmd_state = CMD_IDLE;
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, gripper_fin_dir_Pin|gripper_fin_step_Pin|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8|GPIO_PIN_9|gripper_rot_step_Pin|gripper_rot_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|Enable_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : gripper_fin_switch_Pin unknown_Pin gripper_rot_switch_Pin */
  GPIO_InitStruct.Pin = gripper_fin_switch_Pin|unknown_Pin|gripper_rot_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : gripper_fin_dir_Pin gripper_fin_step_Pin PC5 PC6
                           PC8 PC9 gripper_rot_step_Pin gripper_rot_dir_Pin */
  GPIO_InitStruct.Pin = gripper_fin_dir_Pin|gripper_fin_step_Pin|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_8|GPIO_PIN_9|gripper_rot_step_Pin|gripper_rot_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 Enable_pin_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|Enable_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
