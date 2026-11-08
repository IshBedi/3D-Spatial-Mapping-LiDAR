/* ============================================================================
   Ish Bedi
	 3D LiDAR Scan - ToF + Stepper + UART

   Table of Contents
   Line 1    : File header / overview
   Line 20   : Includes
   Line 30   : User settings and defines
   Line 77   : Global variables and forward declarations
   Line 92   : Wait_ms()
   Line 102  : I2C_Init()
   Line 119  : GPIO init functions
   Line 176  : ClockTest_Run()
   Line 193  : LED helper functions
   Line 210  : Button read helpers
   Line 219  : CheckPause()
   Line 263  : Motor helper functions
   Line 291  : Rotate11_25Deg_Measure()
   Line 304  : JourneyBack()
   Line 321  : FullSensorReinit()
   Line 355  : ResetScans()
   Line 373  : DistanceAnalyz()
   Line 405  : RunScan()
   Line 443  : RunScanAndStore()
   Line 482  : TransmitAllData()
   Line 502  : main()

   Notes for my DEmo
   - CLOCK_TEST_MODE = 1 sends 1kHz signal out on PK0 (used for AD3 testing)
   - CLOCK_TEST_MODE = 0 runs the normal scanning system
   - angles stored as angle x100 (so 1125 = 11.25 deg)
   - this took way lon
   ============================================================================ */

#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


// ============================================================================
// USER SETTINGS + CONSTANTS
// ============================================================================

// 1 = output 1 kHz on PK0 for AD3 test
// 0 = normal scan code
#define CLOCK_TEST_MODE             0

// ---------------- I2C defines ----------------
#define I2C_MCR_MFE                 0x00000010

// ---------------- Stepper defines ----------------
#define CW                          1
#define CCW                         0

#define STEPS_PER_REV               2048
#define STEPS_11_25_DEG             64
#define STEP_DELAY_MS               4
#define STEP_DELAY_FAST_MS          3
#define DEBOUNCE_DELAY_MS           20

// ---------------- Scan defines ----------------
#define SAMPLES_PER_SCAN            32
#define MAX_SCANS                   3

// ---------------- ToF timing / limits ----------------
#define TOF_TIMEOUT_MS              1200
#define TOF_MIN_VALID_MM            20
#define TOF_MAX_VALID_MM            4000

// ---------------- Transition timing ----------------
#define RETURN_SETTLE_MS            500
#define SENSOR_STOP_SETTLE_MS       100
#define SENSOR_CLEAR_SETTLE_MS      50
#define SENSOR_INIT_SETTLE_MS       150
#define SENSOR_START_SETTLE_MS      200
#define PRE_SCAN_SENSOR_SETTLE_MS   250

// ---------------- Clock test output ----------------
// At 26 MHz:
// 13000 cycles = 0.5 ms
// toggle every 0.5 ms -> 1 ms period -> 1 kHz
#define TEST_HALF_PERIOD_CYCLES     13000


// ============================================================================
// GLOBAL STATE + FORWARD DECLARATIONS
// ============================================================================

volatile uint32_t stepPositionCounter = 0;
volatile uint8_t  seqIndex            = 0;

const uint8_t stepSequence[4] = {
    0x0C,
    0x06,
    0x03,
    0x09
};

uint16_t scanAngles[MAX_SCANS][SAMPLES_PER_SCAN];
uint16_t scanDist[MAX_SCANS][SAMPLES_PER_SCAN];
uint8_t  totalScansDone = 0;

volatile uint8_t paused             = 0;
volatile uint8_t scanDirectionState = CW;

uint16_t dev    = 0x29;
int      status = 0;

void Motor_Off(void);
void Wait_ms(uint32_t ms);
int FullSensorReinit(void);


// ============================================================================
// DELAY HELPER
// ============================================================================

// simple delay function based on 26MHz clock
// not super precise but good enough for this
void Wait_ms(uint32_t ms){
    while(ms--){
        // Assumes 26 MHz system/bus timing for this delay helper
        SysTick_Wait(26000);
    }
}


// ============================================================================
// I2C INITIALIZATION
// ============================================================================

void I2C_Init(void){
    SYSCTL_RCGCI2C_R  |= SYSCTL_RCGCI2C_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    while((SYSCTL_PRGPIO_R & 0x0002) == 0){}

    // PB3 = SDA, PB2 = SCL
    GPIO_PORTB_AFSEL_R |= 0x0C;
    GPIO_PORTB_ODR_R   |= 0x08;
    GPIO_PORTB_DEN_R   |= 0x0C;
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200;

    I2C0_MCR_R  = I2C_MCR_MFE;
    I2C0_MTPR_R = 12;
}


// ============================================================================
// GPIO INITIALIZATION
// ============================================================================

void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){}
    GPIO_PORTH_DIR_R   |= 0x0F;
    GPIO_PORTH_AFSEL_R &= ~0x0F;
    GPIO_PORTH_DEN_R   |= 0x0F;
    GPIO_PORTH_AMSEL_R &= ~0x0F;
    GPIO_PORTH_PCTL_R  &= ~0x0000FFFF;
    GPIO_PORTH_DATA_R  &= ~0x0F;
}

void PortM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){}
    GPIO_PORTM_DIR_R   &= ~0x03;
    GPIO_PORTM_AFSEL_R &= ~0x03;
    GPIO_PORTM_DEN_R   |= 0x03;
    GPIO_PORTM_AMSEL_R &= ~0x03;
    GPIO_PORTM_PUR_R   |= 0x03;
}

void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){}
    GPIO_PORTJ_DIR_R   &= ~0x03;
    GPIO_PORTJ_AFSEL_R &= ~0x03;
    GPIO_PORTJ_DEN_R   |= 0x03;
    GPIO_PORTJ_AMSEL_R &= ~0x03;
    GPIO_PORTJ_PUR_R   |= 0x03;
}

void PortF_Init(void){
    // PF4 + PF0 kept for normal LED behavior only
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){}
    GPIO_PORTF_DIR_R   |= 0x11;
    GPIO_PORTF_AFSEL_R &= ~0x11;
    GPIO_PORTF_DEN_R   |= 0x11;
    GPIO_PORTF_AMSEL_R &= ~0x11;
    GPIO_PORTF_PCTL_R  &= ~0x000F000F;
    GPIO_PORTF_DATA_R  &= ~0x11;
}

void PortN_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){}
    GPIO_PORTN_DIR_R   |= 0x02;
    GPIO_PORTN_AFSEL_R &= ~0x02;
    GPIO_PORTN_DEN_R   |= 0x02;
    GPIO_PORTN_AMSEL_R &= ~0x02;
    GPIO_PORTN_DATA_R  &= ~0x02;
}

void PortK_Init(void){
    // Dedicated GPIO output pin for AD3 clock test
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R9) == 0){}

    GPIO_PORTK_DIR_R   |= 0x01;     // PK0 output
    GPIO_PORTK_AFSEL_R &= ~0x01;
    GPIO_PORTK_DEN_R   |= 0x01;
    GPIO_PORTK_AMSEL_R &= ~0x01;
    GPIO_PORTK_PCTL_R  &= ~0x0000000F;
    GPIO_PORTK_DATA_R  &= ~0x01;
}


// ============================================================================
// CLOCK TEST MODE
// ============================================================================

void ClockTest_Run(void){
    UART_printf("Clock test mode\r\n");
    UART_printf("1 kHz signal on PK0\r\n");
    UART_printf("AD3 CH1+ - PK0, AD3 GND -> board GND\r\n");

    while(1){
        // Toggle PK0 every 13000 clock cycles
        // this is used to check if clock is actully 26 MHz
        // if not then timing will be off
        GPIO_PORTK_DATA_R ^= 0x01;
        SysTick_Wait(TEST_HALF_PERIOD_CYCLES);
    }
}


// ============================================================================
// LED HELPERS
// ============================================================================

void LED_Measurement_Flash(void){
    GPIO_PORTF_DATA_R |= 0x01;
    Wait_ms(20);
    GPIO_PORTF_DATA_R &= ~0x01;
}

void LED_PreScan_Blink(void){
    GPIO_PORTF_DATA_R |= 0x10;
    Wait_ms(500);
    GPIO_PORTF_DATA_R &= ~0x10;
}

void LED_Return_On(void)  { GPIO_PORTN_DATA_R |= 0x02; }
void LED_Return_Off(void) { GPIO_PORTN_DATA_R &= ~0x02; }


// ============================================================================
// BUTTON READ HELPERS
// ============================================================================

uint8_t PM0_Pressed(void){ return ((GPIO_PORTM_DATA_R & 0x01) == 0); }
uint8_t PM1_Pressed(void){ return ((GPIO_PORTM_DATA_R & 0x02) == 0); }
uint8_t PJ0_Pressed(void){ return ((GPIO_PORTJ_DATA_R & 0x01) == 0); }
uint8_t PJ1_Pressed(void){ return ((GPIO_PORTJ_DATA_R & 0x02) == 0); }


// ============================================================================
// PAUSE / RESUME + DIRECTION CONTROL
// ============================================================================

void CheckPause(void){
    static uint8_t prevPJ0 = 0;
    static uint8_t prevPJ1 = 0;
    uint8_t curPJ0, curPJ1;

    curPJ0 = PJ0_Pressed();
    curPJ1 = PJ1_Pressed();

    if(curPJ0 && !prevPJ0){
        Wait_ms(DEBOUNCE_DELAY_MS);
        if(PJ0_Pressed()){
            paused = !paused;
            if(paused){
                Motor_Off();
                UART_printf("Paused\r\n");
            }else{
                UART_printf("Resumed\r\n");
            }
        }
    }

    if(curPJ1 && !prevPJ1){
        Wait_ms(DEBOUNCE_DELAY_MS);
        if(PJ1_Pressed()){
            scanDirectionState = (scanDirectionState == CW) ? CCW : CW;
            if(scanDirectionState == CW) UART_printf("Dir: CW scan, CCW return\r\n");
            else                         UART_printf("Dir: CCW scan, CW return\r\n");
        }
    }

    prevPJ0 = curPJ0;
    prevPJ1 = curPJ1;

    while(paused){
        curPJ0 = PJ0_Pressed();
        if(curPJ0 && !prevPJ0){
            Wait_ms(DEBOUNCE_DELAY_MS);
            if(PJ0_Pressed()){
                paused = 0;
                UART_printf("Resumed\r\n");
            }
        }
        prevPJ0 = curPJ0;
    }
}


// ============================================================================
// MOTOR HELPERS
// ============================================================================

void Motor_Off(void){
    GPIO_PORTH_DATA_R &= ~0x0F;
}

void StepMotorOneStep(uint8_t dir){
    if(dir == CW){
        seqIndex = (seqIndex + 3) & 0x03;
        if(stepPositionCounter == 0) stepPositionCounter = STEPS_PER_REV - 1;
        else                         stepPositionCounter--;
    }else{
        seqIndex = (seqIndex + 1) & 0x03;
        stepPositionCounter = (stepPositionCounter + 1) % STEPS_PER_REV;
    }
    GPIO_PORTH_DATA_R = (GPIO_PORTH_DATA_R & ~0x0F) | stepSequence[seqIndex];
}

void StepMeasure(uint8_t dir){
    StepMotorOneStep(dir);
    Wait_ms(STEP_DELAY_MS);
}

void StepFast(uint8_t dir){
    StepMotorOneStep(dir);
    Wait_ms(STEP_DELAY_FAST_MS);
}


// ============================================================================
// 11.25 DEGREE MEASUREMENT MOVE
// ============================================================================

void Rotate11_25Deg_Measure(uint8_t dir){
    uint32_t i;
    for(i = 0; i < STEPS_11_25_DEG; i++){
        CheckPause();
        StepMeasure(dir);
    }
    Motor_Off();
    Wait_ms(80);
}


// ============================================================================
// RETURN TO HOME
// ============================================================================

void JourneyBack(void){
    uint32_t i;
    uint8_t returnDir = (scanDirectionState == CW) ? CCW : CW;

    LED_Return_On();
    for(i = 0; i < STEPS_PER_REV; i++){
        StepFast(returnDir);
    }
    Motor_Off();
    LED_Return_Off();

    UART_printf("Return done, Going Home...\r\n");
    Wait_ms(RETURN_SETTLE_MS);
}


// ============================================================================
// FULL SENSOR REINITIALIZATION
// ============================================================================

// tries to reset sensor if somthing goes wrong
// usually fixes timeout issues but not always
int FullSensorReinit(void){
    int st;

    UART_printf("Reinit sensor...\r\n");

    st = VL53L1X_StopRanging(dev);
    sprintf(printf_buffer, "Stop status = %d\r\n", st);
    UART_printf(printf_buffer);
    Wait_ms(SENSOR_STOP_SETTLE_MS);

    st = VL53L1X_ClearInterrupt(dev);
    sprintf(printf_buffer, "Clear status = %d\r\n", st);
    UART_printf(printf_buffer);
    Wait_ms(SENSOR_CLEAR_SETTLE_MS);

    st = VL53L1X_SensorInit(dev);
    sprintf(printf_buffer, "Init status = %d\r\n", st);
    UART_printf(printf_buffer);
    Wait_ms(SENSOR_INIT_SETTLE_MS);

    if(st != 0){
        UART_printf("Err: sensor init failed\r\n");
        return st;
    }

    st = VL53L1X_StartRanging(dev);
    sprintf(printf_buffer, "Start status = %d\r\n", st);
    UART_printf(printf_buffer);
    Wait_ms(SENSOR_START_SETTLE_MS);

    if(st != 0){
        UART_printf("Err: start ranging failed\r\n");
        return st;
    }

    UART_printf("Sensor ready\r\n");
    return 0;
}


// ============================================================================
// RESET STORED SCAN DATA
// ============================================================================

void ResetScans(void){
    uint8_t s, i;
    totalScansDone = 0;

    for(s = 0; s < MAX_SCANS; s++){
        for(i = 0; i < SAMPLES_PER_SCAN; i++){
            scanAngles[s][i] = 0;
            scanDist[s][i]   = 0;
        }
    }

    UART_printf("Scan data cleared\r\n");
}


// ============================================================================
// READ TOF DISTANCE
// ============================================================================

// reads distance from ToF sensor
// if it times out it will try reinit agian
uint16_t DistanceAnalyz(void){
    uint8_t  dataReady   = 0;
    uint16_t distance    = 0;
    uint8_t  rangeStatus = 0;
    uint16_t timeout     = 0;

    while(dataReady == 0){
        CheckPause();
        status = VL53L1X_CheckForDataReady(dev, &dataReady);
        Wait_ms(1);
        timeout++;

        if(timeout >= TOF_TIMEOUT_MS){
            UART_printf("Warn: ToF timeout, reinit\r\n");
            FullSensorReinit();
            LED_Measurement_Flash();
            return 0;
        }
    }

    status = VL53L1X_GetRangeStatus(dev, &rangeStatus);
    status = VL53L1X_GetDistance(dev, &distance);
    status = VL53L1X_ClearInterrupt(dev);

    if(rangeStatus != 0){
        sprintf(printf_buffer, "Warn: range status = %u\r\n", rangeStatus);
        UART_printf(printf_buffer);
        distance = 0;
    }

    if(distance < TOF_MIN_VALID_MM || distance > TOF_MAX_VALID_MM){
        distance = 0;
    }

    LED_Measurement_Flash();
    return distance;
}


// ============================================================================
// ONE FULL SCAN 
// ============================================================================

// does one full scan and prints data live
// angles increment by 11.25 deg each step
void RunScan(uint8_t scanNum){
    uint8_t  i;
    uint16_t angle_x100;
    uint16_t dist;

    LED_PreScan_Blink();

    UART_printf("Pre-scan settle...\r\n");
    Wait_ms(PRE_SCAN_SENSOR_SETTLE_MS);

    if(FullSensorReinit() != 0){
        UART_printf("Err: pre-scan reinit has failed\r\n");
        return;
    }

    sprintf(printf_buffer, "SCAN_START,%u\r\n", scanNum);
    UART_printf(printf_buffer);

    for(i = 0; i < SAMPLES_PER_SCAN; i++){
        Rotate11_25Deg_Measure(scanDirectionState);
        angle_x100 = (uint16_t)i * 1125;
        dist = DistanceAnalyz();
        sprintf(printf_buffer, "LIVE,%u,%u,%u\r\n", scanNum, angle_x100, dist);
        UART_printf(printf_buffer);
    }

    Motor_Off();

    for(i = 0; i < SAMPLES_PER_SCAN; i++){
        
    }

    sprintf(printf_buffer, "SCAN_DONE,%u\r\n", scanNum);
    UART_printf(printf_buffer);

    JourneyBack();

    UART_printf("return settle...\r\n");
    Wait_ms(PRE_SCAN_SENSOR_SETTLE_MS);

    FullSensorReinit();
}


// ============================================================================
// ONE FULL SCAN WITH STORAGE
// ============================================================================

void RunScanAndStore(uint8_t scanNum){
    uint8_t  i;
    uint16_t angle_x100;
    uint16_t dist;

    LED_PreScan_Blink();

    UART_printf("Pre-scaning.\r\n");
    Wait_ms(PRE_SCAN_SENSOR_SETTLE_MS);

    if(FullSensorReinit() != 0){
        UART_printf("Err: pre-scan reinit failed\r\n");
        return;
    }

    sprintf(printf_buffer, "SCAN_START,%u\r\n", scanNum);
    UART_printf(printf_buffer);

    for(i = 0; i < SAMPLES_PER_SCAN; i++){
        Rotate11_25Deg_Measure(scanDirectionState);

        angle_x100 = (uint16_t)i * 1125;
        dist = DistanceAnalyz();

        scanAngles[scanNum - 1][i] = angle_x100;
        scanDist[scanNum - 1][i]   = dist;

        sprintf(printf_buffer, "LIVE,%u,%u,%u\r\n", scanNum, angle_x100, dist);
        UART_printf(printf_buffer);
    }

    Motor_Off();

    sprintf(printf_buffer, "SCAN_DONE,%u\r\n", scanNum);
    UART_printf(printf_buffer);

    JourneyBack();

    UART_printf("Post-return settle...\r\n");
    Wait_ms(PRE_SCAN_SENSOR_SETTLE_MS);

    FullSensorReinit();
}


// ============================================================================
// TRANSMIT ALL STORED DATA
// ============================================================================

void TransmitAllData(void){
    uint8_t  scan, sample;
    uint16_t angle_x100, dist;

    UART_printf("scan_num,angle_x100,distance_mm\r\n");

    for(scan = 0; scan < totalScansDone; scan++){
        for(sample = 0; sample < SAMPLES_PER_SCAN; sample++){
            angle_x100 = scanAngles[scan][sample];
            dist       = scanDist[scan][sample];

            sprintf(printf_buffer, "%u,%u,%u\r\n", scan + 1, angle_x100, dist);
            UART_printf(printf_buffer);
        }
    }

    UART_printf("ALL SCANS COMPLETE\r\n");
}


// ============================================================================
// MAIN
// ============================================================================

// main loop handles buttons and scan logic
// PM0 = start scan
// PM1 = send data
// PJ0 = pause
// PJ1 = flip direction
int main(void){
    uint8_t  sensorState = 0;
    uint16_t wordData    = 0;
    uint8_t  prevPM0     = 0;
    uint8_t  prevPM1     = 0;
    uint8_t  prevPJ1     = 0;
    uint8_t  curPM0, curPM1, curPJ1;

    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    UART_Init();
    PortF_Init();

    if(CLOCK_TEST_MODE){
        PortK_Init();
        ClockTest_Run();
    }

    I2C_Init();
    PortH_Init();
    PortM_Init();
    PortJ_Init();
    PortN_Init();

    UART_printf("3D scan ready\r\n");
    UART_printf("PM0=scan PM1=send PJ0=pause PJ1=flip dir\r\n");
    UART_printf("Hold PM0+PM1 to clear scan data\r\n");

    status = VL53L1X_GetSensorId(dev, &wordData);
    sprintf(printf_buffer, "Sensor ID: 0x%04X\r\n", wordData);
    UART_printf(printf_buffer);

    if(wordData == 0x0000){
        UART_printf("Err: sensor ID 0x0000, check wiring/power\r\n");
        while(1){
            GPIO_PORTF_DATA_R ^= 0x01;
            Wait_ms(200);
        }
    }

    while(sensorState == 0){
        status = VL53L1X_BootState(dev, &sensorState);
        Wait_ms(100);
    }
    UART_printf("Sensor booted\r\n");

    if(FullSensorReinit() != 0){
        UART_printf("Err: first sensor init failed\r\n");
        while(1){
            GPIO_PORTF_DATA_R ^= 0x01;
            Wait_ms(100);
        }
    }

    while(1){
        curPM0 = PM0_Pressed();
        curPM1 = PM1_Pressed();
        curPJ1 = PJ1_Pressed();

        if(curPM0 && curPM1){
            Wait_ms(DEBOUNCE_DELAY_MS);
            if(PM0_Pressed() && PM1_Pressed()){
                ResetScans();
                while(PM0_Pressed() || PM1_Pressed()){}
                prevPM0 = 0;
                prevPM1 = 0;
                continue;
            }
        }

        if(curPJ1 && !prevPJ1){
            Wait_ms(DEBOUNCE_DELAY_MS);
            if(PJ1_Pressed()){
                scanDirectionState = (scanDirectionState == CW) ? CCW : CW;
                if(scanDirectionState == CW) UART_printf("Dir: CW scan, CCW return\r\n");
                else                         UART_printf("Dir: CCW scan, CW return\r\n");
            }
        }

        if(curPM0 && !prevPM0){
            Wait_ms(DEBOUNCE_DELAY_MS);
            if(PM0_Pressed()){
                if(totalScansDone < MAX_SCANS){
                    totalScansDone++;

                    sprintf(printf_buffer, "Start scan %u/%u\r\n",
                            totalScansDone, MAX_SCANS);
                    UART_printf(printf_buffer);

                    RunScanAndStore(totalScansDone);

                    sprintf(printf_buffer,
                            "Scan %u done, move 10 cm then PM0\r\n",
                            totalScansDone);
                    UART_printf(printf_buffer);

                    if(totalScansDone == MAX_SCANS){
                        UART_printf("3 scans done, press PM1 to send\r\n");
                    }
                }else{
                    UART_printf("Already have 3 scans, PM1 to send or PM0+PM1 to clear\r\n");
                }
            }
        }

        if(curPM1 && !prevPM1){
            Wait_ms(DEBOUNCE_DELAY_MS);
            if(PM1_Pressed()){
                if(totalScansDone == 0){
                    UART_printf("No scans yet, press PM0 first\r\n");
                }else{
                    sprintf(printf_buffer, "Sending %u scan(s)...\r\n", totalScansDone);
                    UART_printf(printf_buffer);
                                    
                                        GPIO_PORTF_DATA_R |= 0x10;
                                        Wait_ms(50);
                                        GPIO_PORTF_DATA_R &= ~0x10;
                    TransmitAllData();
                }
            }
        }

        prevPM0 = curPM0;
        prevPM1 = curPM1;
        prevPJ1 = curPJ1;

        Motor_Off();
    }
}