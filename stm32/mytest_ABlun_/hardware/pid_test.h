#ifndef PID_TEST_H
#define PID_TEST_H

#include "config.h"
#include "pid_motor.h"
#include "encoder.h"

// ============================================================================
// PID Test Mode Definitions
// ============================================================================
typedef enum {
    PID_TEST_STOP = 0,          // Stop Test
    PID_TEST_FORWARD,           // Forward Test
    PID_TEST_BACKWARD,          // Backward Test
    PID_TEST_LEFT_TURN,         // Left Turn Test
    PID_TEST_CIRCLE,            // Circle Test
    PID_TEST_ACCEL_BRAKE        // Acceleration Brake Test
} PID_Test_Mode;

// ============================================================================
// PID Test Control Structure
// ============================================================================
typedef struct {
    PID_Test_Mode mode;         // Current Test Mode
    uint32_t start_time;        // Test Start Time
    uint16_t target_speed;      // Target Speed
    uint8_t test_active;        // Test Active Flag
} PID_Test_t;

// ============================================================================
// Global Variable Declaration
// ============================================================================
extern PID_Test_t pid_test;

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * @brief PID Test Module Initialization
 * @note Initialize PID Controllers and Enable PID Function
 */
void PID_Test_Init(void);

/**
 * @brief PID Test Task Processing Function
 * @note Called in Main Loop to Process Various Test Modes
 */
void PID_Test_Task(void);

/**
 * @brief Start Specified PID Test
 * @param mode Test Mode
 * @param speed Target Speed (10-100)
 */
void PID_Test_Start(PID_Test_Mode mode, uint16_t speed);

/**
 * @brief Stop Current PID Test
 * @note Stop All Motors and Clear Test Status
 */
void PID_Test_Stop(void);

/**
 * @brief Send PID Test Data to Debug UART
 * @note Send Target Speed, Actual Speed, PWM Output Data
 */
void PID_Test_Send_Data(void);

/**
 * @brief Manual Control of Four Wheels
 * @param fl Front Left Wheel Speed (-100 to 100)
 * @param fr Front Right Wheel Speed (-100 to 100)
 * @param rl Rear Left Wheel Speed (-100 to 100)
 * @param rr Rear Right Wheel Speed (-100 to 100)
 */
void PID_Test_Manual_Control(int16_t fl, int16_t fr, int16_t rl, int16_t rr);

/**
 * @brief Automatic Test Sequence
 * @note Execute All Test Modes in Sequence
 */
void PID_Test_Sequence(void);

#endif /* PID_TEST_H */


