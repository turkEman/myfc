#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "BMP180.h"

// ── Exported symbols (extern in main file and baro consumer) ─────────────
QueueHandle_t baroDataQueue = nullptr;

// ── Private ───────────────────────────────────────────────────────────────
static SemaphoreHandle_t s_i2cMutex = nullptr;

static constexpr uint32_t BARO_PERIOD_MS = 1;   // conversion time now dictates rate (~34 Hz at OSS=3)
static constexpr uint32_t BARO_TEMP_PERIOD_MS = 1000; // 1 Hz temp refresh
static constexpr uint32_t BARO_HEARTBEAT_MS = 1;      // 1 ms poll — tighter than mag
static constexpr uint32_t BARO_MUTEX_WAIT_MS = 15;
static constexpr uint32_t BARO_STACK_BYTES = 3072;
static constexpr UBaseType_t BARO_PRIORITY = 6; // above mag (5)
static constexpr BaseType_t BARO_CORE = 1;

enum BaroState
{
    BARO_IDLE,
    BARO_TEMP_CMD,
    BARO_TEMP_WAIT,
    BARO_TEMP_READ,
    BARO_PRESS_CMD,
    BARO_PRESS_WAIT,
    BARO_PRESS_READ,
};

static void baroTaskFn(void *)
{
    BaroState state = BARO_IDLE;
    uint32_t cmdIssuedMs = 0;
    uint32_t lastCycleMs = 0;
    uint32_t lastTempMs = 0;
    bool tempValid = false;
    float lastTempC = 0.0f;

    for (;;)
    {
        uint32_t now = (uint32_t)millis();

        switch (state)
        {
        // ── Wait for next 50 Hz cycle ─────────────────────────────────────
        case BARO_IDLE:
            if ((now - lastCycleMs) < BARO_PERIOD_MS)
                break;

            // Time for a fresh temperature read?
            state = (!tempValid || (now - lastTempMs) >= BARO_TEMP_PERIOD_MS)
                        ? BARO_TEMP_CMD
                        : BARO_PRESS_CMD;
            break;

        // ── Temperature path ──────────────────────────────────────────────
        case BARO_TEMP_CMD:
            if (xSemaphoreTake(s_i2cMutex, pdMS_TO_TICKS(BARO_MUTEX_WAIT_MS)))
            {
                bmp180_start_temp(); // write 0x2E to 0xF4
                xSemaphoreGive(s_i2cMutex);
                cmdIssuedMs = now;
                state = BARO_TEMP_WAIT;
            }
            // If mutex missed, stay in BARO_TEMP_CMD — retry next heartbeat.
            break;

        case BARO_TEMP_WAIT:
            if ((now - cmdIssuedMs) >= BMP180_TEMP_WAIT_MS)
                state = BARO_TEMP_READ;
            break;

        case BARO_TEMP_READ:
            if (xSemaphoreTake(s_i2cMutex, pdMS_TO_TICKS(BARO_MUTEX_WAIT_MS)))
            {
                int32_t UT = bmp180_read_temp_raw(); // sets _B5 correctly, returns raw UT
                xSemaphoreGive(s_i2cMutex);

                // Pass actual UT — idempotent, sets _B5 to the same correct value again
                lastTempC = bmp180_compensate_temp(UT);
                tempValid = true;
                lastTempMs = now;
                state = BARO_PRESS_CMD;
            }
            break;

        // ── Pressure path ─────────────────────────────────────────────────
        case BARO_PRESS_CMD:
            if (xSemaphoreTake(s_i2cMutex, pdMS_TO_TICKS(BARO_MUTEX_WAIT_MS)))
            {
                bmp180_start_pressure(); // write 0x34|(OSS<<6) to 0xF4
                xSemaphoreGive(s_i2cMutex);
                cmdIssuedMs = now;
                state = BARO_PRESS_WAIT;
            }
            break;

        case BARO_PRESS_WAIT:
            if ((now - cmdIssuedMs) >= BMP180_PRESS_WAIT_MS)
                state = BARO_PRESS_READ;
            break;

        case BARO_PRESS_READ:
            if (xSemaphoreTake(s_i2cMutex, pdMS_TO_TICKS(BARO_MUTEX_WAIT_MS)))
            {
                int32_t UP = bmp180_read_pressure_raw();
                xSemaphoreGive(s_i2cMutex);

                // BARO_PRESS_READ in baro_task.cpp
                if (tempValid && UP != INT32_MIN)
                {
                    BaroSample sample;
                    sample.pressurePa = bmp180_compensate_pressure(UP);
                    sample.altitudeM = bmp180_pressure_to_altitude(sample.pressurePa);
                    sample.tempC = lastTempC;

                    xQueueOverwrite(baroDataQueue, &sample);
                }

                lastCycleMs = now;
                state = BARO_IDLE;
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(BARO_HEARTBEAT_MS));
    }
}

void baroTask_start(SemaphoreHandle_t i2cMutex)
{
    s_i2cMutex = i2cMutex;
    baroDataQueue = xQueueCreate(1, sizeof(BaroSample));
    configASSERT(baroDataQueue);

    xTaskCreatePinnedToCore(
        baroTaskFn,
        "baro",
        BARO_STACK_BYTES,
        nullptr,
        BARO_PRIORITY,
        nullptr,
        BARO_CORE);
}