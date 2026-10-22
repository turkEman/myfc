#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include "QMC5338.h"

// ── Exported ──────────────────────────────────────────────────────────────
QueueHandle_t magDataQueue = nullptr;

// ── Private ───────────────────────────────────────────────────────────────
static SemaphoreHandle_t s_i2cMutex = nullptr;

static constexpr uint32_t    MAG_INTERVAL_MS     = 10;   // 100 Hz ceiling
static constexpr uint32_t    MAG_HEARTBEAT_MS    = 2;    // poll granularity
static constexpr uint32_t    MAG_STACK_BYTES     = 3072;
static constexpr UBaseType_t MAG_PRIORITY        = 5;
static constexpr BaseType_t  MAG_CORE            = 1;
static constexpr uint32_t    MAG_MUTEX_TIMEOUT_MS = 15;

// ── Task ──────────────────────────────────────────────────────────────────
static void magTaskFn(void *)
{
    uint32_t lastReadMs = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(MAG_HEARTBEAT_MS));

        const uint32_t now = (uint32_t)millis();
        if ((now - lastReadMs) < MAG_INTERVAL_MS)
            continue;

        // ── Take I2C bus ──────────────────────────────────────────────────
        if (!xSemaphoreTake(s_i2cMutex, pdMS_TO_TICKS(MAG_MUTEX_TIMEOUT_MS)))
            continue;   // bus busy; retry next heartbeat

        // ── Poll STATUS.DRDY ──────────────────────────────────────────────
        const bool rdy = qmc_data_ready();

        if (!rdy)
        {
            xSemaphoreGive(s_i2cMutex);
            continue;   // mid-conversion; retry next heartbeat (≤2 ms, well within 10 ms window)
        }

        // Timestamp as close to the DRDY moment as possible.
        // Error ≤ MAG_HEARTBEAT_MS (2 ms) — acceptable at 100 Hz.
        const int64_t sample_us = esp_timer_get_time();

        // ── Read calibrated vector ────────────────────────────────────────
        Vector3f mag;
        const bool ok = qmc_read_data(&mag);
        xSemaphoreGive(s_i2cMutex);

        if (!ok)
            continue;   // I2C error; lastReadMs not updated so we retry

        // ── Publish ───────────────────────────────────────────────────────
        MagPacket pkt;
        pkt.mag          = mag;
        pkt.timestamp_us = sample_us;
        xQueueOverwrite(magDataQueue, &pkt);
        lastReadMs = now;
    }
}

// ── Public entry point ────────────────────────────────────────────────────
void magTask_start(SemaphoreHandle_t i2cMutex)
{
    s_i2cMutex   = i2cMutex;
    magDataQueue = xQueueCreate(1, sizeof(MagPacket));
    configASSERT(magDataQueue);

    xTaskCreatePinnedToCore(
        magTaskFn, "mag",
        MAG_STACK_BYTES, nullptr,
        MAG_PRIORITY, nullptr,
        MAG_CORE);
}