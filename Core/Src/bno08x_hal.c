#include "main.h"
#include "stm32f3xx_hal.h"
#include "sh2_hal.h"
#include <stdio.h>
#include <string.h>


extern I2C_HandleTypeDef hi2c1;

#define BNO_ADDR (0x4A << 1)

static int hal_open(sh2_Hal_t *self)
{
    (void)self;

    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    HAL_StatusTypeDef rc;

    rc = HAL_I2C_Master_Transmit(&hi2c1, BNO_ADDR, softreset_pkt, sizeof(softreset_pkt), 100);

    printf("hal_open tx rc = %d, addr = 0x%02X\r\n", rc, BNO_ADDR);

    if (rc != HAL_OK)
    {
        return -1;
    }

    HAL_Delay(300);
    return 0;
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;

    uint8_t header[4];
    uint16_t packet_size;
    uint16_t cargo_remaining;
    uint16_t cargo_read = 0;
    HAL_StatusTypeDef status;

    *t_us = HAL_GetTick() * 1000;

    // If INT is high, no packet ready
    if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_SET)
    {
        return 0;
    }

    // First read: get the packet header
    status = HAL_I2C_Master_Receive(&hi2c1, BNO_ADDR, header, 4, 20);
    if (status != HAL_OK)
    {
        return 0;
    }

    packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_size &= ~0x8000;   // clear continuation bit

    if (packet_size < 4 || packet_size > len)
    {
        return 0;
    }

    // Copy first header into destination buffer
    pBuffer[0] = header[0];
    pBuffer[1] = header[1];
    pBuffer[2] = header[2];
    pBuffer[3] = header[3];

    cargo_remaining = packet_size - 4;

    // Read remaining cargo
    while (cargo_remaining > 0)
    {
        uint16_t chunk = cargo_remaining;

        // keep chunk small enough for stack/local buffer
        if (chunk > 32)
        {
            chunk = 32;
        }

        uint8_t tmp[36]; // 4 bytes extra for repeated header

        // Every additional I2C read starts with another 4-byte header
        status = HAL_I2C_Master_Receive(&hi2c1, BNO_ADDR, tmp, chunk + 4, 50);
        if (status != HAL_OK)
        {
            return 0;
        }

        // Skip tmp[0..3] because that is another header
        memcpy(&pBuffer[4 + cargo_read], &tmp[4], chunk);

        cargo_read += chunk;
        cargo_remaining -= chunk;
    }

    return packet_size;
}
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;

    if (HAL_I2C_Master_Transmit(&hi2c1, BNO_ADDR, pBuffer, len, 50) != HAL_OK)
    {
        return 0;
    }

    return (int)len;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return HAL_GetTick() * 1000;
}

sh2_Hal_t bno08x_hal = {
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_getTimeUs
};
