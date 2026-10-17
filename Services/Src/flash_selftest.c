#include "flash_selftest.h"
#include "w25q128.h"
#include "main.h"

#include <stddef.h>

/* The offset is not page aligned on purpose: 56 + 256 + 256 + 32 bytes means
 * 4 pages and 3 page boundaries, which exercises the splitting in w25q_write() */
#define TEST_OFFSET 200u
#define TEST_LEN 600u

static uint8_t s_pattern[TEST_LEN];

/* Position-dependent pattern: neighbouring pages differ, and no byte is 0xFF
 * (a skipped write would be invisible at such a position) */
static void fill_pattern(void)
{
    for (uint32_t i = 0u; i < TEST_LEN; i++)
    {
        uint8_t v = (uint8_t)(i * 31u + 7u);
        v ^= (uint8_t)(i >> 8);
        s_pattern[i] = (v == 0xFFu) ? 0xA5u : v;
    }
}

/* Record the first failure and pass the status through */
static status_t step(flash_selftest_t *res, const char *name, status_t st)
{
    if (st != STATUS_OK)
    {
        res->failed_step = name;
        res->status = st;
    }
    return st;
}

uint32_t flash_selftest_addr(void)
{
    const uint32_t cap = w25q_capacity();
    return (cap >= W25Q_SECTOR_SIZE) ? (cap - W25Q_SECTOR_SIZE) : 0u;
}

status_t flash_selftest_run(flash_selftest_t *res)
{
    if (res == NULL)
    {
        return STATUS_ERR_PARAM;
    }
    *res = (flash_selftest_t){.test_len = TEST_LEN};

    const uint32_t addr = flash_selftest_addr();
    if (addr == 0u)
    {
        res->failed_step = "detect";
        res->status = STATUS_ERR_ID;
        return STATUS_ERR_ID;
    }
    res->test_addr = addr;
    fill_pattern();

    status_t st;
    uint32_t t0;

    t0 = HAL_GetTick();
    st = step(res, "erase", w25q_sector_erase(addr));
    if (st != STATUS_OK)
    {
        return st;
    }
    res->erase_ms = HAL_GetTick() - t0;

    st = step(res, "blank after erase", w25q_check_blank(addr, W25Q_SECTOR_SIZE));
    if (st != STATUS_OK)
    {
        return st;
    }

    t0 = HAL_GetTick();
    st = step(res, "write", w25q_write(addr + TEST_OFFSET, s_pattern, TEST_LEN));
    if (st != STATUS_OK)
    {
        return st;
    }
    res->write_ms = HAL_GetTick() - t0;

    t0 = HAL_GetTick();
    st = step(res, "verify", w25q_verify(addr + TEST_OFFSET, s_pattern, TEST_LEN));
    if (st != STATUS_OK)
    {
        return st;
    }
    res->verify_ms = HAL_GetTick() - t0;

    /* Bytes around the pattern must be untouched: a page-wrap bug would
     * have overwritten the start of a page */
    st = step(res, "area before pattern", w25q_check_blank(addr, TEST_OFFSET));
    if (st != STATUS_OK)
    {
        return st;
    }
    st = step(res, "area after pattern",
              w25q_check_blank(addr + TEST_OFFSET + TEST_LEN,
                               (uint16_t)(W25Q_SECTOR_SIZE - TEST_OFFSET - TEST_LEN)));
    if (st != STATUS_OK)
    {
        return st;
    }

    /* Leave the sector clean */
    return step(res, "final erase", w25q_sector_erase(addr));
}