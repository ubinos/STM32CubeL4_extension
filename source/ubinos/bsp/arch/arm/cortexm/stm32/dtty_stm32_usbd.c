/*
 * Copyright (c) 2021 Sung Ho Park and CSOS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ubinos.h>

#if (INCLUDE__UBINOS__BSP == 1)

#if (UBINOS__BSP__USE_DTTY == 1)

#if (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL)

#if (STM32CUBEL4__DTTY_STM32_USBD_ENABLE == 1)

#if (INCLUDE__UBINOS__UBIK != 1)
    #error "ubik is necessary"
#endif

#include <ubinos/bsp.h>
#include <ubinos/bsp/arch.h>
#include <ubinos/bsp_ubik.h>

#include <assert.h>

#include "main.h"

#define SLEEP_TIMEMS	1

extern int _g_bsp_dtty_init;
extern int _g_bsp_dtty_in_init;
extern int _g_bsp_dtty_echo;
extern int _g_bsp_dtty_autocr;

#define DTTY_UART_ISR_WRITE_BUFFER_SIZE (512)
#define DTTY_UART_READ_BUFFER_SIZE (512)
#define DTTY_UART_WRITE_BUFFER_SIZE (1024 * 10)

#define DTTY_USBD_READ_CHECK_INTERVAL_MS 1000
#define DTTY_USBD_WRITE_CHECK_INTERVAL_MS 1000

cbuf_def_init(_g_dtty_usbd_isr_wbuf, DTTY_UART_ISR_WRITE_BUFFER_SIZE);
cbuf_def_init(_g_dtty_usbd_rbuf, DTTY_UART_READ_BUFFER_SIZE);
cbuf_def_init(_g_dtty_usbd_wbuf, DTTY_UART_WRITE_BUFFER_SIZE);

sem_pt _g_dtty_usbd_rsem = NULL;
sem_pt _g_dtty_usbd_wsem = NULL;

mutex_pt _g_dtty_usbd_putlock = NULL;
mutex_pt _g_dtty_usbd_getlock = NULL;
mutex_pt _g_dtty_usbd_resetlock = NULL;

uint32_t _g_dtty_usbd_rx_overflow_count = 0;
uint32_t _g_dtty_usbd_tx_overflow_count = 0;
uint32_t _g_dtty_usbd_reset_count = 0;

volatile uint8_t _g_dtty_usbd_need_reset = 0;
volatile uint8_t _g_dtty_usbd_need_tx_restart = 0;

volatile uint32_t _g_dtty_usbd_write_trying_size = 0;

static void _dtty_stm32_usbd_reset(void);
static int _dtty_getc_advan(char *ch_p, int blocked);

static void _dtty_stm32_usbd_reset(void)
{
    mutex_lock(_g_dtty_usbd_resetlock);

    if (_g_dtty_usbd_need_reset)
    {
        _g_dtty_usbd_need_reset = 0;
        _g_dtty_usbd_need_tx_restart = 1;

        /* Enable Power Clock*/
        __HAL_RCC_PWR_CLK_ENABLE();
        
        /* enable USB power on Pwrctrl CR2 register */
        HAL_PWREx_EnableVddUSB();

        /* Init Device Library */
        USBD_Init(&USBD_Device, &VCP_Desc, 0);
        
        /* Add Supported Class */
        USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
        
        /* Add CDC Interface Class */
        USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
        
        /* Start Device Process */
        USBD_Start(&USBD_Device);

        _g_dtty_usbd_reset_count++;

        sem_give(_g_dtty_usbd_wsem);
    }

    mutex_unlock(_g_dtty_usbd_resetlock);
}

void dtty_stm32_usbd_rx_callback(uint8_t* buf, uint32_t *len)
{
    uint8_t need_notify = 0;

    if (cbuf_get_len(_g_dtty_usbd_rbuf) == 0)
    {
        need_notify = 1;
    }
    cbuf_write(_g_dtty_usbd_rbuf, buf, *len, NULL);
    if (need_notify && _g_dtty_usbd_rsem != NULL)
    {
        sem_give(_g_dtty_usbd_rsem);
    }

    USBD_CDC_ReceivePacket(&USBD_Device);
}

void dtty_stm32_usbd_tx_callback(void)
{
    uint8_t * buf;
    uint8_t usb_status;

    if (_g_dtty_usbd_write_trying_size > 0)
    {
        cbuf_read(_g_dtty_usbd_wbuf, NULL, _g_dtty_usbd_write_trying_size, NULL);

        if (cbuf_get_len(_g_dtty_usbd_wbuf) > 0)
        {
            _g_dtty_usbd_write_trying_size = cbuf_get_len(_g_dtty_usbd_wbuf);
            buf = cbuf_get_head_addr(_g_dtty_usbd_wbuf);
            USBD_CDC_SetTxBuffer(&USBD_Device, buf, _g_dtty_usbd_write_trying_size);
            usb_status =USBD_CDC_TransmitPacket(&USBD_Device);
            if(usb_status == USBD_OK)
            {
                if (cbuf_get_len(_g_dtty_usbd_wbuf) < (_g_dtty_usbd_wbuf->size / 2))
                {
                    sem_give(_g_dtty_usbd_wsem);
                }
            }
            else
            {
                _g_dtty_usbd_need_tx_restart = 1;
                _g_dtty_usbd_write_trying_size = 0;
                if (_g_dtty_usbd_wsem != NULL)
                {
                    sem_give(_g_dtty_usbd_wsem);
                }
            }
        }
        else
        {
            _g_dtty_usbd_need_tx_restart = 1;
            _g_dtty_usbd_write_trying_size = 0;
            if (_g_dtty_usbd_wsem != NULL)
            {
                sem_give(_g_dtty_usbd_wsem);
            }
        }
    }
}

int dtty_init(void)
{
    int r;
    (void) r;

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_bsp_kernel_active)
        {
            break;
        }

        if (_g_bsp_dtty_init || _g_bsp_dtty_in_init)
        {
            break;
        }

        _g_bsp_dtty_in_init = 1;

        r = semb_create(&_g_dtty_usbd_rsem);
        assert(r == 0);
        r = semb_create(&_g_dtty_usbd_wsem);
        assert(r == 0);
        r = mutex_create(&_g_dtty_usbd_resetlock);
        assert(r == 0);
        r = mutex_create(&_g_dtty_usbd_putlock);
        assert(r == 0);
        r = mutex_create(&_g_dtty_usbd_getlock);
        assert(r == 0);

        _g_bsp_dtty_echo = 1;
        _g_bsp_dtty_autocr = 1;

        _g_dtty_usbd_need_reset = 1;

        _dtty_stm32_usbd_reset();

        _g_dtty_usbd_reset_count = 0;

        _g_bsp_dtty_init = 1;

        cbuf_clear(_g_dtty_usbd_rbuf);

        _g_bsp_dtty_in_init = 0;

        break;
    } while (1);

    return 0;
}

int dtty_enable(void)
{
    return 0;
}

int dtty_disable(void)
{
    return 0;
}

int dtty_geterror(void)
{
    return 0;
}

static int _dtty_getc_advan(char *ch_p, int blocked)
{
    int r;
    ubi_err_t ubi_err;

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        if (!blocked)
        {
            r = mutex_lock_timed(_g_dtty_usbd_getlock, 0);
        }
        else
        {
            r = mutex_lock(_g_dtty_usbd_getlock);
        }
        if (r != 0)
        {
            break;
        }

        for (;;)
        {
            if (_g_dtty_usbd_need_reset)
            {
                _dtty_stm32_usbd_reset();
            }

            ubi_err = cbuf_read(_g_dtty_usbd_rbuf, (uint8_t*) ch_p, 1, NULL);
            if (ubi_err == UBI_ERR_OK)
            {
                r = 0;
                break;
            }
            else
            {
                if (!blocked)
                {
                    break;
                }
                else
                {
                    sem_take_timedms(_g_dtty_usbd_rsem, DTTY_USBD_READ_CHECK_INTERVAL_MS);
                }
            }
        }

        if (0 == r && 0 != _g_bsp_dtty_echo)
        {
            dtty_putc(*ch_p);
        }
        
        mutex_unlock(_g_dtty_usbd_getlock);

        break;
    } while (1);

    return r;
}

int dtty_getc(char *ch_p)
{
    return _dtty_getc_advan(ch_p, 1);
}

int dtty_getc_unblocked(char *ch_p)
{
    return _dtty_getc_advan(ch_p, 0);
}

int dtty_putc(int ch)
{
    int r;
    uint8_t * buf;
    uint16_t len;
    uint32_t written;
    uint8_t data[2];
    uint8_t usb_status;

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            data[0] = (uint8_t) ch;
            len = 1;
            cbuf_write(_g_dtty_usbd_isr_wbuf, data, len, &written);
            if (written != len)
            {
                _g_dtty_usbd_tx_overflow_count++;
            }

            r = 0;
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        mutex_lock(_g_dtty_usbd_putlock);

        do
        {
            if (_g_dtty_usbd_need_reset)
            {
                _dtty_stm32_usbd_reset();
            }

            if (0 != _g_bsp_dtty_autocr && '\n' == ch)
            {
                data[0] = '\r';
                data[1] = '\n';
                len = 2;
            }
            else
            {
                data[0] = (uint8_t) ch;
                len = 1;
            }

            cbuf_write(_g_dtty_usbd_wbuf, data, len, &written);
            if (written != len)
            {
                _g_dtty_usbd_tx_overflow_count++;
            }

            if (_g_dtty_usbd_need_tx_restart)
            {
                _g_dtty_usbd_need_tx_restart = 0;
                _g_dtty_usbd_write_trying_size = cbuf_get_len(_g_dtty_usbd_wbuf);
                buf = cbuf_get_head_addr(_g_dtty_usbd_wbuf);
                USBD_CDC_SetTxBuffer(&USBD_Device, buf, _g_dtty_usbd_write_trying_size);
                usb_status =USBD_CDC_TransmitPacket(&USBD_Device);
                if(usb_status == USBD_OK)
                {
                    r = 0;
                }
                else
                {
                    _g_dtty_usbd_need_tx_restart = 1;
                    _g_dtty_usbd_write_trying_size = 0;
                    r = -1;
                }
            }
            else
            {
                r = 0;
            }

            break;
        } while (1);

        mutex_unlock(_g_dtty_usbd_putlock);

        break;
    } while (1);

    return r;
}

int dtty_flush(void)
{
    return 0;
}

int dtty_putn(const char *str, int len)
{
    int r;
    uint32_t written;

    r = -1;
    do
    {
        if (NULL == str)
        {
            r = -2;
            break;
        }

        if (0 > len)
        {
            r = -3;
            break;
        }

        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            cbuf_write(_g_dtty_usbd_isr_wbuf, (uint8_t *) str, len, &written);
            if (written != len)
            {
                _g_dtty_usbd_tx_overflow_count++;
            }
            r = written;
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        for (r = 0; r < len; r++)
        {
            dtty_putc(*str);
            str++;
        }

        break;
    } while (1);

    return r;
}

int dtty_kbhit(void)
{
    int r;

    r = -1;
    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        if (!_g_bsp_dtty_init)
        {
            dtty_init();
            if (!_g_bsp_dtty_init)
            {
                break;
            }
        }

        if (cbuf_get_len(_g_dtty_usbd_rbuf) != 0)
        {
            r = 1;
        }
        else
        {
            r = 0;
        }

        break;
    } while (1);

    return r;
}

void dtty_write_process(void *arg)
{
    uint8_t * buf;
    uint32_t len;
    cbuf_pt wbuf = _g_dtty_usbd_isr_wbuf;

    do
    {
        if (bsp_isintr() || 0 != _bsp_critcount)
        {
            break;
        }

        while (cbuf_get_len(wbuf) > 0)
        {
            buf = cbuf_get_head_addr(wbuf);
            len = cbuf_get_contig_len(wbuf);
            dtty_putn((const char *) buf, len);
            cbuf_read(wbuf, NULL, len, NULL);
        }

        break;
    } while (1);
}

#endif /* (STM32CUBEL4__DTTY_STM32_USBD_ENABLE == 1) */

#endif /* (UBINOS__BSP__DTTY_TYPE == UBINOS__BSP__DTTY_TYPE__EXTERNAL) */

#endif /* (UBINOS__BSP__USE_DTTY == 1) */

#endif /* (INCLUDE__UBINOS__BSP == 1) */

