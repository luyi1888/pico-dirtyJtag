#include <hardware/clocks.h>
#include "hardware/dma.h"
#include "pio_jtag.h"
#include "cjtag.pio.h"

void jtag_task();//to process USB OUT packets while waiting for DMA to finish


static bool oscan1_mode = false;
static bool isInit = false;

#define DMA

#if 0
static bool pins_source = false; //false: PIO, true: GPIO

static void switch_pins_source(const pio_jtag_inst_t *jtag, bool gpio)
{
    if (pins_source != gpio)
    {
        if (gpio)
        {
            gpio_put(jtag->pin_tdi, gpio_get(jtag->pin_tdi));
            gpio_set_function(jtag->pin_tdi, GPIO_FUNC_SIO);
            gpio_put(jtag->pin_tck, gpio_get(jtag->pin_tck));
            gpio_set_function(jtag->pin_tck, GPIO_FUNC_SIO);
            gpio_set_dir_out_masked((1 << jtag->pin_tdi) | (1 << jtag->pin_tck));
        }
        else
        {
            gpio_set_function(jtag->pin_tdi, GPIO_FUNC_PIO0);
            gpio_set_function(jtag->pin_tck, GPIO_FUNC_PIO0);
        }
        pins_source = gpio;
    }
}
#endif



#ifdef DMA

static int tx_dma_chan = -1;
static int rx_dma_chan;
static dma_channel_config tx_c;
static dma_channel_config rx_c;
#endif

void dma_init()
{
#ifdef DMA
    if (tx_dma_chan == -1)
    {
        // Configure a channel to write a buffer to PIO0
        // SM0's TX FIFO, paced by the data request signal from that peripheral.
        tx_dma_chan = dma_claim_unused_channel(true);
        tx_c = dma_channel_get_default_config(tx_dma_chan);
        channel_config_set_transfer_data_size(&tx_c, DMA_SIZE_8);
        channel_config_set_read_increment(&tx_c, true);
        channel_config_set_dreq(&tx_c, DREQ_PIO0_TX2);
            dma_channel_configure(
            tx_dma_chan,
            &tx_c,
            &pio0_hw->txf[2], // Write address (only need to set this once)
            NULL,             // Don't provide a read address yet
            0,                // Don't provide the count yet
            false             // Don't start yet
        );
        // Configure a channel to read a buffer from PIO0
        // SM0's RX FIFO, paced by the data request signal from that peripheral.
        rx_dma_chan = dma_claim_unused_channel(true);
        rx_c = dma_channel_get_default_config(rx_dma_chan);
        channel_config_set_transfer_data_size(&rx_c, DMA_SIZE_8);
        channel_config_set_write_increment(&rx_c, false);
        channel_config_set_read_increment(&rx_c, false);
        channel_config_set_dreq(&rx_c, DREQ_PIO0_RX2);
        dma_channel_configure(
            rx_dma_chan,
            &rx_c,
            NULL,             // Dont provide a write address yet
            &pio0_hw->rxf[2], // Read address (only need to set this once)
            0,                // Don't provide the count yet
            false             // Don't start yet
            );
    }
#endif

}

static inline bool pio_sm_is_running_stalled(PIO pio, uint sm)
{
    check_pio_param(pio);
    check_sm_param(sm);
    pio->fdebug = 0x1000000 << sm;
    return !!(pio->fdebug & (0x1000000 << sm));
}


static inline void rv_raw_tms(uint32_t command)
{
    pio_sm_put_blocking(pio0, 1, command);
    while(!pio_sm_is_running_stalled(pio0, 1)) tight_loop_contents();
}


static inline uint32_t rv_tap_tick(uint32_t tms, uint32_t tdi)
{
    uint32_t command = 0x1;
    uint32_t rt = 0;
    if(oscan1_mode)
    {
        command |= (tdi & 0x1) << 8;
        command |= (tms & 0x1) << 9;
        command ^= 0x11111100;
        pio_sm_put_blocking(pio0, 2, command);
        while(!pio_sm_is_running_stalled(pio0, 2)) tight_loop_contents();
        rt = pio_sm_get_blocking(pio0, 2);
    }
    else
    {
        command |= (tms & 0x1) << 8;
        pio_sm_put_blocking(pio0, 1, command);
        while(!pio_sm_is_running_stalled(pio0, 1)) tight_loop_contents();
    }
    return rt;
}

static inline void rvl_tap_oac(void)
{
	rv_tap_tick(0 , 0);
	rv_tap_tick(0 , 0);
	rv_tap_tick(1 , 0);
	rv_tap_tick(1 , 0);
}

static inline void rvl_tap_ec(void)
{
	rv_tap_tick(0 , 0);
	rv_tap_tick(0 , 0);
	rv_tap_tick(0 , 0);
	rv_tap_tick(1 , 0);
}

static inline void rvl_tap_cp(void)
{
	rv_tap_tick(0 , 0);
	rv_tap_tick(0 , 0);
	rv_tap_tick(0 , 0);
	rv_tap_tick(0 , 0);
}

static inline void rvl_tap_escape(uint8_t n)
{
	pio_sm_put_blocking(pio0, 0, n);
    while(!pio_sm_is_running_stalled(pio0, 0)) tight_loop_contents();
}

void initOscan1()
{
    oscan1_mode = false;
    rvl_tap_escape(8);
    rv_raw_tms(0xFFFFFF16);
    rv_tap_tick(0, 0);
    rvl_tap_escape(6);
    rvl_tap_oac();
    rvl_tap_ec();
    rvl_tap_cp();
    oscan1_mode = true;
    //rv_tap_tick(0, 0);// TLR to RTI
}

void unsetOscan1()
{
    oscan1_mode = false;
}

void __time_critical_func(pio_jtag_write_blocking)(const pio_jtag_inst_t *jtag, const uint8_t *bsrc, size_t len) 
{
    PIO pio = pio0;
    size_t byte_length = (len+7 >> 3);
    size_t last_shift = ((byte_length << 3) - len);
    size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
    io_rw_8 *txfifo = (io_rw_8 *) &pio->txf[2];
    io_rw_8 *rxfifo = (io_rw_8 *) &pio->rxf[2];
    uint8_t x; // scratch local to receive data
    //kick off the process by sending the len to the tx pipeline
    *(io_rw_32*)txfifo = len-1;
#ifdef DMA
    if (byte_length > 4)
    {
        dma_init();
        channel_config_set_read_increment(&tx_c, true);
        channel_config_set_write_increment(&rx_c, false);
        dma_channel_set_config(rx_dma_chan, &rx_c, false);
        dma_channel_set_config(tx_dma_chan, &tx_c, false);
        dma_channel_transfer_to_buffer_now(rx_dma_chan, (void*)&x, rx_remain);
        dma_channel_transfer_from_buffer_now(tx_dma_chan, (void*)bsrc, tx_remain);
        while (dma_channel_is_busy(rx_dma_chan))
        {
            jtag_task();
            tight_loop_contents();
        }
        // stop the compiler hoisting a non volatile buffer access above the DMA completion.
        __compiler_memory_barrier();
    }
    else
#endif
    {
        while (tx_remain || rx_remain) 
        {
            if (tx_remain && !pio_sm_is_tx_fifo_full(pio, 2))
            {
                *txfifo = *bsrc++;
                --tx_remain;
            }
            if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, 2))
            {
                x = *rxfifo;
                --rx_remain;
            }
        }
    }
}

void __time_critical_func(pio_jtag_write_read_blocking)(const pio_jtag_inst_t *jtag, const uint8_t *bsrc, uint8_t *bdst,
                                                         size_t len) 
{
    PIO pio = pio0;
    size_t byte_length = (len+7 >> 3);
    size_t last_shift = ((byte_length << 3) - len);
    size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
    uint8_t* rx_last_byte_p = &bdst[byte_length-1];
    io_rw_8 *txfifo = (io_rw_8 *) &pio->txf[2];
    io_rw_8 *rxfifo = (io_rw_8 *) &pio->rxf[2];
    //kick off the process by sending the len to the tx pipeline
    *(io_rw_32*)txfifo = len-1;
#ifdef DMA
    if (byte_length > 4)
    {
        dma_init();
        channel_config_set_read_increment(&tx_c, true);
        channel_config_set_write_increment(&rx_c, true);
        dma_channel_set_config(rx_dma_chan, &rx_c, false);
        dma_channel_set_config(tx_dma_chan, &tx_c, false);
        dma_channel_transfer_to_buffer_now(rx_dma_chan, (void*)bdst, rx_remain);
        dma_channel_transfer_from_buffer_now(tx_dma_chan, (void*)bsrc, tx_remain);
        while (dma_channel_is_busy(rx_dma_chan))
        {
            jtag_task();
            tight_loop_contents();
        }
        // stop the compiler hoisting a non volatile buffer access above the DMA completion.
        __compiler_memory_barrier();
    }
    else
#endif
    {
        while (tx_remain || rx_remain) 
        {
            if (tx_remain && !pio_sm_is_tx_fifo_full(pio, 2))
            {
                *txfifo = *bsrc++;
                --tx_remain;
            }
            if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, 2))
            {
                *bdst++ = *rxfifo;
                --rx_remain;
            }
        }
    }
    //fix the last byte
    if (last_shift) 
    {
        *rx_last_byte_p = *rx_last_byte_p << last_shift;
    }
}

uint8_t __time_critical_func(pio_jtag_write_tms_blocking)(const pio_jtag_inst_t *jtag, bool tdi, bool tms, size_t len)
{
    PIO pio = pio0;
    size_t byte_length = (len+7 >> 3);
    size_t last_shift = ((byte_length << 3) - len);
    size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
    io_rw_8 *txfifo = (io_rw_8 *) &pio->txf[2];
    io_rw_8 *rxfifo = (io_rw_8 *) &pio->rxf[2];
    uint8_t x; // scratch local to receive data
    //in cJTAG, tdi is reverse
    uint8_t tdi_word = tdi ? 0x00 : 0xFF;
    uint32_t temp;
    temp = len - 1;
    //kick off the process by sending the len to the tx pipeline
    if(tms)
    {
        *(io_rw_32*)txfifo = (len - 1) | 0x80000000;
    }
    else
    {
        *(io_rw_32*)txfifo = (len - 1) & 0x7FFFFFFF;
    }
#ifdef DMA
    if (byte_length > 4)
    {   
        dma_init();
        channel_config_set_read_increment(&tx_c, false);
        channel_config_set_write_increment(&rx_c, false);
        dma_channel_set_config(rx_dma_chan, &rx_c, false);
        dma_channel_set_config(tx_dma_chan, &tx_c, false);
        dma_channel_transfer_to_buffer_now(rx_dma_chan, (void*)&x, rx_remain);
        dma_channel_transfer_from_buffer_now(tx_dma_chan, (void*)&tdi_word, tx_remain);
        while (dma_channel_is_busy(rx_dma_chan))
        {
            jtag_task();
            tight_loop_contents();
        }
        // stop the compiler hoisting a non volatile buffer access above the DMA completion.
        __compiler_memory_barrier();
    }
    else
#endif
    {
        while (tx_remain || rx_remain) 
        {
            if (tx_remain && !pio_sm_is_tx_fifo_full(pio, 2)) 
            {
                *txfifo = tdi_word;
                --tx_remain;
            }
            if (rx_remain && !pio_sm_is_rx_fifo_empty(pio, 2)) 
            {
                x = *rxfifo;
                --rx_remain;
            }
        }
    }
    //fix the last byte
    if (last_shift)
    {
        x = x << last_shift;
    }
    return x;
}

static void init_pins(uint pin_tck, uint pin_tdi, uint pin_tdo, uint pin_tms, uint pin_rst, uint pin_trst)
{
    #if !( BOARD_TYPE == BOARD_QMTECH_RP2040_DAUGHTERBOARD )
    // emulate open drain with pull up and direction
    gpio_pull_up(pin_rst);
    gpio_clr_mask((1u << pin_tms) | (1u << pin_rst) | (1u << pin_trst));
    gpio_init_mask((1u << pin_tms) | (1u << pin_rst) | (1u << pin_trst));
    gpio_set_dir_masked( (1u << pin_tms) | (1u << pin_trst), 0xffffffffu);
    gpio_set_dir(pin_rst, false);
    #else
    gpio_clr_mask((1u << pin_tms));
    gpio_init_mask((1u << pin_tms));
    gpio_set_dir_masked( (1u << pin_tms), 0xffffffffu);
    #endif
}

void init_jtag(pio_jtag_inst_t* jtag, uint freq, uint pin_tck, uint pin_tdi, uint pin_tdo, uint pin_tms, uint pin_rst, uint pin_trst)
{
    oscan1_mode = false;
    //uint nitx = 0;
    if(!isInit)
    {
        PIO pio = pio0;
        uint offset0 = pio_add_program(pio, &cjtag_escape_program);
        cjtag_escape_program_init(pio, 0, offset0, pin_tck, pin_tms);
        uint offset1 = pio_add_program(pio, &cjtag_writeTMS_program);
        cjtag_writeTMS_program_init(pio, 1, offset1, pin_tck, pin_tms);
        uint offset2 = pio_add_program(pio, &cjtag_tick_program);
        cjtag_tick_program_init(pio, 2, offset2, pin_tck, pin_tms);
        isInit = true;
    }
}

void jtag_set_clk_freq(const pio_jtag_inst_t *jtag, uint freq_khz) {
    uint clk_sys_freq_khz = clock_get_hz(clk_sys) / 1000;
    uint32_t divider = (clk_sys_freq_khz / freq_khz) / 4;
    divider = (divider < 2) ? 2 : divider; //max reliable freq 
    pio_sm_set_clkdiv_int_frac(pio0, jtag->sm, divider, 0);
}

void jtag_transfer(const pio_jtag_inst_t *jtag, uint32_t length, const uint8_t* in, uint8_t* out)
{

    if (out)
        pio_jtag_write_read_blocking(jtag, in, out, length);
    else
        pio_jtag_write_blocking(jtag, in, length);

}


uint8_t jtag_strobe(const pio_jtag_inst_t *jtag, uint32_t length, bool tms, bool tdi)
{
    if(!oscan1_mode)
    {
        initOscan1();
    }
    return pio_jtag_write_tms_blocking(jtag, tdi, tms, length);
}



static uint8_t toggle_bits_out_buffer[4];
static uint8_t toggle_bits_in_buffer[4];

void jtag_set_tdi(const pio_jtag_inst_t *jtag, bool value)
{
    toggle_bits_out_buffer[0] = value ? 1u << 7 : 0;
}

void jtag_set_clk(const pio_jtag_inst_t *jtag, bool value)
{
    if (value)
    {
        toggle_bits_in_buffer[0] = 0; 
        pio_jtag_write_read_blocking(jtag, toggle_bits_out_buffer, toggle_bits_in_buffer, 1);
    }
}

bool jtag_get_tdo(const pio_jtag_inst_t *jtag)
{
    return !! toggle_bits_in_buffer[0];
}


