/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception
 * Pro/Elite matrix wiring, key contacts and Pad pressure scanning.
 * IRQs only scan and enqueue; FM1 control task owns all musical operations. */
#include "smk37_hw.h"
#include "seqadv_clock.h"

static struct {u16 pad[16][5],key[37][5],pedal[5];} trim_vm_cfg;
static const u16 keytrim_init[5]={5,640,800,920,980};
static const u16 padtrim_init[5]={50,550,720,850,950};
static const u16 pedaltrim_init[5]={20,850,920,980,1010};
static volatile unsigned hw_wr, hw_rd, hw_overflow;
static u32 hw_msg[256];
static unsigned scan_started;
volatile u32 smk37_queue_overflows;
volatile u32 smk37_scan_peak_ticks, smk37_adc_peak_ticks;
static FM1_AUDIO_CODE void kbd_hw_msg_wr_atom(u32 message)
{
    unsigned flags=fm1_device_irq_save();
    unsigned next=(hw_wr+1)&255;
    if(next==hw_rd){hw_overflow=1;++smk37_queue_overflows;}
    else {hw_msg[hw_wr]=message;__asm__ volatile("csync" ::: "memory");hw_wr=next;}
    fm1_device_irq_restore(flags);
}
static u8   *io_inbuf_cur;

static FM1_AUDIO_CODE u8 get_io_val(u8 place)
{
    return !(io_inbuf_cur[place>>4]&BIT(place&0xf));
}

//--------------------KEY--------------------
typedef struct
{
  u8  tmp;
  u8  sta;
  u8  jet;
  u32 time;
  u32 cur;
  u32 last;
}t_key;
static t_key key[KEY_CNT];
static const u8 keymap[KEY_CNT][2] __attribute__((section(".data.smk37_scan")))={//琴键左到右
    {0x10,0x52},{0X20,0x72},{0x30,0x92},{0x40,0xb2},{0x50,0xd2},
    {0X60,0xf2},{0x70,0xe2},{0x80,0Xc2},{0x90,0xa2},{0xa0,0x82},
    {0Xc0,0X62},{0xd0,0x42},{0xe0,0X22},{0xf0,0x12},{0xb0,0x32},
    {0x11,0x33},{0X21,0x43},{0x31,0x53},{0x41,0X63},{0x51,0x73},
    {0X61,0x83},{0x71,0x93},{0x81,0xa3},{0x91,0xb3},{0xa1,0Xc3},
    {0Xb1,0xd3},{0Xc1,0xe3},{0xd1,0X23},{0xe1,0x13},{0xf1,0x14},
    {0X64,0X24},{0x74,0x34},{0x84,0x44},{0x94,0x54},{0xa4,0xd4},
    {0xb4,0xf4},{0Xc4,0xe4}
};

static inline FM1_AUDIO_CODE u16 key_trim(u32 val,u8 id){
    if(!val)return 0;
    unsigned x=val>1022?1:1023-val;
    const u16 *row=trim_vm_cfg.key[id];
    if(x<=row[0])return 1;
    /* The final calibration anchor is full scale, not a fifth 256 step. */
    if(x>=row[4])return 1023;
    unsigned i=1;
    while(x>row[i])++i;
    unsigned ret=256*(i-1)+(x-row[i-1])*256/(row[i]-row[i-1]);
    return ret?ret:1;
}

static FM1_AUDIO_CODE void key_scan()
{
    u8 tmp,i;
    for(i=0;i<KEY_CNT;i++)
    {
        tmp = get_io_val(keymap[i][0])|(get_io_val(keymap[i][1])<<1);
        /* Stable released/held contacts need neither debounce transitions nor
         * velocity timing. State 1 keeps advancing its two-contact timer. */
        if(tmp == key[i].sta && tmp != 1 && key[i].last == key[i].cur) {
            key[i].tmp=tmp;key[i].jet=0;continue;
        }
        if(tmp != key[i].tmp)
        {
            key[i].tmp = tmp;
            key[i].jet = 0;
        }
        else if(key[i].tmp != key[i].sta)
        {
            if(++key[i].jet >= KEY_JET)
            {
                if(tmp != 2)
                {
                    if(key[i].sta == 0)
                    {
                        if(key[i].tmp == 1) key[i].time = KEY_JET;
                        else key[i].cur = key[i].time = KEY_JET; //3
                    }
                    else if(key[i].sta == 1)
                    {
                        if(key[i].tmp == 0) key[i].cur = 0;
                        else if(key[i].time) key[i].cur = key[i].time; //3
                    }
                    else if(key[i].sta == 3)
                    {
                        if(key[i].tmp == 0) key[i].cur = key[i].time = 0;
                        else key[i].time = 0;  //1
                    }
                }
                else
                    /* Reverse contact order also ends the held note. */
                    key[i].cur = key[i].time = key[i].tmp = 0;
                key[i].jet = 0;
                key[i].sta = key[i].tmp;
            }
        }
        else
            key[i].jet = 0;
        if(key[i].sta == 1 && key[i].time > 0 && key[i].time < 4294967295)
            key[i].time++;
        if(key[i].last != key[i].cur){
            key[i].last = key[i].cur;
            kbd_hw_msg_wr_atom(KEY_MSG(i,key_trim(key[i].last,i)));
        }
    }
}

//--------------------ENCODER--------------------
typedef struct
{
  u8   val;
  u8   bak;
  u8   offset;
  u32  inc;
  u32  dec;
  u32  cur;
}t_encoder;

static t_encoder encoder[ENCODER_CNT];
static const u8 encodermap[ENCODER_CNT][2] __attribute__((section(".data.smk37_scan")))={//左到右
    {0x97,0xa7},{0x37,0x47},{0x17,0x27},{0xb7,0xc7},
    {0x57,0x67},{0x77,0x87},{0xd7,0xe7},{0xf7,0x07}
};
static const s8 encoder_delta[64] __attribute__((section(".data.smk37_scan")))={
    [0x07]=-1,[0x1e]=-1,[0x38]=-1,[0x21]=-1,
    [0x12]=1,[0x0b]=1,[0x2d]=1,[0x34]=1
};

static FM1_AUDIO_CODE void encoder_scan()
{
    u8 tmp,i;
    for(i=0;i<ENCODER_CNT;i++)
    {
        tmp = get_io_val(encodermap[i][0])|(get_io_val(encodermap[i][1])<<1);
        /* Accept a transition only on its second identical sample. The
         * former zero-threshold counter added no further debounce. */
        if(encoder[i].bak != tmp)encoder[i].bak = tmp;
        else if((encoder[i].val & 3) != tmp) {
            encoder[i].val = ((encoder[i].val <<2)|tmp)&0x3f;
            s8 delta=encoder_delta[encoder[i].val];
            if(delta<0)encoder[i].dec++;
            else if(delta>0)encoder[i].inc++;
        }
    }
}

/* Consume the signed distance between successive modulo-32-bit positions. */
static void encoder_50ms(void *unused)
{
  (void)unused;
  for (int i=0;i<ENCODER_CNT;i++){
        unsigned irq_state;local_irq_save(irq_state);
        __atomic_signal_fence(__ATOMIC_SEQ_CST);
        const u32 cur=encoder[i].inc-encoder[i].dec;
        const s32 delta=(s32)(cur-encoder[i].cur);
        const unsigned shift=encoder[i].offset;
        /* Divide toward zero in both directions. Rounding a negative
         * fraction down emits -1/+1 for a partial turn and reversal, which
         * changes a parameter at its lower bound despite no net rotation. */
        const s32 rounded=delta+(delta<0?(s32)((1U<<shift)-1U):0);
        const s32 cnt=rounded>>shift;
        /* Unsigned arithmetic defines counter wrap and reverse rotation;
         * shifting a negative signed step would be undefined C behavior. */
        encoder[i].cur+=(u32)cnt<<encoder[i].offset;
        __atomic_signal_fence(__ATOMIC_SEQ_CST);
        local_irq_restore(irq_state);
        if(cnt)kbd_hw_msg_wr_atom(ENCODER_MSG(i,(u16)cnt));
  }
}

void encoder_hw_rate(u8 id,u8 offset)
{
    if(id>=ENCODER_CNT || offset>=32)return;
    unsigned irq_state;local_irq_save(irq_state);
    __atomic_signal_fence(__ATOMIC_SEQ_CST);
    encoder[id].cur = encoder[id].inc - encoder[id].dec;
    encoder[id].offset = offset;
    __atomic_signal_fence(__ATOMIC_SEQ_CST);
    local_irq_restore(irq_state);
}

//--------------------BTN--------------------
typedef struct
{
  u8 jet;
  u8 sta;
}t_btn;
static t_btn btn[BTN_CNT];
static const u8 btnmap[BTN_CNT] ={//左到右
    0x26,0x76,
    0x96,0x56,
    0x66,0x36,
    0x75,0x65,0x25,0x45,0x15,0x86,
    0x95,0x55,0x85,0x35,0x16,0x46,
};

static void buttons_read(void)
{
    for(unsigned i=0;i<BTN_CNT;++i) {
        unsigned down=get_io_val(btnmap[i]);
        if(down==btn[i].sta)btn[i].jet=0;
        else if(++btn[i].jet>BTN_JET) {
            btn[i].sta=down;btn[i].jet=0;
            kbd_hw_msg_wr_atom(BTN_MSG(i,down));
        }
    }
}
static u32 ledio[2]={0xffffffff,0xffffffff};
static u32 ledscanio=0xffffffff;

const u8 led_map[BTN_CNT+1]={//左到右
    0x0c,0x12,
    0x00,0x10,
    0x0d,0x1d,

    0x02,0x01,0x09,0x08,0x0b,0x0a,
    0x11,0x19,0x18,0x1a,0x1c,0x1e,0x1b,
                        //play另一个灯
};
void smk37_led(unsigned id,unsigned state)
{
    if(id>=BTN_CNT+1)return;
    unsigned irq_state;local_irq_save(irq_state);
    __atomic_signal_fence(__ATOMIC_SEQ_CST);
    const unsigned pin=led_map[id];
    if(state)ledio[pin>>4]&=~BIT(pin&15);
    else ledio[pin>>4]|=BIT(pin&15);
    __atomic_signal_fence(__ATOMIC_SEQ_CST);
    local_irq_restore(irq_state);
}

static FM1_AUDIO_CODE void led_scan()
{
    static u8 ledscan_cnt=0,ledscan_sta=0;
    if(++ledscan_cnt == 48)
    {
        ledscan_cnt = 0;
        ledscan_sta = !ledscan_sta;//换一组ledcom
        SMK_REG(0x16a0cU)|=0x7000fU; /* USB-B GPIO: both LED commons off. */
        ledscanio = ~0;
    }
    else
    {
        ledscanio = ledio[ledscan_sta];
        unsigned bit=ledscan_sta?BIT(18):BIT(17);
        SMK_REG(0x16a0cU)=(SMK_REG(0x16a0cU)|0x1000fU)&~bit;
    }
}

//--------------------isr--------------------
// ioscan在0-15之间循环| io_inid只有0、1两个值
static u8   ioscan,io_inid;
static u8   scan_inbuf[2][16];
static u32  scandat;
static u32  norscanio[16] =
{
    ~BIT(24),~BIT(25),~BIT(26),~BIT(27),~BIT(28),~BIT(29),~BIT(30),~BIT(31),
    ~BIT(16),~BIT(17),~BIT(18),~BIT(19),~BIT(20),~BIT(21),~BIT(22),~BIT(23)
};

___interrupt static void scan_soft_isr()
{
    SMK_SOFT_CLEAR |= BIT(2);
    u32 started=fm1_device_ticks();
    key_scan();
    encoder_scan();
    led_scan();
    u32 elapsed=fm1_device_ticks()-started;
    if(elapsed>smk37_scan_peak_ticks)smk37_scan_peak_ticks=elapsed;
}

static volatile u8  adc_sta;
static volatile u32 adc_port;
static u32 CD4051 = 0xffffffff;
static inline FM1_AUDIO_CODE void usr_adc_start(u32 ch);

___interrupt static void scan_isr()
{
    /* SeqAdv clock: does nothing while stopped; on a due tick (and every
     * 300 us while ticks wait) wake the control task, which sends the MIDI. */
    if(seqadv_clock_poll(fm1_device_ticks()))fm1_control_request();
    if(adc_sta < 4 && adc_sta)
    {
        if(adc_sta == 3)
            usr_adc_start(adc_port);
        adc_sta++;
    }

    JL_SPI2->CON |= BIT(14);
    const u32 port_h=JL_PORTH->IN,port_a=JL_PORTA->IN;
    scan_inbuf[io_inid][ioscan] = (port_h&0x300)>>2|(port_h&0x40)>>1|
        (port_a&0x200)>>5|(port_a&0xe0)>>4|(port_a&0x4)>>2;
    JL_PORTA->OUT |= BIT(1);//PB4 595刷新时钟置1，这时候数据从595出去
    if(++ioscan >= 16) ioscan = 0;//16次中断将10个扫描口依次置进行扫描0低电平
    scandat = norscanio[ioscan]&ledscanio&CD4051;//
    JL_SPI2->ADR = (u32)&scandat;//SPI 使用的DMA首地址，MSB在前
    JL_SPI2->CNT = 4;//所有595的控制数据位总数
    JL_PORTA->OUT &= ~BIT(1);//595刷新时钟置0
    if(ioscan == 0){//每扫描16次，就是16条扫描线都扫描过了，就将值给当前值的指针
        io_inbuf_cur = scan_inbuf[io_inid];
        io_inid = !io_inid;//用第二组进行存储下一次扫描的值，之前的一组就先用着
        SMK_SOFT_SET |= BIT(2);//触发软中断
    }
}

//--------------------io--------------------
static const u32 io_in_tab[8]={
    IO_PORTH_06,IO_PORTH_08,IO_PORTH_09,//btn2、encoder
    IO_PORTA_02,IO_PORTA_05,IO_PORTA_06,IO_PORTA_07,IO_PORTA_09,//key
};

static __attribute__((cold, minsize, noinline)) void spi_io_init(u32 io)
{
    gpio_set_direction(io, 0);
    gpio_set_die(io, 1);
    gpio_set_pull_up(io, 0);
    gpio_set_pull_down(io, 0);
}

static __attribute__((cold, minsize, noinline)) void in_io_init(u32 io,u8 pu,u8 pd)
{
    gpio_set_die(io, 1);
    gpio_direction_input(io);
    gpio_set_pull_up(io, pu);
    gpio_set_pull_down(io, pd);
}

static __attribute__((cold, minsize, noinline)) void scan_init(void)
{
    int i;
    for(i=0;i<ENCODER_CNT;i++)
        encoder[i].offset = 1;
    for(i=0;i<8;i++)
        in_io_init(io_in_tab[i],1,0);//普通上拉输入

    JL_IOMAP->CON1 &= ~BIT(16);
    JL_IOMAP->CON1 |= BIT(17);//选择SPI2C端口
    gpio_direction_output(IO_PORTA_01,0);//595的刷新时钟线
    spi_io_init(IO_PORTA_03);//时钟线
    spi_io_init(IO_PORTA_04);//数据线4
    io_inbuf_cur = scan_inbuf[io_inid];

    fm1_device_irq_attach(37,scan_isr,5);//进行spi传输进行扫描
    JL_SPI2->CON = BIT(5)|BIT(14)|BIT(13);// 上升沿更新数据 | 清标志位 | 允许中断
    JL_SPI2->BAUD = 5;//6分频
    fm1_device_irq_attach(122,scan_soft_isr,4);

    scandat = norscanio[ioscan]|0xffff;//~0;//
    JL_SPI2->ADR = (u32)&scandat;
    JL_SPI2->CNT = 4;
    JL_SPI2->CON |= BIT(0);
    gpio_direction_output(IO_PORTA_00,0);//这时数据开始传入595
}

/* The pedal is unknown until its first complete ADC averaging window. */
static volatile s16 slowadcval[SLOW_ADC_CNT] = {[4]=-1};//踏板、电池、两个基准
static u8  fastadcflag = 0;
static u8  slowadcflag = 0;
static u32 adc_con = 0;

//--------------------pad--------------------
s16 padgate[PAD_CNT];
typedef struct
{
  s16 last;
  u8  sta;
  s16 cvel;
  s16 dat[PADKEY_JUDGE];
}t_padkey;

typedef struct
{
  s16 val;
  u8  cnt;
  s32 buf;
}t_padadc;

static t_padkey padkey[PAD_CNT];
static t_padadc padadc[PAD_CNT];
static const u8 padmap[PAD_CNT] __attribute__((section(".data.smk37_scan")))={1,2,3,0,4,7,5,6,9,10,11,8,12,15,13,14};
/* Exact 10-bit entries, four per five bytes; two byte loads per lookup. */
static const u8 pad_velocity[1280] __attribute__((section(".data.smk37_scan"))) = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0xd0,0x40,0x03,0x0d,0x35,0xd4,0x50,0x43,0x0d,0x35,0xd4,0x50,0x43,0x0d,
    0x36,0xd8,0x60,0x83,0x0d,0x36,0xd8,0x60,0x83,0x0d,0x37,0xdc,0x70,0xc3,0x0d,0x37,0xdc,0x70,0x03,0x0e,
    0x38,0xe0,0x80,0x03,0x0e,0x38,0xe0,0x80,0x43,0x0e,0x39,0xe4,0x90,0x43,0x0e,0x39,0xe4,0x90,0x83,0x0e,
    0x3a,0xe8,0xa0,0x83,0x0e,0x3a,0xe8,0xb0,0xc3,0x0e,0x3b,0xec,0xb0,0xc3,0x0e,0x3b,0xec,0xc0,0x03,0x0f,
    0x3c,0xf0,0xc0,0x03,0x0f,0x3c,0xf4,0xd0,0x43,0x0f,0x3d,0xf4,0xd0,0x43,0x0f,0x3e,0xf8,0xe0,0x83,0x0f,
    0x3e,0xf8,0xe0,0x83,0x0f,0x3f,0xfc,0xf0,0xc3,0x0f,0x3f,0xfc,0xf0,0x03,0x10,0x40,0x00,0x01,0x04,0x10,
    0x40,0x00,0x11,0x44,0x10,0x41,0x04,0x11,0x44,0x10,0x41,0x08,0x21,0x84,0x10,0x42,0x08,0x21,0x84,0x10,
    0x43,0x0c,0x31,0xc4,0x10,0x43,0x0c,0x41,0x04,0x11,0x44,0x10,0x41,0x04,0x11,0x44,0x14,0x51,0x44,0x11,
    0x45,0x14,0x51,0x84,0x11,0x46,0x18,0x61,0x84,0x11,0x46,0x18,0x71,0xc4,0x11,0x47,0x1c,0x71,0xc4,0x11,
    0x48,0x20,0x81,0x04,0x12,0x48,0x20,0x91,0x44,0x12,0x49,0x24,0x91,0x44,0x12,0x4a,0x28,0xa1,0x84,0x12,
    0x4a,0x28,0xb1,0xc4,0x12,0x4b,0x2c,0xb1,0xc4,0x12,0x4c,0x30,0xc1,0x04,0x13,0x4c,0x30,0xd1,0x44,0x13,
    0x4d,0x34,0xd1,0x44,0x13,0x4e,0x38,0xe1,0x84,0x13,0x4e,0x3c,0xf1,0xc4,0x13,0x4f,0x3c,0xf1,0x04,0x14,
    0x50,0x40,0x01,0x05,0x14,0x51,0x44,0x11,0x45,0x14,0x51,0x44,0x21,0x85,0x14,0x52,0x48,0x21,0xc5,0x14,
    0x53,0x4c,0x31,0xc5,0x14,0x54,0x50,0x41,0x05,0x15,0x54,0x54,0x51,0x45,0x15,0x55,0x54,0x61,0x85,0x15,
    0x56,0x58,0x61,0xc5,0x15,0x57,0x5c,0x71,0xc5,0x15,0x58,0x60,0x81,0x05,0x16,0x58,0x64,0x91,0x45,0x16,
    0x59,0x68,0xa1,0x85,0x16,0x5a,0x68,0xb1,0xc5,0x16,0x5b,0x6c,0xc1,0x05,0x17,0x5c,0x70,0xc1,0x45,0x17,
    0x5d,0x74,0xd1,0x85,0x17,0x5e,0x78,0xe1,0xc5,0x17,0x5f,0x7c,0xf1,0xc5,0x17,0x60,0x80,0x01,0x06,0x18,
    0x61,0x84,0x11,0x46,0x18,0x62,0x88,0x21,0x86,0x18,0x63,0x8c,0x31,0xc6,0x18,0x64,0x90,0x41,0x06,0x19,
    0x65,0x94,0x51,0x46,0x19,0x66,0x98,0x61,0xc6,0x19,0x67,0x9c,0x71,0x06,0x1a,0x68,0xa0,0x81,0x46,0x1a,
    0x69,0xa4,0xa1,0x86,0x1a,0x6a,0xa8,0xb1,0xc6,0x1a,0x6b,0xb0,0xc1,0x06,0x1b,0x6c,0xb4,0xd1,0x46,0x1b,
    0x6e,0xb8,0xe1,0x86,0x1b,0x6f,0xbc,0xf1,0x06,0x1c,0x70,0xc0,0x11,0x47,0x1c,0x71,0xc4,0x21,0x87,0x1c,
    0x72,0xcc,0x31,0xc7,0x1c,0x74,0xd0,0x41,0x47,0x1d,0x75,0xd4,0x61,0x87,0x1d,0x76,0xdc,0x71,0xc7,0x1d,
    0x78,0xe0,0x81,0x47,0x1e,0x79,0xe4,0xa1,0x87,0x1e,0x7a,0xec,0xb1,0xc7,0x1e,0x7c,0xf0,0xc1,0x47,0x1f,
    0x7d,0xf4,0xe1,0x87,0x1f,0x7e,0xfc,0xf1,0x07,0x20,0x80,0x00,0x12,0x48,0x20,0x81,0x08,0x22,0xc8,0x20,
    0x83,0x0c,0x42,0x08,0x21,0x84,0x14,0x52,0x88,0x21,0x86,0x18,0x72,0xc8,0x21,0x88,0x20,0x82,0x48,0x22,
    0x89,0x28,0xa2,0x88,0x22,0x8b,0x2c,0xc2,0x08,0x23,0x8d,0x34,0xd2,0x88,0x23,0x8e,0x3c,0xf2,0x08,0x24,
    0x90,0x40,0x12,0x49,0x24,0x92,0x48,0x32,0xc9,0x24,0x93,0x50,0x42,0x49,0x25,0x95,0x58,0x62,0xc9,0x25,
    0x97,0x60,0x82,0x49,0x26,0x99,0x64,0xa2,0x89,0x26,0x9b,0x6c,0xc2,0x09,0x27,0x9d,0x74,0xe2,0x89,0x27,
    0x9f,0x7c,0x02,0x0a,0x28,0xa1,0x84,0x22,0x8a,0x28,0xa3,0x8c,0x42,0x0a,0x29,0xa5,0x94,0x62,0xca,0x29,
    0xa7,0xa0,0x82,0x4a,0x2a,0xa9,0xa8,0xa2,0xca,0x2a,0xab,0xb0,0xc2,0x4a,0x2b,0xae,0xb8,0xf2,0xca,0x2b,
    0xb0,0xc0,0x12,0x8b,0x2c,0xb2,0xcc,0x32,0x0b,0x2d,0xb5,0xd4,0x62,0x8b,0x2d,0xb7,0xdc,0x82,0x4b,0x2e,
    0xb9,0xe8,0xb2,0xcb,0x2e,0xbc,0xf0,0xd2,0x8b,0x2f,0xbe,0xfc,0x02,0x0c,0x30,0xc1,0x04,0x23,0xcc,0x30,
    0xc3,0x10,0x53,0x4c,0x31,0xc6,0x1c,0x73,0x0c,0x32,0xc9,0x24,0xa3,0xcc,0x32,0xcb,0x30,0xd3,0x8c,0x33,
    0xce,0x3c,0x03,0x0d,0x34,0xd1,0x48,0x33,0xcd,0x34,0xd4,0x54,0x53,0x8d,0x35,0xd7,0x60,0x83,0x4d,0x36,
    0xda,0x6c,0xb3,0x0d,0x37,0xdd,0x78,0xe3,0xcd,0x37,0xe0,0x84,0x23,0x8e,0x38,0xe3,0x90,0x53,0x8e,0x39,
    0xe6,0x9c,0x83,0x4e,0x3a,0xea,0xa8,0xb3,0x0e,0x3b,0xed,0xb8,0xf3,0xce,0x3b,0xf0,0xc4,0x23,0xcf,0x3c,
    0xf4,0xd4,0x53,0x8f,0x3d,0xf7,0xe0,0x93,0x8f,0x3e,0xfb,0xf0,0xd3,0x4f,0x3f,0xfe,0xfc,0x03,0x50,0x40,
    0x02,0x0d,0x44,0x50,0x41,0x06,0x1d,0x84,0x50,0x42,0x0a,0x2d,0xb4,0x10,0x43,0x0d,0x39,0xf4,0x10,0x44,
    0x11,0x49,0x34,0x11,0x45,0x15,0x59,0x74,0x11,0x46,0x19,0x69,0xb4,0x51,0x47,0x1e,0x7d,0x04,0x52,0x48,
    0x22,0x8d,0x44,0x52,0x49,0x26,0x9d,0x84,0x52,0x4a,0x2a,0xb1,0xd4,0x92,0x4b,0x2f,0xc1,0x14,0x93,0x4c,
    0x33,0xd5,0x64,0xd3,0x4d,0x38,0xe5,0xa4,0x13,0x4f,0x3d,0xf9,0xf4,0x13,0x50,0x42,0x0d,0x45,0x54,0x51,
    0x46,0x21,0x95,0x94,0x52,0x4b,0x35,0xe5,0xd4,0x53,0x50,0x49,0x35,0x15,0x55,0x55,0x5d,0x85,0x55,0x56,
    0x5b,0x71,0xd5,0xd5,0x57,0x60,0x85,0x35,0x16,0x59,0x65,0x9d,0x85,0x56,0x5a,0x6b,0xb1,0xe5,0xd6,0x5b,
    0x70,0xc9,0x35,0x57,0x5d,0x76,0xe1,0x95,0x97,0x5e,0x7c,0xf5,0xf5,0x17,0x60,0x82,0x0d,0x56,0x98,0x61,
    0x88,0x25,0xb6,0x18,0x63,0x8e,0x3d,0x16,0xd9,0x64,0x94,0x59,0x76,0x59,0x66,0x9a,0x71,0xe6,0xd9,0x67,
    0xa1,0x89,0x46,0x9a,0x69,0xa7,0xa5,0xb6,0x1a,0x6b,0xae,0xc1,0x16,0xdb,0x6c,0xb5,0xdd,0x86,0x9b,0x6e,
    0xbc,0xf5,0xf6,0x5b,0x70,0xc3,0x11,0x67,0x1c,0x72,0xca,0x31,0xd7,0xdc,0x73,0xd1,0x4d,0x57,0xdd,0x75,
    0xd9,0x69,0xc7,0x9d,0x77,0xe0,0x89,0x47,0x9e,0x79,0xe8,0xa9,0xc7,0x9e,0x7b,0xef,0xc5,0x37,0x5f,0x7d,
    0xf7,0xe5,0xb7,0x5f,0x7f,0xff,0x09,0x48,0xa0,0x81,0x08,0x2a,0xc8,0xa0,0x83,0x10,0x4a,0x48,0xa1,0x85,
    0x19,0x6e,0xd8,0xe1,0x87,0x21,0x8e,0x68,0x22,0x8a,0x2a,0xb2,0xf8,0x62,0x8c,0x33,0xd6,0x88,0xa3,0x8e,
    0x3c,0xfa,0x18,0xe4,0x90,0x45,0x22,0xa9,0x64,0x93,0x4f,0x46,0x49,0xa5,0x95,0x59,0x6e,0xd9,0x25,0x98,
    0x62,0x96,0x79,0xa6,0x9a,0x6c,0xbe,0x19,0x27,0x9d,0x76,0xe6,0xc9,0xa7,0x9f,0x81,0x0e,0x6a,0x68,0xa2,
    0x8b,0x3a,0x1a,0xe9,0xa4,0x96,0x66,0xba,0xa9,0xa7,0xa1,0x92,0x6a,0x6a,0xaa,0xac,0xbe,0x2a,0x2b,0xad,
    0xb7,0xea,0xda,0x2b,0xb0,0xc3,0x1a,0x8b,0xec,0xb2,0xce,0x46,0x4b,0xed,0xb5,0xda,0x76,0x0b,0xee,0xb8,
    0xe6,0xa6,0xcb,0x2e,0xbc,0xf3,0xda,0x9b,0x2f,0xbf,0xff,0x0a,0x6c,0x70,0xc2,0x0c,0x3f,0x2c,0xb1,0xc5,
    0x19,0x73,0xfc,0xf1,0xc8,0x26,0xa7,0xdc,0x32,0xcc,0x34,0xdf,0xac,0xb3,0xcf,0x41,0x17,0x8d,0x34,0xd3,
    0x4f,0x4f,0x6d,0xb5,0xd6,0x5d,0x87,0x4d,0x36,0xda,0x6c,0xbf,0x3d,0xf7,0xdd,0x7a,0xfb,0x2d,0xb8,0xe1,
    0x89,0x37,0x1e,0x79,0xe5,0x99,0x73,0x0e,0x3a,0xe9,0xa8,0xb3,0x0e,0x3b,0xed,0xb8,0xf3,0x0e,0x3c,0xf1,
    0xc8,0x33,0x0f,0x3d,0xf5,0xd8,0x73,0x0f,0x3e,0xf9,0xe9,0xb7,0x1f,0x7f,0xfd,0xfa,0xfb,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
};
static FM1_AUDIO_CODE u16 pad_trim(s16 val,u8 id)
{
    (void)id;
    unsigned sample=(u16)val,byte=sample+(sample>>2);
    unsigned pair=pad_velocity[byte]|((unsigned)pad_velocity[byte+1]<<8);
    return (pair>>((sample&3)*2))&1023;
}
static inline FM1_AUDIO_CODE u16 pad_adc_trim(u16 val)
{
    if(val <= 512)
        val >>= 4;
    else
    {
        val = ((val - 512)>>2) + 32;
        if(val > 127) val = 127;
    }
    return val;
}

static FM1_AUDIO_CODE void pad_prc(u8 id,s16 val)
{
    int i;
    s16 tmp;
    tmp = val - padadc[id].val;
    if(tmp > PADADC_JUDGE || tmp < - PADADC_JUDGE)
    {
        padadc[id].buf += val;
        if (++padadc[id].cnt >= PADADC_JET)
        {
           padadc[id].val = padadc[id].buf >> PADADC_JET_OFFSET;
           kbd_hw_msg_wr_atom(PADADC_MSG(id, pad_adc_trim(padadc[id].val)));
           padadc[id].buf = 0;
           padadc[id].cnt = 0;
        }
    }
    else
    {
        padadc[id].buf = 0;
        padadc[id].cnt = 0;
    }

    if(val < padgate[id])
        padgate[id] --;
    else if(padgate[id] < trim_vm_cfg.pad[id][0])
        padgate[id]+=2;
    /* A cleared history stays zero until the first sample above 300.
     * Keep the assembly kernel off the idle path; thresholds are unchanged. */
    i=(val>300 || padkey[id].dat[3])?smk37_pad_history_step(padkey[id].dat,val):0;

    if(padkey[id].sta < PADKEY_DNJET)
    {
        if(padkey[id].cvel)
            padkey[id].cvel--;
        if(val <= padgate[id] + 10)
        {
            padkey[id].sta = padkey[id].last = 0;
            /* Quiet/released pads must not retain a peak that prevents the
             * next softer attack from forming a rising four-sample window. */
            if(padkey[id].dat[3])memset(padkey[id].dat,0,sizeof padkey[id].dat);
        }
        else
        {
            if(padkey[id].last < val)
                padkey[id].last = val;
            if(i > 2)
            {
                if(++padkey[id].sta >= PADKEY_DNJET)
                {
                    if(padkey[id].last >=  512 +padkey[id].cvel)
                        kbd_hw_msg_wr_atom(PADKEY_MSG(id,pad_trim(padkey[id].last,id)));
                    else
                        padkey[id].sta = PADKEY_DNJET;
                }
            }
            else if(padkey[id].sta > 0)
            {
                if(padkey[id].last > padkey[id].cvel)
                {
                    kbd_hw_msg_wr_atom(PADKEY_MSG(id,pad_trim(padkey[id].last,id)));
                    padkey[id].sta  = PADKEY_DNJET;
                }
                else
                    padkey[id].sta = padkey[id].last = 0;
            }
            else if(padkey[id].last)
            {
                padkey[id].last = 0;
            }
        }
    }
    else
    {
        if(val <= padgate[id] + 5)
        {
            if(++padkey[id].sta >= PADKEY_DNJET + PADKEY_UPJET)
            {
                padkey[id].sta = padkey[id].last = 0;
                padkey[id].cvel = 256;
                memset(padkey[id].dat,0,sizeof padkey[id].dat);
                kbd_hw_msg_wr_atom(PADKEY_MSG(id,0));
            }
        }
        else
            padkey[id].sta = PADKEY_DNJET;
    }
}
//--------------------mp--------------------
typedef struct{
    u16 sum; /* At most 32 ten-bit samples: 32 * 1023 = 32736. */
    u16 curval;//当前通道的值
    u16 adcval;//当前adc的值
    u8  count;//有效值次数
}t_mp;
static t_mp mp[6];
static const u8 mpmap[6] __attribute__((section(".data.smk37_scan")))={1,2,3,0,4,5};

static FM1_AUDIO_CODE void mp_prc(u8 id,s16 value)
{
    s16 tmp = value - mp[id].adcval;
    u16 midval=0;
    /* Soft pickup needs the first averaged position even at the lower stop. */
    if(mp[id].curval==65535 || tmp > MP_ADC_JET || tmp < - MP_ADC_JET)
    {
        mp[id].sum+=value;
        if (++mp[id].count >= MP_ADC_CNT_JET)
        {
            mp[id].adcval = mp[id].sum/MP_ADC_CNT_JET;
            mp[id].sum = 0;
            mp[id].count = 0;

            midval= mp[id].adcval>>3;
            if(mp[id].curval != midval)
            {
                mp[id].curval = midval;
                    kbd_hw_msg_wr_atom(MP_MSG(id,mp[id].curval));
            }
        }
    }
    else
    {
        mp[id].sum = 0;
        mp[id].count = 0;
    }
}

static s8 gl_turn[2];
static FM1_AUDIO_CODE void gl_prc(u8 id,u16 value)
{
    /* Establish both rest positions even when the modulation ADC is zero. */
    if(mp[id].curval==65535) {
        mp[id].sum+=value;
        if(++mp[id].count==32) {
            mp[id].curval=mp[id].adcval=mp[id].sum>>5;
            mp[id].sum=mp[id].count=0;
            kbd_hw_msg_wr_atom(MP_MSG(id,mp[id].curval*5>>3));
        }
        return;
    }
    s16 direction=value>mp[id].adcval?1:value<mp[id].adcval?-1:0;
    if(direction && direction==gl_turn[id-4]) {
        mp[id].sum+=value;
        if(++mp[id].count==32) {
            mp[id].adcval=mp[id].sum>>5;mp[id].sum=mp[id].count=0;
            if(mp[id].curval!=mp[id].adcval) {
                mp[id].curval=mp[id].adcval;
                kbd_hw_msg_wr_atom(MP_MSG(id,mp[id].curval*5>>3));
            }
        }
    } else {mp[id].sum=mp[id].count=0;gl_turn[id-4]=direction;}
}
static s16 pedalval=-1;
FM1_AUDIO_CODE s16 adc_filter(s16 val)
{
    static s16 curval=-1,sum,count;
    s16 tmp = val - curval;
    if(curval<0 || tmp > PEDAL_ADC_JET || tmp < - PEDAL_ADC_JET)
    {
        sum += val;
        if (++count >= PEDAL_ADC_CNT_JET)
        {
            curval = sum>>3;
            count = 0;
            sum = 0;
        }
    }
    else
    {
        count = 0;
        sum = 0;
    }
    return curval;
}

static inline u16 pedal_trim(u16 val)
{
    u16 ret;
    int i;
    if(val <= trim_vm_cfg.pedal[0]) ret = 0;
    else if(val >= trim_vm_cfg.pedal[4]) ret = 1023;
    else{
        for(i=1;i<4;i++){
            if(val <= trim_vm_cfg.pedal[i]) break;
        }
        ret = 256*(i-1) + (val - trim_vm_cfg.pedal[i-1])*256/(trim_vm_cfg.pedal[i] - trim_vm_cfg.pedal[i-1]);
    }
    return ret;
}

static void pedal_10ms()
{
    s16 value=slowadcval[4];
    if(value != pedalval)
    {
        pedalval = value;
        kbd_hw_msg_wr_atom(PEDAL_MSG(0, (u16)pedal_trim(1023 - pedalval)));
    }
}

//--------------------bateery--------------------
static __attribute__((cold, minsize, noinline)) void adc_io_init(u32 io)
{
    gpio_set_die(io, 0);
    gpio_set_direction(io, 1);
    gpio_set_pull_down(io, 0);
    gpio_set_pull_up(io, 0);
}

static const u32 cd4051_adc_port[3] __attribute__((section(".data.smk37_scan")))={ AD_CH_PA08,AD_CH_PC09,AD_CH_PA10};

static inline FM1_AUDIO_CODE void usr_adc_start(u32 channel)
{
    if((channel&0xffff)==AD_CH_PMU)adc_pmu_detect_en(channel>>16);
    JL_ADC->CON=5U|(4U<<12)|BIT(3)|BIT(5)|BIT(6)|((channel&15)<<8);
    JL_ADC->CON|=BIT(4);JL_ADC->CON|=BIT(6);
}
static inline FM1_AUDIO_CODE void usr_adc_pre_start(u8 io,u32 port)
{
    CD4051 = 0xffffffc7|((io&7)<<3);
    adc_port = port;
    adc_sta = 1;
}

static FM1_AUDIO_CODE void adc_complete(u32 con, u16 result)
{
    (void)con;
    if(fastadcflag < FAST_ADC_CNT)
    {
        if(fastadcflag < 16)
            pad_prc(padmap[fastadcflag],result);
        else
            gl_prc(mpmap[fastadcflag-12],result);
        fastadcflag++;
    }
    else
    {
        if(slowadcflag < 4)//推子
            mp_prc(mpmap[slowadcflag],result);
        else if(slowadcflag == 4)//踏板
            slowadcval[slowadcflag] = adc_filter(result);
        else if(slowadcflag == 5)//电量
        {
            slowadcval[slowadcflag] = result;
        }
        else {
            slowadcval[slowadcflag]=result;
            if(result)smk37_adc_reference(slowadcflag==6?AD_CH_VBAT:AD_CH_PMU_VBG,result);
        }
        if(++slowadcflag == SLOW_ADC_CNT)
            slowadcflag = 0;
        fastadcflag = 0;
    }

    if(fastadcflag < FAST_ADC_CNT)
    {
        if(fastadcflag < 16)
            usr_adc_pre_start(fastadcflag,cd4051_adc_port[fastadcflag>>3]);
        else
            usr_adc_pre_start(fastadcflag-12,cd4051_adc_port[2]);
    }
    else if(slowadcflag < SLOW_ADC_CNT)
    {
        if(slowadcflag < 4)
            usr_adc_pre_start(slowadcflag,cd4051_adc_port[2]);
        else if(slowadcflag < 6)
            usr_adc_pre_start(slowadcflag+2,cd4051_adc_port[2]);
        else
        {
            SFR(JL_ANA->WLA_CON0, 14, 1, 0);   //TEST_TO_ADC_EN_12v
            if(slowadcflag == 6)
                usr_adc_pre_start(0,AD_CH_VBAT);
            else if(slowadcflag == 7)
                usr_adc_pre_start(0,AD_CH_PMU_VBG);
        }
    }
}

___interrupt static void smk37_adc_irq(void)
{
    u32 con=JL_ADC->CON;
    if(!(con&BIT(7)))return;
    u32 started=fm1_device_ticks();
    u16 result=JL_ADC->RES;
    JL_ADC->CON=BIT(6);JL_ADC->CON=0;
    adc_complete(con,result);
    u32 elapsed=fm1_device_ticks()-started;
    if(elapsed>smk37_adc_peak_ticks)smk37_adc_peak_ticks=elapsed;
}

/* Versioned calibration is stored separately from FM1 project/sample Flash.
 * Invalid or absent records keep the original board's default curves. */
static __attribute__((cold, minsize, noinline)) void calibration_read(void)
{
    for(unsigned i=0;i<37;++i)memcpy(trim_vm_cfg.key[i],keytrim_init,sizeof keytrim_init);
    for(unsigned i=0;i<16;++i)memcpy(trim_vm_cfg.pad[i],padtrim_init,sizeof padtrim_init);
    memcpy(trim_vm_cfg.pedal,pedaltrim_init,sizeof pedaltrim_init);
    struct {u32 magic;u16 values[100];} record;
    void *dest[]={trim_vm_cfg.key,trim_vm_cfg.key[20],trim_vm_cfg.pad,trim_vm_cfg.pedal};
    const unsigned rows[]={20,17,16,1};
    for(unsigned block=0;block<4;++block) {
        unsigned bytes=4+rows[block]*10;
        if(smk37_syscfg_read(51+block,&record,bytes)!=(int)bytes || record.magic!=0x534d0100U+block)continue;
        unsigned valid=1;
        for(unsigned row=0;row<rows[block];++row) {
            u16 *curve=record.values+5*row;
            for(unsigned i=1;i<5;++i)if(curve[i]<=curve[i-1]||curve[i]>1023)valid=0;
        }
        if(valid)memcpy(dest[block],record.values,bytes-4);
    }
}

__attribute__((cold, minsize, noinline)) void smk37_scan_init(void)
{
    if(scan_started)return;
    calibration_read();
    memset(scan_inbuf,255,sizeof scan_inbuf);
    for(unsigned i=0;i<16;++i)padgate[i]=PADKEY_INIT_DNGATE;
    for(unsigned i=0;i<6;++i)mp[i].curval=65535;
    /* TIMER4 is the FM1 24 MHz monotonic counter; TIMER5 is not needed. */
    SMK_REG(0x10800U)=0;SMK_REG(0x10808U)=0xffffffffU;
    SMK_REG(0x10804U)=0;SMK_REG(0x10800U)=0x4009U;
    gpio_set_pull_up(17,1);gpio_set_die(17,1);gpio_direction_input(17);
    adc_io_init(IO_PORTC_09);adc_io_init(IO_PORTA_08);adc_io_init(IO_PORTA_10);
    JL_ADC->CON=0;fm1_device_irq_attach(24,smk37_adc_irq,5);
    usr_adc_pre_start(0,cd4051_adc_port[0]);
    scan_init();scan_started=1;
}

void smk37_scan_read(void)
{
    encoder_50ms(0);buttons_read();pedal_10ms();
    unsigned flags=fm1_device_irq_save();
    unsigned overflow=hw_overflow;
    if(overflow){hw_rd=hw_wr;hw_overflow=0;}
    fm1_device_irq_restore(flags);
    if(overflow)smk37_controls_panic();
    for(unsigned count=0;count<256;++count) {
        flags=fm1_device_irq_save();
        if(hw_rd==hw_wr){fm1_device_irq_restore(flags);break;}
        u32 message=hw_msg[hw_rd];hw_rd=(hw_rd+1)&255;
        fm1_device_irq_restore(flags);
        unsigned kind=message>>24,id=(message>>16)&255,value=message&65535;
        if(kind==SMK37_KEY||kind==SMK37_PAD||kind==SMK37_PEDAL)
            value=value?(value*127+511)/1023:0;
        if((kind==SMK37_KEY||kind==SMK37_PAD) && (message&65535) && !value)value=1;
        if(kind==SMK37_FADER && id>=4) {
            static u16 centre[2];static u8 ready;
            unsigned bit=1U<<(id-4);
            if(!(ready&bit)){centre[id-4]=value;ready|=bit;}
            int v=(int)value-centre[id-4];
            /* Pitch rests at centre; modulation rests at its lower stop. */
            if(id==4){if(v>-7&&v<7)v=0;v+=64;}
            value=v<0?0:v>127?127:(unsigned)v;
        }
        smk37_control_event(kind,id,kind==SMK37_ENCODER?(s16)value:(int)value);
    }
}

u16 smk37_battery_adc(void) {return slowadcval[5];}
