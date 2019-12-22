// Some definitions, that hopefully are added to IMXRT.h
#ifndef ADC_ETC_CTRL_SOFTRST
typedef struct {
  volatile uint32_t CTRL;              // offset 0
  volatile uint32_t DONE0_1_IRQ;       // offset004
  volatile uint32_t DONE2_ERR_IRQ;     // offset008
  volatile uint32_t DMA_CTRL;          // offset00C
  struct {
    volatile uint32_t CTRL;            //offset010
    volatile uint32_t COUNTER;         //offset014
    volatile uint32_t CHAIN_1_0;
    volatile uint32_t CHAIN_3_2;
    volatile uint32_t CHAIN_5_4;
    volatile uint32_t CHAIN_7_6;
    volatile uint32_t RESULT_1_0;
    volatile uint32_t RESULT_3_2;
    volatile uint32_t RESULT_5_4;
    volatile uint32_t RESULT_7_6;
  } TRIG[7];
} IMXRT_ADC_ETC_t;

#define IMXRT_ADC_ETC           (*(IMXRT_ADC_ETC_t *)0x403B0000)
#define ADC_ETC_CTRL      (IMXRT_ADC_ETC.CTRL)
#define ADC_ETC_DONE0_1_IRQ   (IMXRT_ADC_ETC.DONE0_1_IRQ)
#define ADC_ETC_DONE2_ERR_IRQ   (IMXRT_ADC_ETC.DONE2_ERR_IRQ)
#define ADC_ETC_DMA_CTRL    (IMXRT_ADC_ETC.DMA_CTRL)
#define ADC_ETC_TRIG0_CTRL    (IMXRT_ADC_ETC.TRIG[0].CTRL)
#define ADC_ETC_TRIG0_COUNTER   (IMXRT_ADC_ETC.TRIG[0].COUNTER)
#define ADC_ETC_TRIG0_CHAIN_1_0   (IMXRT_ADC_ETC.TRIG[0].CHAIN_1_0)
#define ADC_ETC_TRIG0_CHAIN_3_2   (IMXRT_ADC_ETC.TRIG[0].CHAIN_3_2)
#define ADC_ETC_TRIG0_CHAIN_5_4   (IMXRT_ADC_ETC.TRIG[0].CHAIN_5_4)
#define ADC_ETC_TRIG0_CHAIN_7_6   (IMXRT_ADC_ETC.TRIG[0].CHAIN_7_6)
#define ADC_ETC_TRIG0_RESULT_1_0  (IMXRT_ADC_ETC.TRIG[0].RESULT_1_0)
#define ADC_ETC_TRIG0_RESULT_3_2  (IMXRT_ADC_ETC.TRIG[0].RESULT_3_2)
#define ADC_ETC_TRIG0_RESULT_5_4  (IMXRT_ADC_ETC.TRIG[0].RESULT_5_4)
#define ADC_ETC_TRIG0_RESULT_7_6  (IMXRT_ADC_ETC.TRIG[0].RESULT_7_6)

#define ADC_ETC_TRIG1_CTRL              (IMXRT_ADC_ETC.TRIG[1].CTRL)
#define ADC_ETC_TRIG1_COUNTER           (IMXRT_ADC_ETC.TRIG[1].COUNTER)
#define ADC_ETC_TRIG1_CHAIN_1_0         (IMXRT_ADC_ETC.TRIG[1].CHAIN_1_0)
#define ADC_ETC_TRIG1_CHAIN_3_2         (IMXRT_ADC_ETC.TRIG[1].CHAIN_3_2)
#define ADC_ETC_TRIG1_CHAIN_5_4         (IMXRT_ADC_ETC.TRIG[1].CHAIN_5_4)
#define ADC_ETC_TRIG1_CHAIN_7_6         (IMXRT_ADC_ETC.TRIG[1].CHAIN_7_6)
#define ADC_ETC_TRIG1_RESULT_1_0        (IMXRT_ADC_ETC.TRIG[1].RESULT_1_0)
#define ADC_ETC_TRIG1_RESULT_3_2        (IMXRT_ADC_ETC.TRIG[1].RESULT_3_2)
#define ADC_ETC_TRIG1_RESULT_5_4        (IMXRT_ADC_ETC.TRIG[1].RESULT_5_4)
#define ADC_ETC_TRIG1_RESULT_7_6        (IMXRT_ADC_ETC.TRIG[1].RESULT_7_6)
#define ADC_ETC_TRIG2_CTRL              (IMXRT_ADC_ETC.TRIG[2].CTRL)
#define ADC_ETC_TRIG2_COUNTER           (IMXRT_ADC_ETC.TRIG[2].COUNTER)
#define ADC_ETC_TRIG2_CHAIN_1_0         (IMXRT_ADC_ETC.TRIG[2].CHAIN_1_0)
#define ADC_ETC_TRIG2_CHAIN_3_2         (IMXRT_ADC_ETC.TRIG[2].CHAIN_3_2)
#define ADC_ETC_TRIG2_CHAIN_5_4         (IMXRT_ADC_ETC.TRIG[2].CHAIN_5_4)
#define ADC_ETC_TRIG2_CHAIN_7_6         (IMXRT_ADC_ETC.TRIG[2].CHAIN_7_6)
#define ADC_ETC_TRIG2_RESULT_1_0        (IMXRT_ADC_ETC.TRIG[2].RESULT_1_0)
#define ADC_ETC_TRIG2_RESULT_3_2        (IMXRT_ADC_ETC.TRIG[2].RESULT_3_2)
#define ADC_ETC_TRIG2_RESULT_5_4        (IMXRT_ADC_ETC.TRIG[2].RESULT_5_4)
#define ADC_ETC_TRIG2_RESULT_7_6        (IMXRT_ADC_ETC.TRIG[2].RESULT_7_6)
#define ADC_ETC_TRIG3_CTRL              (IMXRT_ADC_ETC.TRIG[3].CTRL)
#define ADC_ETC_TRIG3_COUNTER           (IMXRT_ADC_ETC.TRIG[3].COUNTER)
#define ADC_ETC_TRIG3_CHAIN_1_0         (IMXRT_ADC_ETC.TRIG[3].CHAIN_1_0)
#define ADC_ETC_TRIG3_CHAIN_3_2         (IMXRT_ADC_ETC.TRIG[3].CHAIN_3_2)
#define ADC_ETC_TRIG3_CHAIN_5_4         (IMXRT_ADC_ETC.TRIG[3].CHAIN_5_4)
#define ADC_ETC_TRIG3_CHAIN_7_6         (IMXRT_ADC_ETC.TRIG[3].CHAIN_7_6)
#define ADC_ETC_TRIG3_RESULT_1_0        (IMXRT_ADC_ETC.TRIG[3].RESULT_1_0)
#define ADC_ETC_TRIG3_RESULT_3_2        (IMXRT_ADC_ETC.TRIG[3].RESULT_3_2)
#define ADC_ETC_TRIG3_RESULT_5_4        (IMXRT_ADC_ETC.TRIG[3].RESULT_5_4)
#define ADC_ETC_TRIG3_RESULT_7_6        (IMXRT_ADC_ETC.TRIG[3].RESULT_7_6)
#define ADC_ETC_TRIG4_CTRL              (IMXRT_ADC_ETC.TRIG[4].CTRL)
#define ADC_ETC_TRIG4_COUNTER           (IMXRT_ADC_ETC.TRIG[4].COUNTER)
#define ADC_ETC_TRIG4_CHAIN_1_0         (IMXRT_ADC_ETC.TRIG[4].CHAIN_1_0)
#define ADC_ETC_TRIG4_CHAIN_3_2         (IMXRT_ADC_ETC.TRIG[4].CHAIN_3_2)
#define ADC_ETC_TRIG4_CHAIN_5_4         (IMXRT_ADC_ETC.TRIG[4].CHAIN_5_4)
#define ADC_ETC_TRIG4_CHAIN_7_6         (IMXRT_ADC_ETC.TRIG[4].CHAIN_7_6)
#define ADC_ETC_TRIG4_RESULT_1_0        (IMXRT_ADC_ETC.TRIG[4].RESULT_1_0)
#define ADC_ETC_TRIG4_RESULT_3_2        (IMXRT_ADC_ETC.TRIG[4].RESULT_3_2)
#define ADC_ETC_TRIG4_RESULT_5_4        (IMXRT_ADC_ETC.TRIG[4].RESULT_5_4)
#define ADC_ETC_TRIG4_RESULT_7_6        (IMXRT_ADC_ETC.TRIG[4].RESULT_7_6)
#define ADC_ETC_TRIG5_CTRL              (IMXRT_ADC_ETC.TRIG[5].CTRL)
#define ADC_ETC_TRIG5_COUNTER           (IMXRT_ADC_ETC.TRIG[5].COUNTER)
#define ADC_ETC_TRIG5_CHAIN_1_0         (IMXRT_ADC_ETC.TRIG[5].CHAIN_1_0)
#define ADC_ETC_TRIG5_CHAIN_3_2         (IMXRT_ADC_ETC.TRIG[5].CHAIN_3_2)
#define ADC_ETC_TRIG5_CHAIN_5_4         (IMXRT_ADC_ETC.TRIG[5].CHAIN_5_4)
#define ADC_ETC_TRIG5_CHAIN_7_6         (IMXRT_ADC_ETC.TRIG[5].CHAIN_7_6)
#define ADC_ETC_TRIG5_RESULT_1_0        (IMXRT_ADC_ETC.TRIG[5].RESULT_1_0)
#define ADC_ETC_TRIG5_RESULT_3_2        (IMXRT_ADC_ETC.TRIG[5].RESULT_3_2)
#define ADC_ETC_TRIG5_RESULT_5_4        (IMXRT_ADC_ETC.TRIG[5].RESULT_5_4)
#define ADC_ETC_TRIG5_RESULT_7_6        (IMXRT_ADC_ETC.TRIG[5].RESULT_7_6)
#define ADC_ETC_TRIG6_CTRL              (IMXRT_ADC_ETC.TRIG[6].CTRL)
#define ADC_ETC_TRIG6_COUNTER           (IMXRT_ADC_ETC.TRIG[6].COUNTER)
#define ADC_ETC_TRIG6_CHAIN_1_0         (IMXRT_ADC_ETC.TRIG[6].CHAIN_1_0)
#define ADC_ETC_TRIG6_CHAIN_3_2         (IMXRT_ADC_ETC.TRIG[6].CHAIN_3_2)
#define ADC_ETC_TRIG6_CHAIN_5_4         (IMXRT_ADC_ETC.TRIG[6].CHAIN_5_4)
#define ADC_ETC_TRIG6_CHAIN_7_6         (IMXRT_ADC_ETC.TRIG[6].CHAIN_7_6)
#define ADC_ETC_TRIG6_RESULT_1_0        (IMXRT_ADC_ETC.TRIG[6].RESULT_1_0)
#define ADC_ETC_TRIG6_RESULT_3_2        (IMXRT_ADC_ETC.TRIG[6].RESULT_3_2)
#define ADC_ETC_TRIG6_RESULT_5_4        (IMXRT_ADC_ETC.TRIG[6].RESULT_5_4)
#define ADC_ETC_TRIG6_RESULT_7_6        (IMXRT_ADC_ETC.TRIG[6].RESULT_7_6)
#define ADC_ETC_TRIG7_CTRL              (IMXRT_ADC_ETC.TRIG[7].CTRL)
#define ADC_ETC_TRIG7_COUNTER           (IMXRT_ADC_ETC.TRIG[7].COUNTER)
#define ADC_ETC_TRIG7_CHAIN_1_0         (IMXRT_ADC_ETC.TRIG[7].CHAIN_1_0)
#define ADC_ETC_TRIG7_CHAIN_3_2         (IMXRT_ADC_ETC.TRIG[7].CHAIN_3_2)
#define ADC_ETC_TRIG7_CHAIN_5_4         (IMXRT_ADC_ETC.TRIG[7].CHAIN_5_4)
#define ADC_ETC_TRIG7_CHAIN_7_6         (IMXRT_ADC_ETC.TRIG[7].CHAIN_7_6)
#define ADC_ETC_TRIG7_RESULT_1_0        (IMXRT_ADC_ETC.TRIG[7].RESULT_1_0)
#define ADC_ETC_TRIG7_RESULT_3_2        (IMXRT_ADC_ETC.TRIG[7].RESULT_3_2)
#define ADC_ETC_TRIG7_RESULT_5_4        (IMXRT_ADC_ETC.TRIG[7].RESULT_5_4)
#define ADC_ETC_TRIG7_RESULT_7_6        (IMXRT_ADC_ETC.TRIG[7].RESULT_7_6)

#define ADC_ETC_CTRL_SOFTRST            ((uint32_t)(1<<31))
#define ADC_ETC_CTRL_TSC_BYPASS         ((uint32_t)(1<<30))
#define ADC_ETC_CTRL_DMA_MODE_SEL       ((uint32_t)(1<<29))
#define ADC_ETC_CTRL_PRE_DIVIDER(n)     ((uint32_t)(((n) & 0xff) << 16))
#define ADC_ETC_CTRL_EXT1_TRIG_PRIORITY(n)  ((uint32_t)(((n) & 0x07) << 13))
#define ADC_ETC_CTRL_EXT1_TRIG_ENABLE       ((uint32_t)(1<<12))
#define ADC_ETC_CTRL_EXT0_TRIG_PRIORITY(n)  ((uint32_t)(((n) & 0x07) << 9))
#define ADC_ETC_CTRL_EXT0_TRIG_ENABLE       ((uint32_t)(1<<8))
#define ADC_ETC_CTRL_TRIG_ENABLE(n)     ((uint32_t)(((n) & 0xff) << 0))

#define ADC_ETC_DONE0_1_IRQ_TRIG_DONE1(n)  ((uint32_t)(1<<(16 + ((n) &0x7)))
#define ADC_ETC_DONE0_1_IRQ_TRIG_DONE0(n)  ((uint32_t)(1<<((n) &0x7)))

#define ADC_ETC_DONE2_ERR_IRQ_TRIG_ERR(n)  ((uint32_t)(1<<(16 + ((n) &0x7)))
#define ADC_ETC_DONE2_ERR_IRQ_TRIG_DONE2(n)  ((uint32_t)(1<<((n) &0x7)))

#define ADC_ETC_DMA_CTRL_TRIQ_REQ(n)    ((uint32_t)(1<<(16 + ((n) &0x7)))
#define ADC_ETC_DMA_CTRL_TRIQ_ENABLE(n) ((uint32_t)(1<<((n) &0x7)))

// For each TRIG elements in array
#define ADC_ETC_TRIG_CTRL_SYNC_MODE     ((uint32_t)(1<<16))
#define ADC_ETC_TRIG_CTRL_TRIG_PRIORITY(n)  ((uint32_t)(((n) & 0x07) << 12))
#define ADC_ETC_TRIG_CTRL_TRIG_CHAIN(n) ((uint32_t)(((n) & 0x07) << 8))
#define ADC_ETC_TRIG_CTRL_TRIG_MODE     ((uint32_t)(1<<4))
#define ADC_ETC_TRIG_CTRL_SW_TRIG       ((uint32_t)(1<<0))

#define ADC_ETC_TRIG_COUNTER_SAMPLE_INTERVAL(n) ((uint32_t)(((n) & 0xff) << 16))
#define ADC_ETC_TRIG_COUNTER_INIT_DELAY(n)    ((uint32_t)(((n) & 0xff) << 0))

#define ADC_ETC_TRIG_CHAIN_IE1(n)       ((uint32_t)(((n) & 0x03) << 29))
#define ADC_ETC_TRIG_CHAIN_B2B1         ((uint32_t)(1<<28))
#define ADC_ETC_TRIG_CHAIN_HWTS1(n)     ((uint32_t)(((n) & 0xff) << 20))
#define ADC_ETC_TRIG_CHAIN_CSEL1(n)     ((uint32_t)(((n) & 0x0f) << 16))
#define ADC_ETC_TRIG_CHAIN_IE0(n)       ((uint32_t)(((n) & 0x03) << 13))
#define ADC_ETC_TRIG_CHAIN_B2B0         ((uint32_t)(1<<12))
#define ADC_ETC_TRIG_CHAIN_HWTS0(n)     ((uint32_t)(((n) & 0xff) << 4))
#define ADC_ETC_TRIG_CHAIN_CSEL0(n)     ((uint32_t)(((n) & 0x0f) << 0))

#define ADC_ETC_TRIG_RESULT_DATA1(n)    ((uint32_t)(((n) & 0xff) << 16))
#define ADC_ETC_TRIG_RESULT_DATA0(n)    ((uint32_t)(((n) & 0xff) << 0))

#endif
