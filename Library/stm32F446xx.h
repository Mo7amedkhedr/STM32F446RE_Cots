#ifndef STM32F446XX_H
#define STM32F446XX_H


/*************************     Various Memory BASE Address     **********************************/

#define FLASH_BASE_ADDRESS            0X08000000UL
#define SRAM_BASE_ADDRESS             0X20000000UL
#define ROM_BASE_ADDRESS              0x1FFF0000UL

/*************************    AHB1 peripheral BASE Address     **********************************/

#define GPIOA_BASE_ADDRESS                0x40020000U
#define GPIOB_BASE_ADDRESS                0x40020400U
#define GPIOC_BASE_ADDRESS                0x40020800U
#define GPIOD_BASE_ADDRESS                0x40020C00U
#define GPIOE_BASE_ADDRESS                0x40021000U
#define GPIOF_BASE_ADDRESS                0x40021400U
#define GPIOG_BASE_ADDRESS                0x40021800U
#define GPIOH_BASE_ADDRESS                0x40021C00U

#define RCC_BASE_ADDRESS                  0x40023800U



#define DMA1_BASE_ADDRESS 0x40026000UL
#define DMA2_BASE_ADDRESS 0x40026400UL





/*************************    AHB2 peripheral BASE Address     **********************************/

/*************************    AHB3 peripheral BASE Address     **********************************/

/*************************    APB1 peripheral BASE Address     **********************************/

#define UART5_BASE_ADDRESS  0x40005000UL
#define UART4_BASE_ADDRESS  0x40004C00UL
#define USART3_BASE_ADDRESS 0x40004800UL
#define USART2_BASE_ADDRESS 0x40004400UL

#define SPI3_BASE_ADDRESS   0x40003C00UL
#define SPI2_BASE_ADDRESS   0x40003800UL

#define I2C3_BASE_ADDRESS   0x40005C00UL
#define I2C2_BASE_ADDRESS   0x40005800UL
#define I2C1_BASE_ADDRESS   0x40005400UL

/*************************    APB2 peripheral BASE Address     **********************************/
#define EXTI_BASE_ADDRESS   0x40013C00UL
#define SYSCFG_BASE_ADDRESS 0x40013800UL

#define USART6_BASE_ADDRESS 0x40011400UL
#define USART1_BASE_ADDRESS 0x40011000UL

#define SPI4_BASE_ADDRESS   0x40013400UL
#define SPI1_BASE_ADDRESS   0x40013000UL

/*************************    GPIO Register Definition Structure     **********************************/

typedef struct 
{
    volatile uint32_t MODER;               /*GPIO PORT Mode Register*/
	volatile uint32_t OTYPER;              /*GPIO PORT Output type Register*/
	volatile uint32_t OSPEEDR;             /*GPIO PORT Output speed Register*/
	volatile uint32_t PUPDR ;              /*GPIO PORT Pull/down Register*/
	volatile uint32_t IDR;                 /*GPIO PORT Input data Register*/
	volatile uint32_t ODR;                 /*GPIO PORT Output data Register*/
	volatile uint32_t BSRR;                /*GPIO PORT Bit Set/Reset Register*/
	volatile uint32_t LCKR;                /*GPIO PORT Lock Register*/
	volatile uint32_t AFR[2];              /*GPIO PORT Alternate Function Register*/
}GPIO_RegDef_t;

/*************************    RCC Register Definitions Structure    **********************************/
typedef struct
{
	volatile uint32_t CR;		  /*!< RCC clock control register                                   >!*/
	volatile uint32_t PLLCFGR;	  /*!< RCC PLL configuration register                               >!*/
	volatile uint32_t CFGR;		  /*!< RCC clock configuration register                             >!*/
	volatile uint32_t CIR;		  /*!< RCC clock interrupt register                                 >!*/
	volatile uint32_t AHB1RSTR;	  /*!< RCC AHB1 peripheral reset register                           >!*/
	volatile uint32_t AHB2RSTR;	  /*!< RCC AHB2 peripheral reset register                           >!*/
	volatile uint32_t AHB3RSTR;	  /*!< RCC AHB3 peripheral reset register                           >!*/
	uint32_t RESERVED0;			  /*!< Reserved, 0x1C                                               >!*/
	volatile uint32_t APB1RSTR;	  /*!< RCC APB1 peripheral reset register                           >!*/
	volatile uint32_t APB2RSTR;	  /*!< RCC APB2 peripheral reset register                           >!*/
	uint32_t RESERVED1[2];		  /*!< Reserved, 0x28-0x2C                                          >!*/
	volatile uint32_t AHB1ENR;	  /*!< RCC AHB1 peripheral clock register                           >!*/
	volatile uint32_t AHB2ENR;	  /*!< RCC AHB2 peripheral clock register                           >!*/
	volatile uint32_t AHB3ENR;	  /*!< RCC AHB3 peripheral clock register                           >!*/
	uint32_t RESERVED2;			  /*!< Reserved, 0x3C                                               >!*/
	volatile uint32_t APB1ENR;	  /*!< RCC APB1 peripheral clock enable register                    >!*/
	volatile uint32_t APB2ENR;	  /*!< RCC APB2 peripheral clock enable register                    >!*/
	uint32_t RESERVED3[2];		  /*!< Reserved, 0x48-0x4C                                          >!*/
	volatile uint32_t AHB1LPENR;  /*!< RCC AHB1 peripheral clock enable in low power mode register  >!*/
	volatile uint32_t AHB2LPENR;  /*!< RCC AHB2 peripheral clock enable in low power mode register  >!*/
	volatile uint32_t AHB3LPENR;  /*!< RCC AHB3 peripheral clock enable in low power mode register  >!*/
	uint32_t RESERVED4;			  /*!< Reserved, 0x5C                                               >!*/
	volatile uint32_t APB1LPENR;  /*!< RCC APB1 peripheral clock enable in low power mode register  >!*/
	volatile uint32_t APB2LPENR;  /*!< RCC APB2 peripheral clock enable in low power mode register  >!*/
	uint32_t RESERVED5[2];		  /*!< Reserved, 0x68-0x6C                                          >!*/
	volatile uint32_t BDCR;		  /*!< RCC Backup domain control register                           >!*/
	volatile uint32_t CSR;		  /*!< RCC clock control & status register                          >!*/
	uint32_t RESERVED6[2];		  /*!< Reserved, 0x78-0x7C                                          >!*/
	volatile uint32_t SSCGR;	  /*!< RCC spread spectrum clock generation register                >!*/
	volatile uint32_t PLLI2SCFGR; /*!< RCC PLLI2S configuration register                            >!*/
	volatile uint32_t PLLSAICFGR; /*!< RCC PLLSAI configuration register                            >!*/
	volatile uint32_t DCKCFGR;	  /*!< RCC Dedicated Clocks configuration register                  >!*/
	volatile uint32_t CKGATENR;	  /*!< RCC Clocks Gated ENable Register                             >!*/
	volatile uint32_t DCKCFGR2;	  /*!< RCC Dedicated Clocks configuration register 2                >!*/

} RCC_RegDef_t;


/*************************    RCC peripheral Definitions    **********************************/

#define RCC                  ((RCC_RegDef_t*)RCC_BASE_ADDRESS)


/*************************   RCC REGISTERS Bits *************************************************/

/* RCC_CR REG BITS */
#define CR_PLLSAIRDY 29       /* PLLSAI Clock Ready Flag */
#define CR_PLLSAION  28	      /* PLLSAI Enable */
#define CR_PLLI2SRDY 27       /* PLLI2S Clock Ready Flag */
#define CR_PLLI2SON  26	      /* PLLI2S Enable */
#define CR_PLLRDY    25	      /* Main PLL Clock Ready Flag */
#define CR_PLLON     24	      /* Main PLL Enable */
#define CR_CSSON     19		  /* Clock Security System Enable */
#define CR_HSEBYP    18	      /* HSE Clock ByPass */
#define CR_HSERDY    17	      /* HSE Clock Ready Flag */
#define CR_HSEON     16		  /* HSE Clock Enable */
#define CR_HSITRIM   3	      /* HSI Clock Trimming */
#define CR_HSIRDY    1		  /* HSI Clock Ready Flag */
#define CR_HSION     0		  /* HSI Clock Enable */

/* RCC_PLLCFGR REG BITS */
#define PLLCFGR_PLLR   28	  /* Main PLL Division Factor For I2Ss,SAIs & SYSTEM  */
#define PLLCFGR_PLLQ   24	  /* Main PLL Division Factor For USB OTG , SDIO */
#define PLLCFGR_PLLSRC 22     /* Main PLL & PLLI2S entry Clock Source */
#define PLLCFGR_PLLP   16	  /* Main PLL Division Factor For Main System Clock */
#define PLLCFGR_PLLN   6	  /* Main PLL Multiplication Factor */
#define PLLCFGR_PLLM   0	  /* Division Factor For Main PLL Input Clock */

/* RCC_CFGR */
#define CFGR_MCO2    30	/* Microcontroller Clock Output 2 */
#define CFGR_MCO2PRE 27 /* MCO2 Prescaler */
#define CFGR_MCO1PRE 24 /* MCO1 Prescaler */
#define CFGR_MCO1    21	/* Microcontroller Clock Output 1 */
#define CFGR_RTCPRE  16	/* HSE Division Fator for RTC Clock */
#define CFGR_PPRE2   13	/* APB2 Prescaler */
#define CFGR_PPRE1   10	/* APB1 Prescaler */
#define CFGR_HPRE    4	/* AHB Prescaler */
#define CFGR_SWS     2  /* System Clock Switch Status */
#define CFGR_SW      0	/* System Clock Switch */







/*************************    GPIO peripheral Definitions    **********************************/


#define GPIOA                        ((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)  
#define GPIOB                        ((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)  
#define GPIOC                        ((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)  
#define GPIOD                        ((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)  
#define GPIOE                        ((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)  
#define GPIOF                        ((GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)  
#define GPIOG                        ((GPIO_RegDef_t*)GPIOG_BASE_ADDRESS)  
#define GPIOH                        ((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)  
 



/* ------------------------------------------------------------------------------------------------------------- */
/* ------------------------------- CORTEX-M4 INTERNAL PERIPHERALS BASE ADDRESSES ------------------------------- */
/* ------------------------------------------------------------------------------------------------------------- */
#define SYSTICK_BASE_ADDRESS                0xE000E010UL


/* ------------------------------------------------------------------------------------------------------ */
/* ------------------------------- SYSTICK REGISTERS Definition Structure ------------------------------- */
/* ------------------------------------------------------------------------------------------------------ */
typedef struct
{
	volatile uint32_t SYST_CSR;	  /*<! SYSTICK Control and Status Register  >!*/
	volatile uint32_t SYST_RVR;	  /*<! SYSTICK Reload Value Register        >!*/
	volatile uint32_t SYST_CVR;	  /*<! SYSTICK Current Value Register       >!*/
	volatile uint32_t SYST_CALIB; /*<! SYSTICK Calibration Value Register   >!*/
} SYSTICK_RegDef_t;

/* --------------------------------------------------------------------------------------------- */
/* ------------------------------- SYSTICK Peripheral Definition ------------------------------- */
/* --------------------------------------------------------------------------------------------- */
#define SYSTICK ((SYSTICK_RegDef_t *)SYSTICK_BASE_ADDRESS)


/* ------------------------------------------------------------------------------------------------------------- */
/* ------------------------------- CORTEX-M4 INTERNAL PERIPHERALS BASE ADDRESSES ------------------------------- */
/* ------------------------------------------------------------------------------------------------------------- */
#define NVIC_BASE_ADDRESS 0xE000E100UL
#define SCB_BASE_ADDRESS  0xE000E008UL


/* -------------------------------------------------------------------------------------------------- */
/* ------------------------------- NVIC REGISTERS Definition Structure ------------------------------ */
/* -------------------------------------------------------------------------------------------------- */
typedef struct
{
	volatile uint32_t ISER[8]; /* Interrupt Set Enable Registers */
	uint32_t RESERVED0[24];
	volatile uint32_t ICER[8]; /* Interrupt Clear Enable Registers */
	uint32_t RESERVED1[24];
	volatile uint32_t ISPR[8]; /* Interrupt Set Pending Registers */
	uint32_t RESERVED2[24];
	volatile uint32_t ICPR[8]; /* Interrupt Clear Pending Registers */
	uint32_t RESERVED3[24];
	volatile uint32_t IABR[8]; /* Interrupt Active Bit Registers */
	uint32_t RESERVED4[56];
	volatile uint8_t IPR[240]; /* Interrupt Priority Registers */
	uint32_t RESERVED5[643];
	volatile uint32_t STIR;    /* Software Trigger Interrupt Register */
} NVIC_RegDef_t;

/* ------------------------------------------------------------------------------------------ */
/* ------------------------------- NVIC Peripheral Definition ------------------------------- */
/* ------------------------------------------------------------------------------------------ */
#define NVIC ((NVIC_RegDef_t *)NVIC_BASE_ADDRESS)

/* ------------------------------------------------------------------------------------------------- */
/* ------------------------------- SCB REGISTERS Definition Structure ------------------------------ */
/* ------------------------------------------------------------------------------------------------- */
typedef struct
{
	volatile uint32_t ACTLR; /* Auxilary Control Register */
	uint32_t RESERVED0[829];
	volatile uint32_t CPUID;   /* CPUID Base Register */
	volatile uint32_t ICSR;	   /* Interrupt Control and State Register */
	volatile uint32_t VTOR;	   /* Vector Table Offset Register */
	volatile uint32_t AIRCR;   /* Application Interrupt and Reset Control Register */
	volatile uint32_t SCR;	   /* System Control Register */
	volatile uint32_t CCR;	   /* Configuration and Control Register */
	volatile uint32_t SHPR[3]; /* Sytem Handler Priority Registers */
	volatile uint32_t SHCSR;   /* System Handler Control and State Register */
	volatile uint8_t MMFSR;	   /* MemManage Fault Status Register */
	volatile uint8_t BFSR;	   /* BusFault Status Register */
	volatile uint16_t UFSR;	   /* UsageFault Status Register */
	volatile uint32_t HFSR;	   /* HardFault Status Register */
	uint32_t RESERVED1;
	volatile uint32_t MMAR;    /* MemManage Fault Address Register */
	volatile uint32_t BFAR;    /* BusFault Address Register */
	volatile uint32_t AFSR;    /* Auxiliary Fault Status Register */
} SCB_RegDef_t;

/* ----------------------------------------------------------------------------------------- */
/* ------------------------------- SCB Peripheral Definition ------------------------------- */
/* ----------------------------------------------------------------------------------------- */
#define SCB ((SCB_RegDef_t *)SCB_BASE_ADDRESS)

/* ---------------------------------------------------------------------------------------------------- */
/* ------------------------------- SYSCFG REGISTERS Definition Structure ------------------------------ */
/* ---------------------------------------------------------------------------------------------------- */

typedef struct
{
	volatile uint32_t MEMRMP;	 /* SYSCFG Memory Remap Register */
	volatile uint32_t PMC;		 /* SYSCFG Peripheral Mode Configuration Register */
	volatile uint32_t EXTICR[4]; /* SYSCFG External Interrupt Configuration Registers*/
	uint32_t RESERVED0[2];
	volatile uint32_t CMPCR; /* Compensation Cell Control Register */
	uint32_t RESERVED1[2];
	volatile uint32_t CFGR; /* SYSCFG Configuration Register */

} SYSCFG_RegDef_t;


/* -------------------------------------------------------------------------------------------- */
/* ------------------------------- SYSCFG Peripheral Definition ------------------------------- */
/* -------------------------------------------------------------------------------------------- */
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASE_ADDRESS)

/* -------------------------------------------------------------------------------------------------- */
/* ------------------------------- EXTI REGISTERS Definition Structure ------------------------------ */
/* -------------------------------------------------------------------------------------------------- */

typedef struct
{
	volatile uint32_t IMR;	 /* Interrupt Mask Register */
	volatile uint32_t EMR;	 /* Event Mask Register */
	volatile uint32_t RTSR;	 /* Rising Trigger Selection Register */
	volatile uint32_t FTSR;	 /* Falling Trigger Selection Register */
	volatile uint32_t SWIER; /* Software Interrupt Event Register */
	volatile uint32_t PR;	 /* Pending Register */
} EXTI_RegDef_t;

/* ------------------------------------------------------------------------------------------ */
/* ------------------------------- EXTI Peripheral Definition ------------------------------- */
/* ------------------------------------------------------------------------------------------ */
#define EXTI ((EXTI_RegDef_t *)EXTI_BASE_ADDRESS)




/* ------------------------------------------------------------------------------------------------- */
/* ------------------------------- DMA REGISTERS Definition Structure ------------------------------ */
/* ------------------------------------------------------------------------------------------------- */

typedef struct
{
	volatile uint32_t CR;	/* DMA Stream x Configuration Register */
	volatile uint32_t NDTR; /* DMA Stream x Number of Data Register */
	volatile uint32_t PAR;	/* DMA Stream x Peripheral Address Register */
	volatile uint32_t M0AR; /* DMA Stream x Memory 0 Address Register */
	volatile uint32_t M1AR; /* DMA Stream x Memory 1 Address Register */
	volatile uint32_t FCR;	/* DMA Stream x FIFO Control Register */

} DMA_Stream_RegDef_t;


typedef struct
{
	volatile uint16_t ISR[4];				/* DMA Interrupt Status Registers */
	volatile uint16_t IFCR[4];				/* DMA Interrupt Flag Clear Registers */
	volatile DMA_Stream_RegDef_t STREAM[8]; /* DMA Stream Registers */

} DMA_RegDef_t;

/* ----------------------------------------------------------------------------------------- */
/* ------------------------------- DMA Peripheral Definition ------------------------------- */
/* ----------------------------------------------------------------------------------------- */
#define DMA1 ((DMA_RegDef_t *)DMA1_BASE_ADDRESS)
#define DMA2 ((DMA_RegDef_t *)DMA2_BASE_ADDRESS)


/* ---------------------------------------------------------------------------------- */
/* ------------------------------- DMA REGISTERS Bits ------------------------------- */
/* ---------------------------------------------------------------------------------- */

typedef enum
{
	EN = 0,		 /* DMA Stream Enable */
	DMEIE = 1,	 /* Direct Mode Error Interrupt Enable */
	TEIE = 2,	 /* Transfer Error Interrupt Enable */
	HTIE = 3,	 /* Half Transfer Interrupt Enable */
	TCIE = 4,	 /* Transfer Complete Interrupt Enable */
	PFCTRL = 5,	 /* Peripheral Flow Controller */
	DIR = 6,	 /* Data Transfer Direction */
	CIRC = 8,	 /* Circular Mode */
	PINC = 9,	 /* Peripheral Increment Mode */
	MINC = 10,	 /* Memory Increment Mode */
	PSIZE = 11,	 /* Peripheral Size */
	MSIZE = 13,	 /* Memory Size */
	PINCOS = 15, /* Peripheral Increment Offset Size */
	PL = 16,	 /* Priority Level */
	DBM = 18,	 /* Double Buffer Mode */
	CT = 19,	 /* Current Target Memory */
	PBURST = 21, /* Peripheral Burst Transfer Configuration */
	MBURST = 23, /* Memory Burst Transfer Configuration */
	CHSEL = 25	 /* Channel Selection */

} DMA_STREAM_CONFIGURATION_BITS_t;


typedef enum
{
	FTH = 0,   /* FIFO Threshold Selection */
	DMDIS = 2, /* Direct Mode Disable */
	FS = 3,	   /* FIFO Status */
	FEIE = 7   /* FIFO Error Interrupt Enable */

} DMA_FIFO_CONTROL_BITS_t;


/* --------------------------------------------------------------------------------------------------- */
/* ------------------------------- USART REGISTERS Definition Structure ------------------------------ */
/* --------------------------------------------------------------------------------------------------- */

typedef struct
{
	volatile uint16_t SR; /* USART Status Register */
	uint16_t RESERVED0;
	volatile uint16_t DR; /* USART Data Register */
	uint16_t RESERVED1;
	volatile uint16_t BRR; /* USART Baud Rate Register */
	uint16_t RESERVED2;
	volatile uint16_t CR1; /* USART Control Register 1 */
	uint16_t RESERVED3;
	volatile uint16_t CR2; /* USART Control Register 2 */
	uint16_t RESERVED4;
	volatile uint16_t CR3; /* USART Control Register 3 */
	uint16_t RESERVED5;
	volatile uint16_t GTPR; /* USART Guard Time and Prescaler Register */
	uint16_t RESERVED6;

} USART_Reg_t;


/* ------------------------------------------------------------------------------------------- */
/* ------------------------------- USART Peripheral Definition ------------------------------- */
/* ------------------------------------------------------------------------------------------- */
#define USART1 ((USART_Reg_t *)USART1_BASE_ADDRESS)
#define USART2 ((USART_Reg_t *)USART2_BASE_ADDRESS)
#define USART3 ((USART_Reg_t *)USART3_BASE_ADDRESS)
#define UART4 ((USART_Reg_t *)UART4_BASE_ADDRESS)
#define UART5 ((USART_Reg_t *)UART5_BASE_ADDRESS)
#define USART6 ((USART_Reg_t *)USART6_BASE_ADDRESS)

/* ------------------------------------------------------------------------------------ */
/* ------------------------------- USART REGISTERS Bits ------------------------------- */
/* ------------------------------------------------------------------------------------ */

typedef enum
{
	USART_DIV_FRACTION = 0, /* fraction of USARTDIV */
	USART_DIV_MANTISSA = 4  /* mantissa of USARTDIV */

} USART_BaudRateBits_t;

typedef enum
{
	USART_SBK = 0,	   /* Send Break */
	USART_RWU = 1,	   /* Receiver Wakeup */
	USART_RE = 2,		   /* Receiver Enable */
	USART_TE = 3,		   /* Transmitter Enable */
	USART_IDLEIE = 4,	   /* IDLE Interrupt Enable */
	USART_RXNEIE = 5,	   /* RXNE Interrupt Enable */
	USART_TCIE = 6, /* Transmission Complete Interrupt Enable */
	USART_TXEIE = 7,	   /* TXE Interrupt Enable */
	USART_PEIE = 8,	   /* PE Interrupt Enable */
	USART_PS = 9,		   /* Parity Selection */
	USART_PCE = 10,	   /* Parity Control Enable */
	USART_WAKE = 11,	   /* Wakeup Method */
	USART_M = 12,		   /* Word Length */
	USART_UE = 13,	   /* USART Enable */
	USART_OVER8 = 15,	   /* Oversampling Mode */

} USART_CR1_BITS_t;

typedef enum
{
	USART_ADD = 0,	/* Address of the USART node */
	USART_LBDL = 5,	/* LIN Break Detection Length */
	USART_LBDIE = 6,	/* LIN Break Detection Interrupt Enable */
	USART_LBCL = 8,	/* Last Bit Clock Pulse */
	USART_CPHA = 9,	/* Clock Phase */
	USART_CPOL = 10,	/* Clock Polarity */
	USART_CLKEN = 11, /* Clock Enable */
	USART_STOP = 12,	/* STOP bits */
	USART_LINEN = 14, /* LIN mode enable */

} USART_CR2_BITS_t;

typedef enum
{
	USART_EIE = 0,	/* Error Interrupt Enable */
	USART_IREN = 1,	/* IrDA mode Enable */
	USART_IRLP = 2,	/* IrDA Low-Power */
	USART_HDSEL = 3,	/* Half-Duplex Selection */
	USART_NACK = 4,	/* Smartcard NACK enable */
	USART_SCEN = 5,	/* Smartcard mode enable */
	USART_DMAR = 6,	/* DMA Enable Receiver */
	USART_DMAT = 7,	/* DMA Enable Transmitter */
	USART_RTSE = 8,	/* RTS Enable */
	USART_CTSE = 9,	/* CTS Enable */
	USART_CTSIE = 10, /* CTS Interrupt Enable */
	USART_ONEBIT = 11 /* One sample bit method enable */

} USART_CR3_BITS_t;

typedef enum
{
	USART_PSC = 0, /* Prescaler value */
	USART_GT = 8	 /* Guard time value */

} USART_GTPR_BITS_t;

/* ------------------------------------------------------------------------------------------------- */
/* ------------------------------- SPI REGISTERS Definition Structure ------------------------------ */
/* ------------------------------------------------------------------------------------------------- */
#define SPI_REG_t SPI_RegDef_t
typedef struct
{
	volatile uint16_t SPI_CR1; /* SPI Control Register 1 */
	uint16_t RESERVED0;
	volatile uint16_t SPI_CR2; /* SPI Control Register 2 */
	uint16_t RESERVED1;
	volatile uint16_t SPI_SR; /* SPI Status Register */
	uint16_t RESERVED2;
	volatile uint16_t SPI_DR; /* SPI Data Register */
	uint16_t RESERVED3;
	volatile uint16_t SPI_CRCPR; /* SPI CRC Polynomial Register */
	uint16_t RESERVED4;
	volatile uint16_t SPI_RXCRCR; /* SPI RX CRC Register */
	uint16_t RESERVED5;
	volatile uint16_t SPI_TXCRCR; /* SPI TX CRC Register */
	uint16_t RESERVED6;
	volatile uint16_t SPI_I2SCFGR; /* SPI_I2S Configuration Register */
	uint16_t RESERVED7;
	volatile uint16_t SPI_I2SPR; /* SPI_I2S Prescaler Register */
	uint16_t RESERVED8;
} SPI_RegDef_t;

/* ----------------------------------------------------------------------------------------- */
/* ------------------------------- SPI Peripheral Definition ------------------------------- */
/* ----------------------------------------------------------------------------------------- */
#define SPI1 ((SPI_RegDef_t *)SPI1_BASE_ADDRESS)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASE_ADDRESS)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASE_ADDRESS)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASE_ADDRESS)

/* ----------------------------------------------------------------------------------- */
/* ------------------------------- SPI REGISTERS' Bits ------------------------------- */
/* ----------------------------------------------------------------------------------- */

typedef enum
{
	SPI_CPHA = 0,	  /* Clock Phase */
	SPI_CPOL = 1,	  /* Clock Polarity */
	SPI_MSTR = 2,	  /* Master Selection */
	SPI_BR = 3,		  /* Baud Rate Control */
	SPI_SPE = 6,	  /* SPI Enable */
	SPI_LSBFIRST = 7, /* Direction */
	SPI_SSI = 8,	  /* Internal Slave Select */
	SPI_SSM = 9,	  /* Software Slave Management */
	SPI_RXONLY = 10,  /* Receive Only */
	SPI_DFF = 11,	  /* Data Frame Format */
	SPI_CRCNEXT = 12, /* CRC Transfer Next */
	SPI_CRCEN = 13,	  /* Hardware CRC Calculation Enable */
	SPI_BIDIOE = 14,  /* Output Enable in Bidirectional Mode */
	SPI_BIDIMODE = 15 /* Bidirectional Data Mode Enable */

} SPI_CR1_BITS_t;


typedef enum
{
	SPI_RXDMAEN = 0, /* Rx Buffer DMA Enable */
	SPI_TXDMAEN = 1, /* Tx Buffer DMA Enable */
	SPI_SSOE = 2,	 /* SS Output Enable */
	SPI_FRF = 4,	 /* Frame Format */
	SPI_ERRIE = 5,	 /* Error Interrupt Enable */
	SPI_RXNEIE = 6,	 /* RX Buffer Not Empty Interrupt Enable */
	SPI_TXEIE = 7,	 /* Tx Buffer Empty Interrupt Enable */

} SPI_CR2_BITS_t;

typedef enum
{
	SPI_CHSIDE = 2, /* Channel Side */
} SPI_SR_BITS_t;










#end if





