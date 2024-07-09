#include <stdio.h>
#include "NUC029xAN.h"
#define RXBUFSIZE 128

uint8_t datasize[RXBUFSIZE] = {0};

volatile uint32_t karsayi = 0;
volatile uint32_t baslangic = 0;
volatile uint32_t kuyruk = 0;

void SYS_Init(void) {

	/* Unlock protected registers */
	SYS_UnlockReg();

	/* If the macros do not exist in your project, please refer to the related clk.h in Header folder of the tool package */
	/* Enable clock source */
	CLK_EnableXtalRC(
			CLK_PWRCON_OSC10K_EN_Msk | CLK_PWRCON_OSC22M_EN_Msk
					| CLK_PWRCON_XTL12M_EN_Msk);

	/* Waiting for clock source ready */
	CLK_WaitClockReady(
			CLK_CLKSTATUS_OSC10K_STB_Msk | CLK_CLKSTATUS_OSC22M_STB_Msk
					| CLK_CLKSTATUS_XTL12M_STB_Msk);

	/* Disable PLL first to avoid unstable when setting PLL */
	CLK_DisablePLL();

	/* Set PLL frequency */
	CLK->PLLCON = (CLK->PLLCON & ~(0x000FFFFFUL)) | 0x0000C22EUL;

	/* Waiting for PLL ready */
	CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk);

	/* Set HCLK clock */
	CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

	/* Enable IP clock */
	CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk; // TRM sayfa 207 clock enable 16 bool 1 olmali
	CLK->CLKSEL1 |= (3 << CLK_CLKSEL1_UART_S_Pos); //TRM sayfa 213  clock select HIRC
	//CLK->CLKDIV |= (11 << CLK_CLKDIV_UART_N_Pos); //TRM sayfa 216 115200 için 11 bit
	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
	SystemCoreClockUpdate();

	/* Set P3 multi-function pins for UART0 RXD and TXD  */
	SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);
	/* Lock protected registers */
	SYS_LockReg();
}

void UART_Init(void) {
	SYS->IPRSTC2 |= SYS_IPRSTC2_UART0_RST_Msk;
	SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

	UART0->BAUD = 0
			| UART_BAUD_MODE2
			| UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

	UART0->LCR = 0
			| UART_WORD_LEN_8
			| UART_PARITY_NONE
			| UART_STOP_BIT_1; //5609
}

/*void UART0_IRQHandler(void) {
	uint32_t newchar = 0xFF;
	uint32_t flag = UART0->ISR;
	if (flag & UART_ISR_RDA_INT_Msk) {
		printf("\nRX buffer:");

		while (UART0->ISR & UART_ISR_RDA_IF_Msk) {
			newchar = UART0->RBR;
			printf("%c", newchar);
			if (karsayi < RXBUFSIZE) {
				datasize[kuyruk] = newchar;
				kuyruk = (kuyruk == (RXBUFSIZE - 1)) ? 0 : (kuyruk + 1);
				karsayi++;
			}
		}
	}
	if (flag & UART_ISR_THRE_INT_Msk) {
		uint8_t temp = kuyruk;
		if (baslangic != temp) {
			newchar = datasize[baslangic];
			while (((UART0->FSR & UART_FSR_TX_FULL_Msk) >> UART_FSR_TX_FULL_Pos))
				UART0->THR = (newchar & 0xFF);
			baslangic = (baslangic == (RXBUFSIZE - 1)) ? 0 : (baslangic + 1);
			baslangic--;
		}
	}
}*/
void UART0_IRQHandler(void) {
    uint32_t newchar = 0xFF;
    uint32_t flag = UART0->ISR;

    if (flag & UART_ISR_RDA_INT_Msk) {
        while (UART0->ISR & UART_ISR_RDA_IF_Msk) {
            newchar = UART0->RBR;
            printf("%c", newchar);

            if (karsayi < RXBUFSIZE) {
                datasize[kuyruk] = newchar;
                kuyruk = (kuyruk == (RXBUFSIZE - 1)) ? 0 : (kuyruk + 1);
                karsayi++;
                if (newchar == '\n' || newchar == '\r') {
                    UART0->IER |= UART_IER_THRE_IEN_Msk;
                }
            }
        }
        UART0->ISR = UART_ISR_RDA_IF_Msk;
    }

    if (flag & UART_ISR_THRE_INT_Msk) {
        while (baslangic != kuyruk && !(UART0->FSR & UART_FSR_TX_FULL_Msk)) {
            newchar = datasize[baslangic];
            UART0->THR = newchar;
            baslangic = (baslangic == (RXBUFSIZE - 1)) ? 0 : (baslangic + 1);
            karsayi--;
            if (baslangic == kuyruk) {
                UART0->IER &= ~UART_IER_THRE_IEN_Msk;
                break;
            }
        }
        UART0->ISR = UART_ISR_THRE_IF_Msk;
    }
}

int main() {
	SYS_Init();

	UART_Init();

	UART0->IER = 0
			| UART_IER_RDA_IEN_Msk
			| UART_IER_THRE_IEN_Msk;

	NVIC_EnableIRQ(UART0_IRQn);

	while (1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
