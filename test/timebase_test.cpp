#include <Arduino.h>
#include "timebase.h"

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(200);

    us_init();

    Serial.println();
    Serial.println("=== timebase test ===");
    Serial.print("SystemCoreClock  "); Serial.println(SystemCoreClock);
    Serial.print("TIM5 clk         "); Serial.println(timerClkFreq(TIM5));
    Serial.print("PSC              "); Serial.println(TIM5->PSC);
    Serial.print("ARR              "); Serial.println(TIM5->ARR);
    Serial.print("CEN              "); Serial.println(TIM5->CR1 & TIM_CR1_CEN);
    Serial.print("PCLK1            "); Serial.println(HAL_RCC_GetPCLK1Freq());
        Serial.print("PCLK2            "); Serial.println(HAL_RCC_GetPCLK2Freq());
        Serial.print("TIM5 clk         "); Serial.println(timerClkFreq(TIM5));
        Serial.print("TIM9 clk         "); Serial.println(timerClkFreq(TIM9));

    // 1. is it counting at all?
    uint32_t a = us_now(), b = us_now();
    Serial.print("two reads back-to-back: "); Serial.print(a);
    Serial.print(" -> "); Serial.println(b);

    // 2. wrap arithmetic, forced
    TIM5->CNT = 0xFFFFF000UL;
    uint32_t w = us_now();
    delay(100);
    Serial.print("across forced wrap, expect ~100000: ");
    Serial.println(us_since(w));
}

void loop() {
    uint32_t t = us_now();
    delay(1000);
    Serial.print("1000 ms measured as ");
    Serial.print(us_since(t));
    Serial.println(" us");
}