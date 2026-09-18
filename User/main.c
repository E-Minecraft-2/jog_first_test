#include "stm32f10x.h"
#include "App.h"

int main(void)
{
    SystemInit();
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    App_Init();

    while (1)
        App_Process();
}
