ENTRY(_start)

PROVIDE(NMI_Handler = DefaultHandler);
PROVIDE(HardFault_Handler = DefaultHandler);
PROVIDE(MemManage_Handler = DefaultHandler);
PROVIDE(BusFault_Handler = DefaultHandler);
PROVIDE(UsageFault_Handler = DefaultHandler);
PROVIDE(SVC_Handler = DefaultHandler);
PROVIDE(DebugMon_Handler = DefaultHandler);
PROVIDE(PendSV_Handler = DefaultHandler);
PROVIDE(SysTick_Handler = DefaultHandler);

.syntax unified
.thumb

.section .isr_vector, "a", %progbits
.type g_pfnVectors, %object
.size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word DebugMon_Handler
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =_estack
    mov sp, r0
    
    ldr r0, =_data
    ldr r1, =_edata
    ldr r2, =_etext
    movs r3, #0
    b LoopCopyDataInit

CopyDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit
    
    ldr r2, =_bss
    ldr r4, =_ebss
    movs r3, #0
    b LoopFillZerobss

FillZerobss:
    movs r3, #0
    str r3, [r2]
    adds r2, r2, #4

LoopFillZerobss:
    cmp r2, r4
    bcc FillZerobss
    
    bl main
    bx lr

.size Reset_Handler, .-Reset_Handler

.section .text.DefaultHandler,"ax",%progbits
DefaultHandler:
Infinite_Loop:
    b Infinite_Loop
.size DefaultHandler, .-DefaultHandler
