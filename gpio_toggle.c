#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>

#define BCM2711_PERI_BASE        0xFE000000  // Base address for Raspberry Pi 4
#define GPIO_BASE                (BCM2711_PERI_BASE + 0x200000)  // GPIO controller

#define GPIO_PIN                 17  // GPIO pin number

#define BLOCK_SIZE               (4 * 1024)

// GPIO Registers (Section 6.1 of BCM2711 ARM Peripherals manual)
#define GPFSEL1                  0x04  // Function Select for GPIO10-GPIO19
#define GPSET0                   0x1C  // Set pin high
#define GPCLR0                   0x28  // Set pin low

volatile uint32_t *gpio;  // Pointer to GPIO memory-mapped region
volatile int flag = 1;

void setup_gpio() {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open");
        exit(-1);
    }

    void *gpio_map = mmap(
        NULL,                 // Any address in our space will do
        BLOCK_SIZE,           // Map length
        PROT_READ | PROT_WRITE, // Enable reading & writing to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File descriptor to /dev/mem
        GPIO_BASE             // Offset to GPIO peripheral
    );

    if (gpio_map == MAP_FAILED) {
        perror("mmap");
        exit(-1);
    }

    gpio = (volatile uint32_t *)gpio_map;

    close(mem_fd);

    // Set GPIO 17 as output (using GPFSEL1 register, bits 21-23 for GPIO17)
    gpio[GPFSEL1 / 4] &= ~(7 << 21); // Clear bits 21-23 (set to 000 for input)
    gpio[GPFSEL1 / 4] |= (1 << 21);  // Set bits 21-23 to 001 (for output mode)
    flag = 2;
}

void gpio_set(int pin) {
    gpio[GPSET0 / 4] = (1 << pin);
    flag = 3;
}

void gpio_clear(int pin) {
    gpio[GPCLR0 / 4] = (1 << pin);
    flag = 4;
}

int main() {
    setup_gpio();

    while (1) {
        gpio_set(GPIO_PIN);   // Set GPIO pin high
        printf("high---\n");
        sleep(1);             // Wait for 1 second
        gpio_clear(GPIO_PIN); // Set GPIO pin low
        printf("low---\n");
        sleep(1);             // Wait for 1 second
    }

    return 0;
}
