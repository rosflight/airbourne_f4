# Airbourne-F4

Driver interface layer for STM32F4xx chips. Based off https://github.com/superjax/airbourne but specifically only for the F4xx. The goal is to make this similar in simplicity to BreezySTM32

## Quick Reference

Here are some tables to help you see what is linked to what

### DMA

| **DMA1** | Stream0 | Stream1 | Stream2 | Stream3 | Stream4 | Stream5 | Stream6 | Stream7 |
|------|---------|---------|---------|---------|---------|---------|---------|---------|
| CHN0 |         |         | SPI2-RX | SPI1-RX | SPI1-TX | SPI2-TX |         |         |
| CHN1 |   I2C0  |         |         |         |         |         |         |         |
| CHN2 |         |         |         |         |         |         |         |         |
| CHN3 |         |         |         |         |         |         |         | DSHOT3  |
| CHN4 |         | UART2-RX |         | UART2-TX |         |         |         |         |
| CHN5 |         |         | DSHOT0-1 |         |         | UART1-RX | UART1-TX |         |
| CHN6 |         |         |         |         |         |         | DSHOT4-5 |         |
| CHN7 |         |         |   I2C1  |         |         |         |         |         |


| **DMA2** | Stream0 | Stream1 | Stream2 | Stream3 | Stream4 | Stream5 | Stream6 | Stream7 |
|------|---------|---------|---------|---------|---------|---------|---------|---------|
| CHN0 |         |         |         |         |         |         |         |         |
| CHN1 |         |         |         |         |         |         |         |         |
| CHN2 |         |         |         |         |         |         |         |         |
| CHN3 |         |         | SPI0-RX | SPI0-TX |         |         |         |         |
| CHN4 |         |         |         |         |         | UART0-RX |         | UART0-TX |
| CHN5 |         |         |         |         |         |         |         |         |
| CHN6 |         |         |         |         |         |         |         |         |
| CHN7 |         | DSHOT6-9 |         |         |         |         |         |         |