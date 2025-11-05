# STM32L4-AD8282-ECG

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](/LICENSE)
![MCU](https://img.shields.io/badge/MCU-STM32L476-blue)
![Toolchain](https://img.shields.io/badge/Toolchain-STM32CubeIDE-informational)
![Language](https://img.shields.io/badge/C-99-blueviolet)
![Baud](https://img.shields.io/badge/UART-115200-9cf)
![Sampling](https://img.shields.io/badge/Sampling-250%20Hz-orange)

> ECG (Electrocardiogram) acquisition on STM32L476G-DISCO with AD8232/AD8282 analog front end.  
> TIM6 → ADC1 (DMA, 12-bit, 250 Hz) → UART (PD5, 115200) stream to PC.

---

## Wiring (Quick Reference)

| AD8232 pin | STM32L476G-DISCO | Function |
|---|---|---|
| 3V3 | 3V3 | Power |
| GND | GND | Ground |
| OUTPUT | **PA0 (ADC1_IN5)** | ECG analog output |
| SDN̅ | **PA1 (GPIO OUT, set HIGH)** | Enable / Shutdown (active low) |
| LO+ | **PB0 (GPIO IN)** | Lead-off + (optional) |
| LO− | **PB1 (GPIO IN)** | Lead-off − (optional) |
| RA | Electrode | Right arm |
| LA | Electrode | Left arm |
| RL | Electrode | Right leg (reference) |

### Wiring Diagram (Mermaid)

```mermaid
flowchart LR
  subgraph AD8232_Module [AD8232 Module]
    V33[3V3]
    GNDm[GND]
    OUT[OUTPUT]
    SDN[SDN̅]
    LOplus[LO+]
    LOminus[LO−]
    RA_IN[RA]
    LA_IN[LA]
    RL_IN[RL]
  end

  subgraph STM32L476G_DISCO [STM32L476G-DISCO]
    V3[3V3]
    GNDs[GND]
    PA0[PA0 / ADC1_IN5]
    PA1[PA1 / GPIO OUT]
    PB0[PB0 / GPIO IN]
    PB1[PB1 / GPIO IN]
    PD5[PD5 / USART2_TX]
    PD6[PD6 / USART2_RX]
  end

  %% Power & ground
  V33 --- V3
  GNDm --- GNDs

  %% Signal wiring
  OUT --> PA0
  SDN --> PA1
  LOplus --> PB0
  LOminus --> PB1

  %% UART to PC
  PD5 -->|UART 115200 8N1| PC[PC Serial RX]

  %% Electrodes
  RA_E[Right Arm Electrode] --- RA_IN
  LA_E[Left Arm Electrode]  --- LA_IN
  RL_E[Right Leg Electrode] --- RL_IN
