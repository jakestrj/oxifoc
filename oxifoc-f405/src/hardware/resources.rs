//! Resource assignment — one `assign_resources!` block per supported board.
//!
//! Both boards follow the VESC reference layout, so motor/hall/uart blocks
//! are identical; only the DRV8301 bus wiring (and MK5's extra board-control
//! pins) differ. See docs/hw/cheap-focer2-notes.md and docs/hw/vesc6-mk5.md.

use assign_resources::assign_resources;
use embassy_stm32::{Peri, peripherals};

#[cfg(feature = "board-cf2")]
assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,   // Phase A high
        pa9: PA9,   // Phase B high
        pa10: PA10, // Phase C high
        pb13: PB13, // Phase A low
        pb14: PB14, // Phase B low
        pb15: PB15, // Phase C low
    }
    drv: DrvResources {
        spi3: SPI3,
        pc9: PC9,   // SPI3_CS
        pc10: PC10, // SPI3_SCK
        pc11: PC11, // SPI3_MISO
        pc12: PC12, // SPI3_MOSI
        pb5: PB5,   // EN_GATE
        pb7: PB7,   // nFAULT pin
        exti7: EXTI7, // EXTI for nFAULT interrupt
    }
    hall: HallResources {
        pc6: PC6,   // Hall sensor 1
        pc7: PC7,   // Hall sensor 2
        pc8: PC8,   // Hall sensor 3
    }
    can: CanResources {
        can: CAN1,
        rx: PB8,   // CAN1 RX
        tx: PB9,   // CAN1 TX
    }
}

#[cfg(feature = "board-vesc6-mk5")]
assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,   // Phase A high
        pa9: PA9,   // Phase B high
        pa10: PA10, // Phase C high
        pb13: PB13, // Phase A low
        pb14: PB14, // Phase B low
        pb15: PB15, // Phase C low
    }
    // DRV8301 on bit-bang SPI: PB3/PB4 are not a valid hardware-SPI mapping
    // (PC11/PC12 are the NRF51 UART on MK5), VESC bit-bangs it too.
    drv: DrvResources {
        pc9: PC9,   // CS
        pc10: PC10, // SCK
        pb3: PB3,   // MISO (JTDO after reset; GPIO reconfig leaves SWD alone)
        pb4: PB4,   // MOSI (NJTRST after reset)
        pb5: PB5,   // EN_GATE
        pb7: PB7,   // nFAULT pin
        exti7: EXTI7, // EXTI for nFAULT interrupt
    }
    hall: HallResources {
        pc6: PC6,   // Hall sensor 1
        pc7: PC7,   // Hall sensor 2
        pc8: PC8,   // Hall sensor 3
    }
    can: CanResources {
        can: CAN1,
        rx: PB8,   // CAN1 RX
        tx: PB9,   // CAN1 TX
    }
    board_ctrl: BoardCtrlResources {
        pc5: PC5,   // Shutdown latch — hold high or the button powers us off
        pd2: PD2,   // CURRENT_FILTER enable (active high)
        pc13: PC13, // PHASE_FILTER enable (active high, MK5+)
    }
}

#[cfg(feature = "board-jbldc")]
assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,   // Phase U high (UH IN)
        pa9: PA9,   // Phase V high (VH IN)
        pa10: PA10, // Phase W high (WH IN)
        pb13: PB13, // Phase U low  (UL IN)
        pb14: PB14, // Phase V low  (VL IN)
        pb15: PB15, // Phase W low  (WL IN)
    }
    drv: DrvResources {
        i2c1: I2C1,
        pb6: PB6,   // I2C1_SCL (EiceDRIVERs + 0x48 GD temp sensor)
        pb7: PB7,   // I2C1_SDA
        // Per-switch RDYC pins (open-drain ready / chip-select / fault-reset)
        pb4: PB4,   // RDYC_UH
        pc10: PC10, // RDYC_UL
        pc12: PC12, // RDYC_VH
        pc5: PC5,   // RDYC_VL
        pc7: PC7,   // RDYC_WH
        pa3: PA3,   // RDYC_WL
        // Per-switch FLT_N pins (active-low fault outputs)
        pc11: PC11, // FLT_N_UH
        pa15: PA15, // FLT_N_UL
        pd2: PD2,   // FLT_N_VH
        pb0: PB0,   // FLT_N_VL
        pc6: PC6,   // FLT_N_WH
        pa4: PA4,   // FLT_N_WL
    }
    adc: AdcResources {
        adc1: ADC1,
        adc2: ADC2,
        adc3: ADC3,
        pc2: PC2,   // CURR1: Phase A current (ADC123_IN12)
        pc1: PC1,   // CURR2: Phase B current (ADC123_IN11)
        pc0: PC0,   // CURR3: Phase C current (ADC123_IN10)
        pa0: PA0,   // VIN_SENSE: HVDC bus voltage (ADC123_IN0)
        pb1: PB1,   // NTC_SENSE: IGBT module temperature (ADC12_IN9)
    }
    can: CanResources {
        can: CAN1,
        rx: PB8,    // CAN1 RX
        tx: PB9,    // CAN1 TX
    }
}
