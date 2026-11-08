//! CAN bus transport layer for STM32G474 (`fdcan` on FDCAN1, PB8 RX / PB9 TX).
//!
//! Uses `streaming-protocol` + `nmea` fast-packet framing over 500 kbps CAN
//! with arbitration IDs `0x7E5` (RX) and `0x7ED` (TX), modeled after
//! `hv-bms-firmware-rs/src/comms/can_task.rs`.

use embassy_stm32::bind_interrupts;
use embassy_stm32::can::{
    CanConfigurator, CanRx, CanTx, Frame, IT0InterruptHandler, IT1InterruptHandler,
};
use embassy_stm32::peripherals::FDCAN1;
use embassy_time::Timer;
use nmea::nmea_message::Message as NmeaMessage;
use oxifoc_core::runtime::{CommsError, MessageSender, NmeaCanReceiver, send_messages};

use crate::config::{
    CAN_BITRATE, NMEA_INTER_FRAME_DELAY_MS, NMEA_RX_ARBITRATION_ID, NMEA_TX_ARBITRATION_ID,
    PWM_CONFIG,
};
use crate::hardware::CanResources;
use crate::protocol::{DeviceState, set_device_state};
use crate::{FAULT_REGISTRY, STATE};

bind_interrupts!(pub struct CanIrqs {
    FDCAN1_IT0 => IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => IT1InterruptHandler<FDCAN1>;
});

struct CanTxSender<'a> {
    can_tx: CanTx<'a>,
}

impl MessageSender for CanTxSender<'_> {
    async fn send_message_internal(&mut self, encoded_buf: &mut [u8]) -> Result<(), CommsError> {
        let mut msg = NmeaMessage::from_payload(encoded_buf, 0);
        while let Some(f) = msg.pop_frame() {
            let Ok(frame) = Frame::new_standard(NMEA_TX_ARBITRATION_ID, &f.bytes) else {
                return Err(CommsError::CanError);
            };
            self.can_tx.write(&frame).await;
            Timer::after_millis(NMEA_INTER_FRAME_DELAY_MS).await;
        }
        Ok(())
    }
}

/// Initialize FDCAN1 on PB8 (RX) / PB9 (TX) at 500 kbps in normal mode and
/// return the split `(CanTx, CanRx)` handles.
pub fn init_can(res: CanResources) -> (CanTx<'static>, CanRx<'static>) {
    let mut can = CanConfigurator::new(res.can, res.rx, res.tx, CanIrqs);
    can.set_bitrate(CAN_BITRATE);
    let can = can.into_normal_mode();
    let (can_tx, can_rx, _props) = can.split();
    (can_tx, can_rx)
}

/// Read incoming 8-byte CAN frames on `NMEA_RX_ARBITRATION_ID` (`0x7E5`),
/// reassemble NMEA fast-packets, decode `streaming-protocol` messages, and
/// dispatch commands to `STATE`, `CMD_CHANNEL`, and `FAULT_REGISTRY`.
pub async fn can_task_read(mut can_rx: CanRx<'static>) {
    let mut receiver = NmeaCanReceiver::new();
    let Ok(expected_frame) = Frame::new_standard(NMEA_RX_ARBITRATION_ID, &[]) else {
        return;
    };
    let expected_id = *expected_frame.id();

    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                if *envelope.frame.id() != expected_id {
                    continue;
                }
                match receiver.handle_frame(
                    envelope.frame.data(),
                    &STATE,
                    &FAULT_REGISTRY,
                    PWM_CONFIG.pwm_freq_hz,
                ) {
                    Ok(true) => {
                        set_device_state(DeviceState::Linked);
                    }
                    Ok(false) => {}
                    Err(e) => {
                        defmt::warn!("CAN NMEA decode error: {:?}", e);
                    }
                }
            }
            Err(err) => {
                defmt::error!("Error in CAN frame read: {:?}", err);
            }
        }
    }
}

/// Watch telemetry channels and transmit `streaming-protocol` + `nmea` frames
/// on `NMEA_TX_ARBITRATION_ID` (`0x7ED`).
pub async fn can_task_send(can_tx: CanTx<'static>) {
    let mut sender = CanTxSender { can_tx };
    send_messages(&mut sender).await;
}
