//! CAN bus transport layer for STM32F405 (`bxcan` on CAN1, PB8 RX / PB9 TX).
//!
//! Uses `streaming-protocol` + `nmea` fast-packet framing over 500 kbps CAN
//! with arbitration IDs `0x7E5` (RX) and `0x7ED` (TX), modeled after
//! `hv-bms-firmware-rs/src/comms/can_task.rs`.

use embassy_stm32::bind_interrupts;
use embassy_stm32::can::{
    Can, CanRx, CanTx, Fifo, Frame, Id, Rx0InterruptHandler, Rx1InterruptHandler,
    SceInterruptHandler, StandardId, TxInterruptHandler, filter::Mask32,
};
use embassy_stm32::peripherals::CAN1;
use nmea::nmea_message::Message as NmeaMessage;
use oxifoc_core::runtime::{CommsError, MessageSender, NmeaCanReceiver, send_messages};

use crate::config::{CAN_BITRATE, NMEA_RX_ARBITRATION_ID, NMEA_TX_ARBITRATION_ID, PWM_CONFIG};
use crate::hardware::resources::CanResources;
use crate::{FAULT_REGISTRY, STATE};

bind_interrupts!(pub struct CanIrqs {
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
    CAN1_TX => TxInterruptHandler<CAN1>;
});

struct CanTxSender<'a> {
    can_tx: CanTx<'a>,
    seq_counter: u8,
}

impl MessageSender for CanTxSender<'_> {
    async fn send_message_internal(&mut self, encoded_buf: &mut [u8]) -> Result<(), CommsError> {
        let Some(std_id) = StandardId::new(NMEA_TX_ARBITRATION_ID) else {
            return Err(CommsError::CanError);
        };
        let mut msg = NmeaMessage::from_payload(encoded_buf, self.seq_counter);
        self.seq_counter = (self.seq_counter + 1) & 0x07;
        while let Some(f) = msg.pop_frame() {
            let Ok(frame) = Frame::new_data(std_id, &f.bytes) else {
                return Err(CommsError::CanError);
            };
            self.can_tx.write(&frame).await;
        }
        Ok(())
    }
}

/// Initialize CAN1 on PB8 (RX) / PB9 (TX) at 500 kbps with accept-all filter.
///
/// Returns the owned [`Can`] instance so it can be moved into the `can_comm`
/// task before calling [`Can::split`]. Splitting inside `init_can` would drop
/// `Can` on return, triggering `Drop for Can` which master-resets `CAN1` into
/// Sleep Mode (`MCR.RESET = 1`).
pub async fn init_can(res: CanResources) -> Can<'static> {
    let mut can = Can::new(res.can, res.rx, res.tx, CanIrqs);
    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
    can.set_bitrate(CAN_BITRATE);
    can.set_tx_fifo_scheduling(true);
    can.enable().await;
    can
}

/// Read incoming 8-byte CAN frames on `NMEA_RX_ARBITRATION_ID` (`0x7E5`),
/// reassemble NMEA fast-packets, decode `streaming-protocol` messages, and
/// dispatch commands to `STATE`, `CMD_CHANNEL`, and `FAULT_REGISTRY`.
pub async fn can_task_read(mut can_rx: CanRx<'static>) {
    let mut receiver = NmeaCanReceiver::new();
    let Some(expected_std_id) = StandardId::new(NMEA_RX_ARBITRATION_ID) else {
        return;
    };
    let expected_id = Id::Standard(expected_std_id);

    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                if *envelope.frame.id() != expected_id {
                    continue;
                }
                if let Err(e) = receiver.handle_frame(
                    envelope.frame.data(),
                    &STATE,
                    &FAULT_REGISTRY,
                    PWM_CONFIG.pwm_freq_hz,
                ) {
                    defmt::warn!("CAN NMEA decode error: {:?}", e);
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
    let mut sender = CanTxSender {
        can_tx,
        seq_counter: 0,
    };
    send_messages(&mut sender).await;
}
