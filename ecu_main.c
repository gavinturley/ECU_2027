#include <optional>
#include <setjmp.h>
#include <stdint.h>

#include "assert.hpp"
#include "util.hpp"
#include "can_serde.hpp"
#include "ecu_logic.hpp"

void assert_failed_handler(AssertLevel level, LineInfo info, AssertCode error_code) {
    /* Shut everything down. (shutdown) */
    CAN_message_t shutdown_message = empty_can_message(MessageId::ControlCommand, 8);
    /* `empty_can_message` is guaranteed to generate a message full of zeroes,
    * and since this is panic wind down code, it's best to keep it as simple as
    * possible, so we don't use the normal message creation function. */
    MotorCAN.write(shutdown_message);

    /* Loop forever so we never do anything after the panic. */
    while (true) {
        Serial.printf("Safety assertion failed! In file %s:%d with error code %d\n", info.filename, info.line_no, error_code);
        Serial.printf("File hash %lu\n", (unsigned long) str_hash(info.filename));

        CriticalFault fault_msg;
        fault_msg.error_code = error_code;
        fault_msg.assert_failure_line = info.line_no;
        fault_msg.file_name_hash = str_hash(info.filename);

        MotorCAN.write(create_critical_fault_command(fault_msg));

        pinMode(13, OUTPUT);
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        delay(500);
    }
}

int main(void)
{
    Ecu ECU = {};
    
    Timer broadcast_build_info_timer(0, BROADCAST_INFO_INTERVAL_MS);
    Timer debug_pacing(0, 500);


    // First things first, register the panic handler. If something goes
    // wrong during setup, we'll wind everything down.
    register_assert_failed_handler(assert_failed_handler);

    Serial.begin(SERIAL_BAUD_RATE);

    MotorCAN.begin();
    MotorCAN.setBaudRate(CAN_BAUD_RATE);
    DataCAN.begin();
    DataCAN.setBaudRate(CAN_BAUD_RATE);


    Serial.println("============================================");
    Serial.println("==========Motor CAN initialized=============");
    Serial.println("============================================");



	while (true) {
            
        /* The ECU does not keep track of what time it is, nor does it use `millis`,
        * so we always have to tell it what time it is. */
        uint32_t current_time_ms = millis();

        /* Receive a message and have the ECU process it. */
        CAN_message_t rmsg;
        if (MotorCAN.read(rmsg)) {
            ECU.processMessage(current_time_ms, rmsg);
        }

        /* Generate all outgoing messages and send each. */
        while (true) {
            std::optional<CAN_message_t> to_send = ECU.pollCan(current_time_ms);
            if (to_send.has_value()) {
                MotorCAN.write(*to_send);
            } else {
                break;
            }
        }

        auto state = ECU.pollGpioState(current_time_ms);
        digitalWrite(HORN_PIN, state.horn_on);
        digitalWrite(BRAKE_LIGHT_PIN, state.brake_light_on);

        if (broadcast_build_info_timer.shouldFire(current_time_ms)) {
            MotorCAN.write(create_code_hash_message(GIT_COMMIT_HASH_U64));
            MotorCAN.write(create_commit_author_message(GIT_COMMIT_AUTHOR));
            MotorCAN.write(create_uploader_message(GIT_UPLOADER));
        }

        #ifdef ENABLE_DEBUGGING
        if (debug_pacing.shouldFire(current_time_ms)) {
            ECU.printState();
        }
        #endif
    }
}