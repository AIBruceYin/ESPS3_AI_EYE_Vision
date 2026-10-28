#include "servo_controller.h"

#include <algorithm>
#include <cstdio>

#include <esp_err.h>
#include <esp_log.h>

#include "mcp_server.h"

namespace {
constexpr const char* TAG = "ServoController";
}

ServoController::ServoController(const std::array<gpio_num_t, kServoCount>& pins)
    : pins_(pins),
      channels_({
          LEDC_CHANNEL_1,
          LEDC_CHANNEL_2,
          LEDC_CHANNEL_3,
          LEDC_CHANNEL_4,
      }),
      angles_({-1, -1, -1, -1}) {
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        abort();
    }

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kSpeedMode;
    timer_config.duty_resolution = kResolution;
    timer_config.timer_num = kTimer;
    timer_config.freq_hz = kPwmFrequencyHz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    for (int index = 0; index < kServoCount; ++index) {
        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num = pins_[index];
        channel_config.speed_mode = kSpeedMode;
        channel_config.channel = channels_[index];
        channel_config.intr_type = LEDC_INTR_DISABLE;
        channel_config.timer_sel = kTimer;
        channel_config.duty = 0;
        channel_config.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
        ESP_LOGI(TAG, "Servo %d initialized on GPIO%d", index + 1, pins_[index]);
    }

    RegisterMcpTools();
}

ServoController::~ServoController() {
    for (ledc_channel_t channel : channels_) {
        ledc_stop(kSpeedMode, channel, 0);
    }
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
    }
}

int ServoController::GetAngle(int servo_number) const {
    if (servo_number < 1 || servo_number > kServoCount) {
        return -1;
    }
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return -1;
    }
    const int angle = angles_[servo_number - 1];
    xSemaphoreGive(mutex_);
    return angle;
}

bool ServoController::SetPulseWidthUnlocked(int servo_index,
                                             uint32_t pulse_width_us) {
    pulse_width_us =
        std::clamp(pulse_width_us, kMinPulseWidthUs, kMaxPulseWidthUs);
    constexpr uint32_t max_duty = (1u << 13) - 1;
    const uint32_t duty =
        (pulse_width_us * max_duty + kPeriodUs / 2) / kPeriodUs;

    esp_err_t err =
        ledc_set_duty(kSpeedMode, channels_[servo_index], duty);
    if (err != ESP_OK) {
        return false;
    }
    return ledc_update_duty(kSpeedMode, channels_[servo_index]) == ESP_OK;
}

bool ServoController::SetPulseWidthUs(int servo_number,
                                      uint32_t pulse_width_us) {
    if (servo_number < 1 || servo_number > kServoCount) {
        return false;
    }
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    const uint32_t clamped_pulse_width =
        std::clamp(pulse_width_us, kMinPulseWidthUs, kMaxPulseWidthUs);
    const bool success =
        SetPulseWidthUnlocked(servo_number - 1, clamped_pulse_width);
    if (success) {
        angles_[servo_number - 1] =
            kMinAngle +
            static_cast<int>(
                ((clamped_pulse_width - kMinPulseWidthUs) *
                     (kMaxAngle - kMinAngle) +
                 (kMaxPulseWidthUs - kMinPulseWidthUs) / 2) /
                (kMaxPulseWidthUs - kMinPulseWidthUs));
    }
    xSemaphoreGive(mutex_);
    return success;
}

bool ServoController::SetAngle(int servo_number, int angle_degrees) {
    if (servo_number < 1 || servo_number > kServoCount ||
        angle_degrees < kMinAngle || angle_degrees > kMaxAngle) {
        return false;
    }
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    const uint32_t pulse_width_us =
        kMinPulseWidthUs +
        static_cast<uint32_t>(angle_degrees) *
            (kMaxPulseWidthUs - kMinPulseWidthUs) /
            (kMaxAngle - kMinAngle);
    const bool success =
        SetPulseWidthUnlocked(servo_number - 1, pulse_width_us);
    if (success) {
        angles_[servo_number - 1] = angle_degrees;
    }
    xSemaphoreGive(mutex_);
    return success;
}

void ServoController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool(
        "self.servo.set_angle",
        "Move one of the four board servos to an absolute angle. "
        "Servo 1=GPIO11, 2=GPIO12, 3=GPIO13, 4=GPIO14. "
        "Use angles from 0 to 180 degrees.",
        PropertyList({
            Property("servo", kPropertyTypeInteger, 1, kServoCount),
            Property("angle", kPropertyTypeInteger, 90, kMinAngle, kMaxAngle),
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            return SetAngle(properties["servo"].value<int>(),
                            properties["angle"].value<int>());
        });

    mcp_server.AddTool(
        "self.servo.get_angles",
        "Get the last commanded angles of the four board servos.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            (void)properties;
            char json[128];
            snprintf(json, sizeof(json),
                     "{\"servo1\":%d,\"servo2\":%d,\"servo3\":%d,"
                     "\"servo4\":%d}",
                     GetAngle(1), GetAngle(2), GetAngle(3), GetAngle(4));
            return std::string(json);
        });

    mcp_server.AddTool(
        "self.servo.center",
        "Move all four board servos to their 90 degree center position.",
        PropertyList(),
        [this](const PropertyList& properties) -> ReturnValue {
            (void)properties;
            bool success = true;
            for (int servo = 1; servo <= kServoCount; ++servo) {
                success = SetAngle(servo, 90) && success;
            }
            return success;
        });
}
