#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <array>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class ServoController {
public:
    explicit ServoController(const std::array<gpio_num_t, 4>& pins);
    ~ServoController();

    bool SetAngle(int servo_number, int angle_degrees);
    bool SetPulseWidthUs(int servo_number, uint32_t pulse_width_us);
    int GetAngle(int servo_number) const;

private:
    static constexpr int kServoCount = 4;
    static constexpr int kMinAngle = 0;
    static constexpr int kMaxAngle = 180;
    static constexpr uint32_t kPwmFrequencyHz = 50;
    static constexpr uint32_t kPeriodUs = 1000000 / kPwmFrequencyHz;
    static constexpr uint32_t kMinPulseWidthUs = 500;
    static constexpr uint32_t kMaxPulseWidthUs = 2500;
    static constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
    static constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_bit_t kResolution = LEDC_TIMER_13_BIT;

    std::array<gpio_num_t, kServoCount> pins_;
    std::array<ledc_channel_t, kServoCount> channels_;
    std::array<int, kServoCount> angles_;
    SemaphoreHandle_t mutex_ = nullptr;

    void RegisterMcpTools();
    bool SetPulseWidthUnlocked(int servo_index, uint32_t pulse_width_us);
};

#endif  // SERVO_CONTROLLER_H
