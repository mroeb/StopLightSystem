#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <algorithm>
#include <memory>
#include <format>
#include "pilo.hpp" // Including the real Pilo::GPIO library

// ====================================================================
// Traffic Light Application Logic (C++ OOP) - HARDWARE READY
// ====================================================================

// Utility for pausing the execution
void wait_ms(int ms)
{
    // Note: The real hardware application will use longer, more realistic timing.
    // The simulation used short times (400ms) for quick console output.
    // We will use 1000ms here as a minimum for real hardware testing.
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// --- 1. The Light Abstraction ---

/**
 * @brief Represents a single colored LED in the system.
 */
class Light
{
private:
    Pilo::GPIO& gpio_chip_;
    unsigned int line_number_;
    std::string color_;

public:
    // Expose line_number_ for CarTrafficLight's state check
    const unsigned int line_number_;

    Light(Pilo::GPIO& chip, unsigned int line, std::string color)
        : gpio_chip_(chip), line_number_(line), color_(std::move(color))
    {
        // Add as an Output line. Initial state is LOW (off).
        gpio_chip_.add_lines(Pilo::Direction::Output, {line_number_}, Pilo::InitialValue::Low);
    }

    void on() const
    {
        gpio_chip_.write(line_number_, true);
    }

    void off() const
    {
        gpio_chip_.write(line_number_, false);
    }

    [[nodiscard]] std::string get_status() const
    {
        return std::format("{}", color_);
    }
};

class TrafficUnit
{
protected:
    Pilo::GPIO& gpio_chip_;
    std::string name_;
    std::map<std::string, std::unique_ptr<Light>> lights_;

public:
    TrafficUnit(Pilo::GPIO& chip, std::string name)
        : gpio_chip_(chip), name_(std::move(name)) {}

    virtual ~TrafficUnit() = default;

    [[nodiscard]] std::string show_status() const
    {
        std::string status = std::format("{}: [ ", name_);
        for (const auto& [color, light] : lights_)
        {
            status += light->get_status() + " ";
        }
        status += "]";
        return status;
    }
};

class CarTrafficLight : public TrafficUnit
{
private:
    Light& red() { return *lights_.at("Red"); }
    Light& yellow() { return *lights_.at("Yellow"); }
    Light& green() { return *lights_.at("Green"); }

public:
    CarTrafficLight(Pilo::GPIO& chip, std::string name, unsigned int red_pin, unsigned int yellow_pin, unsigned int green_pin)
        : TrafficUnit(chip, std::move(name))
    {
        lights_["Red"] = std::make_unique<Light>(gpio_chip_, red_pin, "Red");
        lights_["Yellow"] = std::make_unique<Light>(gpio_chip_, yellow_pin, "Yellow");
        lights_["Green"] = std::make_unique<Light>(gpio_chip_, green_pin, "Green");
    }

    void set_red()
    {
        red().on();
        yellow().off();
        green().off();
    }

    void cycle_to_green(int delay_ms)
    {
        std::cout << std::format("  -- {} Phase: RED-YELLOW --", name_);
        red().on();
        yellow().on();
        green().off();
        std::cout << " (Wait " << delay_ms << "ms)" << std::endl;
        wait_ms(delay_ms);

        std::cout << std::format("  -- {} Phase: GREEN --", name_) << std::endl;
        red().off();
        yellow().off();
        green().on();
    }
    void cycle_to_red(int delay_ms)
    {
        std::cout << std::format("  -- {} Phase: YELLOW --", name_);
        red().off();
        yellow().on();
        green().off();
        std::cout << " (Wait " << delay_ms << "ms)" << std::endl;
        wait_ms(delay_ms);

        std::cout << std::format("  -- {} Phase: RED --", name_) << std::endl;
        set_red();
    }
};

class PedestrianLight : public TrafficUnit
{
private:
    Light& red() { return *lights_.at("Red"); }
    Light& green() { return *lights_.at("Green"); }

public:
    PedestrianLight(Pilo::GPIO& chip, std::string name, unsigned int red_pin, unsigned int green_pin)
        : TrafficUnit(chip, std::move(name))
    {
        lights_["Red"] = std::make_unique<Light>(gpio_chip_, red_pin, "Red");
        lights_["Green"] = std::make_unique<Light>(gpio_chip_, green_pin, "Green");

        stop_crossing();
    }
    void allow_crossing()
    {
        std::cout << std::format("  -- {} Phase: GREEN (WALK) --", name_) << std::endl;
        red().off();
        green().on();
    }

    void stop_crossing()
    {
        std::cout << std::format("  -- {} Phase: RED (STOP) --", name_) << std::endl;
        red().on();
        green().off();
    }
};


class TrafficController
{
private:
    Pilo::GPIO gpio_chip_;

    CarTrafficLight light_A_;
    CarTrafficLight light_B_;
    PedestrianLight ped_light_;

    unsigned int button_pin_;
    bool is_request_pending_{false};

    std::chrono::milliseconds debounce_time_{100};
    std::chrono::high_resolution_clock::time_point last_button_read_time_;

    static constexpr int WAIT_TIME_MS = 5000;
    static constexpr int CYCLE_TIME_MS = 1000;
    static constexpr int WALK_TIME_MS = 6000;

public:
    TrafficController()
        : gpio_chip_("/dev/gpiochip0", "German_Traffic_Control"),

          light_A_(gpio_chip_, "Car Light A", 17, 27, 22),

          light_B_(gpio_chip_, "Car Light B", 5, 6, 13),

          ped_light_(gpio_chip_, "Pedestrian Light", 19, 26),

          button_pin_(21)
    {
        gpio_chip_.add_lines(Pilo::Direction::Input, {button_pin_});
        std::cout << "\n--- Traffic Controller Initialized (BCM Pins Used) ---" << std::endl;
        std::cout << "Starting Phase: Light A Green (Main Road Flow)" << std::endl;

        light_A_.set_red();
        light_B_.set_red();
        ped_light_.stop_crossing();

        light_A_.cycle_to_green(CYCLE_TIME_MS);

        show_current_state();
    }

    void show_current_state() const
    {
        std::cout << "\n[STATUS] ========================================" << std::endl;
        std::cout << light_A_.show_status() << " (BCM 17, 27, 22)" << std::endl;
        std::cout << light_B_.show_status() << " (BCM 5, 6, 13)" << std::endl;
        std::cout << ped_light_.show_status() << " (BCM 19, 26)" << std::endl;
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << std::format("Pedestrian Request Pending: {}", is_request_pending_ ? "YES" : "NO") << std::endl;
        std::cout << "=================================================" << std::endl;
    }

    void check_button()
    {
        auto now = std::chrono::high_resolution_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_button_read_time_) < debounce_time_)
        {
             return;
        }

        if (gpio_chip_.read(button_pin_))
        {
            if (!is_request_pending_)
            {
                std::cout << "\n*** PHYSICAL PEDESTRIAN BUTTON PRESSED (BCM 21)! Request pending. ***" << std::endl;
                is_request_pending_ = true;
                last_button_read_time_ = now;
            }
        }
    }

    void run()
    {
        std::cout << "\nStarting main hardware loop. Monitoring BCM 21 for pedestrian request." << std::endl;

        while (true)
        {
            std::cout << "\n[LOOP] Waiting on main phase (Light A Green) for " << WAIT_TIME_MS/1000 << "s..." << std::endl;

            for (int elapsed = 0; elapsed < WAIT_TIME_MS; elapsed += 100)
            {
                wait_ms(100);
                check_button();
                if (is_request_pending_) break;
            }

            if (is_request_pending_)
            {
                std::cout << "\n--- STARTING TRAFFIC CYCLE FOR PEDESTRIAN CROSSING ---" << std::endl;
                is_request_pending_ = false;

                light_A_.cycle_to_red(CYCLE_TIME_MS);
                show_current_state();
                wait_ms(CYCLE_TIME_MS);

                std::cout << "\n--- Pedestrian Crossing Phase (WALK) ---" << std::endl;
                ped_light_.allow_crossing();
                show_current_state();
                wait_ms(WALK_TIME_MS);

                ped_light_.stop_crossing();
                show_current_state();
                wait_ms(CYCLE_TIME_MS);

                std::cout << "\n--- Transitioning to Car Light B Green ---" << std::endl;
                light_B_.cycle_to_green(CYCLE_TIME_MS);
                show_current_state();

                std::cout << "\n[LOOP] Waiting on cross-road phase (Light B Green) for " << WAIT_TIME_MS/1000 << "s..." << std::endl;

                wait_ms(WAIT_TIME_MS);

                light_B_.cycle_to_red(CYCLE_TIME_MS);
                show_current_state();
                wait_ms(CYCLE_TIME_MS);

                std::cout << "\n--- Transitioning back to Car Light A Green ---" << std::endl;
                light_A_.cycle_to_green(CYCLE_TIME_MS);
                show_current_state();

            } else {
                std::cout << "[LOOP] No pending request. Resuming wait." << std::endl;
            }
        }
    }
};

int main()
{
    try
    {
        TrafficController controller;
        controller.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        std::cerr << "Ensure you have the correct permissions (e.g., running as root or with udev rules) and the chip path is correct." << std::endl;
        return 1;
    }

    std::cout << "\nProgram Ended." << std::endl;
    return 0;
}