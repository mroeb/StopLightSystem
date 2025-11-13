//
// Created by mroeb on 11/1/25.
//

#ifndef PILOTEST_PILO_H
#define PILOTEST_PILO_H


#include  <algorithm>
#include  <cstring>
#include  <iostream>
#include  <unordered_map>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <ranges>
#include <unistd.h>
#include <linux/gpio.h>
#include <format>

#include <type_traits>

namespace Pilo
{
    enum class Direction : uint8_t
    {
        Input,
        Output
    };

    struct LineHandles
    {
        using HandleMap = std::unordered_map<unsigned int, int>;
        HandleMap InputHandles{};
        HandleMap OutputHandles{};
    };

    class GPIO
    {
    public:
        GPIO(const std::string& chip_name, const std::string& name): name(name)
        {
            chip_fd = open(chip_name.c_str(), O_RDWR);
            if (chip_fd < 0)
            {
                throw std::runtime_error(std::format("Failed to open the chip: {}", chip_name.c_str()));
            }
        }

        ~GPIO()
        {
            for (const auto &fd: line_handles.InputHandles | std::views::values)
            {
                close(fd);
            }
            for (const auto &fd: line_handles.OutputHandles | std::views::values)
            {
                close(fd);
            }
            if (chip_fd >= 0)
            {
                close(chip_fd);
            }
        };

        template<Direction dir, unsigned int... Lines>
        void add_lines()
        {
            (add_handle<dir>(Lines), ...);
        }

        template<unsigned int Line>
        void write(const bool value) const
        {
            if (chip_fd < 0)
            {
                return;
            }
            if (not line_handles.OutputHandles.contains(Line))
            {
                throw std::runtime_error(std::format("Failed to write to Line: {}. The Line is not Registered", Line));
            }
            gpiohandle_data data{};
            data.values[0] = value ? 1 : 0;
            ioctl(line_handles.OutputHandles.at(Line), GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        }

        template<unsigned int Line>
        [[nodiscard]] bool read() const
        {
            if (chip_fd < 0)
            {
                return false;
            }
            if (not line_handles.InputHandles.contains(Line))
            {
                throw std::runtime_error(std::format("Failed to read Line: {}. The Line is not Registered", Line));
            }
            gpiohandle_data data{};
            ioctl(line_handles.InputHandles.at(Line), GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
            return data.values[0];
        }

    private:
        int chip_fd{-1};
        LineHandles line_handles;
        std::string name{};

        template<Direction direction>
        void add_handle(const unsigned int line)
        {
            if constexpr (static_cast<int>(direction) == 0)
            {
                line_handles.InputHandles[line] = request_handle(line, GPIOHANDLE_REQUEST_INPUT);
            }

            if constexpr (static_cast<int>(direction) == 1)
            {
                line_handles.OutputHandles[line] = request_handle(line, GPIOHANDLE_REQUEST_OUTPUT);
            }
        }

        [[nodiscard]] int request_handle(const unsigned int line, const uint32_t flag)
        {
            gpiohandle_request req{};
            req.lineoffsets[0] = line;
            req.flags = flag;
            req.lines = 1;
            std::strncpy(req.consumer_label, name.c_str(), sizeof(req.consumer_label) - 1);
            req.consumer_label[sizeof(req.consumer_label) - 1] = '\0';
            if (ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0)
            {
                throw std::runtime_error("Failed to request GPIO Line: " + std::to_string(line));
            }
            return req.fd;
        }
    };
};

#endif //PILOTEST_PILO_H