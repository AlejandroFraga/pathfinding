#ifndef TIMER_H
#define TIMER_H
#pragma once

#include <iostream>
#include <chrono>


namespace Benchmark {

    class Timer
    {
        typedef std::chrono::high_resolution_clock          hr_clock;
        typedef std::chrono::time_point<hr_clock>           time_point;
        typedef std::chrono::microseconds                   microseconds;

    private:
        const time_point m_start;
        const std::string m_title;

    public:
        Timer()
            : m_start(hr_clock::now()), m_title("Unknown") {}

        Timer(std::string title)
            : m_start(hr_clock::now()), m_title(title) {}

        ~Timer() { Stop(); }

        void Stop()
        {
            auto m_stop = hr_clock::now();

            auto start = std::chrono::time_point_cast<microseconds>(m_start).time_since_epoch().count();
            auto stop = std::chrono::time_point_cast<microseconds>(m_stop).time_since_epoch().count();

            auto duration = stop - start;
            double ms = duration * 0.001;

            std::cout << "Timer(" << m_title << "): " << duration << "us (" << ms << "ms)" << std::endl;
        }
    };
}

#endif