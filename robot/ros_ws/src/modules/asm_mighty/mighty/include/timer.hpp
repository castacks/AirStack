#include <chrono>
#include <iostream>

// avoid redefinition of Timer
#ifndef TIMER_HPP
#define TIMER_HPP

namespace timer
{

  class Timer
  {
      
      typedef std::chrono::high_resolution_clock high_resolution_clock;
      typedef std::chrono::milliseconds milliseconds;

    private:

      high_resolution_clock::time_point _start;

    public:

      explicit Timer(bool run = false)
    {
        if (run)
          Reset();
      }

      void Reset()
      {
        _start = high_resolution_clock::now();
      }
      
      double getElapsedMs() const
      {
        return (std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start)).count();
      }

      void printMs(const std::string &str)
      {
        std::cout << str << " " << getElapsedMs() << " ms" << std::endl;
      }

      double getElapsedMicros() const
      {
        return std::chrono::duration_cast<std::chrono::microseconds>(high_resolution_clock::now() - _start).count();
      }

      template <typename T, typename Traits>
      friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const Timer& timer)
      {
        return out << timer.getElapsedMs();
      }

  };
      
}  // namespace timer

#endif  // TIMER_HPP

