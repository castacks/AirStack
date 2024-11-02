set environment LD_PRELOAD /extras/drag_and_drop/inject.so

b _ZN7HALSITL9Scheduler10stop_clockEm
catch signal SIGSEGV

commands 1
  silent
  #info registers rsi
  call (void)injected_function($rsi)
  continue

commands 2
  bt