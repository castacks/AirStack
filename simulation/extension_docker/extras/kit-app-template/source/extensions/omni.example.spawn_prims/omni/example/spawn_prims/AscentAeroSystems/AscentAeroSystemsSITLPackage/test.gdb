set environment LD_PRELOAD /extras/kit-app-template/source/extensions/omni.example.spawn_prims/omni/example/spawn_prims/AscentAeroSystems/AscentAeroSystemsSITLPackage/inject.so

b _ZN7HALSITL9Scheduler10stop_clockEm
catch signal SIGSEGV

commands 1
  #info registers rsi
  call (void)injected_function($rsi)
  continue

commands 2
  bt