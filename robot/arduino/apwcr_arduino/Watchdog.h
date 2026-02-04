/*
  Watchdog.h

  Purpose:
  Declares a watchdog utility that ensures the robot stops safely if communication from Python is lost.

  Responsibilities:
  - Track the time of the last valid command received
  - Provide an expired() check used by controllers
  - Support a clear safety policy: stale command means motors stop
*/
