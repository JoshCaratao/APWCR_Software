/*
  EncoderQuad.h

  Purpose:
  Declares an encoder wrapper used by APWCR controllers. This class provides a clean interface for:
  - absolute tick count
  - tick rate (ticks per second)
  - angular speed (rad per second) of the output shaft

  Implementation note:
  The low-level quadrature decoding can be handled by an existing encoder library. This wrapper adds:
  - speed estimation using delta ticks over delta time
  - conversion to physical units using configured CPR, quadrature factor, and gear ratio
  - consistent sign conventions
*/
