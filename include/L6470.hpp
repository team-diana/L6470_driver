/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <Core/HW/EXT.hpp>
#include <Core/HW/GPIO.hpp>
#include <Core/HW/SPI.hpp>

namespace drivers {
   class L6470
   {
public:
      L6470(
         Core::HW::SPIDevice&  spi,
         Core::HW::EXTChannel& ext,
         Core::HW::Pad&        stby,
         Core::HW::Pad&        flag
      );

      virtual
      ~L6470();

      bool
      probe();

      uint16_t
      getStatus();

      void
      run(
         int32_t speed
      );

      void
      move(
         int32_t steps
      );

      void
      moveto(
         int32_t position
      );

      void
      resetPosition();


protected:
      Core::HW::SPIDevice&  _spi;
      Core::HW::EXTChannel& _ext;
      Core::HW::Pad&        _stby;
      Core::HW::Pad&        _flag;

private:
      void
      transfer(
         uint16_t n,
         uint8_t* txbuf,
         uint8_t* rxbuf
      );

      uint32_t
      getParam(
         uint8_t param
      );

      void
      setParam(
         uint8_t  param,
         uint32_t value
      );
   };
}
