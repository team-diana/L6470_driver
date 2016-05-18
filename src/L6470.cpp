/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <cmath>
#include <Module.hpp>

#include <L6470_driver/L6470.hpp>

uint32_t status = 0;

// REGISTERS
#define REG_ABS_POS              0x01
#define REG_EL_POS               0x02
#define REG_MARK                 0x03
#define REG_SPEED                0x04
#define REG_ACC                  0x05
#define REG_DEC                  0x06
#define REG_MAX_SPEED            0x07
#define REG_MIN_SPEED            0x08
#define REG_FS_SPD               0x15
#define REG_KVAL_HOLD            0x09
#define REG_KVAL_RUN             0x0A
#define REG_KVAL_ACC             0x0B
#define REG_KVAL_DEC             0x0C
#define REG_INT_SPD              0x0D
#define REG_ST_SLP               0x0E
#define REG_FN_SLP_ACC           0x0F
#define REG_FN_SLP_DEC           0x10
#define REG_K_THERM              0x11
#define REG_ADC_OUT              0x12
#define REG_OCD_TH               0x13
#define REG_STALL_TH             0x14
#define REG_STEP_MODE            0x16
#define REG_ALARM_EN             0x17
#define REG_CONFIG               0x18
#define REG_STATUS               0x19

// COMMANDS
#define CMD_NOP                  0x00
#define CMD_SET_PARAM            0x00
#define CMD_GET_PARAM            0x20
#define CMD_RUN                  0x50
#define CMD_STEP_CLOCK           0x58
#define CMD_MOVE                 0x40
#define CMD_GOTO                 0x60
#define CMD_GOTO_DIR             0x68
#define CMD_GO_UNTIL             0x82
#define CMD_RELEASE_SW           0x92
#define CMD_GO_HOME              0x70
#define CMD_GO_MARK              0x78
#define CMD_RESET_POS            0xD8
#define CMD_RESET_DEVICE         0xC0
#define CMD_SOFT_STOP            0xB0
#define CMD_HARD_STOP            0xB8
#define CMD_SOFT_HIZ             0xA0
#define CMD_HARD_HIZ             0xA8
#define CMD_GET_STATUS           0xD0

// Constant definitions provided by ST

// constant definitions for overcurrent thresholds. Write these values to
//  register OCD_TH to set the level at which an overcurrent even occurs.
#define OCD_TH_375mA  0x00
#define OCD_TH_750mA  0x01
#define OCD_TH_1125mA 0x02
#define OCD_TH_1500mA 0x03
#define OCD_TH_1875mA 0x04
#define OCD_TH_2250mA 0x05
#define OCD_TH_2625mA 0x06
#define OCD_TH_3000mA 0x07
#define OCD_TH_3375mA 0x08
#define OCD_TH_3750mA 0x09
#define OCD_TH_4125mA 0x0A
#define OCD_TH_4500mA 0x0B
#define OCD_TH_4875mA 0x0C
#define OCD_TH_5250mA 0x0D
#define OCD_TH_5625mA 0x0E
#define OCD_TH_6000mA 0x0F

// STEP_MODE option values.
// First comes the "microsteps per step" options...
#define STEP_MODE_STEP_SEL 0x07  // Mask for these bits only.
#define STEP_SEL_1     0x00
#define STEP_SEL_1_2   0x01
#define STEP_SEL_1_4   0x02
#define STEP_SEL_1_8   0x03
#define STEP_SEL_1_16  0x04
#define STEP_SEL_1_32  0x05
#define STEP_SEL_1_64  0x06
#define STEP_SEL_1_128 0x07

// ...next, define the SYNC_EN bit. When set, the BUSYN pin will instead
//  output a clock related to the full-step frequency as defined by the
//  SYNC_SEL bits below.
#define STEP_MODE_SYNC_EN  0x80  // Mask for this bit
#define SYNC_EN 0x80

// ...last, define the SYNC_SEL modes. The clock output is defined by
//  the full-step frequency and the value in these bits- see the datasheet
//  for a matrix describing that relationship (page 46).
#define STEP_MODE_SYNC_SEL 0x70
#define SYNC_SEL_1_2 0x00
#define SYNC_SEL_1   0x10
#define SYNC_SEL_2   0x20
#define SYNC_SEL_4   0x30
#define SYNC_SEL_8   0x40
#define SYNC_SEL_16  0x50
#define SYNC_SEL_32  0x60
#define SYNC_SEL_64  0x70

// Bit names for the ALARM_EN register.
//  Each of these bits defines one potential alarm condition.
//  When one of these conditions occurs and the respective bit in ALARM_EN is set,
//  the FLAG pin will go low. The register must be queried to determine which event
//  caused the alarm.
#define ALARM_EN_OVERCURRENT       0x01
#define ALARM_EN_THERMAL_SHUTDOWN  0x02
#define ALARM_EN_THERMAL_WARNING   0x04
#define ALARM_EN_UNDER_VOLTAGE     0x08
#define ALARM_EN_STALL_DET_A       0x10
#define ALARM_EN_STALL_DET_B       0x20
#define ALARM_EN_SW_TURN_ON        0x40
#define ALARM_EN_WRONG_NPERF_CMD   0x80

// CONFIG register renames.

// Oscillator options.
// The dSPIN needs to know what the clock frequency is because it uses that for some
//  calculations during operation.
#define CONFIG_OSC_SEL                 0x000F // Mask for this bit field.
#define CONFIG_INT_16MHZ               0x0000 // Internal 16MHz, no output
#define CONFIG_INT_16MHZ_OSCOUT_2MHZ   0x0008 // Default; internal 16MHz, 2MHz output
#define CONFIG_INT_16MHZ_OSCOUT_4MHZ   0x0009 // Internal 16MHz, 4MHz output
#define CONFIG_INT_16MHZ_OSCOUT_8MHZ   0x000A // Internal 16MHz, 8MHz output
#define CONFIG_INT_16MHZ_OSCOUT_16MHZ  0x000B // Internal 16MHz, 16MHz output
#define CONFIG_EXT_8MHZ_XTAL_DRIVE     0x0004 // External 8MHz crystal
#define CONFIG_EXT_16MHZ_XTAL_DRIVE    0x0005 // External 16MHz crystal
#define CONFIG_EXT_24MHZ_XTAL_DRIVE    0x0006 // External 24MHz crystal
#define CONFIG_EXT_32MHZ_XTAL_DRIVE    0x0007 // External 32MHz crystal
#define CONFIG_EXT_8MHZ_OSCOUT_INVERT  0x000C // External 8MHz crystal, output inverted
#define CONFIG_EXT_16MHZ_OSCOUT_INVERT 0x000D // External 16MHz crystal, output inverted
#define CONFIG_EXT_24MHZ_OSCOUT_INVERT 0x000E // External 24MHz crystal, output inverted
#define CONFIG_EXT_32MHZ_OSCOUT_INVERT 0x000F // External 32MHz crystal, output inverted

// Configure the functionality of the external switch input
#define CONFIG_SW_MODE                 0x0010 // Mask for this bit.
#define CONFIG_SW_HARD_STOP            0x0000 // Default; hard stop motor on switch.
#define CONFIG_SW_USER                 0x0010 // Tie to the GoUntil and ReleaseSW
//  commands to provide jog function.
//  See page 25 of datasheet.

// Configure the motor voltage compensation mode (see page 34 of datasheet)
#define CONFIG_EN_VSCOMP               0x0020  // Mask for this bit.
#define CONFIG_VS_COMP_DISABLE         0x0000  // Disable motor voltage compensation.
#define CONFIG_VS_COMP_ENABLE          0x0020  // Enable motor voltage compensation.

// Configure overcurrent detection event handling
#define CONFIG_OC_SD                   0x0080  // Mask for this bit.
#define CONFIG_OC_SD_DISABLE           0x0000  // Bridges do NOT shutdown on OC detect
#define CONFIG_OC_SD_ENABLE            0x0080  // Bridges shutdown on OC detect

// Configure the slew rate of the power bridge output
#define CONFIG_POW_SR                  0x0300  // Mask for this bit field.
#define CONFIG_SR_180V_us              0x0000  // 180V/us
#define CONFIG_SR_290V_us              0x0200  // 290V/us
#define CONFIG_SR_530V_us              0x0300  // 530V/us

// Integer divisors for PWM sinewave generation
//  See page 32 of the datasheet for more information on this.
#define CONFIG_F_PWM_DEC               0x1C00      // mask for this bit field
#define CONFIG_PWM_MUL_0_625           (0x00) << 10
#define CONFIG_PWM_MUL_0_75            (0x01) << 10
#define CONFIG_PWM_MUL_0_875           (0x02) << 10
#define CONFIG_PWM_MUL_1               (0x03) << 10
#define CONFIG_PWM_MUL_1_25            (0x04) << 10
#define CONFIG_PWM_MUL_1_5             (0x05) << 10
#define CONFIG_PWM_MUL_1_75            (0x06) << 10
#define CONFIG_PWM_MUL_2               (0x07) << 10

// Multiplier for the PWM sinewave frequency
#define CONFIG_F_PWM_INT               0xE000     // mask for this bit field.
#define CONFIG_PWM_DIV_1               (0x00) << 13
#define CONFIG_PWM_DIV_2               (0x01) << 13
#define CONFIG_PWM_DIV_3               (0x02) << 13
#define CONFIG_PWM_DIV_4               (0x03) << 13
#define CONFIG_PWM_DIV_5               (0x04) << 13
#define CONFIG_PWM_DIV_6               (0x05) << 13
#define CONFIG_PWM_DIV_7               (0x06) << 13

// Status register bit renames- read-only bits conferring information about the
//  device to the user.
#define STATUS_HIZ                     0x0001 // high when bridges are in HiZ mode
#define STATUS_BUSY                    0x0002 // mirrors BUSY pin
#define STATUS_SW_F                    0x0004 // low when switch open, high when closed
#define STATUS_SW_EVN                  0x0008 // active high, set on switch falling edge,
//  cleared by reading STATUS
#define STATUS_DIR                     0x0010 // Indicates current motor direction.
//  High is FWD, Low is REV.
#define STATUS_NOTPERF_CMD             0x0080 // Last command not performed.
#define STATUS_WRONG_CMD               0x0100 // Last command not valid.
#define STATUS_UVLO                    0x0200 // Undervoltage lockout is active
#define STATUS_TH_WRN                  0x0400 // Thermal warning
#define STATUS_TH_SD                   0x0800 // Thermal shutdown
#define STATUS_OCD                     0x1000 // Overcurrent detected
#define STATUS_STEP_LOSS_A             0x2000 // Stall detected on A bridge
#define STATUS_STEP_LOSS_B             0x4000 // Stall detected on B bridge
#define STATUS_SCK_MOD                 0x8000 // Step clock mode is active

// Status register motor status field
#define STATUS_MOT_STATUS                0x0060      // field mask
#define STATUS_MOT_STATUS_STOPPED       (0x0000) << 13 // Motor stopped
#define STATUS_MOT_STATUS_ACCELERATION  (0x0001) << 13 // Motor accelerating
#define STATUS_MOT_STATUS_DECELERATION  (0x0002) << 13 // Motor decelerating
#define STATUS_MOT_STATUS_CONST_SPD     (0x0003) << 13 // Motor at constant speed

namespace drivers {
   L6470::L6470(
      Core::HW::SPIDevice&  spi,
      Core::HW::EXTChannel& ext,
      Core::HW::Pad&        stby,
      Core::HW::Pad&        flag
   ) : _spi(spi), _ext(ext), _stby(stby), _flag(flag) {}

   L6470::~L6470()
   {}

   void
   L6470::transfer(
      uint16_t n,
      uint8_t* txbuf,
      uint8_t* rxbuf
   )
   {
      _spi.acquireBus();

      for (int i = 0; i < n; i++) {
         _spi.select();

         if (!rxbuf) {
            _spi.send(1, &txbuf[i]);
         } else if (!txbuf) {
            _spi.receive(1, &rxbuf[i]);
         } else {
            _spi.exchange(1, &txbuf[i], &rxbuf[i]);
         }

         _spi.deselect();
      }

      _spi.releaseBus();
   } // L6470::transfer

   uint32_t
   L6470::getParam(
      uint8_t param
   )
   {
      uint8_t  txbuf = 0;
      uint8_t  rxbuf = 0;
      uint32_t rx    = 0;

      _spi.acquireBus();

      /* Send GetParam operation code to dSPIN */
      txbuf = (uint8_t)CMD_GET_PARAM | (uint8_t)param;

      _spi.select();
      _spi.send(1, &txbuf);
      _spi.deselect();

      switch (param) {
        case REG_ABS_POS:
        case REG_MARK:
        case REG_SPEED:
           _spi.select();
           _spi.receive(1, &rxbuf);
           _spi.deselect();

           rx |= rxbuf << 16;
        case REG_EL_POS:
        case REG_ACC:
        case REG_DEC:
        case REG_MAX_SPEED:
        case REG_MIN_SPEED:
        case REG_FS_SPD:
        case REG_INT_SPD:
        case REG_CONFIG:
        case REG_STATUS:
           _spi.select();
           _spi.receive(1, &rxbuf);
           _spi.deselect();

           rx |= rxbuf << 8;
        default:
           _spi.select();
           _spi.receive(1, &rxbuf);
           _spi.deselect();

           rx |= rxbuf;
      } // switch

      _spi.releaseBus();

      return rx;
   } // L6470::getParam

   void
   L6470::setParam(
      uint8_t  param,
      uint32_t value
   )
   {
      uint8_t txbuf;

      _spi.acquireBus();

      /* Send SetParam operation code to dSPIN */
      txbuf = (uint8_t)CMD_SET_PARAM | (uint8_t)param;

      _spi.select();
      _spi.send(1, &txbuf);
      _spi.deselect();

      switch (param) {
        case REG_ABS_POS:
           ;
        case REG_MARK:
           ;
           /* Send parameter - byte 2 to dSPIN */
           txbuf = (uint8_t)(value >> 16);

           _spi.select();
           _spi.send(1, &txbuf);
           _spi.deselect();
        case REG_EL_POS:
           ;
        case REG_ACC:
           ;
        case REG_DEC:
           ;
        case REG_MAX_SPEED:
           ;
        case REG_MIN_SPEED:
           ;
        case REG_FS_SPD:
           ;
#if defined(L6470)
        case REG_INT_SPD:
           ;
#endif /* defined(L6470) */
        case REG_CONFIG:
           ;
        case REG_STATUS:
           txbuf = (uint8_t)(value >> 8);

           _spi.select();
           _spi.send(1, &txbuf);
           _spi.deselect();
        default:
           txbuf = (uint8_t)(value);

           _spi.select();
           _spi.send(1, &txbuf);
           _spi.deselect();
      } // switch

      _spi.releaseBus();
   } // L6470::setParam

   uint16_t
   L6470::getStatus()
   {
      uint8_t txbuf = 0;
      uint8_t rxbuf[2];

      _spi.acquireBus();

      /* Send GetParam operation code to dSPIN */
      txbuf = (uint8_t)CMD_GET_STATUS;

      _spi.select();
      _spi.send(1, &txbuf);
      _spi.deselect();


      _spi.select();
      _spi.receive(1, &rxbuf[0]);
      _spi.deselect();

      _spi.select();
      _spi.receive(1, &rxbuf[1]);
      _spi.deselect();

      _spi.releaseBus();

      return rxbuf[0] << 8 | rxbuf[1];
   } // L6470::getStatus

   void
   L6470::run(
      int32_t speed
   )
   {
      uint8_t txbuf[4];

      if (speed >= 0) {
         txbuf[0] = (uint8_t)CMD_RUN | 1;
      } else {
         txbuf[0] = (uint8_t)CMD_RUN;
         speed    = -speed;
      }

      txbuf[1] = speed >> 16;
      txbuf[2] = speed >> 8;
      txbuf[3] = speed;

      transfer(4, txbuf, NULL);
   }

   void
   L6470::move(
      int32_t steps
   )
   {
      uint8_t txbuf[4];

      if (steps >= 0) {
         txbuf[0] = (uint8_t)CMD_MOVE | 1;
      } else {
         txbuf[0] = (uint8_t)CMD_MOVE;
         steps    = -steps;
      }

      txbuf[1] = steps >> 16;
      txbuf[2] = steps >> 8;
      txbuf[3] = steps;

      transfer(4, txbuf, NULL);
   }

   void
   L6470::moveto(
      int32_t position
   )
   {
      uint8_t txbuf[4];

      if (position >= 0) {
         txbuf[0] = (uint8_t)CMD_GOTO_DIR | 1;
      } else {
         txbuf[0] = (uint8_t)CMD_GOTO_DIR;
         position = -position;
      }

      txbuf[1] = position >> 16;
      txbuf[2] = position >> 8;
      txbuf[3] = position;

      transfer(4, txbuf, NULL);
   }

   void
   L6470::resetPosition()
   {
      uint8_t txbuf = CMD_RESET_POS;

      transfer(1, &txbuf, NULL);
   }

   bool
   L6470::probe()
   {
      _stby.set();
      Core::MW::Thread::sleep(Core::MW::Time::ms(5));
      _stby.clear();
      Core::MW::Thread::sleep(Core::MW::Time::ms(5));
      _stby.set();
      Core::MW::Thread::sleep(Core::MW::Time::ms(5));

      status = getStatus();

      uint32_t tmp = getParam(REG_CONFIG);

      if (tmp != 0x2e88) {
         return false;
      }

      setParam(REG_STEP_MODE, !SYNC_EN | STEP_SEL_1_128 | SYNC_SEL_1);
      setParam(REG_MAX_SPEED, (unsigned long int)std::ceil(1000 * .065536));
      setParam(REG_FS_SPD, (50 * .065536) - .5);
      setParam(REG_ACC, 0x0ff);
      setParam(REG_DEC, 0x0ff);
      setParam(REG_OCD_TH, OCD_TH_2250mA);
      setParam(REG_CONFIG, CONFIG_PWM_DIV_1 | CONFIG_PWM_MUL_2 | CONFIG_SR_180V_us | CONFIG_OC_SD_DISABLE | CONFIG_VS_COMP_DISABLE | CONFIG_SW_HARD_STOP | CONFIG_INT_16MHZ);
#if 1
      setParam(REG_KVAL_HOLD, 53);
      setParam(REG_KVAL_ACC, 53);
      setParam(REG_KVAL_DEC, 53);
      setParam(REG_KVAL_RUN, 53);
      setParam(REG_INT_SPD, 7417);
      setParam(REG_ST_SLP, 34);
      setParam(REG_FN_SLP_ACC, 34);
      setParam(REG_FN_SLP_DEC, 34);
#endif

      Core::MW::Thread::sleep(Core::MW::Time::ms(5));
      tmp = getParam(REG_CONFIG);
      Core::MW::Thread::sleep(Core::MW::Time::ms(5));
      status = getStatus();
      Core::MW::Thread::sleep(Core::MW::Time::ms(5));
      status = getStatus();
      Core::MW::Thread::sleep(Core::MW::Time::ms(5));
      return true;
   } // L6470::probe
}
