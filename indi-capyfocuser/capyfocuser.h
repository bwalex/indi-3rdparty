#pragma once

#include "indifocuser.h"

#include <time.h>           // for nsleep() R Brown June 2021
#include <errno.h>          // for nsleep() R Brown June 2021

#include <chrono>

class CapyFocuser : public INDI::Focuser {
public:
    CapyFocuser();
    ~CapyFocuser();

    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

protected:
    virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual bool Handshake() override;
    virtual bool saveConfigItems(FILE *fp) override;

    virtual void TimerHit() override;

    /***************************************************/
    /*** INDI Focuser interface                      ***/
    /***************************************************/

    /**
     * \brief MoveFocuser the focuser to an absolute position.
     * \param ticks The new position of the focuser.
     * \return Return IPS_OK if motion is completed and focuser reached requested position. Return
     * IPS_BUSY if focuser started motion to requested position and is in progress.
     * Return IPS_ALERT if there is an error.
     */
    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;

    /**
     * \brief MoveFocuser the focuser to an relative position.
     * \param dir Direction of focuser, either FOCUS_INWARD or FOCUS_OUTWARD.
     * \param ticks The relative ticks to move.
     * \return Return IPS_OK if motion is completed and focuser reached requested position. Return
     * IPS_BUSY if focuser started motion to requested position and is in progress.
     * Return IPS_ALERT if there is an error.
     */
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;

    /**
     * @brief ReverseFocuser Reverse focuser motion direction
     * @param enabled If true, normal default focuser motion is reversed. If false, the direction is set to the default focuser motion.
     * @return True if successful, false otherwise.
     */
    virtual bool ReverseFocuser(bool enabled) override;

    /**
     * @brief SyncFocuser Set current position to ticks without moving the focuser.
     * @param ticks Desired new sync position.
     * @return True if successful, false otherwise.
     */
    virtual bool SyncFocuser(uint32_t ticks) override;

    /**
     * @brief SetFocuserMaxPosition Set Focuser Maximum position limit in the hardware.
     * @param ticks maximum steps permitted
     * @return True if successful, false otherwise.
     * @note If setting maximum position limit in the hardware is not available or not supported, do not override this function as the default
     * implementation will always return true.
     */
    virtual bool SetFocuserMaxPosition(uint32_t ticks) override;

    /**
     * @brief AbortFocuser all focus motion
     * @return True if abort is successful, false otherwise.
     */
    virtual bool AbortFocuser() override;

private:
    bool fetchConfig();
    bool fetchSeq();
    bool fetchMaxPos();
    bool ack();

    bool wrcfg(uint16_t reg, const void *data, size_t sz);
    bool rdcfg(uint16_t reg, void *data, size_t sz);
    bool rdseq(uint16_t seq, void *data, size_t sz);
    bool probe(void *data, size_t sz);
    bool wrreg(uint16_t reg, const void *data, size_t sz);
    bool rdreg(uint16_t reg, void *data, size_t sz);
    bool stop();
    bool autohome();
    bool moveRel(uint8_t dir, uint32_t nsteps);
    bool moveAbs(uint32_t pos);

    bool xceive(uint8_t opcode, const uint16_t *regp, const uint8_t *wrmsg, size_t wrsz, uint8_t *rdmsg, size_t rdsz, size_t rdoff);
    bool xceive_ll(const uint8_t *wrmsg, size_t wrsz, uint8_t *rdmsg, size_t rdsz);
    uint16_t crc16_init();
    uint16_t crc16_update(uint16_t crc, const uint8_t *buf, size_t len);

    bool configRead;

    INDI::PropertyText FirmwareRevisionTP {1};

    //  IDLE, MOVING, HOMING, FAULT...
    INDI::PropertyText DriverStatusTP {1};

    //  Motor enabled/disabled
    INDI::PropertySwitch CoilEnergizedSP {2};
    enum
    {
        COIL_ENERGIZED_OFF,
        COIL_ENERGIZED_ON
    };

    // Operating mode
    //  * Hybrid
    //  * StealthChop
    //  * SpreadCycle
    INDI::PropertySwitch OpModeSP {3};
    enum
    {
        OP_MODE_HYBRID,
        OP_MODE_STEALTHCHOP,
        OP_MODE_SPREADCYCLE
    };

    // Microsteps
    //  * 1, 2, 4, 8, 16, 32
    INDI::PropertySwitch StepModeSP {6};
    enum
    {
        STEP_MODE_1,
        STEP_MODE_2,
        STEP_MODE_4,
        STEP_MODE_8,
        STEP_MODE_16,
        STEP_MODE_32
    };

    // Full steps per rotation
    //  * Typically 200, but depends on motor
    INDI::PropertyNumber FullStepsPerRotationNP {1};

    // Move speed (in RPM)
    //  * 15 - 600
    INDI::PropertyNumber MoveSpeedNP {1};

    // Move current (in mA)
    //  * 100-1000mA
    INDI::PropertyNumber MoveCurrentNP {1};

    // Hold current (in % of move current)
    //  * 0-100%
    INDI::PropertyNumber HoldCurrentPctNP {1};


    INDI::PropertySwitch AutoHomeSP {1};

    // Homing speed (in RPM)
    //  * 15 - 600
    INDI::PropertyNumber HomingSpeedNP {1};

    // Homing current (in mA)
    //  * 100-1000mA
    INDI::PropertyNumber HomingCurrentNP {1};

    // Homing SGTHRS
    //  * 0-255
    INDI::PropertyNumber HomingSGTHRSNP {1};


    //  * temperature probe OK
    INDI::PropertyLight TemperatureFlagsLP {3};
    enum
    {
        TEMPERATURE_FLAGS_VALID,
        TEMPERATURE_FLAGS_NO_DEV,
        TEMPERATURE_FLAGS_COMMS_ERR
    };

    // Temperature (in celsius)
    INDI::PropertyNumber TemperatureNP {1};
};
