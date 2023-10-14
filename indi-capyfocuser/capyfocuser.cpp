#include "indicom.h"
#include "capyfocuser.h"
#include "config.h"
#include "connectionplugins/connectionserial.h"

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

static std::unique_ptr<CapyFocuser> capyFocuser(new CapyFocuser());

static const char *MOTOR_TAB  = "Motor";

CapyFocuser::CapyFocuser()
{
    configRead = false;

    setVersion(CAPYFOCUSER_VERSION_MAJOR, CAPYFOCUSER_VERSION_MINOR);

    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT |
        FOCUSER_CAN_REVERSE | FOCUSER_CAN_SYNC);
}

CapyFocuser::~CapyFocuser()
{
}

const char *CapyFocuser::getDefaultName()
{
    return "CapyFocuser";
}

bool
CapyFocuser::initProperties()
{
    INDI::Focuser::initProperties();

    serialConnection->setWordSize(8);
    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);


    FirmwareRevisionTP[0].fill("MI_FW_VERSION", "Firmware Version", nullptr);
    FirmwareRevisionTP.fill(getDeviceName(), "FOCUSER_FIRMWARE", "Firmware", CONNECTION_TAB, IP_RO, 0 ,IPS_IDLE);

    DriverStatusTP[0].fill("DRIVER_STATUS", "Driver Status", nullptr);
    DriverStatusTP.fill(getDeviceName(), "DRIVER", "Driver", MAIN_CONTROL_TAB, IP_RO, 0 ,IPS_IDLE);

    CoilEnergizedSP[COIL_ENERGIZED_ON].fill("COIL_ENERGIZED_ON", "On", ISS_OFF);
    CoilEnergizedSP[COIL_ENERGIZED_OFF].fill("COIL_ENERGIZED_OFF", "Off", ISS_ON);
    CoilEnergizedSP.fill(getDeviceName(), "COIL_ENERGIZED", "Motor power", MAIN_CONTROL_TAB,
                       IP_RW, ISR_1OFMANY, 0, IPS_IDLE);


    OpModeSP[OP_MODE_HYBRID].fill("OP_MODE_HYBRID", "Hybrid", ISS_ON);
    OpModeSP[OP_MODE_STEALTHCHOP].fill("OP_MODE_STEALTHCHOP", "StealthChop", ISS_OFF);
    OpModeSP[OP_MODE_SPREADCYCLE].fill("OP_MODE_SPREADCYCLE", "SpreadCycle", ISS_OFF);
    OpModeSP.fill(getDeviceName(), "OP_MODE", "Driver mode", MOTOR_TAB,
                       IP_RW, ISR_1OFMANY, 0, IPS_IDLE);


    StepModeSP[STEP_MODE_1].fill("STEP_MODE_1", "Full steps", ISS_OFF);
    StepModeSP[STEP_MODE_2].fill("STEP_MODE_2", "1/2 steps", ISS_OFF);
    StepModeSP[STEP_MODE_4].fill("STEP_MODE_4", "1/4 steps", ISS_OFF);
    StepModeSP[STEP_MODE_8].fill("STEP_MODE_8", "1/8 steps", ISS_OFF);
    StepModeSP[STEP_MODE_16].fill("STEP_MODE_16", "1/16 steps", ISS_OFF);
    StepModeSP[STEP_MODE_32].fill("STEP_MODE_32", "1/32 steps", ISS_OFF);
    StepModeSP.fill(getDeviceName(), "STEP_MODE", "Step mode", MOTOR_TAB,
                       IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    FullStepsPerRotationNP[0].fill("FULL_STEPS_PER_ROTATION", "fsteps/rot", "%.0f", 1, 1000, 1, 200);
    FullStepsPerRotationNP.fill(getDeviceName(), "FULL_STEPS_PER_ROTATION", "Full steps per rotation", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    MoveSpeedNP[0].fill("MOVE_SPEED", "rpm", "%.0f", 15, 600, 1, 60);
    MoveSpeedNP.fill(getDeviceName(), "MOVE_SPEED", "Move speed", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    MoveCurrentNP[0].fill("MOVE_CURRENT", "mA", "%.0f", 100, 1000, 1, 500);
    MoveCurrentNP.fill(getDeviceName(), "MOVE_CURRENT", "Move current", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    HoldCurrentPctNP[0].fill("HOLD_CURRENT_PCT", "% of move current", "%.0f", 1, 100, 1, 30);
    HoldCurrentPctNP.fill(getDeviceName(), "HOLD_CURRENT_PCT", "Hold current", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    AutoHomeSP[0].fill("AUTO_HOME", "Start homing", ISS_OFF);
    AutoHomeSP.fill(getDeviceName(), "AUTO_HOME", "Homing", MAIN_CONTROL_TAB,
                    IP_RW, ISR_ATMOST1, 0, IPS_IDLE);


    HomingSpeedNP[0].fill("HOMING_SPEED", "rpm", "%.0f", 15, 600, 1, 30);
    HomingSpeedNP.fill(getDeviceName(), "HOMING_SPEED", "Homing speed", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    HomingCurrentNP[0].fill("HOMING_CURRENT", "mA", "%.0f", 100, 1000, 1, 300);
    HomingCurrentNP.fill(getDeviceName(), "HOMING_CURRENT", "Homing current", MOTOR_TAB, IP_RW, 0, IPS_IDLE);

    HomingSGTHRSNP[0].fill("HOMING_SGTHRS", "(sgthrs)", "%.0f", 5, 255, 1, 15);
    HomingSGTHRSNP.fill(getDeviceName(), "HOMING_SGTHRS", "Homing sensitivty", MOTOR_TAB, IP_RW, 0, IPS_IDLE);


    TemperatureFlagsLP[TEMPERATURE_FLAGS_VALID].fill("OK", "", IPS_IDLE);
    TemperatureFlagsLP[TEMPERATURE_FLAGS_NO_DEV].fill("No device", "", IPS_IDLE);
    TemperatureFlagsLP[TEMPERATURE_FLAGS_COMMS_ERR].fill("Comms error", "", IPS_IDLE);
    TemperatureFlagsLP.fill(getDeviceName(), "TEMPERATURE_FLAGS", "Temperature sensor", MAIN_CONTROL_TAB, IPS_IDLE);

    TemperatureNP[0].fill("TEMPERATURE", "Celsius", "%.2f", -50, 70., 0., 0.);
    TemperatureNP.fill(getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    addAuxControls();

    setDefaultPollingPeriod(750);

    return true;
}

bool
CapyFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();
    if (isConnected()) {
        defineProperty(FirmwareRevisionTP);
        defineProperty(DriverStatusTP);
        defineProperty(CoilEnergizedSP);
        defineProperty(OpModeSP);
        defineProperty(StepModeSP);
        defineProperty(FullStepsPerRotationNP);
        defineProperty(MoveSpeedNP);
        defineProperty(MoveCurrentNP);
        defineProperty(HoldCurrentPctNP);
        defineProperty(AutoHomeSP);
        defineProperty(HomingSpeedNP);
        defineProperty(HomingCurrentNP);
        defineProperty(HomingSGTHRSNP);
        defineProperty(TemperatureFlagsLP);
        defineProperty(TemperatureNP);

        return fetchConfig() && fetchSeq();
    } else {
        deleteProperty(FirmwareRevisionTP);
        deleteProperty(DriverStatusTP);
        deleteProperty(CoilEnergizedSP);
        deleteProperty(OpModeSP);
        deleteProperty(StepModeSP);
        deleteProperty(FullStepsPerRotationNP);
        deleteProperty(MoveSpeedNP);
        deleteProperty(MoveCurrentNP);
        deleteProperty(HoldCurrentPctNP);
        deleteProperty(AutoHomeSP);
        deleteProperty(HomingSpeedNP);
        deleteProperty(HomingCurrentNP);
        deleteProperty(HomingSGTHRSNP);
        deleteProperty(TemperatureFlagsLP);
        deleteProperty(TemperatureNP);

        configRead = false;

        return true;
    }
}

bool
CapyFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    uint32_t prevVal;
    uint32_t val;

    if (dev == nullptr || (strcmp(dev, getDeviceName()) != 0))
        return INDI::Focuser::ISNewNumber(dev, name, values, names, n);

    if (FullStepsPerRotationNP.isNameMatch(name)) {
        prevVal = static_cast<uint32_t>(FullStepsPerRotationNP[0].getValue());
        FullStepsPerRotationNP.update(values, names, n);
        val = static_cast<uint32_t>(FullStepsPerRotationNP[0].getValue());

        if (!wrcfg(0x20, &val, sizeof(val))) {
            FullStepsPerRotationNP[0].setValue(prevVal);
            FullStepsPerRotationNP.setState(IPS_ALERT);
        } else {
            FullStepsPerRotationNP.setState(IPS_OK);
        }
        FullStepsPerRotationNP.apply();
        return true;
    } else if (MoveSpeedNP.isNameMatch(name)) {
        prevVal = static_cast<uint32_t>(MoveSpeedNP[0].getValue());
        MoveSpeedNP.update(values, names, n);
        val = static_cast<uint32_t>(MoveSpeedNP[0].getValue());

        if (!wrcfg(0x21, &val, sizeof(val))) {
            MoveSpeedNP[0].setValue(prevVal);
            MoveSpeedNP.setState(IPS_ALERT);
        } else {
            MoveSpeedNP.setState(IPS_OK);
        }
        MoveSpeedNP.apply();
        return true;
    } else if (MoveCurrentNP.isNameMatch(name)) {
        prevVal = static_cast<uint32_t>(MoveCurrentNP[0].getValue());
        MoveCurrentNP.update(values, names, n);
        val = static_cast<uint32_t>(MoveCurrentNP[0].getValue());

        if (!wrcfg(0x22, &val, sizeof(val))) {
            MoveCurrentNP[0].setValue(prevVal);
            MoveCurrentNP.setState(IPS_ALERT);
        } else {
            MoveCurrentNP.setState(IPS_OK);
        }
        MoveCurrentNP.apply();
        return true;
    } else if (HoldCurrentPctNP.isNameMatch(name)) {
        prevVal = static_cast<uint32_t>(HoldCurrentPctNP[0].getValue());
        HoldCurrentPctNP.update(values, names, n);
        val = static_cast<uint32_t>(HoldCurrentPctNP[0].getValue());

        if (!wrcfg(0x23, &val, sizeof(val))) {
            HoldCurrentPctNP[0].setValue(prevVal);
            HoldCurrentPctNP.setState(IPS_ALERT);
        } else {
            HoldCurrentPctNP.setState(IPS_OK);
        }
        HoldCurrentPctNP.apply();
        return true;
    } else if (HomingSpeedNP.isNameMatch(name)) {
        prevVal = static_cast<uint32_t>(HomingSpeedNP[0].getValue());
        HomingSpeedNP.update(values, names, n);
        val = static_cast<uint32_t>(HomingSpeedNP[0].getValue());

        if (!wrcfg(0x31, &val, sizeof(val))) {
            HomingSpeedNP[0].setValue(prevVal);
            HomingSpeedNP.setState(IPS_ALERT);
        } else {
            HomingSpeedNP.setState(IPS_OK);
        }
        HomingSpeedNP.apply();
        return true;
    } else if (HomingCurrentNP.isNameMatch(name)) {
        prevVal = static_cast<uint32_t>(HomingCurrentNP[0].getValue());
        HomingCurrentNP.update(values, names, n);
        val = static_cast<uint32_t>(HomingCurrentNP[0].getValue());

        if (!wrcfg(0x32, &val, sizeof(val))) {
            HomingCurrentNP[0].setValue(prevVal);
            HomingCurrentNP.setState(IPS_ALERT);
        } else {
            HomingCurrentNP.setState(IPS_OK);
        }
        HomingCurrentNP.apply();
        return true;
    } else if (HomingSGTHRSNP.isNameMatch(name)) {
        prevVal = static_cast<uint32_t>(HomingSGTHRSNP[0].getValue());
        HomingSGTHRSNP.update(values, names, n);
        val = static_cast<uint32_t>(HomingSGTHRSNP[0].getValue());

        if (!wrcfg(0x34, &val, sizeof(val))) {
            HomingSGTHRSNP[0].setValue(prevVal);
            HomingSGTHRSNP.setState(IPS_ALERT);
        } else {
            HomingSGTHRSNP.setState(IPS_OK);
        }
        HomingSGTHRSNP.apply();
        return true;
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool
CapyFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    uint32_t val;

    if (dev == nullptr || (strcmp(dev, getDeviceName()) != 0))
        return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);

    if (CoilEnergizedSP.isNameMatch(name)) {
        auto prevIdx = CoilEnergizedSP.findOnSwitchIndex();
        CoilEnergizedSP.update(states, names, n);
        auto currIdx = CoilEnergizedSP.findOnSwitchIndex();

        if (currIdx == COIL_ENERGIZED_ON) {
            val = 1;
        } else {
            val = 0;
        }

        if (!wrreg(0x00, &val, sizeof(val))) {
            CoilEnergizedSP[prevIdx].setState(ISS_ON);
            CoilEnergizedSP.setState(IPS_ALERT);
        }

        CoilEnergizedSP.apply();
        return true;
    } else if (OpModeSP.isNameMatch(name)) {
        auto prevIdx = OpModeSP.findOnSwitchIndex();
        OpModeSP.update(states, names, n);
        auto currIdx = OpModeSP.findOnSwitchIndex();

        val =
            (currIdx == OP_MODE_STEALTHCHOP) ? 0x00 :
            (currIdx == OP_MODE_SPREADCYCLE) ? 0x03 : 0x02;

        if (!wrcfg(0x10, &val, sizeof(val))) {
            OpModeSP[prevIdx].setState(ISS_ON);
            OpModeSP.setState(IPS_ALERT);
        }

        OpModeSP.apply();
        return true;
    } else if (StepModeSP.isNameMatch(name)) {
        auto prevIdx = StepModeSP.findOnSwitchIndex();
        StepModeSP.update(states, names, n);
        auto currIdx = StepModeSP.findOnSwitchIndex();

        val =
            (currIdx == STEP_MODE_2) ? 2 :
            (currIdx == STEP_MODE_4) ? 4 :
            (currIdx == STEP_MODE_8) ? 8 :
            (currIdx == STEP_MODE_16) ? 16 :
            (currIdx == STEP_MODE_32) ? 32 : 1;

        if (!wrcfg(0x11, &val, sizeof(val))) {
            StepModeSP[prevIdx].setState(ISS_ON);
            StepModeSP.setState(IPS_ALERT);
        }

        StepModeSP.apply();
        return true;
    } else if (AutoHomeSP.isNameMatch(name)) {
        AutoHomeSP.update(states, names, n);

        AutoHomeSP.reset();

        if (!autohome()) {
            AutoHomeSP.setState(IPS_ALERT);
        } else {
            AutoHomeSP.setState(IPS_BUSY);
        }

        AutoHomeSP.apply();
        return true;
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool CapyFocuser::Handshake()
{
    if (ack()) {
        LOG_INFO("CapyFocuser is online.");
        return true;
    }

    LOG_INFO("Error connecting to CapyFocuser.");
    return false;
}

void
CapyFocuser::TimerHit()
{
    if (!isConnected() || !configRead) {
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    if (!fetchSeq()) {
        LOG_WARN("Unable to read focuser status....");
    }

    SetTimer(getCurrentPollingPeriod());
}

bool
CapyFocuser::ack()
{
    struct __attribute__((__packed__)) {
        uint16_t addr;
        uint8_t type;
        uint16_t firmware_version;
    } data;

    if (!probe((uint8_t *)&data, sizeof(data))) {
        LOG_DEBUG("ack: probe error");
        return false;
    }

    if (data.type != 0x41) {
        LOGF_DEBUG("ack: type mismatch: %x (expected: %x)", data.type, 0x41);
        return false;
    }

    return true;
}

bool
CapyFocuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);

    OpModeSP.save(fp);
    StepModeSP.save(fp);
    FullStepsPerRotationNP.save(fp);
    MoveSpeedNP.save(fp);
    MoveCurrentNP.save(fp);
    HoldCurrentPctNP.save(fp);
    HomingSpeedNP.save(fp);
    HomingCurrentNP.save(fp);
    HomingSGTHRSNP.save(fp);

    return true;
}

/**
 * \brief MoveFocuser the focuser to an absolute position.
 * \param ticks The new position of the focuser.
 * \return Return IPS_OK if motion is completed and focuser reached requested position. Return
 * IPS_BUSY if focuser started motion to requested position and is in progress.
 * Return IPS_ALERT if there is an error.
 */
IPState
CapyFocuser::MoveAbsFocuser(uint32_t targetTicks)
{
    if (moveAbs(targetTicks)) {
        return IPS_BUSY;
    } else {
        return IPS_ALERT;
    }
}

/**
 * \brief MoveFocuser the focuser to an relative position.
 * \param dir Direction of focuser, either FOCUS_INWARD or FOCUS_OUTWARD.
 * \param ticks The relative ticks to move.
 * \return Return IPS_OK if motion is completed and focuser reached requested position. Return
 * IPS_BUSY if focuser started motion to requested position and is in progress.
 * Return IPS_ALERT if there is an error.
 */
IPState
CapyFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    uint8_t hwdir = (dir == FOCUS_INWARD) ? 0 : 1;

    if (moveRel(hwdir, ticks)) {
        return IPS_BUSY;
    } else {
        return IPS_ALERT;
    }
}

/**
 * @brief ReverseFocuser Reverse focuser motion direction
 * @param enabled If true, normal default focuser motion is reversed. If false, the direction is set to the default focuser motion.
 * @return True if successful, false otherwise.
 */
bool
CapyFocuser::ReverseFocuser(bool enabled)
{
    uint32_t val = enabled ? 1 : 0;

    return wrcfg(0x24, &val, sizeof(val));
}

/**
 * @brief SyncFocuser Set current position to ticks without moving the focuser.
 * @param ticks Desired new sync position.
 * @return True if successful, false otherwise.
 */
bool
CapyFocuser::SyncFocuser(uint32_t ticks)
{
    return wrreg(0x10, &ticks, sizeof(ticks));
}

/**
 * @brief SetFocuserMaxPosition Set Focuser Maximum position limit in the hardware.
 * @param ticks maximum steps permitted
 * @return True if successful, false otherwise.
 * @note If setting maximum position limit in the hardware is not available or not supported, do not override this function as the default
 * implementation will always return true.
 */
bool
CapyFocuser::SetFocuserMaxPosition(uint32_t ticks)
{
    if (!wrcfg(0x00, &ticks, sizeof(ticks)))
        return false;

    return fetchMaxPos();
}

/**
 * @brief AbortFocuser all focus motion
 * @return True if abort is successful, false otherwise.
 */
bool
CapyFocuser::AbortFocuser()
{
    return stop();
}

bool
CapyFocuser::fetchSeq()
{
    struct __attribute__((__packed__)) {
        struct __attribute__((__packed__)) {
            uint8_t flags;
            uint16_t reading;
        } temperature;

        struct __attribute__((__packed__)) {
            uint8_t flags;
            uint8_t state;
            uint8_t curr_dir;
            uint32_t curr_pos;
        } stepper;
    } data;

    if (!rdseq(0x00, &data, sizeof(data)))
        return false;

    if (data.temperature.flags == 0x01) {
        double val = (double)data.temperature.reading / 16.0;
        TemperatureNP[0].setValue(val);
        TemperatureNP.setState(IPS_OK);
        TemperatureNP.apply();
    } else {
        TemperatureNP[0].setValue(-273.15); /* Zero kelvin */
        TemperatureNP.setState(IPS_ALERT);
        TemperatureNP.apply();
    }

    TemperatureFlagsLP[TEMPERATURE_FLAGS_VALID].setState((data.temperature.flags & 0x01) ? IPS_OK : IPS_IDLE);
    TemperatureFlagsLP[TEMPERATURE_FLAGS_NO_DEV].setState((data.temperature.flags & 0x02) ? IPS_ALERT : IPS_IDLE);
    TemperatureFlagsLP[TEMPERATURE_FLAGS_COMMS_ERR].setState((data.temperature.flags & 0x04) ? IPS_ALERT : IPS_IDLE);
    TemperatureFlagsLP.apply();

    CoilEnergizedSP.reset();
    if (data.stepper.flags & (1U << 1)) { /* Driver enabled? */
        CoilEnergizedSP[COIL_ENERGIZED_ON].setState(ISS_ON);
        CoilEnergizedSP.setState(IPS_OK);
    } else {
        CoilEnergizedSP[COIL_ENERGIZED_OFF].setState(ISS_ON);
        CoilEnergizedSP.setState(IPS_IDLE);
    }
    CoilEnergizedSP.apply();

    switch (data.stepper.state) {
    case 0: /* IDLE */
        DriverStatusTP[0].setText("Idle");
        DriverStatusTP.setState(IPS_IDLE);

        if (FocusAbsPosNP.s != IPS_OK) {
            FocusAbsPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
        }

        if (FocusRelPosNP.s != IPS_OK) {
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusRelPosNP, nullptr);
        }

        if (AutoHomeSP.getState() == IPS_BUSY) {
            if (!fetchMaxPos())
                return false;
        }

        if (AutoHomeSP.getState() != IPS_IDLE) {
            AutoHomeSP.setState(IPS_IDLE);
            AutoHomeSP.apply();
        }
        break;
    case 1: /* MOVING */
        DriverStatusTP[0].setText("Moving");
        DriverStatusTP.setState(IPS_BUSY);

        FocusAbsPosNP.s = IPS_BUSY;
        FocusRelPosNP.s = IPS_BUSY;
        IDSetNumber(&FocusAbsPosNP, nullptr);
        IDSetNumber(&FocusRelPosNP, nullptr);
        break;
    case 2: /* HOMING */
        DriverStatusTP[0].setText("Homing");
        DriverStatusTP.setState(IPS_BUSY);

        FocusAbsPosNP.s = IPS_BUSY;
        FocusRelPosNP.s = IPS_BUSY;
        IDSetNumber(&FocusAbsPosNP, nullptr);
        IDSetNumber(&FocusRelPosNP, nullptr);

        AutoHomeSP.setState(IPS_BUSY);
        AutoHomeSP.apply();
        break;
    case 3: /* FAULT */
        DriverStatusTP[0].setText("Fault");
        DriverStatusTP.setState(IPS_ALERT);

        FocusAbsPosNP.s = IPS_ALERT;
        FocusRelPosNP.s = IPS_ALERT;
        IDSetNumber(&FocusAbsPosNP, nullptr);
        IDSetNumber(&FocusRelPosNP, nullptr);

        AutoHomeSP.setState(IPS_ALERT);
        AutoHomeSP.apply();
        break;
    default:
        DriverStatusTP[0].setText("???");
        DriverStatusTP.setState(IPS_ALERT);
        break;
    }

    DriverStatusTP.apply();

    FocusAbsPosN[0].value = data.stepper.curr_pos;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return true;
}

bool
CapyFocuser::fetchMaxPos()
{
    uint32_t val;

    /* Max pos */
    if (!rdcfg(0x00, &val, sizeof(val)))
        return false;

    FocusMaxPosNP.s = IPS_OK;
    FocusMaxPosN[0].value = val;
    IDSetNumber(&FocusMaxPosNP, nullptr);

    return true;
}

bool
CapyFocuser::fetchConfig()
{
    uint32_t val;

    if (!fetchMaxPos())
        return false;

    /* Mode */
    if (!rdcfg(0x10, &val, sizeof(val)))
        return false;

    OpModeSP.reset();
    switch (val) {
    case 0x00: /* StealthChop */
        OpModeSP[OP_MODE_STEALTHCHOP].setState(ISS_ON);
        break;
    case 0x02: /* Hybrid */
        OpModeSP[OP_MODE_HYBRID].setState(ISS_ON);
        break;
    case 0x03: /* SpreadCycle */
        OpModeSP[OP_MODE_SPREADCYCLE].setState(ISS_ON);
        break;
    default:
        return false;
    }
    OpModeSP.setState(IPS_OK);
    OpModeSP.apply();

    /* Microsteps */
    if (!rdcfg(0x11, &val, sizeof(val)))
        return false;

    StepModeSP.reset();
    switch (val) {
    case 1:
        StepModeSP[STEP_MODE_1].setState(ISS_ON);
        break;
    case 2:
        StepModeSP[STEP_MODE_2].setState(ISS_ON);
        break;
    case 4:
        StepModeSP[STEP_MODE_4].setState(ISS_ON);
        break;
    case 8:
        StepModeSP[STEP_MODE_8].setState(ISS_ON);
        break;
    case 16:
        StepModeSP[STEP_MODE_16].setState(ISS_ON);
        break;
    case 32:
        StepModeSP[STEP_MODE_32].setState(ISS_ON);
        break;
    default:
        return false;
    }
    StepModeSP.setState(IPS_OK);
    StepModeSP.apply();

    /* Full steps per rotation */
    if (!rdcfg(0x20, &val, sizeof(val)))
        return false;

    FullStepsPerRotationNP[0].setValue(val);
    FullStepsPerRotationNP.setState(IPS_OK);
    FullStepsPerRotationNP.apply();

    /* Move speed (RPM) */
    if (!rdcfg(0x21, &val, sizeof(val)))
        return false;

    MoveSpeedNP[0].setValue(val);
    MoveSpeedNP.setState(IPS_OK);
    MoveSpeedNP.apply();

    /* Move current (mA) */
    if (!rdcfg(0x22, &val, sizeof(val)))
        return false;

    MoveCurrentNP[0].setValue(val);
    MoveCurrentNP.setState(IPS_OK);
    MoveCurrentNP.apply();

    /* Hold current (% of move current) */
    if (!rdcfg(0x23, &val, sizeof(val)))
        return false;

    HoldCurrentPctNP[0].setValue(val);
    HoldCurrentPctNP.setState(IPS_OK);
    HoldCurrentPctNP.apply();

    /* Reverse motion (boolean) */
    if (!rdcfg(0x24, &val, sizeof(val)))
        return false;

    IUResetSwitch(&FocusReverseSP);
    FocusReverseS[INDI_ENABLED].s = val ? ISS_ON : ISS_OFF;
    FocusReverseS[INDI_DISABLED].s = val ? ISS_OFF : ISS_ON;
    FocusReverseSP.s = IPS_OK;
    IDSetSwitch(&FocusReverseSP, nullptr);

    /* Homing speed (RPM) */
    if (!rdcfg(0x31, &val, sizeof(val)))
        return false;

    HomingSpeedNP[0].setValue(val);
    HomingSpeedNP.setState(IPS_OK);
    HomingSpeedNP.apply();

    /* Homing current (mA) */
    if (!rdcfg(0x32, &val, sizeof(val)))
        return false;

    HomingCurrentNP[0].setValue(val);
    HomingCurrentNP.setState(IPS_OK);
    HomingCurrentNP.apply();

    /* Homing threshold (SGTHRS) */
    if (!rdcfg(0x34, &val, sizeof(val)))
        return false;

    HomingSGTHRSNP[0].setValue(val);
    HomingSGTHRSNP.setState(IPS_OK);
    HomingSGTHRSNP.apply();

    configRead = true;

    return true;
}

bool
CapyFocuser::wrcfg(uint16_t reg, const void *data, size_t sz)
{
    return xceive(0x10, &reg, (const uint8_t *)data, sz+2, NULL, 0, 0);
}

bool
CapyFocuser::rdcfg(uint16_t reg, void *data, size_t sz)
{
    return xceive(0x11, &reg, NULL, 0, (uint8_t *)data, sz, 2);
}

bool
CapyFocuser::wrreg(uint16_t reg, const void *data, size_t sz)
{
    return xceive(0x17, &reg, (const uint8_t *)data, sz, NULL, 0, 0);
}

bool
CapyFocuser::rdreg(uint16_t reg, void *data, size_t sz)
{
    return xceive(0x18, &reg, NULL, 0, (uint8_t *)data, sz, 2);
}

bool
CapyFocuser::rdseq(uint16_t seq, void *data, size_t sz)
{
    return xceive(0x19, &seq, NULL, 0, (uint8_t *)data, sz, 0);
}

bool
CapyFocuser::probe(void *data, size_t sz)
{
    return xceive(0x03, NULL, NULL, 0, (uint8_t *)data, sz, 0);
}

bool
CapyFocuser::stop()
{
    return xceive(0x21, NULL, NULL, 0, NULL, 0, 0);
}

bool
CapyFocuser::autohome()
{
    return xceive(0x22, NULL, NULL, 0, NULL, 0, 0);
}

bool
CapyFocuser::moveRel(uint8_t dir, uint32_t nsteps)
{
    uint8_t buf[5];

    buf[0] = dir;
    memcpy(&buf[1], &nsteps, sizeof(nsteps));

    return xceive(0x20, NULL, (const uint8_t *)buf, sizeof(buf), NULL, 0, 0);
}

bool
CapyFocuser::moveAbs(uint32_t pos)
{
    return wrreg(0x08, &pos, sizeof(pos));
}

bool
CapyFocuser::xceive(uint8_t opcode, const uint16_t *regp, const uint8_t *wrmsg, size_t wrsz, uint8_t *rdmsg, size_t rdsz, size_t rdoff)
{
    static uint8_t msgid_ctr = 0;
    uint8_t msgid = ++msgid_ctr;
    uint8_t buf[48];
    size_t extrasz = (regp != NULL) ? 6 : 4;

    if (wrsz > (sizeof(buf) - extrasz)) {
        LOGF_DEBUG("xceive: invalid wrsz: %zu", wrsz);
        return false;
    }

    if (rdsz > (sizeof(buf) - 4 - rdoff)) {
        LOGF_DEBUG("xceive: invalid rdsz: %zu", rdsz);
        return false;
    }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = msgid;
    buf[3] = opcode;

    if (regp != NULL) {
        memcpy(&buf[4], regp, sizeof(*regp));
    }

    if (wrsz > 0)
        memcpy(&buf[extrasz], wrmsg, wrsz);

    if (!xceive_ll(buf, wrsz+extrasz, buf, sizeof(buf))) {
        LOG_DEBUG("xceive: xceive_ll call failed!");
        return false;
    }

    if (buf[2] != msgid) { /* not the same msgid */
        LOGF_DEBUG("xceive: msgid mismatch: %x (expected: %x)", buf[2], msgid);
        return false;
    }

    if (!(buf[3] & (1U << 7)) || /* not a response opcode */
        ((buf[3] & 0x3F) != (opcode & 0x3F))) {/* not the same opcode */
        LOGF_DEBUG("xceive: opcode mismatch: %x (sent: %x)", buf[3], opcode);
        return false;
    }

    if (buf[3] & (1U << 6)) { /* error response opcode */
        LOGF_DEBUG("xceive: error response opcode: %x", buf[3]);
        return false;
    }

    if (rdmsg)
        memcpy(rdmsg, &buf[4+rdoff], rdsz);

    return true;
}

#define ICN_MAGIC1 0xab
#define ICN_MAGIC2 0xba

#define ICN_MAX_PKT_SIZE 48

bool
CapyFocuser::xceive_ll(const uint8_t *wrmsg, size_t wrsz, uint8_t *rdmsg, size_t rdsz)
{
    int r;
    int nbwr;
    int nbrd;

    struct __attribute__((__packed__)) {
        uint8_t sync[2];
        uint8_t flitsz;
    } flit_hdr;

    uint8_t crcbuf[2];
    uint16_t crc, crc_rcvd;

    tcflush(PortFD, TCIOFLUSH);

    if (wrsz > (ICN_MAX_PKT_SIZE - 5)) {
        LOGF_DEBUG("xceive_ll: illegal wrsz, %zu (max: %zu)", wrsz, (ICN_MAX_PKT_SIZE - 5));
        return false;
    }

    flit_hdr.sync[0] = ICN_MAGIC1;
    flit_hdr.sync[1] = ICN_MAGIC2;
    flit_hdr.flitsz = (uint8_t)wrsz;

    crc = crc16_init();
    crc = crc16_update(crc, (const uint8_t *)&flit_hdr, sizeof(flit_hdr));
    crc = crc16_update(crc, wrmsg, wrsz);

    crcbuf[0] = (uint8_t)((crc >> 8) & 0xff);
    crcbuf[1] = (uint8_t)(crc & 0xff);

    if ((r = tty_write(PortFD, (const char *)&flit_hdr, (int)sizeof(flit_hdr), &nbwr)) != TTY_OK) {
        LOG_DEBUG("xceive_ll: tty_write flit_hdr failed!");
        return false;
    }

    if ((r = tty_write(PortFD, (const char *)wrmsg, (int)wrsz, &nbwr)) != TTY_OK) {
        LOG_DEBUG("xceive_ll: tty_write wrmsg failed!");
        return false;
    }

    if ((r = tty_write(PortFD, (const char *)crcbuf, (int)sizeof(crcbuf), &nbwr)) != TTY_OK) {
        LOG_DEBUG("xceive_ll: tty_write crcbuf failed!");
        return false;
    }

    tcdrain(PortFD);

    if ((r = tty_read_expanded(PortFD, (char *)&flit_hdr, (int)sizeof(flit_hdr), 1, 500, &nbrd)) != TTY_OK) {
        LOG_DEBUG("xceive_ll: tty_read flit_hdr failed!");
        return false;
    }

    if ((size_t)nbrd < sizeof(flit_hdr)) {
        LOGF_DEBUG("xceive_ll: tty_read flit_hdr returned insufficient bytes: %zd (expected: %zd)", (size_t)nbrd, sizeof(flit_hdr));
        return false;
    }

    if ((flit_hdr.sync[0] != ICN_MAGIC1) ||
        (flit_hdr.sync[1] != ICN_MAGIC2) ||
        (flit_hdr.flitsz > rdsz)) {
        LOGF_DEBUG("xceive_ll: flit header error: [%x, %x, %x]", flit_hdr.sync[0], flit_hdr.sync[1], flit_hdr.flitsz);
        return false;
    }

    if ((r = tty_read_expanded(PortFD, (char *)rdmsg, (int)flit_hdr.flitsz, 1, 100, &nbrd)) != TTY_OK) {
        LOG_DEBUG("xceive_ll: tty_read rdmsg failed!");
        return false;
    }

    if (nbrd != (int)flit_hdr.flitsz) {
        LOGF_DEBUG("xceive_ll: tty_read rdmsg returned insufficient bytes: %zd (expected: %zd)", (size_t)nbrd, (size_t)flit_hdr.flitsz);
        return false;
    }

    if ((r = tty_read_expanded(PortFD, (char *)crcbuf, (int)sizeof(crcbuf), 1, 100, &nbrd)) != TTY_OK) {
        LOG_DEBUG("xceive_ll: tty_read crcbuf failed!");
        return false;
    }

    if (nbrd != (int)sizeof(crcbuf)) {
        LOGF_DEBUG("xceive_ll: tty_read crcbuf returned insufficient bytes: %zd (expected: %zd)", (size_t)nbrd, sizeof(crcbuf));
        return false;
    }

    crc = crc16_init();
    crc = crc16_update(crc, (const uint8_t *)&flit_hdr, sizeof(flit_hdr));
    crc = crc16_update(crc, rdmsg, (size_t)flit_hdr.flitsz);

    crc_rcvd = ((uint16_t)crcbuf[0] << 8) | crcbuf[1];

    if (crc != crc_rcvd) {
        LOGF_DEBUG("xceive_ll: crc mismatch: %x (expected: %x)", crc_rcvd, crc);
        return false;
    }

    return true;
}

uint16_t
CapyFocuser::crc16_init(void)
{
    return 0xFFFF;
}

uint16_t
CapyFocuser::crc16_update(uint16_t crc, const uint8_t *data_p, size_t len)
{
    uint8_t x;
    uint8_t b;

    while (len--) {
        b = *data_p++;

        x = crc >> 8 ^ b;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }

    return crc;
}
