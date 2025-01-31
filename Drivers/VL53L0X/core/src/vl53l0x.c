
#include "vl53l0x_api.h"
#include "vl53l0x_api_core.h"
#include "vl53l0x.h"

#define REG_IDENTIFICATION_MODEL_ID (0xC0)
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define REG_RESULT_INTERRUPT_STATUS (0x13)
#define REG_SYSRANGE_START (0x00)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define REG_RESULT_RANGE_STATUS (0x14)
#define REG_SLAVE_DEVICE_ADDRESS (0x8A)

#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define VL53L0X_DEFAULT_ADDRESS (0x29)

/* There are two types of SPAD: aperture and non-aperture. My understanding
 * is that aperture ones let it less light (they have a smaller opening), similar
 * to how you can change the aperture on a digital camera. Only 1/4 th of the
 * SPADs are of type non-aperture. */
#define SPAD_TYPE_APERTURE (0x01)
/* The total SPAD array is 16x16, but we can only activate a quadrant spanning 44 SPADs at
 * a time. In the ST api code they have (for some reason) selected 0xB4 (180) as a starting
 * point (lies in the middle and spans non-aperture (3rd) quadrant and aperture (4th) quadrant). */
#define SPAD_START_SELECT (0xB4)
/* The total SPAD map is 16x16, but we should only activate an area of 44 SPADs at a time. */
#define SPAD_MAX_COUNT (44)
/* The 44 SPADs are represented as 6 bytes where each bit represents a single SPAD.
 * 6x8 = 48, so the last four bits are unused. */
#define SPAD_MAP_ROW_COUNT (6)
#define SPAD_ROW_SIZE (8)
/* Since we start at 0xB4 (180), there are four quadrants (three aperture, one aperture),
 * and each quadrant contains 256 / 4 = 64 SPADs, and the third quadrant is non-aperture, the
 * offset to the aperture quadrant is (256 - 64 - 180) = 12 */
#define SPAD_APERTURE_START_INDEX (12)

typedef enum
{
    CALIBRATION_TYPE_VHV,
    CALIBRATION_TYPE_PHASE
} calibration_type_t;

static uint8_t stop_variable = 0;

/**
 * We can read the model id to confirm that the device is booted.
 * (There is no fresh_out_of_reset as on the vl6180x)
 */
static bool device_is_booted(VL53L0X_DEV Dev)
{
    uint8_t device_id = 0;
    if (VL53L0X_ERROR_NONE != VL53L0X_RdByte(Dev, REG_IDENTIFICATION_MODEL_ID, &device_id)) {
        return false;
    }
    vl53l0x_log("Device id %d", device_id);
    return device_id == VL53L0X_EXPECTED_DEVICE_ID;
}

/**
 * One time device initialization
 */
static bool data_init(VL53L0X_DEV Dev)
{
    VL53L0X_Error error = VL53L0X_ERROR_NONE;

    /* Set 2v8 mode */
    uint8_t vhv_config_scl_sda = 0;
    if (VL53L0X_ERROR_NONE != VL53L0X_RdByte(Dev, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &vhv_config_scl_sda)) {
        return false;
    }
    vhv_config_scl_sda |= 0x01;
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, vhv_config_scl_sda)) {
        return false;
    }

    /* Set I2C standard mode */
    error = VL53L0X_WrByte(Dev, 0x88, 0x00);

    error |= VL53L0X_WrByte(Dev, 0x80, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x00);
    /* It may be unnecessary to retrieve the stop variable for each sensor */
    error |= VL53L0X_RdByte(Dev, 0x91, &stop_variable);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x80, 0x00);

    vl53l0x_log("Data init done");

    return error == VL53L0X_ERROR_NONE;
}

/**
 * Wait for strobe value to be set. This is used when we read values
 * from NVM (non volatile memory).
 */
static bool read_strobe(VL53L0X_DEV Dev)
{
    VL53L0X_Error error = VL53L0X_ERROR_NONE;
    uint8_t strobe = 0;
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, 0x83, 0x00)) {
        return false;
    }
    do {
        error = VL53L0X_RdByte(Dev, 0x83, &strobe);
    } while (error == VL53L0X_ERROR_NONE && (strobe == 0));
    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, 0x83, 0x01)) {
        return false;
    }
    return true;
}

/**
 * Gets the spad count, spad type och "good" spad map stored by ST in NVM at
 * their production line.
 * .
 * According to the datasheet, ST runs a calibration (without cover glass) and
 * saves a "good" SPAD map to NVM (non volatile memory). The SPAD array has two
 * types of SPADs: aperture and non-aperture. By default, all of the
 * good SPADs are enabled, but we should only enable a subset of them to get
 * an optimized signal rate. We should also only enable either only aperture
 * or only non-aperture SPADs. The number of SPADs to enable and which type
 * are also saved during the calibration step at ST factory and can be retrieved
 * from NVM.
 */
static bool get_spad_info_from_nvm(VL53L0X_DEV Dev, uint8_t *spad_count, uint8_t *spad_type, uint8_t good_spad_map[6])
{
	VL53L0X_Error error = VL53L0X_ERROR_NONE;
    uint8_t tmp_data8 = 0;
    uint32_t tmp_data32 = 0;

    /* Setup to read from NVM */
    error  = VL53L0X_WrByte(Dev, 0x80, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x00);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x06);
    error |= VL53L0X_RdByte(Dev, 0x83, &tmp_data8);
    error |= VL53L0X_WrByte(Dev, 0x83, tmp_data8 | 0x04);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x07);
    error |= VL53L0X_WrByte(Dev, 0x81, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x80, 0x01);
    if (error != VL53L0X_ERROR_NONE) {
      return false;
    }

    /* Get the SPAD count and type */
    error |= VL53L0X_WrByte(Dev, 0x94, 0x6b);
    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }
    if (!read_strobe(Dev)) {
        return false;
    }
    error |= VL53L0X_RdDWord(Dev, 0x90, &tmp_data32);
    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }
    *spad_count = (tmp_data32 >> 8) & 0x7f;
    *spad_type = (tmp_data32 >> 15) & 0x01;

    vl53l0x_log("spad_count=%d, spad_type=%d", *spad_count, *spad_type);

    /* Restore after reading from NVM */
    error |= VL53L0X_WrByte(Dev, 0x81, 0x00);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x06);
    error |= VL53L0X_RdByte(Dev, 0x83, &tmp_data8);
    error |= VL53L0X_WrByte(Dev, 0x83, tmp_data8 & 0xfb);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x80, 0x00);

    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }

    /* When we haven't configured the SPAD map yet, the SPAD map register actually
     * contains the good SPAD map, so we can retrieve it straight from this register
     * instead of reading it from the NVM. */
    if (VL53L0X_ERROR_NONE != VL53L0X_ReadMulti(Dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, good_spad_map, 6)) {
        return false;
    }
    vl53l0x_log("spad map: %02x %02x %02x %02x %02x %02x",
        good_spad_map[0], good_spad_map[1], good_spad_map[2],
        good_spad_map[3], good_spad_map[4], good_spad_map[5]);

    vl53l0x_log("Get spad count done");
    return true;
}

/**
 * Sets the SPADs according to the value saved to NVM by ST during production. Assuming
 * similar conditions (e.g. no cover glass), this should give reasonable readings and we
 * can avoid running ref spad management (tedious code).
 */
static bool set_spads_from_nvm(VL53L0X_DEV Dev)
{
    uint8_t spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t good_spad_map[SPAD_MAP_ROW_COUNT] = { 0 };
    uint8_t spads_enabled_count = 0;
    uint8_t spads_to_enable_count = 0;
    uint8_t spad_type = 0;
    volatile uint32_t total_val = 0;

    if (!get_spad_info_from_nvm(Dev, &spads_to_enable_count, &spad_type, good_spad_map)) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        total_val += good_spad_map[i];
    }

    VL53L0X_Error error = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    error |= VL53L0X_WrByte(Dev, REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, REG_GLOBAL_CONFIG_REF_EN_START_SELECT, SPAD_START_SELECT);
    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }

    uint8_t offset = (spad_type == SPAD_TYPE_APERTURE) ? SPAD_APERTURE_START_INDEX : 0;

    /* Create a new SPAD array by selecting a subset of the SPADs suggested by the good SPAD map.
     * The subset should only have the number of type enabled as suggested by the reading from
     * the NVM (spads_to_enable_count and spad_type). */
    for (int row = 0; row < SPAD_MAP_ROW_COUNT; row++) {
        for (int column = 0; column < SPAD_ROW_SIZE; column++) {
            int index = (row * SPAD_ROW_SIZE) + column;
            if (index >= SPAD_MAX_COUNT) {
                return false;
            }
            if (spads_enabled_count == spads_to_enable_count) {
                /* We are done */
                break;
            }
            if (index < offset) {
                continue;
            }
            if ((good_spad_map[row] >> column) & 0x1) {
                spad_map[row] |= (1 << column);
                spads_enabled_count++;
            }
        }
        if (spads_enabled_count == spads_to_enable_count) {
            /* To avoid looping unnecessarily when we are already done. */
            break;
        }
    }

    if (spads_enabled_count != spads_to_enable_count) {
        return false;
    }

    /* Write the new SPAD configuration */
    if (VL53L0X_ERROR_NONE != VL53L0X_WriteMulti(Dev, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, SPAD_MAP_ROW_COUNT)) {
        return false;
    }

    vl53l0x_log("Get spad from nvm done");

    return true;
}

/**
 * Load tuning settings (same as default tuning settings provided by ST api code)
 */
static bool load_default_tuning_settings(VL53L0X_DEV Dev)
{
    VL53L0X_Error error = VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x00);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x09, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x10, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x11, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x24, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x25, 0xFF);
    error |= VL53L0X_WrByte(Dev, 0x75, 0x00);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x4E, 0x2C);
    error |= VL53L0X_WrByte(Dev, 0x48, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x30, 0x20);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x30, 0x09);
    error |= VL53L0X_WrByte(Dev, 0x54, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x31, 0x04);
    error |= VL53L0X_WrByte(Dev, 0x32, 0x03);
    error |= VL53L0X_WrByte(Dev, 0x40, 0x83);
    error |= VL53L0X_WrByte(Dev, 0x46, 0x25);
    error |= VL53L0X_WrByte(Dev, 0x60, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x27, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x50, 0x06);
    error |= VL53L0X_WrByte(Dev, 0x51, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x52, 0x96);
    error |= VL53L0X_WrByte(Dev, 0x56, 0x08);
    error |= VL53L0X_WrByte(Dev, 0x57, 0x30);
    error |= VL53L0X_WrByte(Dev, 0x61, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x62, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x64, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x65, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x66, 0xA0);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x22, 0x32);
    error |= VL53L0X_WrByte(Dev, 0x47, 0x14);
    error |= VL53L0X_WrByte(Dev, 0x49, 0xFF);
    error |= VL53L0X_WrByte(Dev, 0x4A, 0x00);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x7A, 0x0A);
    error |= VL53L0X_WrByte(Dev, 0x7B, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x78, 0x21);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x23, 0x34);
    error |= VL53L0X_WrByte(Dev, 0x42, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x44, 0xFF);
    error |= VL53L0X_WrByte(Dev, 0x45, 0x26);
    error |= VL53L0X_WrByte(Dev, 0x46, 0x05);
    error |= VL53L0X_WrByte(Dev, 0x40, 0x40);
    error |= VL53L0X_WrByte(Dev, 0x0E, 0x06);
    error |= VL53L0X_WrByte(Dev, 0x20, 0x1A);
    error |= VL53L0X_WrByte(Dev, 0x43, 0x40);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x34, 0x03);
    error |= VL53L0X_WrByte(Dev, 0x35, 0x44);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x31, 0x04);
    error |= VL53L0X_WrByte(Dev, 0x4B, 0x09);
    error |= VL53L0X_WrByte(Dev, 0x4C, 0x05);
    error |= VL53L0X_WrByte(Dev, 0x4D, 0x04);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x44, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x45, 0x20);
    error |= VL53L0X_WrByte(Dev, 0x47, 0x08);
    error |= VL53L0X_WrByte(Dev, 0x48, 0x28);
    error |= VL53L0X_WrByte(Dev, 0x67, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x70, 0x04);
    error |= VL53L0X_WrByte(Dev, 0x71, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x72, 0xFE);
    error |= VL53L0X_WrByte(Dev, 0x76, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x77, 0x00);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x0D, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x80, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x01, 0xF8);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x8E, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x80, 0x00);

    vl53l0x_log("load tuning done: %d", error);
    return error == VL53L0X_ERROR_NONE;
}

static bool configure_interrupt(VL53L0X_DEV Dev)
{
    /* Interrupt on new sample ready */
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) {
        return false;
    }

    /* Configure active low since the pin is pulled-up on most breakout boards */
    uint8_t gpio_hv_mux_active_high = 0;
    if (VL53L0X_ERROR_NONE != VL53L0X_RdByte(Dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high)) {
        return false;
    }
    gpio_hv_mux_active_high &= ~0x10;
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high)) {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }
    vl53l0x_log("config intr done");
    return true;
}


/**
 * Enable (or disable) specific steps in the sequence
 */
static bool set_sequence_steps_enabled(VL53L0X_DEV Dev, uint8_t sequence_step)
{
    return VL53L0X_ERROR_NONE == VL53L0X_WrByte(Dev, REG_SYSTEM_SEQUENCE_CONFIG, sequence_step);
}

/**
 * Basic device initialization
 */
static bool static_init(VL53L0X_DEV Dev)
{
    if (!set_spads_from_nvm(Dev)) {
        return false;
    }

    if (!load_default_tuning_settings(Dev)) {
        return false;
    }

    if (!configure_interrupt(Dev)) {
        return false;
    }

    if (!set_sequence_steps_enabled(Dev, RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        return false;
    }
    vl53l0x_log("static init done");

    return true;
}

static bool perform_single_ref_calibration(VL53L0X_DEV Dev, calibration_type_t calib_type)
{
    uint8_t sysrange_start = 0;
    uint8_t sequence_config = 0;
    switch (calib_type)
    {
    case CALIBRATION_TYPE_VHV:
        sequence_config = 0x01;
        sysrange_start = 0x01 | 0x40;
        break;
    case CALIBRATION_TYPE_PHASE:
        sequence_config = 0x02;
        sysrange_start = 0x01 | 0x00;
        break;
    }
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSTEM_SEQUENCE_CONFIG, sequence_config)) {
        return false;
    }
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSRANGE_START, sysrange_start)) {
        return false;
    }
    /* Wait for interrupt */
    uint8_t interrupt_status = 0;
    VL53L0X_Error error = VL53L0X_ERROR_NONE;
    do {
        error = VL53L0X_RdByte(Dev, REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (error == VL53L0X_ERROR_NONE && ((interrupt_status & 0x07) == 0));
    if (VL53L0X_ERROR_NONE != error) {
        return false;
    }
    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSRANGE_START, 0x00)) {
        return false;
    }

    vl53l0x_log("single ref cal done %d", calib_type);
    return true;
}

/**
 * Temperature calibration needs to be run again if the temperature changes by
 * more than 8 degrees according to the datasheet.
 */
static bool perform_ref_calibration(VL53L0X_DEV Dev)
{
    if (!perform_single_ref_calibration(Dev, CALIBRATION_TYPE_VHV)) {
        return false;
    }
    if (!perform_single_ref_calibration(Dev, CALIBRATION_TYPE_PHASE)) {
        return false;
    }
    /* Restore sequence steps enabled */
    if (!set_sequence_steps_enabled(Dev, RANGE_SEQUENCE_STEP_DSS +
                                    RANGE_SEQUENCE_STEP_PRE_RANGE +
                                    RANGE_SEQUENCE_STEP_FINAL_RANGE)) {
        return false;
    }
    vl53l0x_log("ref cal done");
    return true;
}

/* Sets the address of a single VL53L0X sensor.
 * This functions assumes that all non-configured VL53L0X are still
 * in hardware standby. */
static bool init_address(VL53L0X_DEV Dev)
{
    /* The datasheet doesn't say how long we must wait to leave hw standby,
     * but using the same delay as vl6180x seems to work fine. */
    HAL_Delay(1);

    if (!device_is_booted(Dev)) {
        return false;
    }
    return true;
}

/**
 * Initializes the sensors by putting them in hw standby and then
 * waking them up one-by-one as described in AN4846.
 */
static bool init_addresses(VL53L0X_DEV Dev)
{
    /* Wake each sensor up one by one and set a unique address for each one */
    if (!init_address(Dev)) {
        return false;
    }

    return true;
}

static bool init_config(VL53L0X_DEV Dev)
{
    if (!data_init(Dev)) {
        return false;
    }
    if (!static_init(Dev)) {
        return false;
    }
    if (!perform_ref_calibration(Dev)) {
        return false;
    }
    return true;
}

bool vl53l0x_init(VL53L0X_DEV Dev)
{
    if (!init_addresses(Dev)) {
        return false;
    }
    if (!init_config(Dev)) {
        return false;
    }
    return true;
}

bool vl53l0x_read_range_single(VL53L0X_DEV Dev, uint16_t *range)
{
    VL53L0X_Error error = VL53L0X_WrByte(Dev, 0x80, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x01);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x91, stop_variable);
    error |= VL53L0X_WrByte(Dev, 0x00, 0x01);
    error |= VL53L0X_WrByte(Dev, 0xFF, 0x00);
    error |= VL53L0X_WrByte(Dev, 0x80, 0x00);
    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSRANGE_START, 0x01)) {
        return false;
    }

    uint8_t sysrange_start = 0;
    do {
        error = VL53L0X_RdByte(Dev, REG_SYSRANGE_START, &sysrange_start);
    } while (error != VL53L0X_ERROR_NONE && (sysrange_start & 0x01));
    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }

    uint8_t interrupt_status = 0;
    do {
        error = VL53L0X_RdByte(Dev, REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    } while (error != VL53L0X_ERROR_NONE && ((interrupt_status & 0x07) == 0));
    if (error != VL53L0X_ERROR_NONE) {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_RdWord(Dev, REG_RESULT_RANGE_STATUS + 10, range)) {
        return false;
    }

    if (VL53L0X_ERROR_NONE != VL53L0X_WrByte(Dev, REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    /* 8190 or 8191 may be returned when obstacle is out of range. */
    if (*range == 8190 || *range == 8191) {
        *range = VL53L0X_OUT_OF_RANGE;
    }

    return true;
}
