/**
 * @file types.h
 * @brief Ouster client datatypes and constants. 
 */

#pragma once

#include <json/json.h>
#include <Eigen/Eigen>
#include <set>
#include <string>
#include <vector>

#include "optional-lite/optional.hpp"

// Declare namespaces from optional-lite
using nonstd::nullopt;
using nonstd::optional;

namespace ouster {

  /** For image operations. */
  template < typename T >
  using img_t = Eigen::Array < T, -1, -1, Eigen::RowMajor >;

  /** Used for transformations */
  using mat4d = Eigen::Matrix < double, 4, 4, Eigen::DontAlign >;

  namespace sensor {

    using AzimuthWindow = std::pair<int, int>;
    using ColumnWindow = std::pair<int, int>;

    /** Unit of range from sensor packet, in meters. */
    constexpr double range_unit = 0.001;

    /** Design values for altitude and azimuth offset angles for gen1 sensors. */
    extern const std::vector < double > gen1_altitude_angles;
    extern const std::vector < double > gen1_azimuth_angles;

    /** Design values for imu and lidar to sensor-frame transforms. */
    extern const mat4d default_imu_to_sensor_transform;
    extern const mat4d default_lidar_to_sensor_transform;

    enum configuration_version
    {
      FW_2_0 = 3,
      FW_2_2 = 4
    };

    /**
 * Constants used for configuration. Refer to the sensor documentation for the
 * meaning of each option.
 */
    enum lidar_mode {
      MODE_UNSPEC = 0,  ///< lidar mode: unspecified
      MODE_512x10,      ///< lidar mode: 10 scans of 512 columns per second
      MODE_512x20,      ///< lidar mode: 20 scans of 512 columns per second
      MODE_1024x10,     ///< lidar mode: 10 scans of 1024 columns per second
      MODE_1024x20,     ///< lidar mode: 20 scans of 1024 columns per second
      MODE_2048x10,     ///< lidar mode: 10 scans of 2048 columns per second
      MODE_4096x5       ///< lidar mode: 5 scans of 4096 columns per second. Only
                      ///< available on select sensors
    };

    enum timestamp_mode
    {
      TIME_FROM_UNSPEC = 0,
      TIME_FROM_INTERNAL_OSC,
      TIME_FROM_SYNC_PULSE_IN,
      TIME_FROM_PTP_1588
    };

    /**
 * Flags for set_config()
 */
    enum config_flags : uint8_t {
      CONFIG_UDP_DEST_AUTO    = (1 << 0), ///< Set udp_dest automatically
      CONFIG_PERSIST          = (1 << 1), ///< Make configuration persistent
      CONFIG_FORCE_REINIT     = (1 << 2)  ///< Forces the sensor to re-init during
                                      ///< set_config even when config params
                                      ///< have not changed
    };

    enum OperatingMode 
    { 
      OPERATING_NORMAL = 1, 
      OPERATING_STANDBY 
    };

    enum MultipurposeIOMode 
    {
      MULTIPURPOSE_OFF = 1,
      MULTIPURPOSE_INPUT_NMEA_UART,
      MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC,
      MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN,
      MULTIPURPOSE_OUTPUT_FROM_PTP_1588,
      MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE
    };

    enum Polarity 
    { 
       POLARITY_ACTIVE_LOW = 1, 
       POLARITY_ACTIVE_HIGH 
    };

    enum NMEABaudRate 
    { 
      BAUD_9600 = 1, 
      BAUD_115200 
    };

    /** Profile indicating packet format of lidar data. */
    enum UDPProfileLidar {
      /** Legacy lidar data */
      PROFILE_LIDAR_LEGACY = 1,
      /** Dual Returns data */
      PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL,
      /** Single Returns data */
      PROFILE_RNG19_RFL8_SIG16_NIR16,
      /** Single Returns Low Data Rate */
      PROFILE_RNG15_RFL8_NIR8,
      /** Five Word Profile */
      PROFILE_FIVE_WORD_PIXEL,
    };

    /** Profile indicating packet format of IMU data. */
    enum UDPProfileIMU {
      PROFILE_IMU_LEGACY = 1  ///< Legacy IMU data
    };

    /** Thermal Shutdown status. */
    enum ThermalShutdownStatus {
      THERMAL_SHUTDOWN_NORMAL = 0x00,    ///< Normal operation
      THERMAL_SHUTDOWN_IMMINENT = 0x01,  ///< Thermal Shutdown imminent
    };

    /** Shot Limiting status. */
    enum ShotLimitingStatus {
      SHOT_LIMITING_NORMAL = 0x00,    ///< Normal operation
      SHOT_LIMITING_IMMINENT = 0x01,  ///< Shot limiting imminent
      SHOT_LIMITING_REDUCTION_0_10 =
              0x02,  //< Shot limiting reduction by 0 to 10%
      SHOT_LIMITING_REDUCTION_10_20 =
              0x03,  ///< Shot limiting reduction by 10 to 20%
      SHOT_LIMITING_REDUCTION_20_30 =
              0x04,  ///< Shot limiting reduction by 20 to 30%
      SHOT_LIMITING_REDUCTION_30_40 =
              0x05,  ///< Shot limiting reduction by 30 to 40%
      SHOT_LIMITING_REDUCTION_40_50 =
              0x06,  ///< Shot limiting reduction by 40 to 50%
      SHOT_LIMITING_REDUCTION_50_60 =
              0x07,  ///< Shot limiting reduction by 50 to 60%
      SHOT_LIMITING_REDUCTION_60_70 =
              0x08,  ///< Shot limiting reduction by 60 to 70%
      SHOT_LIMITING_REDUCTION_70_75 =
              0x09,  ///< Shot limiting reduction by 70 to 80%
    };

    /** Tag to identitify a paricular value reported in the sensor channel data
 * block. */
    enum ChanField {
      RANGE = 1,            ///< 1st return range in mm
      RANGE2 = 2,           ///< 2nd return range in mm
      INTENSITY = 3,        ///< @deprecated Use SIGNAL instead
      SIGNAL = 3,           ///< 1st return signal in photons
      SIGNAL2 = 4,          ///< 2nd return signal in photons
      REFLECTIVITY = 5,     ///< 1st return reflectivity, calibrated by range and sensor
                          ///< sensitivity in FW 2.1+. See sensor docs for more details
      REFLECTIVITY2 = 6,    ///< 2nd return reflectivity, calibrated by range and sensor
                          ///< sensitivity in FW 2.1+. See sensor docs for more details
      AMBIENT = 7,          ///< @deprecated Use NEAR_IR instead
      NEAR_IR = 7,          ///< near_ir in photons
      FLAGS = 8,            ///< 1st return flags
      FLAGS2 = 9,           ///< 2nd return flags
      RAW_HEADERS = 40,     ///< raw headers for packet/footer/column for dev use
      RAW32_WORD5 = 45,     ///< raw word access to packet for dev use
      RAW32_WORD6 = 46,     ///< raw word access to packet for dev use
      RAW32_WORD7 = 47,     ///< raw word access to packet for dev use
      RAW32_WORD8 = 48,     ///< raw word access to packet for dev use
      RAW32_WORD9 = 49,     ///< raw word access to packet for dev use
      CUSTOM0 = 50,         ///< custom user field
      CUSTOM1 = 51,         ///< custom user field
      CUSTOM2 = 52,         ///< custom user field
      CUSTOM3 = 53,         ///< custom user field
      CUSTOM4 = 54,         ///< custom user field
      CUSTOM5 = 55,         ///< custom user field
      CUSTOM6 = 56,         ///< custom user field
      CUSTOM7 = 57,         ///< custom user field
      CUSTOM8 = 58,         ///< custom user field
      CUSTOM9 = 59,         ///< custom user field
      RAW32_WORD1 = 60,     ///< raw word access to packet for dev use
      RAW32_WORD2 = 61,     ///< raw word access to packet for dev use
      RAW32_WORD3 = 62,     ///< raw word access to packet for dev use
      RAW32_WORD4 = 63,     ///< raw word access to packet for dev use
      CHAN_FIELD_MAX = 64,  ///< max which allows us to introduce future fields
    };

    /**
 * Types of channel fields.
 */
    enum ChanFieldType { VOID = 0, UINT8, UINT16, UINT32, UINT64 };

    /** Stores data format information. */
    struct data_format {
      uint32_t pixels_per_column;   ///< pixels per column
      uint32_t columns_per_packet;  ///< columns per packet
      uint32_t
              columns_per_frame;  ///< columns per frame, should match with lidar mode
      std::vector<int>
              pixel_shift_by_row;      ///< shift of pixels by row to enable destagger
      ColumnWindow column_window;  ///< window of columns over which sensor fires
      UDPProfileLidar udp_profile_lidar;  ///< profile of lidar packet
      UDPProfileIMU udp_profile_imu;      ///< profile of imu packet
      uint16_t fps;                ///< frames per second
    };

    /** Stores necessary information from sensor to parse and project sensor data.
 */
    struct sensor_info {
      std::string
              name;               ///< @deprecated Will be removed in the next version
      std::string sn;         ///< sensor serial number
      std::string fw_rev;     ///< fw revision
      lidar_mode mode;        ///< lidar mode of sensor
      std::string prod_line;  ///< prod line
      data_format format;     ///< data format of sensor
      std::vector<double>
              beam_azimuth_angles;  ///< beam azimuth angles for 3D projection
      std::vector<double>
              beam_altitude_angles;  ///< beam altitude angles for 3D projection
      double lidar_origin_to_beam_origin_mm;  ///< distance between lidar origin
                                            ///< and beam origin in mm
      mat4d beam_to_lidar_transform;  ///< transform between beam and lidar frame
      mat4d imu_to_sensor_transform;  ///< transform between sensor coordinate
                                            ///< frame and imu
      mat4d lidar_to_sensor_transform;  ///< transform between lidar and sensor
                                            ///< coordinate frames
      mat4d extrinsic;                  ///< extrinsic matrix
      uint32_t init_id;         ///< initialization ID updated every reinit
      uint16_t udp_port_lidar;  ///< the lidar destination port
      uint16_t udp_port_imu;    ///< the imu destination port
    };

    /**
 * Struct for sensor configuration parameters.
 */
    struct sensor_config {
      /**
       * The destination address for the
       * lidar/imu data to be sent to
       */
      optional<std::string> udp_dest;
      /**
       * The destination port for the lidar
       * data to be sent to
       */
      optional<int> udp_port_lidar;
      /**
       * The destination port for the imu data
       * to be sent to
       */
      optional<int> udp_port_imu;

      /**
     * The timestamp mode for the sensor to use.
     * Refer to timestamp_mode for more details.
     */
      optional<timestamp_mode> ts_mode;
      /**
     * The lidar mode for the sensor to use.
     * Refer to lidar_mode for more details.
     */
      optional<lidar_mode> ld_mode;
      /**
     * The operating mode for the sensor to use.
     * Refer to OperatingMode for more details.
     */
      optional<OperatingMode> operating_mode;
      /**
     * The multipurpose io mode for the sensor to use.
     * Refer to MultipurposeIOMode for more details.
     */
      optional<MultipurposeIOMode> multipurpose_io_mode;

      /**
     * The azimuth window for the sensor to use.
     * Refer to AzimuthWindow for more details.
     */
      optional<AzimuthWindow> azimuth_window;
      /**
     * Multiplier for signal strength of sensor. See the sensor docs for
     * more details on usage.
     */
      optional<double> signal_multiplier;

      /**
     * The nmea polarity for the sensor to use.
     * Refer to Polarity for more details.
     */
      optional<Polarity> nmea_in_polarity;
      /**
     * Whether NMEA UART input $GPRMC messages should be ignored.
     * Refer to the sensor docs for more details.
     */
      optional<bool> nmea_ignore_valid_char;
      /**
     * The nmea baud rate for the sensor to use.
     * Refer to Polarity> for more details.
     */
      optional<NMEABaudRate> nmea_baud_rate;
      /**
     * Number of leap seconds added to UDP timestamp.
     * See the sensor docs for more details.
     */
      optional<int> nmea_leap_seconds;

      /**
     * Polarity of SYNC_PULSE_IN input.
     * See Polarity for more details.
     */
      optional<Polarity> sync_pulse_in_polarity;
      /**
     * Polarity of SYNC_PULSE_OUT output.
     * See Polarity for more details.
     */
      optional<Polarity> sync_pulse_out_polarity;
      /**
     * Angle in degrees that sensor traverses between each SYNC_PULSE_OUT
     * pulse. See senor docs for more details.
     */
      optional<int> sync_pulse_out_angle;
      /**
     * Width of SYNC_PULSE_OUT pulse in ms.
     * See sensor docs for more details.
     */
      optional<int> sync_pulse_out_pulse_width;
      /**
     * Frequency of SYNC_PULSE_OUT pulse in Hz.
     * See sensor docs for more details.
     */
      optional<int> sync_pulse_out_frequency;

      /**
     * Whether phase locking is enabled.
     * See sensor docs for more details.
     */
      optional<bool> phase_lock_enable;
      /**
     * Angle that sensors are locked to in millidegrees.
     * See sensor docs for more details.
     */
      optional<int> phase_lock_offset;

      /**
     * Columns per packet.
     * See sensor docs for more details.
     */
      optional<int> columns_per_packet;
      /**
     * The lidar profile for the sensor to use.
     * Refer to UDPProfileLidar for more details.
     */
      optional<UDPProfileLidar> udp_profile_lidar;
      /**
     * The imu profile for the sensor to use.
     * Refer to UDPProfileIMU for more details.
     */
      optional<UDPProfileIMU> udp_profile_imu;
    };

    /** Equality/Not-Equality for data_format */
    bool operator==(const data_format& lhs, const data_format& rhs);
    bool operator!=(const data_format& lhs, const data_format& rhs);

    /** Equality/Not-Equality for sensor_info */
    bool operator==(const sensor_info& lhs, const sensor_info& rhs);
    bool operator!=(const sensor_info& lhs, const sensor_info& rhs);

    /** Equality/Not Equality for sensor config */
    bool operator==(const sensor_config& lhs, const sensor_config& rhs);
    bool operator!=(const sensor_config& lhs, const sensor_config& rhs);

    /**
     * Get a default sensor_info for the given lidar mode.
     *
     * @param lidar_mode
     * @return default sensor_info for the OS1-64
     */
    sensor_info default_sensor_info(lidar_mode mode);

    /**
     * Get string representation of a lidar mode.
     *
     * @param lidar_mode
     * @return string representation of the lidar mode, or "UNKNOWN"
     */
    std::string to_string(lidar_mode mode);

    /**
     * Get lidar mode from string.
     *
     * @param string
     * @return lidar mode corresponding to the string, or 0 on error
     */
    lidar_mode lidar_mode_of_string(const std::string & s);

    /**
     * Get number of columns in a scan for a lidar mode.
     *
     * @param lidar_mode
     * @return number of columns per rotation for the mode
     */
    uint32_t n_cols_of_lidar_mode(lidar_mode mode);

    /**
     * Get the lidar rotation frequency from lidar mode.
     *
     * @param lidar_mode
     * @return lidar rotation frequency in Hz
     */
    int frequency_of_lidar_mode(lidar_mode mode);

    /**
     * Get string representation of a timestamp mode.
     *
     * @param timestamp_mode
     * @return string representation of the timestamp mode, or "UNKNOWN"
     */
    std::string to_string(timestamp_mode mode);

    /**
     * Get timestamp mode from string.
     *
     * @param string
     * @return timestamp mode corresponding to the string, or 0 on error
     */
    timestamp_mode timestamp_mode_of_string(const std::string & s);

    /**
     * Get string representation of an operating mode.
     *
     * @param mode
     * @return string representation of the operating mode, or "UNKNOWN"
     */
    std::string to_string(OperatingMode mode);

    /**
     * Get operating mode from string.
     *
     * @param string
     * @return operating mode corresponding to the string, or 0 on error
     */
    optional<OperatingMode> operating_mode_of_string(const std::string& s);

    /**
     * Get string representation of a multipurpose io mode.
     *
     * @param mode
     * @return string representation of the multipurpose io mode, or "UNKNOWN"
     */
    std::string to_string(MultipurposeIOMode mode);

    /**
     * Get multipurpose io mode from string.
     *
     * @param string
     * @return multipurpose io mode corresponding to the string, or 0 on error
     */
    optional<MultipurposeIOMode> multipurpose_io_mode_of_string(
        const std::string& s);

    /**
     * Get string representation of a polarity.
     *
     * @param polarity
     * @return string representation of the polarity, or "UNKNOWN"
     */
    std::string to_string(Polarity polarity);

    /**
     * Get polarity from string.
     *
     * @param string
     * @return polarity corresponding to the string, or 0 on error
     */
    optional<Polarity> polarity_of_string(const std::string& s);

    /**
     * Get string representation of a NMEA Baud Rate
     *
     * @param rate
     * @return string representation of the NMEA baud rate, or "UNKNOWN"
     */
    std::string to_string(NMEABaudRate rate);

    /**
     * Get nmea baud rate from string.
     *
     * @param string
     * @return nmea baud rate corresponding to the string, or 0 on error
     */
    optional<NMEABaudRate> nmea_baud_rate_of_string(const std::string& s);

    /**
     * Get string representation of an Azimuth Window
     *
     * @param azimuth_window
     * @return string representation of the azimuth window
     */
    std::string to_string(AzimuthWindow azimuth_window);

    /**
 * Get string representation of a lidar profile.
 *
 * @param[in] profile The profile to get the string representation of.
 *
 * @return string representation of the lidar profile.
 */
    std::string to_string(UDPProfileLidar profile);

    /**
 * Get lidar profile from string.
 *
 * @param[in] s The string to decode into a lidar profile.
 *
 * @return lidar profile corresponding to the string, or nullopt on error.
 */
    optional<UDPProfileLidar> udp_profile_lidar_of_string(const std::string& s);

    /**
 * Get string representation of an IMU profile.
 *
 * @param[in] profile The profile to get the string representation of.
 *
 * @return string representation of the lidar profile.
 */
    std::string to_string(UDPProfileIMU profile);

    /**
 * Get imu profile from string
 *
 * @param[in] s The string to decode into an imu profile.
 *
 * @return imu profile corresponding to the string, or nullopt on error.
 */
    optional<UDPProfileIMU> udp_profile_imu_of_string(const std::string& s);

    /**
 * Get string representation of a Shot Limiting Status.
 *
 * @param[in] shot_limiting_status The shot limiting status to get the string
 * representation of.
 *
 * @return string representation of the shot limiting status.
 */
    std::string to_string(ShotLimitingStatus shot_limiting_status);

    /**
 * Get string representation of Thermal Shutdown Status.
 *
 * @param[in] thermal_shutdown_status The thermal shutdown status to get the
 * string representation of.
 *
 * @return string representation of thermal shutdown status.
 */
    std::string to_string(ThermalShutdownStatus thermal_shutdown_status);

    /**
 * Determine validity of provided signal multiplier value
 *
 * @param[in] signal_multiplier Signal multiplier value.
 */
    void check_signal_multiplier(double signal_multiplier);

    /**
     * Parse metadata text blob from the sensor into a sensor_info struct.
     *
     * String and vector fields will have size 0 if the parameter cannot
     * be found or parsed, while lidar_mode will be set to 0 (invalid).
     *
     * @throw runtime_error if the text is not valid json
     * @param metadata a text blob returned by get_metadata from client.h
     * @return a sensor_info struct populated with a subset of the metadata
     */
    sensor_info parse_metadata(const std::string & metadata);

    /**
     * Parse metadata given path to a json file.
     *
     * @throw runtime_error if json file does not exist or is malformed
     * @param json_file path to a json file containing sensor metadata
     * @return a sensor_info struct populated with a subset of the metadata
     */
    sensor_info metadata_from_json(const std::string & json_file);

    /**
     * Get a string representation of metadata. All fields included.
     *
     * @param metadata a struct of sensor metadata
     * @return a json metadata string
     */
    std::string to_string(const sensor_info & metadata);

    /**
     * Parse config text blob from the sensor into a sensor_config struct
     *
     * All fields are optional, and will only be set if found.
     *
     * @throw runtime_error if the text is not valid json
     * @param metadata a text blob given by get_config from client.h
     * @return a sensor_config struct populated with the sensor config
     * parameters
     */
    sensor_config parse_config(const std::string& config);

    /**
     * Get a string representation of sensor config. Only set fields will be
     * represented.
     *
     * @param config a struct of sensor config
     * @return a json sensor config string
     */
    std::string to_string(const sensor_config& config);

    /**
 * Convert non-legacy string representation of metadata to legacy.
 *
 * @param[in] metadata non-legacy string representation of metadata.
 *
 * @return legacy string representation of metadata.
 */
    std::string convert_to_legacy(const std::string& metadata);


    Json::Value to_json(const sensor_config &config);

    /**
 * Get string representation of a channel field.
 *
 * @param[in] field The field to get the string representation of.
 *
 * @return string representation of the channel field.
 */
    std::string to_string(ChanField field);

    /**
 * Get the size of the ChanFieldType in bytes.
 *
 * @param[in] ft the field type
 *
 * @return size of the field type in bytes
 */
    size_t field_type_size(ChanFieldType ft);

    /**
 * Get string representation of a channel field.
 *
 * @param[in] ft The field type to get the string representation of.
 *
 * @return string representation of the channel field type.
 */
    std::string to_string(ChanFieldType ft);

    /**
     * Table of accessors for extracting data from imu and lidar packets.
     *
     * In the user guide, refer to section 9 for the lidar packet format and section
     * 10 for imu packets.
     *
     * For 0 <= n < columns_per_packet, nth_col(n, packet_buf) returns a pointer to
     * the nth measurement block. For 0 <= m < pixels_per_column, nth_px(m, col_buf)
     * returns the mth channel data block.
     *
     * Use imu_la_{x,y,z} to access the acceleration in the corresponding
     * direction. Use imu_av_{x,y,z} to read the angular velocity.
     */
    struct packet_format
    {
      const size_t lidar_packet_size;
      const size_t imu_packet_size;
      const int columns_per_packet;
      const int pixels_per_column;
      const int encoder_ticks_per_rev;

      // Measurement block accessors
      const uint8_t * (* const nth_col)(int n, const uint8_t * lidar_buf);
      uint64_t(*const col_timestamp)(const uint8_t * col_buf);
      uint32_t(*const col_encoder)(const uint8_t * col_buf);
      uint16_t(*const col_measurement_id)(const uint8_t * col_buf);
      uint16_t(*const col_frame_id)(const uint8_t * col_buf);
      uint32_t(*const col_status)(const uint8_t * col_buf);

      // Channel data block accessors
      const uint8_t * (* const nth_px)(int n, const uint8_t * col_buf);
      uint32_t(*const px_range)(const uint8_t * px_buf);
      uint16_t(*const px_reflectivity)(const uint8_t * px_buf);
      uint16_t(*const px_signal)(const uint8_t * px_buf);
      uint16_t(*const px_ambient)(const uint8_t * px_buf);

      // IMU packet accessors
      uint64_t(*const imu_sys_ts)(const uint8_t * imu_buf);
      uint64_t(*const imu_accel_ts)(const uint8_t * imu_buf);
      uint64_t(*const imu_gyro_ts)(const uint8_t * imu_buf);
      float(*const imu_la_x)(const uint8_t * imu_buf);
      float(*const imu_la_y)(const uint8_t * imu_buf);
      float(*const imu_la_z)(const uint8_t * imu_buf);
      float(*const imu_av_x)(const uint8_t * imu_buf);
      float(*const imu_av_y)(const uint8_t * imu_buf);
      float(*const imu_av_z)(const uint8_t * imu_buf);
    };

    /**
     * Get a packet parser for a particular data format.
     *
     * @param data_format parameters provided by the sensor
     * @return a packet_format suitable for parsing UDP packets sent by the sensor
     */
    const packet_format & get_format(const sensor_info & info);

  } // namespace sensor
}  // namespace ouster
