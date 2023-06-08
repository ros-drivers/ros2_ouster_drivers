/**
 * @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "ros2_ouster/client/types.h"

namespace ouster {

namespace impl {
// forward declaration
struct LidarScan;

using sensor::ChanFieldType;

template <typename T>
struct FieldTag;

template <>
struct FieldTag<uint8_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT8;
};

template <>
struct FieldTag<uint16_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT16;
};

template <>
struct FieldTag<uint32_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT32;
};

template <>
struct FieldTag<uint64_t> {
  static constexpr ChanFieldType tag = ChanFieldType::UINT64;
};

/*
 * Tagged union for LidarScan fields
 */
struct FieldSlot {
  ChanFieldType tag;
  union {
    img_t<uint8_t> f8;
    img_t<uint16_t> f16;
    img_t<uint32_t> f32;
    img_t<uint64_t> f64;
  };

  FieldSlot(ChanFieldType t, size_t w, size_t h) : tag{t} {
    switch (t) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        new (&f8) img_t<uint8_t>{h, w};
        f8.setZero();
        break;
      case ChanFieldType::UINT16:
        new (&f16) img_t<uint16_t>{h, w};
        f16.setZero();
        break;
      case ChanFieldType::UINT32:
        new (&f32) img_t<uint32_t>{h, w};
        f32.setZero();
        break;
      case ChanFieldType::UINT64:
        new (&f64) img_t<uint64_t>{h, w};
        f64.setZero();
        break;
    }
  }

  FieldSlot() : FieldSlot{ChanFieldType::VOID, 0, 0} {};

  ~FieldSlot() { clear(); }

  FieldSlot(const FieldSlot& other) {
    switch (other.tag) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        new (&f8) img_t<uint8_t>{other.f8};
        break;
      case ChanFieldType::UINT16:
        new (&f16) img_t<uint16_t>{other.f16};
        break;
      case ChanFieldType::UINT32:
        new (&f32) img_t<uint32_t>{other.f32};
        break;
      case ChanFieldType::UINT64:
        new (&f64) img_t<uint64_t>{other.f64};
        break;
    }
    tag = other.tag;
  }

  FieldSlot(FieldSlot&& other)  noexcept { set_from(other); }

  FieldSlot& operator=(FieldSlot other) {
    clear();
    set_from(other);
    return *this;
  }

  template <typename T>
  Eigen::Ref<img_t<T>> get() {
    if (tag == FieldTag<T>::tag)
      return get_unsafe<T>();
    else
      throw std::invalid_argument("Accessed field at wrong type");
  }

  template <typename T>
  Eigen::Ref<const img_t<T>> get() const {
    if (tag == FieldTag<T>::tag)
      return get_unsafe<T>();
    else
      throw std::invalid_argument("Accessed field at wrong type");
  }

  friend bool operator==(const FieldSlot& l, const FieldSlot& r) {
    if (l.tag != r.tag) return false;
    switch (l.tag) {
      case ChanFieldType::VOID:
        return true;
      case ChanFieldType::UINT8:
        return (l.f8 == r.f8).all();
      case ChanFieldType::UINT16:
        return (l.f16 == r.f16).all();
      case ChanFieldType::UINT32:
        return (l.f32 == r.f32).all();
      case ChanFieldType::UINT64:
        return (l.f64 == r.f64).all();
      default:
        assert(false);
    }
    // unreachable, appease older gcc
    return false;
  }

  private:
  void set_from(FieldSlot& other) {
    switch (other.tag) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        new (&f8) img_t<uint8_t>{std::move(other.f8)};
        break;
      case ChanFieldType::UINT16:
        new (&f16) img_t<uint16_t>{std::move(other.f16)};
        break;
      case ChanFieldType::UINT32:
        new (&f32) img_t<uint32_t>{std::move(other.f32)};
        break;
      case ChanFieldType::UINT64:
        new (&f64) img_t<uint64_t>{std::move(other.f64)};
        break;
    }
    tag = other.tag;
    other.clear();
  }

  void clear() {
    switch (tag) {
      case ChanFieldType::VOID:
        break;
      case ChanFieldType::UINT8:
        f8.~img_t<uint8_t>();
        break;
      case ChanFieldType::UINT16:
        f16.~img_t<uint16_t>();
        break;
      case ChanFieldType::UINT32:
        f32.~img_t<uint32_t>();
        break;
      case ChanFieldType::UINT64:
        f64.~img_t<uint64_t>();
        break;
    }
    tag = ChanFieldType::VOID;
  }

  template <typename T>
  Eigen::Ref<img_t<T>> get_unsafe();

  template <typename T>
  Eigen::Ref<const img_t<T>> get_unsafe() const;
};

template <>
inline Eigen::Ref<img_t<uint8_t>> FieldSlot::get_unsafe() {
  return f8;
}

template <>
inline Eigen::Ref<img_t<uint16_t>> FieldSlot::get_unsafe() {
  return f16;
}

template <>
inline Eigen::Ref<img_t<uint32_t>> FieldSlot::get_unsafe() {
  return f32;
}

template <>
inline Eigen::Ref<img_t<uint64_t>> FieldSlot::get_unsafe() {
  return f64;
}

template <>
inline Eigen::Ref<const img_t<uint8_t>> FieldSlot::get_unsafe() const {
  return f8;
}

template <>
inline Eigen::Ref<const img_t<uint16_t>> FieldSlot::get_unsafe() const {
  return f16;
}

template <>
inline Eigen::Ref<const img_t<uint32_t>> FieldSlot::get_unsafe() const {
  return f32;
}

template <>
inline Eigen::Ref<const img_t<uint64_t>> FieldSlot::get_unsafe() const {
  return f64;
}

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the LidarScan channel field f
 * Example code for the operation<T>:
 * \code
 * struct print_field_size {
 *   template <typename T>
 *   void operator()(Eigen::Ref<img_t<T>> field) {
 *       std::cout << "Rows: " + field.rows() << std::endl;
 *       std::cout << "Cols: " + field.cols() << std::endl;
 *   }
 * };
 * \endcode
 */
template <typename SCAN, typename OP, typename... Args>
void visit_field(SCAN&& ls, sensor::ChanField f, OP&& op, Args&&... args) {
  switch (ls.field_type(f)) {
    case sensor::ChanFieldType::UINT8:
      op.template operator()(ls.template field<uint8_t>(f),
                             std::forward<Args>(args)...);
      break;
    case sensor::ChanFieldType::UINT16:
      op.template operator()(ls.template field<uint16_t>(f),
                             std::forward<Args>(args)...);
      break;
    case sensor::ChanFieldType::UINT32:
      op.template operator()(ls.template field<uint32_t>(f),
                             std::forward<Args>(args)...);
      break;
    case sensor::ChanFieldType::UINT64:
      op.template operator()(ls.template field<uint64_t>(f),
                             std::forward<Args>(args)...);
      break;
    default:
      throw std::invalid_argument("Invalid field for LidarScan");
  }
}

/*
 * Call a generic operation op<T>(f, Args...) for each field of the lidar scan
 * with type parameter T having the correct field type
 */
template <typename SCAN, typename OP, typename... Args>
void foreach_field(SCAN&& ls, OP&& op, Args&&... args) {
  for (const auto& ft : ls)
    visit_field(std::forward<SCAN>(ls), ft.first, std::forward<OP>(op),
                ft.first, std::forward<Args>(args)...);
}

// Read LidarScan field and cast to the destination
struct read_and_cast {
  template <typename T, typename U>
  void operator()(Eigen::Ref<const img_t<T>> src, Eigen::Ref<img_t<U>> dest) {
    dest = src.template cast<U>();
  }
  template <typename T, typename U>
  void operator()(Eigen::Ref<img_t<T>> src, Eigen::Ref<img_t<U>> dest) {
    dest = src.template cast<U>();
  }
  template <typename T, typename U>
  void operator()(Eigen::Ref<img_t<T>> src, img_t<U>& dest) {
    dest = src.template cast<U>();
  }
  template <typename T, typename U>
  void operator()(Eigen::Ref<const img_t<T>> src, img_t<U>& dest) {
    dest = src.template cast<U>();
  }
};

// Copy fields from `ls_source` LidarScan to `field_dest` img with casting
// to the img_t<T> type of `field_dest`.
struct copy_and_cast {
  template <typename T>
  void operator()(Eigen::Ref<img_t<T>> field_dest, const LidarScan& ls_source,
                  sensor::ChanField ls_source_field) {
    visit_field(ls_source, ls_source_field, read_and_cast(),
                field_dest);
  }
};
}  // namespace impl

/**
 * Flags for frame_status
 */
enum frame_status_masks : uint64_t {
    FRAME_STATUS_THERMAL_SHUTDOWN_MASK = 0x0f,  ///< Mask to get thermal shutdown status
    FRAME_STATUS_SHOT_LIMITING_MASK = 0xf0      ///< Mask to get shot limting status
};

enum frame_status_shifts: uint64_t {
    FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT = 0,    ///< No shift for thermal shutdown
    FRAME_STATUS_SHOT_LIMITING_SHIFT = 4        /// shift 4 for shot limiting
};

/**
 * Alias for the lidar scan field types
 */
using LidarScanFieldTypes =
        std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>;

/**
 * Datastructure for efficient operations on aggregated lidar data.
 *
 * Stores each field (range, intensity, etc.) contiguously as a H x W block of
 * 4-byte unsigned integers, where H is the number of beams and W is the
 * horizontal resolution (e.g. 512, 1024, 2048).
 *
 * Note: this is the "staggered" representation where each column corresponds
 * to a single measurement in time. Use the destagger() function to create an
 * image where columns correspond to a single azimuth angle.
 */
  class LidarScan {
public:
    template <typename T>
    using Header = Eigen::Array<T, Eigen::Dynamic, 1>;  ///< Header typedef

    /** XYZ coordinates with dimensions arranged contiguously in columns */
    using Points = Eigen::Array < double, Eigen::Dynamic, 3 >;

private:
    Header<uint64_t> timestamp_;
    Header<uint16_t> measurement_id_;
    Header<uint32_t> status_;
    std::map<sensor::ChanField, impl::FieldSlot> fields_;
    LidarScanFieldTypes field_types_;

    LidarScan(size_t w, size_t h, LidarScanFieldTypes field_types);

public:
    /**
     * Pointer offsets to deal with strides.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    std::ptrdiff_t w{0};

    /**
     * Pointer offsets to deal with strides.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    std::ptrdiff_t h{0};

    /**
     * Frame status - information from the packet header which corresponds to a
     * frame
     *
     * @warning Member variables: use with caution, some of these will become
     * private.
     */
    uint64_t frame_status{0};

    /**
     * The current frame ID.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    int32_t frame_id{-1};

    using FieldIter =
            decltype(field_types_)::const_iterator;  ///< An STL Iterator of the
                                                   ///< field types

    /** The default constructor creates an invalid 0 x 0 scan. */
    LidarScan() = default;

    /**
     * Initialize a scan with fields configured for the LEGACY udp profile.
     *
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     * scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     */
    LidarScan(size_t w, size_t h)
        : LidarScan{w, h, sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY} {}

    /**
     * Initialize a scan with the default fields for a particular udp profile.
     *
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     * scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] profile udp profile.
     */
    LidarScan(size_t w, size_t h, sensor::UDPProfileLidar profile);

    /**
     * Initialize a scan with a custom set of fields.
     *
     * @tparam Iterator A standard template iterator for the custom fields.
     *
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     * scan.
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] begin begin iterator of pairs of channel fields and types.
     * @param[in] end end iterator of pairs of channel fields and types.
     */
    template <typename Iterator>
    LidarScan(size_t w, size_t h, Iterator begin, Iterator end)
        : LidarScan(w, h, {begin, end}){};

    /**
     * Initialize a lidar scan from another lidar scan.
     *
     * @param[in] other The other lidar scan to initialize from.
     */
    LidarScan(const LidarScan& other) = default;

    /** @copydoc LidarScan(const LidarScan& other) */
    LidarScan(LidarScan&& other) = default;

    /**
     * Copy via Move semantic.
     *
     * @param[in] other The lidar scan to copy from.
     */
    LidarScan& operator=(const LidarScan& other) = default;

    /** @copydoc operator=(const LidarScan& other) */
    LidarScan& operator=(LidarScan&& other) = default;

    /**
     * Lidar scan destructor.
     */
    ~LidarScan() = default;

    /**
     * Get frame shot limiting status
     */
    inline sensor::ShotLimitingStatus shot_limiting() const {
      return static_cast<sensor::ShotLimitingStatus>(
              (frame_status & frame_status_masks::FRAME_STATUS_SHOT_LIMITING_MASK) >>
              frame_status_shifts::FRAME_STATUS_SHOT_LIMITING_SHIFT);
    }

    /**
     * Get frame thermal shutdown status
     */
    inline sensor::ThermalShutdownStatus thermal_shutdown() const {
      return static_cast<sensor::ThermalShutdownStatus>(
              (frame_status &
               frame_status_masks::FRAME_STATUS_THERMAL_SHUTDOWN_MASK) >>
              frame_status_shifts::FRAME_STATUS_THERMAL_SHUTDOWN_SHIFT);
    }

    /**
     * Access a lidar data field.
     *
     * @throw std::invalid_argument if T does not match the runtime field type.
     *
     * @tparam T The type parameter T must match the dynamic type of the field.
     * See the constructor documentation for expected field types or query
     * dynamically for generic operations.
     *
     * @param[in] f the field to view.
     *
     * @return a view of the field data.
     */
    template <typename T = uint32_t,
             typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
    inline Eigen::Ref<img_t<T>> field(sensor::ChanField f) {
      return fields_.at(f).get<T>();
    }

    /** @copydoc field(Field f) */
    template <typename T = uint32_t,
             typename std::enable_if<std::is_unsigned<T>::value, T>::type = 0>
    inline Eigen::Ref<const img_t<T>> field(sensor::ChanField f) const {
      return fields_.at(f).get<T>();
    }

    /**
     * Get the type of the specified field.
     *
     * @param[in] f the field to query.
     *
     * @return the type tag associated with the field.
     */
    inline sensor::ChanFieldType field_type(sensor::ChanField f) const {
      return fields_.count(f) ? fields_.at(f).tag : sensor::ChanFieldType::VOID;
    }

    /** A const forward iterator over field / type pairs. */
    inline FieldIter begin() const { return field_types_.cbegin(); }

    /** @copydoc begin() */
    inline FieldIter end() const { return field_types_.cend(); }

    /**
     * Access the measurement timestamp headers.
     *
     * @return a view of timestamp as a w-element vector.
     */
    inline Eigen::Ref<Header<uint64_t>> timestamp() {
      return timestamp_;
    }

    /**
     * @copydoc timestamp()
     */
    inline Eigen::Ref<const Header<uint64_t>> timestamp() const {
      return timestamp_;
    }

    /**
     * Access the measurement id headers.
     *
     * @return a view of measurement ids as a w-element vector.
     */
    inline Eigen::Ref<Header<uint16_t>> measurement_id() {
      return measurement_id_;
    }

    /** @copydoc measurement_id() */
    inline Eigen::Ref<const Header<uint16_t>> measurement_id() const {
      return measurement_id_;
    }

    /**
     * Access the measurement status headers.
     *
     * @return a view of measurement statuses as a w-element vector.
     */
    inline Eigen::Ref<Header<uint32_t>> status() {
      return status_;
    }

    /** @copydoc status() */
    inline Eigen::Ref<const Header<uint32_t>> status() const {
      return status_;
    }

    /**
     * Assess completeness of scan.
     * @param[in] window The column window to use for validity assessment
     * @return whether all columns within given column window were valid
     */
    bool complete(sensor::ColumnWindow window) const;

    friend inline bool operator==(const LidarScan& a, const LidarScan& b);
  };

  /**
 * Get string representation of lidar scan field types.
 *
 * @param[in] field_types The field types to get the string representation of.
 *
 * @return string representation of the lidar scan field types.
 */
  std::string to_string(const LidarScanFieldTypes& field_types);

  /**
 * Get the lidar scan field types from a lidar scan
 *
 * @param[in] ls The lidar scan to get the lidar scan field types from.
 *
 * @return The lidar scan field types
 */
  inline LidarScanFieldTypes get_field_types(const LidarScan& ls) {
    return {ls.begin(), ls.end()};
  }

  /**
 * Get the lidar scan field types from sensor info
 *
 * @param[in] info The sensor info to get the lidar scan field types from.
 *
 * @return The lidar scan field types
 */
  LidarScanFieldTypes get_field_types(const sensor::sensor_info& info);

  /**
 * Get string representation of a lidar scan.
 *
 * @param[in] ls The lidar scan to get the string representation of.
 *
 * @return string representation of the lidar scan.
 */
  std::string to_string(const LidarScan& ls);

  /** Equality for scans. */
  inline bool operator == (const LidarScan & a, const LidarScan & b)
  {
    return a.frame_id == b.frame_id && a.w == b.w && a.h == b.h &&
           a.frame_status == b.frame_status && a.fields_ == b.fields_ &&
           a.field_types_ == b.field_types_ &&
           (a.timestamp() == b.timestamp()).all() &&
           (a.measurement_id() == b.measurement_id()).all() &&
           (a.status() == b.status()).all();
  }

  /** Not Equality for scans. */
  inline bool operator!=(const LidarScan& a, const LidarScan& b) 
  {
      return !(a == b);
  }

  /** Lookup table of beam directions and offsets. */
  struct XYZLut
  {
    LidarScan::Points direction;
    LidarScan::Points offset;
  };

  /**
   * Generate a set of lookup tables useful for computing cartesian coordinates
   * from ranges.
   *
   * The lookup tables are:
   * - direction: a matrix of unit vectors pointing radially outwards
   * - offset: a matrix of offsets dependent on beam origin distance from lidar
   *           origin
   *
   * Each table is an n x 3 array of doubles stored in column-major order where
   * each row corresponds to the nth point in a lidar scan, with 0 <= n < h*w.
   *
   * @param w number of columns in the lidar scan. e.g. 512, 1024, or 2048
   * @param h number of rows in the lidar scan
   * @param range_unit the unit, in meters, of the range,  e.g. sensor::range_unit
   * @param lidar_origin_to_beam_origin_mm the radius to the beam origin point of
   *        the unit, in millimeters
   * @param transform additional transformation to apply to resulting points
   * @param azimuth_angles_deg azimuth offsets in degrees for each of h beams
   * @param altitude_angles_deg altitude in degrees for each of h beams
   * @return xyz direction and offset vectors for each point in the lidar scan
   */
  XYZLut make_xyz_lut(
    size_t w, 
    size_t h, 
    double range_unit,
    const mat4d& beam_to_lidar_transform,
    const mat4d & transform,
    const std::vector < double > & azimuth_angles_deg,
    const std::vector < double > & altitude_angles_deg);

  /**
   * Convenient overload that uses parameters from the supplied sensor_info.
   *
   * @param sensor metadata returned from the client
   * @return xyz direction and offset vectors for each point in the lidar scan
   */
  inline XYZLut make_xyz_lut(const sensor::sensor_info & sensor)
  {
    return make_xyz_lut(
            sensor.format.columns_per_frame, sensor.format.pixels_per_column,
            sensor::range_unit, sensor.beam_to_lidar_transform,
            sensor.lidar_to_sensor_transform, sensor.beam_azimuth_angles,
            sensor.beam_altitude_angles);
  }

  /**
 * Convert LidarScan to Cartesian points.
 *
 * @param[in] scan a LidarScan.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
  LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut);

  /**
 * Convert a staggered range image to Cartesian points.
 *
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarScan.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarScan where i = row * w + col.
 */
  LidarScan::Points cartesian(const Eigen::Ref<const img_t<uint32_t>>& range,
                              const XYZLut& lut);

  /**
   * Generate a destaggered version of a channel field.
   *
   * In the default staggered representation, each column corresponds to a single
   * timestamp. In the destaggered representation, each column corresponds to a
   * single azimuth angle, compensating for the azimuth offset of each beam.
   *
   * Destaggering is used for visualizing lidar data as an image or for algorithms
   * that exploit the structure of the lidar data, such as beam_uniformity in
   * ouster_viz, or computer vision algorithms.
   *
   * For example:
   *     destagger(lidarscan.field(Field::INTENSITY))
   *     destagger(lidarscan.field(Field::INTENSITY).cast<double>())
   *
   * @param img the channel field
   * @param pixel_shift_by_row offsets, usually queried from the sensor
   * @param inverse perform the inverse operation
   * @return destaggered version of the image
   */
  template < typename T >
  inline img_t < T > destagger(
    const Eigen::Ref < const img_t < T >> &img,
    const std::vector < int > &pixel_shift_by_row,
    bool inverse = false) {
    const size_t h = img.rows();
    const size_t w = img.cols();

    if (pixel_shift_by_row.size() != h) {
      throw std::invalid_argument {"image height does not match shifts size"};
    }

    img_t < T > destaggered {h, w};
    for (size_t u = 0; u < h; u++) {
      const std::ptrdiff_t offset =
        ((inverse ? -1 : 1) * pixel_shift_by_row[u] + w) % w;

      destaggered.row(u).segment(offset, w - offset) =
        img.row(u).segment(0, w - offset);
      destaggered.row(u).segment(0, offset) =
        img.row(u).segment(w - offset, offset);
    }
    return destaggered;
  }

  /**
   * Generate a staggered version of a channel field.
   *
   * @param img
   * @param pixel_shift_by_row
   * @return staggered version of the image
   */
  template < typename T >
  inline img_t < T > stagger(
    const Eigen::Ref < const img_t < T >> &img,
    const std::vector < int > &pixel_shift_by_row) {
    return destagger(img, pixel_shift_by_row, true);
  }

  /**
   * Parse lidar packets into a LidarScan.
   *
   * Make a function that batches a single scan (revolution) of data to a
   * LidarScan.
   */
  class ScanBatcher 
  {
    std::ptrdiff_t w;
    std::ptrdiff_t h;
    uint16_t next_valid_m_id;
    uint16_t next_headers_m_id;
    LidarScan ls_write;

  public:

    sensor::packet_format pf;

    /**
     * Create a batcher given information about the scan and packet format.
     *
     * @param w number of columns in the lidar scan. One of 512, 1024, or 2048
     * @param pf expected format of the incoming packets used for parsing
     * @param profile expected profile of the incoming packets used for parsing
     */
    ScanBatcher(size_t w, const sensor::packet_format & pf)
        : w(w), h(pf.pixels_per_column), next_valid_m_id(0), next_headers_m_id(0), pf(pf),
          ls_write(w, h, pf.udp_profile_lidar) {}

    /**
     * Create a batcher given information about the scan and packet format.
     *
     * @param[in] info sensor metadata returned from the client.
     */
    explicit ScanBatcher(const sensor::sensor_info& info)
        : ScanBatcher(info.format.columns_per_frame, sensor::get_format(info)) {}

    /**
     * Add a packet to the scan.
     *
     * @param packet_buf the lidar packet
     * @param lidar scan to populate
     * @return true when the provided lidar scan is ready to use
     */
    bool operator()(const uint8_t * packet_buf, LidarScan & ls);
  };

}  // namespace ouster
