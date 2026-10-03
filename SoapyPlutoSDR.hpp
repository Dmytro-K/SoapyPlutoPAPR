#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Types.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iio.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

typedef enum plutosdrStreamFormat
{
    PLUTO_SDR_CF32,
    PLUTO_SDR_CS16,
    PLUTO_SDR_CS12,
    PLUTO_SDR_CS8,
    PLUTO_SDR_CF32_TEZUKA,
    PLUTO_SDR_CS16_TEZUKA,
    PLUTO_SDR_CS12_TEZUKA,
    PLUTO_SDR_CS8_TEZUKA
} plutosdrStreamFormat;

typedef enum WireFormat
{
    WIRE_CS16,
    WIRE_CS8,
    WIRE_CS12
} WireFormat;

// CS12 wire format (Tezuka PL packing, see cs12_cs8mux.v / cs12_sync_frame.v)
static constexpr size_t CS12_BURST = 24;                      // 8 IQ samples
static constexpr uint32_t CS12_BURSTS_PER_SYNC = 262144 / 8;  // must match SYNC_PERIOD_SAMPLES

// RX data transport, device arg tezuka_transport=iio|udp.
// udp: IQ data comes from the board's iqnet service (zero-copy UDP, see
// iqnet_proto.h) instead of an IIO buffer; AD9361 control stays on libiio.
struct IqNetConfig
{
    bool enabled = false;       // tezuka_transport=udp
    std::string host;           // board IPv4 address or hostname (control + UDP source)
    uint16_t udp_port = 30432;  // tezuka_udp_port: local UDP port the board sends to
    size_t rcvbuf = 32 << 20;   // tezuka_udp_rcvbuf: requested SO_RCVBUF, bytes
    std::string start_options;  // " blocks=.. block_size=.. gso=.." appended to START
};

class rx_streamer
{
public:
    rx_streamer(const iio_device *dev, const plutosdrStreamFormat format,
                const WireFormat wire_format, const std::vector<size_t> &channels,
                const SoapySDR::Kwargs &args, const IqNetConfig &net = IqNetConfig());
    ~rx_streamer();
    size_t recv(void *const *buffs, const size_t numElems, int &flags, long long &timeNs,
                const long timeoutUs = 100000);
    int start(const int flags, const long long timeNs, const size_t numElems);

    int stop(const int flags, const long long timeNs = 100000);

    void set_buffer_size_by_samplerate(const size_t _samplerate);

    size_t get_mtu_size();

private:
    void set_buffer_size(const size_t _buffer_size, const size_t num_kernel);
    void set_mtu_size(const size_t mtu_size);

    bool has_direct_copy();

    std::vector<iio_channel *> channel_list;
    const iio_device *dev;

    size_t buffer_size;
    size_t byte_offset;
    size_t items_in_buffer;
    iio_buffer *buf;
    const plutosdrStreamFormat format;
    const WireFormat wire_format;
    bool direct_copy;
    size_t mtu_size;
    // bool UseExtendedTezukaFeatures=false;

    // CS12 wire format (Tezuka PL packing, see cs12_cs8mux.v / cs12_sync_frame.v)
    size_t recv_cs12(void *const *buffs, const size_t numElems);
    void decode_cs12(const uint8_t *d, const size_t len);
    void decode_cs12_burst(const uint8_t *b);
    void reset_cs12();
    size_t output_iq(void *dst, const size_t numElems);

    // wire CS16/CS8 items -> host format (shared by the IIO and UDP paths)
    void convert_direct(const uint8_t *src, void *dst, const size_t items);

    // UDP transport (PlutoSDR_IqNet.cpp)
    struct UdpRx;
    struct UdpRxDeleter
    {
        void operator()(UdpRx *p) const;
    };
    std::unique_ptr<UdpRx, UdpRxDeleter> udp;
    void udp_open(const IqNetConfig &net);
    int udp_start();
    void udp_stop();
    int udp_recv(void *const *buffs, const size_t numElems, const long timeoutUs);
    int udp_fill(const long long waitUs);
    void udp_gap_cs12(const uint8_t *&data, size_t &len, const uint64_t lost, const bool restart);
    size_t udp_mtu() const;

    std::vector<int16_t> iq;     // decoded I,Q,I,Q...
    size_t iq_pos = 0;           // read position, in IQ pairs
    std::vector<uint8_t> carry;  // undecoded tail of the previous refill
    std::vector<uint8_t> work;   // carry + current refill
    bool synced = false;
    uint32_t bursts_since_sync = 0;
    uint32_t last_counter = 0;
    bool overflow = false;
};

class tx_streamer
{

public:
    tx_streamer(const iio_device *dev, const plutosdrStreamFormat format,
                const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
    ~tx_streamer();
    int send(const void *const *buffs, const size_t numElems, int &flags, const long long timeNs,
             const long timeoutUs);
    int flush();
    void set_buffer_size_by_samplerate(const size_t _samplerate);
    size_t get_mtu_size();

private:
    int send_buf();
    bool has_direct_copy();
    void set_buffer_size(const size_t _buffer_size, const size_t num_kernel);
    void set_mtu_size(const size_t mtu_size);

    std::vector<iio_channel *> channel_list;
    const iio_device *dev;
    const plutosdrStreamFormat format;

    iio_buffer *buf;
    size_t buffer_size;
    size_t items_in_buffer = 0;
    bool direct_copy;
    size_t mtu_size;
};

// A local spin_mutex usable with std::lock_guard
// for lightweight locking for short periods.
class pluto_spin_mutex
{

public:
    pluto_spin_mutex() = default;

    pluto_spin_mutex(const pluto_spin_mutex &) = delete;

    pluto_spin_mutex &operator=(const pluto_spin_mutex &) = delete;

    ~pluto_spin_mutex()
    {
        lock_state.clear(std::memory_order_release);
    }

    void lock()
    {
        while (lock_state.test_and_set(std::memory_order_acquire))
            ;
    }

    void unlock()
    {
        lock_state.clear(std::memory_order_release);
    }

private:
    std::atomic_flag lock_state = ATOMIC_FLAG_INIT;
};

class SoapyPlutoSDR : public SoapySDR::Device
{

public:
    SoapyPlutoSDR(const SoapySDR::Kwargs &args);
    ~SoapyPlutoSDR();

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const;

    std::string getHardwareKey(void) const;

    SoapySDR::Kwargs getHardwareInfo(void) const;

    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels(const int) const;

    bool getFullDuplex(const int direction, const size_t channel) const;

    /*******************************************************************
     * Stream API
     ******************************************************************/

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

    std::string getNativeStreamFormat(const int direction, const size_t channel,
                                      double &fullScale) const;

    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                  const std::vector<size_t> &channels = std::vector<size_t>(),
                                  const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

    void closeStream(SoapySDR::Stream *stream);

    size_t getStreamMTU(SoapySDR::Stream *stream) const;

    int activateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0,
                       const size_t numElems = 0);

    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0,
                         const long long timeNs = 0);

    int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags,
                   long long &timeNs, const long timeoutUs = 100000);

    int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems,
                    int &flags, const long long timeNs = 0, const long timeoutUs = 100000);

    int readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs,
                         const long timeoutUs);

    /*******************************************************************
     * Sensor API
     ******************************************************************/

    std::vector<std::string> listSensors(void) const;

    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const;

    std::string readSensor(const std::string &key) const;

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const;

    void writeSetting(const std::string &key, const std::string &value);

    std::string readSetting(const std::string &key) const;

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;

    void setAntenna(const int direction, const size_t channel, const std::string &name);

    std::string getAntenna(const int direction, const size_t channel) const;

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/

    bool hasDCOffsetMode(const int direction, const size_t channel) const;

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(const int direction, const size_t channel) const;

    bool hasGainMode(const int direction, const size_t channel) const;

    void setGainMode(const int direction, const size_t channel, const bool automatic);

    bool getGainMode(const int direction, const size_t channel) const;

    void setGain(const int direction, const size_t channel, const double value);

    void setGain(const int direction, const size_t channel, const std::string &name,
                 const double value);

    double getGain(const int direction, const size_t channel, const std::string &name) const;

    SoapySDR::Range getGainRange(const int direction, const size_t channel,
                                 const std::string &name) const;

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency(const int direction, const size_t channel, const std::string &name,
                      const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

    double getFrequency(const int direction, const size_t channel, const std::string &name) const;

    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;

    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel,
                                          const std::string &name) const;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/

    void setSampleRate(const int direction, const size_t channel, const double rate);

    double getSampleRate(const int direction, const size_t channel) const;

    std::vector<double> listSampleRates(const int direction, const size_t channel) const;

    void setBandwidth(const int direction, const size_t channel, const double bw);

    double getBandwidth(const int direction, const size_t channel) const;

    std::vector<double> listBandwidths(const int direction, const size_t channel) const;

    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;

private:
    bool IsValidRxStreamHandle(SoapySDR::Stream *handle) const;
    bool IsValidTxStreamHandle(SoapySDR::Stream *handle) const;

    bool is_sensor_channel(struct iio_channel *chn) const;
    double double_from_buf(const char *buf) const;
    double get_sensor_value(struct iio_channel *chn) const;
    std::string id_to_unit(const std::string &id) const;
    void parse_iqnet_args(const SoapySDR::Kwargs &args);

    iio_device *dev;
    iio_device *rx_dev;
    iio_device *tx_dev;
    bool gainMode;

    mutable pluto_spin_mutex rx_device_mutex;
    mutable pluto_spin_mutex tx_device_mutex;

    bool decimation, interpolation;
    std::unique_ptr<rx_streamer> rx_stream;
    std::unique_ptr<tx_streamer> tx_stream;
    bool UseExtendedTezukaFeatures = false;
    WireFormat wire_format = WIRE_CS16;
    IqNetConfig iqnet;
};
