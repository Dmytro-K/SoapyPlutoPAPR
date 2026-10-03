#include "SoapyPlutoSDR.hpp"
#include "iqnet_proto.h"
#include <cstring>
#ifdef HAS_AD9361_IIO
#include <ad9361.h>
#endif

static iio_context *ctx = nullptr;

SoapyPlutoSDR::SoapyPlutoSDR(const SoapySDR::Kwargs &args)
    : dev(nullptr), rx_dev(nullptr), tx_dev(nullptr), decimation(false), interpolation(false),
      rx_stream(nullptr)
{

    gainMode = false;

    if (args.count("label") != 0)
        SoapySDR_logf(SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());
    fprintf(stderr, "Using URI %s\n", args.at("uri").c_str());
    if (ctx == nullptr)
    {
        if (args.count("uri") != 0)
        {

            ctx = iio_create_context_from_uri(args.at("uri").c_str());
            fprintf(stderr, "Using URI %s\n", args.at("uri").c_str());
        }
        else if (args.count("hostname") != 0)
        {
            ctx = iio_create_network_context(args.at("hostname").c_str());
        }
        else
        {
            ctx = iio_create_default_context();
        }
    }

    if (ctx == nullptr)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "no device context found.");
        throw std::runtime_error("no device context found");
    }

    dev = iio_context_find_device(ctx, "ad9361-phy");
    rx_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    tx_dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");

    if (dev == nullptr || rx_dev == nullptr || tx_dev == nullptr)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "no device found in this context.");
        throw std::runtime_error("no device found in this context");
    }

    this->setAntenna(SOAPY_SDR_RX, 0, "A_BALANCED");
    this->setGainMode(SOAPY_SDR_RX, 0, false);
    this->setAntenna(SOAPY_SDR_TX, 0, "A");
    SoapySDR::Kwargs test = getHardwareInfo();
    SoapySDR::Kwargs::iterator it;
    if (test.count("fw_version") != 0)
    {
        // fprintf(stderr,"fw_version  %s\n", test.at("fw_version").c_str());
        if (strncmp(test.at("fw_version").c_str(), "tezuka", strlen("tezuka")) == 0)
        {
            fprintf(stderr, "Tezuka firmware\n");
            UseExtendedTezukaFeatures = true;
        }
        else
        {
            fprintf(stderr, "Standard firmware\n");
            UseExtendedTezukaFeatures = false;
        }

        UseExtendedTezukaFeatures = false;
        wire_format = WireFormat::WIRE_CS16;

        if (args.count("tezuka_format") != 0)
        {
            if (strncmp(args.at("tezuka_format").c_str(), "CS8", strlen("CS8")) == 0)
            {
                UseExtendedTezukaFeatures = true;
                wire_format = WireFormat::WIRE_CS8;
            }
            else if (strncmp(args.at("tezuka_format").c_str(), "CS12", strlen("CS12")) == 0)
            {
                UseExtendedTezukaFeatures = true;
                wire_format = WireFormat::WIRE_CS12;
            }
        }
    }

    try
    {
        parse_iqnet_args(args);
    }
    catch (...)
    {
        iio_context_destroy(ctx);
        ctx = nullptr;
        throw;
    }
}

// Board address for the iqnet control/data connection: the host part of an
// "ip:<host>[:<port>]" URI, the hostname arg, or the address libiio resolved.
static std::string iqnet_host_from_uri(const std::string &uri)
{
    if (uri.compare(0, 3, "ip:") != 0)
        return "";
    std::string host = uri.substr(3);
    if (!host.empty() && host[0] == '[')
    {
        const size_t end = host.find(']');
        return end == std::string::npos ? "" : host.substr(1, end - 1);
    }
    const size_t colon = host.find(':');
    if (colon != std::string::npos && host.find(':', colon + 1) == std::string::npos)
        host.resize(colon);  // strip ":port" (more than one ':' = bare IPv6 literal)
    return host;
}

// upper bound for tezuka_udp_block_size (one IIO DMA block)
static constexpr unsigned long IQNET_MAX_BLOCK_SIZE = 16ul << 20;

static unsigned long iqnet_arg_ulong(const SoapySDR::Kwargs &args, const char *key,
                                     const unsigned long min, const unsigned long max)
{
    const std::string &value = args.at(key);
    unsigned long v = 0;
    size_t used = 0;
    try
    {
        v = std::stoul(value, &used, 0);
    }
    catch (const std::exception &)
    {
        used = 0;
    }
    if (used == 0 || used != value.size() || v < min || v > max)
        throw std::runtime_error(std::string("invalid ") + key + "=" + value);
    return v;
}

void SoapyPlutoSDR::parse_iqnet_args(const SoapySDR::Kwargs &args)
{
    iqnet = IqNetConfig();

    const std::string transport =
        args.count("tezuka_transport") != 0 ? args.at("tezuka_transport") : "iio";
    if (transport == "iio")
        return;
    if (transport != "udp")
        throw std::runtime_error("invalid tezuka_transport=" + transport + " (iio or udp)");

    iqnet.enabled = true;

    if (args.count("tezuka_udp_host") != 0)
        iqnet.host = args.at("tezuka_udp_host");
    if (iqnet.host.empty() && args.count("uri") != 0)
        iqnet.host = iqnet_host_from_uri(args.at("uri"));
    if (iqnet.host.empty() && args.count("hostname") != 0)
        iqnet.host = args.at("hostname");
    if (iqnet.host.empty())
    {
        const char *ip = iio_context_get_attr_value(ctx, "ip,ip-addr");
        if (ip != nullptr)
            iqnet.host = ip;
    }
    if (iqnet.host.empty())
        throw std::runtime_error("tezuka_transport=udp needs a network device: use uri=ip:<host>, "
                                 "hostname=<host> or tezuka_udp_host=<host>");

    if (args.count("tezuka_udp_port") != 0)
        iqnet.udp_port = uint16_t(iqnet_arg_ulong(args, "tezuka_udp_port", 1, 65535));
    if (args.count("tezuka_udp_rcvbuf") != 0)
        iqnet.rcvbuf = iqnet_arg_ulong(args, "tezuka_udp_rcvbuf", 65536, 1ul << 30);

    // optional START parameters, passed through to the board (it validates them too)
    static const struct
    {
        const char *arg, *key;
        unsigned long min, max;
    } start_keys[] = {{"tezuka_udp_blocks", "blocks", 1, IQNET_MAX_BLOCKS},
                      {"tezuka_udp_block_size", "block_size", 0, 0xFFFFFFFFul},
                      {"tezuka_udp_gso", "gso", 0, IQNET_MAX_GSO}};
    for (const auto &k : start_keys)
    {
        if (args.count(k.arg) == 0)
            continue;
        const unsigned long v = iqnet_arg_ulong(args, k.arg, k.min, k.max);
        // the block must hold whole datagrams and stay within what the board can allocate
        if (std::string(k.arg) == "tezuka_udp_block_size" &&
            (v == 0 || v % IQNET_DEFAULT_PAYLOAD != 0 || v > IQNET_MAX_BLOCK_SIZE))
            throw std::runtime_error(
                "invalid tezuka_udp_block_size=" + args.at(k.arg) +
                ": must be a non-zero multiple of " + std::to_string(IQNET_DEFAULT_PAYLOAD) +
                " bytes and at most " + std::to_string(IQNET_MAX_BLOCK_SIZE) + " (16 MiB)");
        iqnet.start_options += std::string(" ") + k.key + "=" + std::to_string(v);
    }

    SoapySDR_logf(SOAPY_SDR_INFO, "RX transport: UDP from %s, local port %u%s", iqnet.host.c_str(),
                  (unsigned)iqnet.udp_port, iqnet.start_options.c_str());
}

SoapyPlutoSDR::~SoapyPlutoSDR(void)
{

    long long samplerate = 0;
    if (decimation)
    {
        iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),
                                       "sampling_frequency", &samplerate);
        iio_channel_attr_write_longlong(iio_device_find_channel(rx_dev, "voltage0", false),
                                        "sampling_frequency", samplerate);
    }

    if (interpolation)
    {
        iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),
                                       "sampling_frequency", &samplerate);
        iio_channel_attr_write_longlong(iio_device_find_channel(tx_dev, "voltage0", true),
                                        "sampling_frequency", samplerate);
    }

    if (ctx)
    {
        iio_context_destroy(ctx);
        ctx = nullptr;
    }
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyPlutoSDR::getDriverKey(void) const
{
    return "PlutoSDR";
}

std::string SoapyPlutoSDR::getHardwareKey(void) const
{
    return "ADALM-PLUTO";
}

SoapySDR::Kwargs SoapyPlutoSDR::getHardwareInfo(void) const
{
    SoapySDR::Kwargs info;

    unsigned int major, minor;
    char git_tag[8];
    iio_library_get_version(&major, &minor, git_tag);
    char lib_ver[100];
    snprintf(lib_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
    info["library_version"] = lib_ver;

    iio_context_get_version(ctx, &major, &minor, git_tag);
    char backend_ver[100];
    snprintf(backend_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
    info["backend_version"] = backend_ver;

    unsigned int nb_ctx_attrs = iio_context_get_attrs_count(ctx);
    for (unsigned int i = 0; i < nb_ctx_attrs; i++)
    {
        const char *key, *value;
        iio_context_get_attr(ctx, i, &key, &value);
        info[key] = value;
    }

    return info;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyPlutoSDR::getNumChannels(const int dir) const
{
    return (1);
}

bool SoapyPlutoSDR::getFullDuplex(const int direction, const size_t channel) const
{
    return (true);
}

/*******************************************************************
 * Sensor API
 ******************************************************************/

bool SoapyPlutoSDR::is_sensor_channel(struct iio_channel *chn) const
{
    return (!iio_channel_is_output(chn) &&
            (iio_channel_find_attr(chn, "raw") || iio_channel_find_attr(chn, "input")));
}

double SoapyPlutoSDR::double_from_buf(const char *buf) const
{
    std::istringstream val_as_string(buf);
    val_as_string.imbue(std::locale::classic());  // ignore global C++ locale

    double val = 0.0;
    val_as_string >> val;

    return val;
}

double SoapyPlutoSDR::get_sensor_value(struct iio_channel *chn) const
{
    char buf[32];
    double val = 0.0;

    if (iio_channel_find_attr(chn, "input"))
    {
        if (iio_channel_attr_read(chn, "input", buf, sizeof(buf)) > 0)
        {
            val = double_from_buf(buf);
        }
    }
    else
    {
        if (iio_channel_attr_read(chn, "raw", buf, sizeof(buf)) > 0)
        {
            val = double_from_buf(buf);
        }

        if (iio_channel_find_attr(chn, "offset"))
        {
            if (iio_channel_attr_read(chn, "offset", buf, sizeof(buf)) > 0)
            {
                val += double_from_buf(buf);
            }
        }

        if (iio_channel_find_attr(chn, "scale"))
        {
            if (iio_channel_attr_read(chn, "scale", buf, sizeof(buf)) > 0)
            {
                val *= double_from_buf(buf);
            }
        }
    }

    return val / 1000.0;
}

std::string SoapyPlutoSDR::id_to_unit(const std::string &id) const
{
    static std::map<std::string, std::string> id_to_unit_table = {
        {"current", "A"},
        {"power", "W"},
        {"temp", "C"},
        {"voltage", "V"},
    };

    for (auto it_match : id_to_unit_table)
    {

        // if the id starts with a known prefix, retreive its unit.
        if (id.substr(0, it_match.first.size()) == it_match.first)
        {
            return it_match.second;
        }
    }
    return std::string();
}

std::vector<std::string> SoapyPlutoSDR::listSensors(void) const
{
    /*
    iio:device2: xadc
        10 channels found:
            temp0:  (input)
            voltage0: vccint (input)
            voltage1: vccaux (input)
            voltage2: vccbram (input)
            voltage3: vccpint (input)
            voltage4: vccpaux (input)
            voltage5: vccoddr (input)
            voltage6: vrefp (input)
            voltage7: vrefn (input)
            voltage8:  (input)
    iio:device0: adm1177
        2 channels found:
            current0:  (input)
            voltage0:  (input)
    iio:device1: ad9361-phy
        9 channels found:
            temp0:  (input)
            voltage2:  (input)
    */
    std::vector<std::string> sensors;

    sensors.push_back("xadc_temp0");
    sensors.push_back("xadc_voltage0");
    sensors.push_back("xadc_voltage1");
    sensors.push_back("xadc_voltage2");
    sensors.push_back("xadc_voltage3");
    sensors.push_back("xadc_voltage4");
    sensors.push_back("xadc_voltage5");
    sensors.push_back("xadc_voltage6");
    sensors.push_back("xadc_voltage7");
    sensors.push_back("xadc_voltage8");
    sensors.push_back("adm1177_current0");
    sensors.push_back("adm1177_voltage0");
    sensors.push_back("ad9361-phy_temp0");
    sensors.push_back("ad9361-phy_voltage2");

    return sensors;
}

SoapySDR::ArgInfo SoapyPlutoSDR::getSensorInfo(const std::string &key) const
{
    SoapySDR::ArgInfo info;

    std::size_t dash = key.find("_");
    if (dash < std::string::npos)
    {
        std::string deviceStr = key.substr(0, dash);
        std::string channelStr = key.substr(dash + 1);

        iio_device *dev = iio_context_find_device(ctx, deviceStr.c_str());
        if (!dev)
            return info;
        iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
        if (!chn)
            return info;

        const char *name = iio_channel_get_name(chn);
        info.key = key;
        if (name)
            info.name = name;
        info.type = SoapySDR::ArgInfo::FLOAT;
        info.value = "0.0";
        info.units = id_to_unit(channelStr);
    }

    return info;
}

std::string SoapyPlutoSDR::readSensor(const std::string &key) const
{
    std::string sensorValue;

    std::size_t dash = key.find("_");
    if (dash < std::string::npos)
    {
        std::string deviceStr = key.substr(0, dash);
        std::string channelStr = key.substr(dash + 1);

        iio_device *dev = iio_context_find_device(ctx, deviceStr.c_str());
        if (!dev)
            return sensorValue;
        iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
        if (!chn)
            return sensorValue;

        double value = get_sensor_value(chn);
        sensorValue.assign(std::to_string(value));
    }

    return sensorValue;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyPlutoSDR::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    return setArgs;
}

void SoapyPlutoSDR::writeSetting(const std::string &key, const std::string &value)
{
}

std::string SoapyPlutoSDR::readSetting(const std::string &key) const
{
    std::string info;

    return info;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyPlutoSDR::listAntennas(const int direction,
                                                     const size_t channel) const
{
    std::vector<std::string> options;
    if (direction == SOAPY_SDR_RX)
        options.push_back("A_BALANCED");
    if (direction == SOAPY_SDR_TX)
        options.push_back("A");
    return (options);
}

void SoapyPlutoSDR::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    if (direction == SOAPY_SDR_RX)
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
        iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false), "rf_port_select",
                               name.c_str());
    }

    else if (direction == SOAPY_SDR_TX)
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
        iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", true), "rf_port_select",
                               name.c_str());
    }
}

std::string SoapyPlutoSDR::getAntenna(const int direction, const size_t channel) const
{
    std::string options;

    if (direction == SOAPY_SDR_RX)
    {
        options = "A_BALANCED";
    }
    else if (direction == SOAPY_SDR_TX)
    {

        options = "A";
    }
    return options;
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyPlutoSDR::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return (false);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyPlutoSDR::listGains(const int direction, const size_t channel) const
{
    std::vector<std::string> options;
    options.push_back("PGA");
    return (options);
}

bool SoapyPlutoSDR::hasGainMode(const int direction, const size_t channel) const
{
    if (direction == SOAPY_SDR_RX)
        return true;
    return false;
}

void SoapyPlutoSDR::setGainMode(const int direction, const size_t channel, const bool automatic)
{

    gainMode = automatic;
    if (direction == SOAPY_SDR_RX)
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
        if (gainMode)
        {

            iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false),
                                   "gain_control_mode", "slow_attack");
        }
        else
        {

            iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false),
                                   "gain_control_mode", "manual");
        }
    }
}

bool SoapyPlutoSDR::getGainMode(const int direction, const size_t channel) const
{
    return gainMode;
}

void SoapyPlutoSDR::setGain(const int direction, const size_t channel, const double value)
{
    long long gain = (long long)value;
    if (direction == SOAPY_SDR_RX)
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),
                                        "hardwaregain", gain);
    }

    else if (direction == SOAPY_SDR_TX)
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
        gain = gain - 89;
        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),
                                        "hardwaregain", gain);
    }
}

void SoapyPlutoSDR::setGain(const int direction, const size_t channel, const std::string &name,
                            const double value)
{
    this->setGain(direction, channel, value);
}

double SoapyPlutoSDR::getGain(const int direction, const size_t channel,
                              const std::string &name) const
{
    long long gain = 0;

    if (direction == SOAPY_SDR_RX)
    {

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),
                                           "hardwaregain", &gain) != 0)
            return 0;
    }

    else if (direction == SOAPY_SDR_TX)
    {

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),
                                           "hardwaregain", &gain) != 0)
            return 0;
        gain = gain + 89;
    }
    return double(gain);
}

SoapySDR::Range SoapyPlutoSDR::getGainRange(const int direction, const size_t channel,
                                            const std::string &name) const
{
    if (direction == SOAPY_SDR_RX)
        return (SoapySDR::Range(0, 71));
    return (SoapySDR::Range(0, 89));
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyPlutoSDR::setFrequency(const int direction, const size_t channel,
                                 const std::string &name, const double frequency,
                                 const SoapySDR::Kwargs &args)
{
    long long freq = (long long)frequency;
    if (direction == SOAPY_SDR_RX)
    {

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "altvoltage0", true),
                                        "frequency", freq);
    }

    else if (direction == SOAPY_SDR_TX)
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "altvoltage1", true),
                                        "frequency", freq);
    }
}

double SoapyPlutoSDR::getFrequency(const int direction, const size_t channel,
                                   const std::string &name) const
{
    long long freq = 0;

    if (direction == SOAPY_SDR_RX)
    {

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(dev, "altvoltage0", true),
                                           "frequency", &freq) != 0)
            return 0;
    }

    else if (direction == SOAPY_SDR_TX)
    {

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(dev, "altvoltage1", true),
                                           "frequency", &freq) != 0)
            return 0;
    }

    return double(freq);
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getFrequencyArgsInfo(const int direction,
                                                          const size_t channel) const
{

    SoapySDR::ArgInfoList freqArgs;

    return freqArgs;
}

std::vector<std::string> SoapyPlutoSDR::listFrequencies(const int direction,
                                                        const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    return (names);
}

SoapySDR::RangeList SoapyPlutoSDR::getFrequencyRange(const int direction, const size_t channel,
                                                     const std::string &name) const
{
    return (SoapySDR::RangeList(1, SoapySDR::Range(46000000, 6000000000ull)));
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/
void SoapyPlutoSDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
    long long samplerate = (long long)rate;
#ifdef HAS_AD9361_IIO
    int const fir = 4;  // assume ad9361_set_bb_rate() will load x4 FIR if needed
#else
    int const fir = 1;
#endif

    // note: sample rates below 25e6/12 need x8 decimation/interpolation or x4 FIR to 25e6/48,
    // below 25e6/96 need x8 decimation/interpolation and x4 FIR, minimum is 25e6/384
    // if libad9361 is available it will load an approporiate FIR.
    if (direction == SOAPY_SDR_RX)
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
        decimation = false;
        if (samplerate < (25e6 / (12 * fir)))
        {
            if (samplerate * 8 < (25e6 / 48))
            {
                SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
            }
            else if (samplerate * 8 < (25e6 / 12))
            {
                SoapySDR_logf(SOAPY_SDR_NOTICE, "sample rate needs a FIR setting loaded.");
            }

            decimation = true;
            samplerate = samplerate * 8;
        }

        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),
                                        "sampling_frequency", samplerate);

#ifdef HAS_AD9361_IIO
        if (ad9361_set_bb_rate(dev, (unsigned long)samplerate))
            SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to set BB rate.");
#endif

        // FPGA data port rate must be set AFTER ad9361_set_bb_rate() which
        // reconfigures the entire decimation chain and resets the FPGA rate.
        iio_channel_attr_write_longlong(iio_device_find_channel(rx_dev, "voltage0", false),
                                        "sampling_frequency",
                                        decimation ? samplerate / 8 : samplerate);

        if (rx_stream)
            rx_stream->set_buffer_size_by_samplerate(decimation ? samplerate / 8 : samplerate);
    }

    else if (direction == SOAPY_SDR_TX)
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
        interpolation = false;
        if (samplerate < (25e6 / (12 * fir)))
        {
            if (samplerate * 8 < (25e6 / 48))
            {
                SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
            }
            else if (samplerate * 8 < (25e6 / 12))
            {
                SoapySDR_logf(SOAPY_SDR_NOTICE, "sample rate needs a FIR setting loaded.");
            }

            interpolation = true;
            samplerate = samplerate * 8;
        }

        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),
                                        "sampling_frequency", samplerate);

#ifdef HAS_AD9361_IIO
        if (ad9361_set_bb_rate(dev, (unsigned long)samplerate))
            SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to set BB rate.");
#endif

        // FPGA data port rate must be set AFTER ad9361_set_bb_rate()
        iio_channel_attr_write_longlong(iio_device_find_channel(tx_dev, "voltage0", true),
                                        "sampling_frequency",
                                        interpolation ? samplerate / 8 : samplerate);
    }
}

double SoapyPlutoSDR::getSampleRate(const int direction, const size_t channel) const
{
    long long samplerate = 0;

    if (direction == SOAPY_SDR_RX)
    {

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(rx_dev, "voltage0", false),
                                           "sampling_frequency", &samplerate) != 0)
            return 0;
    }

    else if (direction == SOAPY_SDR_TX)
    {

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(tx_dev, "voltage0", true),
                                           "sampling_frequency", &samplerate) != 0)
            return 0;
    }

    return double(samplerate);
}

std::vector<double> SoapyPlutoSDR::listSampleRates(const int direction, const size_t channel) const
{
    std::vector<double> options;

    for (int i = 0; i < 61; i++)
        options.push_back(i * 1e6);
    /*
        options.push_back(65105);//25M/48/8+1
        options.push_back(1e6);
        options.push_back(2e6);
        options.push_back(3e6);
        options.push_back(4e6);
        options.push_back(5e6);
        options.push_back(6e6);
        options.push_back(7e6);
        options.push_back(8e6);
        options.push_back(9e6);
        options.push_back(10e6);
        options.push_back(12e6);
        options.push_back(15e6);
        options.push_back(50e6);*/
    return (options);
}

SoapySDR::RangeList SoapyPlutoSDR::getSampleRateRange(const int direction,
                                                      const size_t channel) const
{
    SoapySDR::RangeList results;

    // note that there are some gaps and rounding errors since we get truncated values form IIO
    // e.g. 25e6/12 = 2083333.333 is read as 2083333 but written as 2083334
#ifdef HAS_AD9361_IIO
    // assume ad9361_set_bb_rate(), if available, will load x4 FIR as needed
    // below 25e6/96 needs x8 decimation/interpolation and x4 FIR, minimum is 25e6/384
    results.push_back(SoapySDR::Range(25e6 / 384, 61440000));
#else
    // sample rates below 25e6/12 need x8 decimation/interpolation (or x4 FIR to 25e6/48)
    results.push_back(SoapySDR::Range(25e6 / 96, 61440000));
#endif

    return results;
}

void SoapyPlutoSDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    long long bandwidth = (long long)bw;
    if (direction == SOAPY_SDR_RX)
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),
                                        "rf_bandwidth", bandwidth);
    }

    else if (direction == SOAPY_SDR_TX)
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
        iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),
                                        "rf_bandwidth", bandwidth);
    }
}

double SoapyPlutoSDR::getBandwidth(const int direction, const size_t channel) const
{
    long long bandwidth = 0;

    if (direction == SOAPY_SDR_RX)
    {
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),
                                           "rf_bandwidth", &bandwidth) != 0)
            return 0;
    }

    else if (direction == SOAPY_SDR_TX)
    {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

        if (iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),
                                           "rf_bandwidth", &bandwidth) != 0)
            return 0;
    }

    return double(bandwidth);
}

std::vector<double> SoapyPlutoSDR::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> options;
    // options.push_back(0.2e6);
    for (int i = 0; i < 56; i++)
        options.push_back(i * 1e6);
    /*
options.push_back(1e6);
options.push_back(2e6);
options.push_back(3e6);
options.push_back(4e6);
options.push_back(5e6);
options.push_back(6e6);
options.push_back(7e6);
options.push_back(8e6);
options.push_back(9e6);
options.push_back(10e6);
options.push_back(12e6);
options.push_back(15e6);
*/
    return (options);
}
