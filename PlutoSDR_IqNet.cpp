// UDP RX transport (device arg tezuka_transport=udp).
//
// The board's iqnet service streams the raw IIO DMA bytes (CS12/CS8/CS16 exactly
// as the PL wrote them) as UDP datagrams to this host, see iqnet_proto.h:
//   [struct iqnet_hdr: magic, seq, u64 stream offset][payload, multiple of 24 bytes]
// A TCP connection to IQNET_CTRL_PORT starts the stream (START), stops it (STOP,
// or by closing it) and reports board statistics (STATS).
//
// Loss handling: the stream offset in every header tells exactly how many bytes
// were lost. Datagrams lost in the network or the socket buffer show up as a seq
// and offset jump; bytes the board itself skipped (DMA overflow, dropped short
// block) as an offset jump with contiguous seq. Both are handled the same way:
// each gap is reported once as SOAPY_SDR_OVERFLOW, at the point of the stream
// where it happened (data received before the gap is delivered first).
// seq == 0 && offset == 0 after data was received marks a new stream started by
// the board: it is reported as an overflow too, and the decoder starts over.
// For CS12 the decoder does not have to search the sync magic again after a gap:
// payload_len is a multiple of the 24 byte burst, so every datagram starts at the
// same burst phase and the lost byte count is a whole number of bursts. The
// decoder position inside the 32768 burst sync period is advanced by that count
// (and the expected sync counter by the number of skipped sync bursts); the next
// sync burst is still checked, so an inconsistency falls back to the magic search.

#include "SoapyPlutoSDR.hpp"
#include "iqnet_proto.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static_assert(sizeof(iqnet_hdr) == IQNET_HDR_LEN, "iqnet_hdr must be 16 bytes");
static_assert(IQNET_BURST == CS12_BURST, "iqnet payload granularity must be one CS12 burst");

namespace
{

typedef std::chrono::steady_clock clk;

constexpr unsigned UDP_BATCH = 64;  // datagrams per recvmmsg()
constexpr size_t UDP_SLOT = 2048;   // > IQNET_HDR_LEN + IQNET_MAX_PAYLOAD, bigger = MSG_TRUNC
constexpr int CTRL_CONNECT_MS = 3000;
constexpr int CTRL_START_MS = 10000;  // the board allocates and enables the IIO DMA blocks
constexpr int CTRL_STOP_MS = 12000;   // STOP returns after every in-flight block is back
                                      // (the board may take up to 10 s)
constexpr int CTRL_STATS_MS = 1000;
// a datagram older than this (bytes before the expected offset) is not reordering but a
// different stream (e.g. stale datagrams of the previous START): resynchronise to it
constexpr uint64_t REORDER_WINDOW = 4u << 20;

#ifdef __linux__
typedef struct mmsghdr udp_mmsg;
#else
struct udp_mmsg
{
    struct msghdr msg_hdr;
    unsigned int msg_len;
};
#endif

uint32_t get_le32(const uint8_t *p)
{
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}

uint64_t get_le64(const uint8_t *p)
{
    return uint64_t(get_le32(p)) | uint64_t(get_le32(p + 4)) << 32;
}

void set_cloexec(const int fd)
{
    const int fl = fcntl(fd, F_GETFD);
    if (fl >= 0)
        fcntl(fd, F_SETFD, fl | FD_CLOEXEC);
}

std::string addr_str(const in_addr &a)
{
    char s[INET_ADDRSTRLEN] = "?";
    inet_ntop(AF_INET, &a, s, sizeof(s));
    return s;
}

int ms_until(const clk::time_point &deadline)
{
    const long long ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clk::now()).count();
    return ms <= 0 ? 0 : int(std::min<long long>(ms, INT_MAX));
}

// bytes per output element of the host stream format
size_t host_elem_size(const plutosdrStreamFormat f)
{
    switch (f)
    {
        case PLUTO_SDR_CF32:
        case PLUTO_SDR_CF32_TEZUKA:
            return 2 * sizeof(float);
        case PLUTO_SDR_CS16:
        case PLUTO_SDR_CS16_TEZUKA:
            return 2 * sizeof(int16_t);
        case PLUTO_SDR_CS12:
        case PLUTO_SDR_CS12_TEZUKA:
            return 3;
        case PLUTO_SDR_CS8:
        case PLUTO_SDR_CS8_TEZUKA:
            return 2 * sizeof(int8_t);
    }
    return 2 * sizeof(int16_t);
}

const char *wire_mode(const WireFormat w)
{
    return w == WIRE_CS12 ? "cs12" : w == WIRE_CS8 ? "cs8" : "cs16";
}

// Connect the control socket (IPv4 only: the board sends UDP to the IPv4 TCP peer).
int ctrl_connect(const std::string &host, in_addr &peer, std::string &err)
{
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *res = nullptr;
    const std::string port = std::to_string(IQNET_CTRL_PORT);
    const int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
    if (rc != 0)
    {
        err = "cannot resolve " + host + ": " + gai_strerror(rc);
        return -1;
    }

    int fd = -1;
    for (const struct addrinfo *ai = res; ai != nullptr; ai = ai->ai_next)
    {
        const in_addr a = ((const struct sockaddr_in *)ai->ai_addr)->sin_addr;
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0)
        {
            err = std::string("socket: ") + strerror(errno);
            continue;
        }
        set_cloexec(fd);
        const int fl = fcntl(fd, F_GETFL);
        fcntl(fd, F_SETFL, fl | O_NONBLOCK);

        int r = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (r != 0 && errno == EINPROGRESS)
        {
            struct pollfd p = {fd, POLLOUT, 0};
            r = poll(&p, 1, CTRL_CONNECT_MS);
            if (r == 0)
            {
                errno = ETIMEDOUT;
                r = -1;
            }
            else if (r > 0)
            {
                int e = 0;
                socklen_t l = sizeof(e);
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &e, &l);
                errno = e;
                r = e == 0 ? 0 : -1;
            }
        }
        if (r == 0)
        {
            fcntl(fd, F_SETFL, fl);
            const int one = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
#ifdef SO_NOSIGPIPE
            setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
            peer = a;
            break;
        }
        err = "connect " + addr_str(a) + ":" + port + ": " + strerror(errno);
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    return fd;
}

bool send_all(const int fd, const std::string &s)
{
    size_t off = 0;
    while (off < s.size())
    {
        const ssize_t r = send(fd, s.data() + off, s.size() - off, MSG_NOSIGNAL);
        if (r < 0 && errno == EINTR)
            continue;
        if (r <= 0)
            return false;
        off += size_t(r);
    }
    return true;
}

// pop one complete line (without "\r\n") from buf
bool pop_line(std::string &buf, std::string &line)
{
    const size_t nl = buf.find('\n');
    if (nl == std::string::npos)
        return false;
    line = buf.substr(0, nl);
    buf.erase(0, nl + 1);
    if (!line.empty() && line[line.size() - 1] == '\r')
        line.resize(line.size() - 1);
    return true;
}

// Value of `key` in a "STATS key=value key=value ..." line. Tolerant: unknown keys,
// tokens without '=' and non-numeric values are skipped.
bool stats_value(const std::string &line, const char *key, unsigned long long &v)
{
    const size_t kl = strlen(key);
    size_t p = line.find(' ');
    while (p != std::string::npos)
    {
        p = line.find_first_not_of(" \t", p);
        if (p == std::string::npos)
            break;
        const size_t e = std::min(line.find_first_of(" \t", p), line.size());
        if (e - p > kl + 1 && line.compare(p, kl, key) == 0 && line[p + kl] == '=')
        {
            const std::string val = line.substr(p + kl + 1, e - p - kl - 1);
            char *end = nullptr;
            errno = 0;
            const unsigned long long r = strtoull(val.c_str(), &end, 10);
            if (errno == 0 && end != nullptr && *end == 0 && val[0] >= '0' && val[0] <= '9')
            {
                v = r;
                return true;
            }
        }
        p = e;
    }
    return false;
}

}  // namespace

struct rx_streamer::UdpRx
{
    IqNetConfig cfg;
    int udp_fd = -1;
    int ctrl_fd = -1;
    in_addr board = {};   // TCP peer address = expected UDP source address
    std::string ctrl_in;  // control bytes received but not consumed yet
    bool started = false;
    size_t payload_len = 0;
    size_t block_size = 0;

    // one recvmmsg() batch
    std::vector<uint8_t> slab;
    std::vector<udp_mmsg> msgs;
    std::vector<struct iovec> iov;
    std::vector<struct sockaddr_in> from;

    // accepted datagrams of the current batch, consumed in order by udp_recv()
    struct Packet
    {
        const uint8_t *data;
        size_t len;
        uint64_t lost;  // bytes missing before this datagram
        bool gap;       // report SOAPY_SDR_OVERFLOW before this datagram
        bool restart;   // discontinuity of unknown size (new stream)
    };
    std::vector<Packet> pkts;
    size_t pkt_idx = 0;
    size_t pkt_pos = 0;  // bytes of pkts[pkt_idx] already delivered (CS16/CS8)

    // stream position: offset/seq expected in the next datagram
    uint64_t next_offset = 0;
    uint32_t next_seq = 0;

    // statistics since START
    uint64_t datagrams = 0, bytes = 0, gaps = 0, lost_datagrams = 0, lost_bytes = 0;
    uint64_t board_gaps = 0, board_lost_bytes = 0;
    uint64_t late = 0, bad = 0, foreign = 0, restarts = 0;

    void close_ctrl()
    {
        if (ctrl_fd >= 0)
            close(ctrl_fd);
        ctrl_fd = -1;
        ctrl_in.clear();
    }

    // read one reply line from the control socket
    bool read_line(const int timeout_ms, std::string &line)
    {
        const clk::time_point deadline = clk::now() + std::chrono::milliseconds(timeout_ms);
        while (!pop_line(ctrl_in, line))
        {
            if (ctrl_fd < 0)
                return false;
            struct pollfd p = {ctrl_fd, POLLIN, 0};
            const int r = poll(&p, 1, ms_until(deadline));
            if (r < 0 && errno == EINTR)
                continue;
            if (r <= 0)
                return false;
            char tmp[512];
            const ssize_t n = ::recv(ctrl_fd, tmp, sizeof(tmp), 0);
            if (n < 0 && errno == EINTR)
                continue;
            if (n <= 0)
                return false;
            ctrl_in.append(tmp, size_t(n));
        }
        return true;
    }

    int recv_batch()
    {
        for (unsigned i = 0; i < UDP_BATCH; i++)
        {
            msgs[i].msg_hdr.msg_namelen = sizeof(from[i]);
            msgs[i].msg_hdr.msg_flags = 0;
            msgs[i].msg_len = 0;
        }
#ifdef __linux__
        return recvmmsg(udp_fd, msgs.data(), UDP_BATCH, MSG_DONTWAIT, nullptr);
#else
        unsigned n = 0;
        for (; n < UDP_BATCH; n++)
        {
            const ssize_t r = recvmsg(udp_fd, &msgs[n].msg_hdr, MSG_DONTWAIT);
            if (r < 0)
            {
                if (n == 0)
                    return -1;
                break;
            }
            msgs[n].msg_len = unsigned(r);
        }
        return int(n);
#endif
    }

    void drain()
    {
        uint8_t tmp[UDP_SLOT];
        unsigned n = 0;
        while (::recv(udp_fd, tmp, sizeof(tmp), MSG_DONTWAIT) >= 0)
            n++;
        if (n != 0)
            SoapySDR_logf(SOAPY_SDR_DEBUG, "iqnet: dropped %u stale datagrams", n);
    }

    void reset_stream()
    {
        pkts.clear();
        pkt_idx = pkt_pos = 0;
        next_offset = 0;
        next_seq = 0;
        datagrams = bytes = gaps = lost_datagrams = lost_bytes = 0;
        board_gaps = board_lost_bytes = 0;
        late = bad = foreign = restarts = 0;
    }

    ~UdpRx()
    {
        close_ctrl();
        if (udp_fd >= 0)
            close(udp_fd);
    }
};

void rx_streamer::UdpRxDeleter::operator()(UdpRx *p) const
{
    delete p;
}

void rx_streamer::udp_open(const IqNetConfig &net)
{
    if (channel_list.size() != (wire_format == WIRE_CS16 ? 2u : 1u))
        throw std::runtime_error("tezuka_transport=udp supports RX channel 0 only");

    udp.reset(new UdpRx);
    UdpRx &u = *udp;
    u.cfg = net;

    u.udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (u.udp_fd < 0)
        throw std::runtime_error(std::string("iqnet: UDP socket: ") + strerror(errno));
    set_cloexec(u.udp_fd);

    // A large receive buffer absorbs scheduling hiccups of the reading thread.
    // SO_RCVBUFFORCE ignores net.core.rmem_max but needs CAP_NET_ADMIN.
    const int want = int(std::min<size_t>(net.rcvbuf, INT_MAX / 2));
#ifdef SO_RCVBUFFORCE
    if (setsockopt(u.udp_fd, SOL_SOCKET, SO_RCVBUFFORCE, &want, sizeof(want)) != 0)
#endif
        setsockopt(u.udp_fd, SOL_SOCKET, SO_RCVBUF, &want, sizeof(want));
    int got = 0;
    socklen_t got_len = sizeof(got);
    getsockopt(u.udp_fd, SOL_SOCKET, SO_RCVBUF, &got, &got_len);
#ifdef __linux__
    got /= 2;  // Linux reports twice the requested size (bookkeeping overhead)
#endif
    if (got < want)
        SoapySDR_logf(SOAPY_SDR_WARNING,
                      "iqnet: UDP receive buffer is %d bytes, %d requested; raise it with "
                      "'sysctl -w net.core.rmem_max=%d' to avoid drops",
                      got, want, want);

    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(net.udp_port);
    if (bind(u.udp_fd, (const struct sockaddr *)&a, sizeof(a)) != 0)
        throw std::runtime_error("iqnet: cannot bind UDP port " + std::to_string(net.udp_port) +
                                 ": " + strerror(errno));

    u.slab.assign(size_t(UDP_BATCH) * UDP_SLOT, 0);
    u.msgs.resize(UDP_BATCH);
    u.iov.resize(UDP_BATCH);
    u.from.resize(UDP_BATCH);
    for (unsigned i = 0; i < UDP_BATCH; i++)
    {
        u.iov[i].iov_base = u.slab.data() + size_t(i) * UDP_SLOT;
        u.iov[i].iov_len = UDP_SLOT;
        memset(&u.msgs[i], 0, sizeof(u.msgs[i]));
        u.msgs[i].msg_hdr.msg_name = &u.from[i];
        u.msgs[i].msg_hdr.msg_namelen = sizeof(u.from[i]);
        u.msgs[i].msg_hdr.msg_iov = &u.iov[i];
        u.msgs[i].msg_hdr.msg_iovlen = 1;
    }

    SoapySDR_logf(SOAPY_SDR_INFO, "iqnet: UDP port %u, receive buffer %d bytes",
                  (unsigned)net.udp_port, got);
}

size_t rx_streamer::udp_mtu() const
{
    // samples in one full recvmmsg() batch of default-size datagrams
    const size_t bytes = size_t(UDP_BATCH) * IQNET_DEFAULT_PAYLOAD;
    if (wire_format == WIRE_CS12)
        return bytes / CS12_BURST * 8;
    return bytes / (wire_format == WIRE_CS8 ? 2 : 4);
}

int rx_streamer::udp_start()
{
    UdpRx &u = *udp;
    u.close_ctrl();
    u.reset_stream();
    u.started = false;

    // datagrams of a previous stream must not be taken for this one
    u.drain();

    std::string err;
    u.ctrl_fd = ctrl_connect(u.cfg.host, u.board, err);
    if (u.ctrl_fd < 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "iqnet: %s (is iqnetd running on the board?)", err.c_str());
        return SOAPY_SDR_STREAM_ERROR;
    }

    const std::string cmd = "START " + std::to_string(u.cfg.udp_port) + " " +
                            wire_mode(wire_format) + u.cfg.start_options + "\n";
    std::string reply;
    if (!send_all(u.ctrl_fd, cmd) || !u.read_line(CTRL_START_MS, reply))
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "iqnet: no reply to START from %s",
                      addr_str(u.board).c_str());
        u.close_ctrl();
        return SOAPY_SDR_STREAM_ERROR;
    }

    unsigned long payload_len = 0, block_size = 0;
    if (sscanf(reply.c_str(), "OK %lu %lu", &payload_len, &block_size) != 2 || payload_len == 0 ||
        payload_len % IQNET_BURST != 0 || payload_len > IQNET_MAX_PAYLOAD)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "iqnet: START refused by %s: %s", addr_str(u.board).c_str(),
                      reply.c_str());
        u.close_ctrl();
        return SOAPY_SDR_STREAM_ERROR;
    }

    u.payload_len = payload_len;
    u.block_size = block_size;
    u.started = true;

    SoapySDR_logf(SOAPY_SDR_INFO,
                  "iqnet: streaming %s from %s to UDP port %u, %lu B/datagram, %lu B/block",
                  wire_mode(wire_format), addr_str(u.board).c_str(), (unsigned)u.cfg.udp_port,
                  payload_len, block_size);
    return 0;
}

void rx_streamer::udp_stop()
{
    if (!udp)
        return;
    UdpRx &u = *udp;

    if (u.ctrl_fd >= 0)
    {
        std::string line;
        if (send_all(u.ctrl_fd, "STATS\n") && u.read_line(CTRL_STATS_MS, line))
        {
            SoapySDR_logf(SOAPY_SDR_INFO, "iqnet board: %s", line.c_str());
            unsigned long long ovf = 0, shrt = 0, serr = 0;
            const bool h_ovf = stats_value(line, "overflows", ovf);
            const bool h_shrt = stats_value(line, "short_blocks", shrt);
            const bool h_serr = stats_value(line, "send_errors", serr);
            if ((h_ovf && ovf != 0) || (h_shrt && shrt != 0) || (h_serr && serr != 0))
                SoapySDR_logf(SOAPY_SDR_WARNING,
                              "iqnet board lost data: overflows=%llu short_blocks=%llu "
                              "send_errors=%llu",
                              ovf, shrt, serr);
        }
        if (!send_all(u.ctrl_fd, "STOP\n") || !u.read_line(CTRL_STOP_MS, line) ||
            line.compare(0, 2, "OK") != 0)
            SoapySDR_logf(SOAPY_SDR_WARNING, "iqnet: STOP not confirmed (%s)", line.c_str());
        u.close_ctrl();
    }

    if (u.started)
    {
        SoapySDR_logf(SOAPY_SDR_INFO,
                      "iqnet host: datagrams=%llu bytes=%llu gaps=%llu lost_datagrams=%llu "
                      "lost_bytes=%llu board_gaps=%llu board_lost_bytes=%llu late=%llu bad=%llu "
                      "foreign=%llu restarts=%llu",
                      (unsigned long long)u.datagrams, (unsigned long long)u.bytes,
                      (unsigned long long)u.gaps, (unsigned long long)u.lost_datagrams,
                      (unsigned long long)u.lost_bytes, (unsigned long long)u.board_gaps,
                      (unsigned long long)u.board_lost_bytes, (unsigned long long)u.late,
                      (unsigned long long)u.bad, (unsigned long long)u.foreign,
                      (unsigned long long)u.restarts);
        u.started = false;
    }
    u.pkts.clear();
    u.pkt_idx = u.pkt_pos = 0;
}

// Receive and validate the next batch of datagrams into udp->pkts.
// Waits up to waitUs for the first one (0 = poll only).
// Returns the number of accepted datagrams, 0 if none, or a SOAPY_SDR_* error.
int rx_streamer::udp_fill(const long long waitUs)
{
    UdpRx &u = *udp;
    u.pkts.clear();
    u.pkt_idx = 0;
    u.pkt_pos = 0;

    if (waitUs > 0)
    {
        struct pollfd fds[2] = {{u.udp_fd, POLLIN, 0}, {u.ctrl_fd, POLLIN, 0}};
        const nfds_t nfds = u.ctrl_fd >= 0 ? 2 : 1;
        const int ms = int(std::min<long long>((waitUs + 999) / 1000, INT_MAX));
        const int r = poll(fds, nfds, ms);
        if (r < 0)
        {
            if (errno == EINTR)
                return 0;
            SoapySDR_logf(SOAPY_SDR_ERROR, "iqnet: poll: %s", strerror(errno));
            return SOAPY_SDR_STREAM_ERROR;
        }
        if (r == 0)
            return 0;
        if ((fds[0].revents & POLLIN) == 0)
        {
            if (nfds < 2 || fds[1].revents == 0)
                return 0;
            // control socket readable while no data is queued
            char tmp[512];
            const ssize_t n = ::recv(u.ctrl_fd, tmp, sizeof(tmp), MSG_DONTWAIT);
            if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
            {
                SoapySDR_logf(SOAPY_SDR_ERROR,
                              "iqnet: control connection to %s closed, stream stopped by board",
                              addr_str(u.board).c_str());
                u.close_ctrl();
                return SOAPY_SDR_STREAM_ERROR;
            }
            if (n > 0)
            {
                u.ctrl_in.append(tmp, size_t(n));
                std::string line;
                while (pop_line(u.ctrl_in, line))
                    SoapySDR_logf(SOAPY_SDR_WARNING, "iqnet board: %s", line.c_str());
            }
            return 0;
        }
    }

    const int n = u.recv_batch();
    if (n < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            return 0;
        SoapySDR_logf(SOAPY_SDR_ERROR, "iqnet: recvmmsg: %s", strerror(errno));
        return SOAPY_SDR_STREAM_ERROR;
    }

    for (int i = 0; i < n; i++)
    {
        const udp_mmsg &m = u.msgs[i];
        const uint8_t *d = u.slab.data() + size_t(i) * UDP_SLOT;
        const size_t len = m.msg_len;

        if (u.from[i].sin_addr.s_addr != u.board.s_addr)
        {
            if (u.foreign++ == 0)
                SoapySDR_logf(SOAPY_SDR_WARNING,
                              "iqnet: ignoring datagrams from %s (expected source %s)",
                              addr_str(u.from[i].sin_addr).c_str(), addr_str(u.board).c_str());
            continue;
        }
        if ((m.msg_hdr.msg_flags & MSG_TRUNC) != 0 || len < IQNET_HDR_LEN ||
            get_le32(d) != IQNET_MAGIC)
        {
            u.bad++;
            continue;
        }
        const size_t plen = len - IQNET_HDR_LEN;
        if (plen == 0 || plen % IQNET_BURST != 0 || plen > IQNET_MAX_PAYLOAD)
        {
            u.bad++;
            continue;
        }
        const uint32_t seq = get_le32(d + 4);
        const uint64_t offset = get_le64(d + 8);

        UdpRx::Packet p = {d + IQNET_HDR_LEN, plen, 0, false, false};
        if (seq == 0 && offset == 0 && u.next_offset != 0)
        {
            // first datagram of a new stream (the board restarted it): unknown distance
            p.gap = true;
            p.restart = true;
            u.restarts++;
            SoapySDR_logf(SOAPY_SDR_WARNING,
                          "iqnet: new stream from the board (seq 0, offset 0; expected seq %u, "
                          "offset %llu), resync",
                          (unsigned)u.next_seq, (unsigned long long)u.next_offset);
        }
        else if (offset > u.next_offset && (offset - u.next_offset) % IQNET_BURST == 0)
        {
            // datagrams lost in the network / socket buffer (seq jump) and/or bytes the
            // board skipped (DMA overflow, dropped short block: offset jump only)
            p.gap = true;
            p.lost = offset - u.next_offset;
            u.gaps++;
            u.lost_bytes += p.lost;
            const int32_t dseq = int32_t(seq - u.next_seq);
            const uint64_t net_dgrams = dseq > 0 ? uint64_t(dseq) : 0;
            const uint64_t net_bytes = std::min<uint64_t>(p.lost, net_dgrams * u.payload_len);
            const uint64_t board_bytes = p.lost - net_bytes;
            u.lost_datagrams += net_dgrams;
            if (board_bytes != 0)
            {
                u.board_gaps++;
                u.board_lost_bytes += board_bytes;
            }
            const bool first = u.gaps == 1;
            SoapySDR_logf(first ? SOAPY_SDR_WARNING : SOAPY_SDR_DEBUG,
                          "iqnet: %llu bytes lost at offset %llu (%llu datagrams in the network "
                          "or socket buffer, %llu bytes dropped on the board)%s",
                          (unsigned long long)p.lost, (unsigned long long)u.next_offset,
                          (unsigned long long)net_dgrams, (unsigned long long)board_bytes,
                          first && net_dgrams != 0
                              ? " (raise tezuka_udp_rcvbuf / net.core.rmem_max if this repeats)"
                              : "");
        }
        else if (offset < u.next_offset && u.next_offset - offset <= REORDER_WINDOW)
        {
            u.late++;  // duplicate or reordered behind a newer datagram: already skipped
            continue;
        }
        else if (offset != u.next_offset)
        {
            p.gap = true;
            p.restart = true;
            u.restarts++;
            SoapySDR_logf(SOAPY_SDR_WARNING,
                          "iqnet: stream discontinuity (offset %llu, expected %llu), resync",
                          (unsigned long long)offset, (unsigned long long)u.next_offset);
        }

        u.next_offset = offset + plen;
        u.next_seq = seq + 1;
        u.datagrams++;
        u.bytes += plen;
        u.pkts.push_back(p);
    }
    return int(u.pkts.size());
}

// Apply a gap of `lost` bytes in front of the datagram (data, len) to the CS12 decoder.
void rx_streamer::udp_gap_cs12(const uint8_t *&data, size_t &len, const uint64_t lost,
                               const bool restart)
{
    if (restart)
    {
        reset_cs12();  // unknown distance: search the sync magic again
        return;
    }
    if (!synced)
    {
        carry.clear();  // a magic cannot straddle a gap
        return;
    }

    // carry = head of the burst at the current decoder position, its tail is lost.
    // lost % 24 == 0, so the new datagram starts at the same phase inside a burst:
    // its first `skip` bytes are the tail of a burst whose head is lost.
    const size_t c = carry.size();
    const size_t skip = (CS12_BURST - c % CS12_BURST) % CS12_BURST;
    carry.clear();

    const uint64_t pos = uint64_t(bursts_since_sync) + (c + lost + skip) / CS12_BURST;
    bursts_since_sync = uint32_t(pos % CS12_BURSTS_PER_SYNC);
    last_counter += uint32_t(pos / CS12_BURSTS_PER_SYNC);  // sync bursts skipped

    const size_t s = std::min(skip, len);
    data += s;
    len -= s;
}

int rx_streamer::udp_recv(void *const *buffs, const size_t numElems, const long timeoutUs)
{
    UdpRx &u = *udp;
    if (!u.started || numElems == 0)
        return 0;

    uint8_t *out = (uint8_t *)buffs[0];
    const size_t out_size = host_elem_size(format);
    const size_t item = wire_format == WIRE_CS8 ? 2 : 4;  // raw bytes per IQ pair
    const clk::time_point deadline =
        clk::now() + std::chrono::microseconds(timeoutUs > 0 ? timeoutUs : 0);
    size_t done = 0;

    for (;;)
    {
        // 1. hand out what is already decoded / received
        if (wire_format == WIRE_CS12)
        {
            if (overflow)  // sync burst missing or counter jump inside the received data
            {
                if (done != 0)
                    return int(done);
                overflow = false;
                return SOAPY_SDR_OVERFLOW;
            }
            done += output_iq(out + done * out_size, numElems - done);
        }
        else if (u.pkt_idx < u.pkts.size() && !u.pkts[u.pkt_idx].gap)
        {
            const UdpRx::Packet &p = u.pkts[u.pkt_idx];
            const size_t n = std::min(numElems - done, (p.len - u.pkt_pos) / item);
            convert_direct(p.data + u.pkt_pos, out + done * out_size, n);
            done += n;
            u.pkt_pos += n * item;
            if (u.pkt_pos >= p.len)
            {
                u.pkt_idx++;
                u.pkt_pos = 0;
            }
        }
        if (done == numElems)
            return int(done);

        // 2. nothing queued: receive, blocking only while nothing was delivered yet
        if (u.pkt_idx >= u.pkts.size())
        {
            long long waitUs = 0;
            if (done == 0)
                waitUs =
                    std::chrono::duration_cast<std::chrono::microseconds>(deadline - clk::now())
                        .count();
            const int r = udp_fill(waitUs);
            if (r < 0)
                return done != 0 ? int(done) : r;
            if (r == 0)
            {
                if (done != 0)
                    return int(done);
                if (clk::now() >= deadline)
                    return SOAPY_SDR_TIMEOUT;
                continue;
            }
        }

        // 3. next datagram
        UdpRx::Packet &p = u.pkts[u.pkt_idx];
        if (p.gap)
        {
            if (done != 0)
                return int(done);  // data from before the gap goes out first
            p.gap = false;
            if (wire_format == WIRE_CS12)
                udp_gap_cs12(p.data, p.len, p.lost, p.restart);
            return SOAPY_SDR_OVERFLOW;
        }
        if (wire_format == WIRE_CS12)
        {
            // the iq queue is empty here
            iq.clear();
            iq_pos = 0;
            if (carry.empty())
            {
                decode_cs12(p.data, p.len);
            }
            else
            {
                work.assign(carry.begin(), carry.end());
                work.insert(work.end(), p.data, p.data + p.len);
                carry.clear();
                decode_cs12(work.data(), work.size());
            }
            u.pkt_idx++;
        }
    }
}
