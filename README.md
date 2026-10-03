# Soapy SDR module for Pluto SDR

## Installation instructions

```
git clone https://github.com/F5OEO/SoapyPlutoPAPR
cd SoapyPlutoPAPR
mkdir build
cd build
cmake ..
make
sudo make install
```

## Dependencies

- [libiio](https://github.com/analogdevicesinc/libiio)
- [libad9361](https://github.com/analogdevicesinc/libad9361-iio)
- [SoapySDR](https://github.com/pothosware/SoapySDR)

## Documentation

* https://github.com/pothosware/SoapyPlutoSDR/wiki

Note that the Frequency Correction API is not implemented,
it's recommended that you adjust the `xo_correction` value with the observed PPM in the Pluto device `config.txt`.

## Tezuka device arguments

| Argument | Values | Meaning |
|---|---|---|
| `tezuka_format` | `CS16` (default), `CS8`, `CS12` | RX wire format packed by the Tezuka PL |
| `tezuka_transport` | `iio` (default), `udp` | RX data path: libiio buffer, or zero-copy UDP from the board's iqnet service |
| `tezuka_udp_port` | default `30432` | local UDP port the board sends to |
| `tezuka_udp_host` | IPv4 / hostname | board address; default: host of `uri=ip:...`, `hostname=`, or the address libiio resolved |
| `tezuka_udp_rcvbuf` | bytes, default 32 MiB | socket receive buffer (needs `net.core.rmem_max` at least this big, or CAP_NET_ADMIN) |
| `tezuka_udp_blocks`, `tezuka_udp_block_size`, `tezuka_udp_gso` | integers (blocks 1..64, block_size a multiple of 1440 up to 16 MiB, gso 0..44) | passed to the board as `START ... blocks= block_size= gso=` |

With `tezuka_transport=udp` only the IQ data moves to UDP (see `iqnet_proto.h`); AD9361
control (frequency, gain, sample rate) stays on libiio. The host connects to TCP port 30433
of the board on `activateStream()` and sends `START`, `deactivateStream()` sends `STOP`.
Lost data is reported once per gap as `SOAPY_SDR_OVERFLOW`, whether datagrams were lost in the
network / socket buffer or the board skipped samples (DMA overflow, dropped short block: an
offset jump in the iqnet header). A datagram with `seq == 0 && offset == 0` in the middle of
a stream means the board started a new stream: it is reported as an overflow and the decoder
starts over. Only datagrams from the board address are accepted. Only RX channel 0 is supported.

```
SoapySDRUtil --args="driver=tezuka,hostname=10.10.11.20,tezuka_format=CS12,tezuka_transport=udp" --rate=10e6 --direction=RX
sudo sysctl -w net.core.rmem_max=33554432
```

## PothosSDR

Note that installation with PothosSDR is optional as "PlutoSDR SoapySDR binding (experimental)" and not selected by default.

This is due to possible problems with other libusb devices,
see [#24](https://github.com/pothosware/SoapyPlutoSDR/issues/24)
and [libiio#586](https://github.com/analogdevicesinc/libiio/issues/586)

## Licensing information

GNU LESSER GENERAL PUBLIC LICENSE Version 2.1, February 1999
