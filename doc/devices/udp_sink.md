anyloop:udp_sink
================

Types and units: `[T_ANY, U_ANY] -> [T_UNCHANGED, U_UNCHANGED]`.

This device writes the current pipeline state to a UDP port as an AYLP file. See
[filetype.md](../filetype.md) for documentation on the AYLP file format.

Parameters
----------

- `ip` (string) (required)
  - The IP address to send the data to.
- `port` (string) (required)
  - The port to send the data to.
- `decimation` (integer) (optional)
  - Send only every `decimation`'th iteration; skipped iterations return
    immediately without copying or writing anything. Defaults to 1 (send every
    iteration).
  - Use this when the loop runs much faster than whatever is reading the port.
    A viewer that redraws at tens of Hz cannot consume a multi-kHz frame stream;
    without decimation the kernel drops the excess anyway, but the loop still
    pays for the copy and the `writev` on every iteration. Setting
    `decimation` to roughly `loop_rate / viewer_rate` keeps the traffic (and the
    syscall) off the hot path.

