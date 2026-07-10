#!/usr/bin/julia
# plotting script to watch a single-aperture center_of_mass from udp
# expects matrix output on 64730 and [y,x] vector output on 64731
#
# The loop can push frames far faster than this script can redraw them (~2300
# fps at 64x64 vs a few tens of Hz for a GR heatmap). Two rules keep the display
# honest:
#
#   1. Give the two udp_sink devices feeding 64730 and 64731 the *same*
#      `decimation`. Both sinks are proc'd every iteration and count down from
#      the same start, so they emit on the same loop iterations -- the frame and
#      the CoM vector drawn together are then from one iteration, by
#      construction. Pick decimation ~ loop_rate/30 so the arrival rate lands
#      near the redraw rate.
#   2. Drain each socket to its newest datagram before rendering. If a redraw
#      does fall behind, we show fresh data instead of grinding through a stale
#      backlog. Because both sinks fire on the same iteration, draining both
#      preserves the pairing from (1).
#
# Without these, the displayed frames are separated by ~50-100 ms of real beam
# motion, and the spot appears to teleport between uncorrelated positions rather
# than move. That is an artifact of this viewer, not of the control loop -- judge
# the loop from watch_error_ts.jl or the logged .aylp, never from this heatmap.

include("anyloop.jl")
using .Anyloop
using Plots; gr()
using Sockets

const SOL_SOCKET = Cint(1)
const SO_RCVBUF = Cint(8)
const MSG_DONTWAIT = Cint(0x40)
const POLLIN = Cshort(1)

struct PollFd
    fd::Cint
    events::Cshort
    revents::Cshort
end

function sockfd(sock)
    fd = Ref{Cint}(0)
    ccall(:uv_fileno, Cint, (Ptr{Cvoid}, Ptr{Cint}), sock.handle, fd)
    return fd[]
end

function set_rcvbuf!(fd, size)
    bufsize = Ref{Cint}(size)
    ccall(:setsockopt, Cint, (Cint, Cint, Cint, Ptr{Cint}, Cuint),
        fd, SOL_SOCKET, SO_RCVBUF, bufsize, sizeof(Cint))
end

# one non-blocking datagram; < 0 means the socket is empty (EAGAIN)
function try_recv(fd, buf)
    return ccall(:recv, Cssize_t, (Cint, Ptr{UInt8}, Csize_t, Cint),
        fd, buf, Csize_t(length(buf)), MSG_DONTWAIT)
end

# libuv puts its fds in non-blocking mode, so a bare recv() can't block for us;
# poll() first when there is nothing queued
function wait_readable(fd, timeout_ms)
    pfd = Ref(PollFd(fd, POLLIN, 0))
    return ccall(:poll, Cint, (Ptr{PollFd}, Culong, Cint), pfd, 1, timeout_ms) > 0
end

# Pull datagrams until the socket is empty and decode the last one, mirroring the
# queue drain in asi_source.c. Blocks for one datagram if the socket was already
# empty. Returns nothing on timeout.
function recv_newest(fd, buf, timeout_ms=2000)
    n = -1
    while (r = try_recv(fd, buf)) >= 0
        n = r
    end
    if n < 0
        wait_readable(fd, timeout_ms) || return nothing
        n = try_recv(fd, buf)
        n < 0 && return nothing
    end
    return read(IOBuffer(buf[1:n]), AYLPChunk)
end

sock0 = UDPSocket()
if !bind(sock0, ip"0.0.0.0", 64730)
    throw(SystemError("couldn't open port"))
end
sock1 = UDPSocket()
if !bind(sock1, ip"0.0.0.0", 64731)
    throw(SystemError("couldn't open port"))
end
fd0 = sockfd(sock0)
fd1 = sockfd(sock1)

# keep the receive buffers small: we drain to the newest datagram anyway, and a
# deep buffer only means more to throw away (and more lag if we ever miss)
set_rcvbuf!(fd0, 131072)
set_rcvbuf!(fd1, 8192)

println("listening on 64730 (matrix) and 64731 (vector)")

buf0 = Vector{UInt8}(undef, 65536)
buf1 = Vector{UInt8}(undef, 65536)

for i in 1:100000
    try
        # drain the image first, then the CoM: the CoM sink writes microseconds
        # after the image sink in the same iteration, so by the time we finish
        # draining sock0 the matching vector is already queued on sock1
        data0 = recv_newest(fd0, buf0)
        data0 === nothing && continue
        data1 = recv_newest(fd1, buf1)
        data1 === nothing && continue

        if length(data1.data) != 2
            @warn "unexpected chunk on 64731" typeof(data1.data) length(data1.data) data1
            continue
        end
        # CoM output is [y, x] each in -1:1, where 0 is centre of the frame
        com_y = data1.data[1]
        com_x = data1.data[2]

        nrows, ncols = size(data0.data)
        # convert from -1:1 to pixel coordinates
        px = (com_x + 1) / 2 * (ncols - 1) + 1
        py = (com_y + 1) / 2 * (nrows - 1) + 1

        heatmap(data0.data, aspect_ratio=:equal, size=(800,800))
        display(scatter!([px], [py],
            marker=:cross, markersize=10, color=:magenta, label="CoM"
        ))
    catch e
        e isa InterruptException && rethrow()
        # show why this iteration failed instead of crashing silently
        @error "watch_steering iteration $i failed" exception=(e, catch_backtrace())
        continue
    end
end

close(sock0)
close(sock1)
