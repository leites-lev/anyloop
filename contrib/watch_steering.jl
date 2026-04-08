#!/usr/bin/julia
# plotting script to watch a single-aperture center_of_mass from udp
# expects matrix output on 64730 and [y,x] vector output on 64731
include("anyloop.jl")
using .Anyloop
using Plots; gr()
using Sockets

sock0 = UDPSocket()
if !bind(sock0, ip"0.0.0.0", 64730)
    throw(SystemError("couldn't open port"))
end
sock1 = UDPSocket()
if !bind(sock1, ip"0.0.0.0", 64731)
    throw(SystemError("couldn't open port"))
end

println("listening on 64730 (matrix) and 64731 (vector)")

function set_rcvbuf!(sock, size)
    fd = Ref{Cint}(0)
    ccall(:uv_fileno, Cint, (Ptr{Cvoid}, Ptr{Cint}), sock.handle, fd)
    bufsize = Ref{Cint}(size)
    ccall(:setsockopt, Cint, (Cint, Cint, Cint, Ptr{Cint}, Cuint),
        fd[], 1, 8, bufsize, sizeof(Cint))
end

# limit receive buffers so the OS drops stale packets rather than queuing them
set_rcvbuf!(sock0, 131072)  # ~2 image frames
set_rcvbuf!(sock1, 512)     # a handful of CoM packets

for i in 1:10000
    data0 = read(IOBuffer(recv(sock0)), AYLPChunk)
    data1 = read(IOBuffer(recv(sock1)), AYLPChunk)

    @assert length(data1.data) == 2
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
end

close(sock0)
close(sock1)


