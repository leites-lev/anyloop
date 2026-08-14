#!/usr/bin/julia
# Rolling time-series + PSD plot of the [y, x] error signal from center_of_mass.
# Listens on UDP port 64732.
#
# Optionally also plots beam intensity over time, listening on port 64733 (see
# --iport). Intensity is NOT in the CoM stream -- center_of_mass emits only
# [y, x] -- so it has to come from the image, which means a second udp_sink in
# the config:
#
#   {"uri": "anyloop:udp_sink", "params": {
#       "ip": "127.0.0.1", "port": "64733", "reduce": "stats"}}
#
# placed anywhere after asi_source and before center_of_mass. "reduce": "stats"
# makes the loop send [peak, mean, saturated] as three doubles instead of the
# frame: 24 bytes against 61.5 kB for a 248x248 image, so it runs UNDECIMATED at
# the full loop rate. Sending frames that fast is not possible -- 233 MB/s at
# 3788 Hz -- which is why this panel was stuck at loop_rate/decimation before.
#
# A plain frame sink (no "reduce") still works and is reduced here instead, but
# then it must be decimated, and the trace updates at loop_rate/decimation.
#
# The port must be its own rather than the 64730 that watch_steering.jl uses:
# two processes bound to one UDP port either refuse to bind or (with
# SO_REUSEPORT) split the datagrams between them, which would starve the
# heatmap. The panel is skipped silently when nothing arrives, so configs
# without the sink behave exactly as before.
include(joinpath(@__DIR__, "anyloop-examples", "source", "anyloop.jl"))
using .Anyloop
using Plots; gr()
using Sockets
using FFTW
using Dates

const FS = 471.0
const DEFAULT_SECONDS = 30.0  # rolling time-series window length, seconds

# --- output configuration (override on the command line) ---
#   julia watch_error_ts.jl [--file NAME] [--timestamp] [--seconds SECS]
#                           [--fmin HZ] [--fmax HZ] [--xtick SPACING]
# --file/-f NAME: path/name for the saved figure (default "spectrum.png",
#                 relative to this script's directory). The extension sets the
#                 format. A bare positional NAME also works.
# --timestamp/-t: append a timestamp before the extension on each save, so
#                 successive captures get unique names instead of overwriting
#                 (e.g. spectrum_20260625-143052.png).
# --seconds/-s SECS: length of the rolling time-series window in seconds
#                 (default 10 s). Longer windows show more history and give
#                 finer spectral resolution at the cost of slower updates.
# --pixel/-p SIZE: the pixel span the center_of_mass output is normalized across,
#                 used to convert it into pixels for the time-series panels
#                 (default 32, matching the ROI every current config uses; it
#                 was 64 until 2026-07-30, which reported everything 2.03x high).
#                 The conversion is pixels = value * (SIZE - 1)/2, so getting
#                 this wrong scales every px figure by (SIZE-1)/(actual-1).
#                 With track mode this is the IMAGE size (asi_source width /
#                 height), because the tracking window reports absolute image
#                 coordinates; without track it is the region/subaperture size.
#                 Assumes a square frame. Only affects the time series and the
#                 --com readout; the spectrum is unchanged.
# --fmin/-m HZ:   lower limit of the spectrum frequency axis in Hz (default 0,
#                 i.e. from DC). Ignored unless it is below the effective max
#                 frequency. Set it above your drift/1-f corner to actually see
#                 what's underneath: the 0 dB reference and the peak search both
#                 follow the displayed band, so cutting the low end rescales the
#                 plot rather than just sliding the axis. --fmin 0 turns it off.
# --fmax/-F HZ:   upper limit of the spectrum frequency axis in Hz (default:
#                 Nyquist = fs/2). Values above Nyquist are clamped to it.
# --xtick/-x SPACING: spacing between spectrum x-axis ticks in Hz (default:
#                 auto, ~8 ticks across the displayed band rounded to 10 Hz).
# --com/-c: text mode. Instead of plotting, print the [y, x] CoM to stdout
#                 (throttled to ~5 Hz) with a running mean, for reading the
#                 spot's resting position. No figure is produced.
# --iport/-P PORT: UDP port carrying the intensity stream, either in-loop stats
#                 or raw frames (default 64733; 0 disables the panel and never
#                 binds). Needs the extra udp_sink described at the top. The
#                 panel plots per-frame peak counts against a FIXED 0-255 axis
#                 -- peak is what saturation and center_of_mass's min_peak are
#                 both measured against -- plus mean counts on a right-hand
#                 axis, since a saturated peak pins at 255 and hides a beam that
#                 is dimming underneath it.
function parse_args(args)
    outfile = joinpath(@__DIR__, "..", "spectrum.png")
    timestamp = false
    seconds = DEFAULT_SECONDS
    com = false
    pixel = 384.0   # span the CoM is normalized across (image size in track mode)
                    # Measured 1 kHz setup: 384x384 at 471 Hz. A stale pixel
                    # span scales every displayed offset and RMS directly.
                    # MUST match asi_source width/height -- pass -p if it does not.
    fmin  = 0.0     # spectrum min frequency (Hz); 0 = from DC
    fmax  = 0.0     # spectrum max frequency (Hz); 0 = auto (Nyquist)
    xtick = 0.0     # spectrum x-axis tick spacing (Hz); 0 = auto
    iport = 64733   # raw-frame port for the intensity panel; 0 = disabled
    i = 1
    while i <= length(args)
        a = args[i]
        if a in ("--file", "-f")
            i < length(args) || error("$a needs a NAME argument")
            outfile = args[i+1]; i += 2
        elseif startswith(a, "--file=")
            outfile = split(a, "=", limit=2)[2]; i += 1
        elseif a in ("--timestamp", "-t")
            timestamp = true; i += 1
        elseif a in ("--seconds", "-s")
            i < length(args) || error("$a needs a SECS argument")
            seconds = parse(Float64, args[i+1]); i += 2
        elseif startswith(a, "--seconds=")
            seconds = parse(Float64, split(a, "=", limit=2)[2]); i += 1
        elseif a in ("--pixel", "-p")
            i < length(args) || error("$a needs a SIZE argument")
            pixel = parse(Float64, args[i+1]); i += 2
        elseif startswith(a, "--pixel=")
            pixel = parse(Float64, split(a, "=", limit=2)[2]); i += 1
        elseif a in ("--fmin", "-m")
            i < length(args) || error("$a needs an HZ argument")
            fmin = parse(Float64, args[i+1]); i += 2
        elseif startswith(a, "--fmin=")
            fmin = parse(Float64, split(a, "=", limit=2)[2]); i += 1
        elseif a in ("--fmax", "-F")
            i < length(args) || error("$a needs an HZ argument")
            fmax = parse(Float64, args[i+1]); i += 2
        elseif startswith(a, "--fmax=")
            fmax = parse(Float64, split(a, "=", limit=2)[2]); i += 1
        elseif a in ("--xtick", "-x")
            i < length(args) || error("$a needs a SPACING argument")
            xtick = parse(Float64, args[i+1]); i += 2
        elseif startswith(a, "--xtick=")
            xtick = parse(Float64, split(a, "=", limit=2)[2]); i += 1
        elseif a in ("--iport", "-P")
            i < length(args) || error("$a needs a PORT argument")
            iport = parse(Int, args[i+1]); i += 2
        elseif startswith(a, "--iport=")
            iport = parse(Int, split(a, "=", limit=2)[2]); i += 1
        elseif a in ("--com", "-c")
            com = true; i += 1
        elseif startswith(a, "-")
            error("unknown option: $a")
        else
            outfile = a; i += 1   # bare positional name
        end
    end
    seconds > 0 || error("--seconds must be positive")
    pixel > 1 || error("--pixel must be greater than 1")
    fmin >= 0 || error("--fmin must be non-negative")
    fmax >= 0 || error("--fmax must be non-negative")
    xtick >= 0 || error("--xtick must be non-negative")
    0 <= iport <= 65535 || error("--iport must be a valid port (or 0)")
    return outfile, timestamp, seconds, com, pixel, fmin, fmax, xtick, iport
end

const OUTFILE, TIMESTAMP, SECONDS, COM, PIXEL, FMIN, FMAX, XTICK, IPORT =
    parse_args(ARGS)
# center_of_mass output is normalized to -1:1 across PIXEL px -- the whole image
# in track mode, the region otherwise; convert to pixels with value*(PIXEL-1)/2
const PIX_PER_UNIT = (PIXEL - 1) / 2
const N = max(round(Int, SECONDS * FS), 2)
const SAVE_EVERY = 40000.0  # seconds between saves

# build the name to save under, inserting a timestamp if requested
function save_path()
    TIMESTAMP || return OUTFILE
    base, ext = splitext(OUTFILE)
    stamp = Dates.format(Dates.now(), "yyyymmdd-HHMMSS")
    return string(base, "_", stamp, ext)
end

sock = UDPSocket()
if !bind(sock, ip"0.0.0.0", 64732)
    throw(SystemError("couldn't open port 64732"))
end
println("listening on 64732 ([y, x] CoM error)")

# --com: text readout of the centroid, no plotting. Reports, per axis:
#   pos    running-mean position (normalized -1:1), for the resting spot location
#   std    total RMS jitter of the position (px) -- includes real motion + noise
#   noise  x1-x2 measurement-noise floor: std(Δ)/√2 over successive-sample
#          differences (px). Differencing cancels the common-mode true position
#          (and slow drift), leaving √2× the per-sample noise, so this is the
#          floor the sensor can't beat regardless of how the beam moves.
#   motion √(std² - noise²): the real beam motion left after removing the noise
#          floor. motion ≈ 0 with std ≈ noise means all you see is sensor noise
#          (or a centroid pinned by background -- confirm with a push test).
# Note std/motion are CUMULATIVE, so slow drift inflates them over a long run;
# noise stays put. Ratios matter more than absolutes.
if COM
    println("--com: [y,x] CoM stats (~2 Hz); Ctrl-C to stop")
    println("  std = total RMS jitter; noise = x1-x2 floor std(Δ)/√2; ",
            "motion = √(std²-noise²); all px")
    println("  normalized across ", round(Int, PIXEL), " px (",
            round(PIX_PER_UNIT, digits=1), " px/unit)")
    let n = 0, sum_y = 0.0, sum_x = 0.0, sqy = 0.0, sqx = 0.0,
        nd = 0, sdy = 0.0, sdx = 0.0, sdqy = 0.0, sdqx = 0.0,
        prev_y = 0.0, prev_x = 0.0, prev_valid = false, last_print = 0.0
        while true
            chunk = read(IOBuffer(recv(sock)), AYLPChunk)
            @assert length(chunk.data) == 2 "expected 2-element [y,x] vector"
            y = chunk.data[1]; x = chunk.data[2]
            # skip non-finite centroids (empty/low-signal frames) so one bad
            # frame doesn't poison the running statistics with NaN
            if isfinite(y) && isfinite(x)
                n += 1
                sum_y += y; sum_x += x; sqy += y*y; sqx += x*x
                # only difference against an immediately-preceding valid sample,
                # so a dropped NaN frame doesn't widen the interval and bias the
                # noise estimate high
                if prev_valid
                    dy = y - prev_y; dx = x - prev_x
                    nd += 1
                    sdy += dy; sdx += dx; sdqy += dy*dy; sdqx += dx*dx
                end
                prev_y = y; prev_x = x; prev_valid = true
            else
                prev_valid = false
            end
            now = time()
            if now - last_print >= 0.5 && n > 1
                std_y = sqrt(max(sqy/n - (sum_y/n)^2, 0.0)) * PIX_PER_UNIT
                std_x = sqrt(max(sqx/n - (sum_x/n)^2, 0.0)) * PIX_PER_UNIT
                nf_y = nd > 1 ?
                    sqrt(max(sdqy/nd - (sdy/nd)^2, 0.0)) / sqrt(2) * PIX_PER_UNIT : NaN
                nf_x = nd > 1 ?
                    sqrt(max(sdqx/nd - (sdx/nd)^2, 0.0)) / sqrt(2) * PIX_PER_UNIT : NaN
                mot_y = sqrt(max(std_y^2 - nf_y^2, 0.0))
                mot_x = sqrt(max(std_x^2 - nf_x^2, 0.0))
                r(v) = round(v, sigdigits=3)
                println("y  pos=", round(sum_y/n, digits=4),
                        "  std=", r(std_y), "  noise=", r(nf_y),
                        "  motion=", r(mot_y))
                println("x  pos=", round(sum_x/n, digits=4),
                        "  std=", r(std_x), "  noise=", r(nf_x),
                        "  motion=", r(mot_x), "   (N=", n, ")")
                flush(stdout)   # stdout is block-buffered when piped/redirected
                last_print = now
            end
        end
    end
    close(sock)
    exit()
end

println("time-series window: ", round(N / FS, digits=2), " s (", N, " samples)")
println("saving figures to ", TIMESTAMP ? save_path() * " (timestamped)" : OUTFILE,
        " every ", SAVE_EVERY, " s")

# Hann window to reduce spectral leakage
hann = 0.5 .* (1 .- cos.(2π .* (0:N-1) ./ (N-1)))

# Measured sample rate from actual packet arrivals. FS is only a fallback /
# buffer-sizing hint: the real loop rate changes with camera exposure, loop
# load, etc., so the frequency axis must come from the measured rate or every
# frequency is mis-scaled. Averages only recent packets (via the inter-arrival
# EMA) so it re-converges quickly after a rate change; returns FS until warmed up.
function meas_fs()
    (npkt[] > 50 && dt_ema[] > 0.0) ? 1.0 / dt_ema[] : FS
end

# --- decoupled acquisition + rendering ---------------------------------
# A background task drains the UDP socket as fast as packets arrive and
# writes each sample into a shared ring buffer. The main task renders on a
# fixed ~5 Hz cadence. Splitting the two means a slow plot render never
# blocks the socket: at worst the kernel briefly queues packets while a
# frame draws, and the acquisition task drains them the instant the render
# yields. Without this split the per-sample render (a ~1.2 ms budget at
# 815 Hz) overruns, the kernel receive buffer overflows, and dropped
# datagrams punch gaps into the time base that corrupt the spectrum.

const buf_y = zeros(N)
const buf_x = zeros(N)
const buf_lock = ReentrantLock()
const widx = Ref(1)   # next write position in the ring (also the oldest sample)
const npkt = Ref(0)   # packets received so far (for the sample-rate warmup gate)
# exponential moving average of the packet inter-arrival time, so the sample-rate
# estimate tracks recent packets and re-converges quickly when the rate changes
# (e.g. a new pixel/region size shifts the camera/loop rate). Smaller EMA_N =
# faster response, more jitter.
const EMA_N  = 64
const dt_ema = Ref(0.0) # smoothed inter-arrival time (s); 0 until first interval
const t_last = Ref(0.0) # wall-clock time of the previous packet

# windowed RMS of the on-screen error is shown only after a warmup delay, so a
# noisy estimate from the first few packets isn't displayed
const AVG_DELAY = 5.0   # seconds to wait after the first packet before showing RMS
const tstart = Ref(0.0) # wall-clock time of the first packet

# Mean and RMS-about-the-mean, both in px, over the newest nvalid samples of an
# oldest->newest window. The tail selection skips the zero-prefilled part of the
# ring before it has filled.
#
# These are two different things and both are worth seeing. The mean is the
# beam's steady-state offset from the setpoint: center_of_mass reports absolute
# image coordinates (in track mode the window follows the beam but the output is
# still normalized across the whole image), so the PID's setpoint is 0 = frame
# centre, and a nonzero mean is real pointing error. The RMS about that mean is
# the jitter. Drawing ±RMS about zero — as this used to — puts the lines nowhere
# near the trace as soon as the beam rests off centre.
function mean_rms_px(win, nvalid, scale)
    nvalid < 1 && return (NaN, NaN)
    v = @view win[end-nvalid+1:end]
    m = sum(v) / nvalid
    r = sqrt(sum(x -> abs2(x - m), v) / nvalid)
    return (m * scale, r * scale)
end

# oldest -> newest copy of a ring buffer, given the next-write index i
snapshot(buf, i) = vcat(buf[i:end], buf[1:i-1])

# Inclusive index range of the bins whose frequency lies in [fmin, fmax]. freqs
# is increasing, so a binary search suffices. Falls back to the whole spectrum if
# the band somehow selects nothing.
function band_range(freqs, fmin, fmax)
    lo = searchsortedfirst(freqs, fmin)
    hi = searchsortedlast(freqs, fmax)
    (lo > hi || lo < 1 || hi > length(freqs)) && return 1:length(freqs)
    return lo:hi
end

# Amplitude spectrum in dB, referenced to the loudest bin inside [fmin, fmax].
# Bins outside the band can therefore come out above 0 dB; they're off-axis and
# clipped by ylim, which is the point — the band sets the scale.
function todb(spec, freqs, fmin, fmax)
    r = band_range(freqs, fmin, fmax)
    ref = maximum(@view spec[r])
    return 20 .* log10.(spec ./ max(ref, 1e-12))
end

# Up to n dominant spectral peaks within [0, fmax], as (freq, dB) pairs sorted
# by descending magnitude. Picks local maxima (a bin above both neighbors) so a
# single wide peak doesn't claim several adjacent bins, and skips the DC bin.
function top_peaks(freqs, db, fmin, fmax, n)
    peaks = Tuple{Float64,Float64}[]
    @inbounds for i in 2:length(db)-1
        freqs[i] > fmax && break        # past the displayed band (freqs increasing)
        freqs[i] < fmin && continue     # below the displayed band
        if db[i] > db[i-1] && db[i] >= db[i+1]
            push!(peaks, (freqs[i], db[i]))
        end
    end
    sort!(peaks; by = p -> p[2], rev = true)
    return peaks[1:min(n, length(peaks))]
end

# legend label listing the peak frequencies (biggest first), e.g.
# "peaks: 94.2, 81.6, 156.0 Hz"; falls back to fallback when there are none
function peak_label(peaks, fallback)
    isempty(peaks) && return fallback
    "peaks: " * join((string(round(p[1], digits=1)) for p in peaks), ", ") * " Hz"
end

# background acquisition: never plots, just keeps the ring buffer current
acq = @async while true
    chunk = read(IOBuffer(recv(sock)), AYLPChunk)
    # drop malformed frames instead of killing the task
    length(chunk.data) == 2 || continue
    lock(buf_lock) do
        buf_y[widx[]] = chunk.data[1]
        buf_x[widx[]] = chunk.data[2]
        widx[] = widx[] == N ? 1 : widx[] + 1
        now = time()
        if t_last[] > 0.0
            dt = now - t_last[]
            # warm up to the first interval, then track recent packets with the EMA
            dt_ema[] = dt_ema[] == 0.0 ? dt : dt_ema[] + (dt - dt_ema[]) / EMA_N
        end
        t_last[] = now
        npkt[] == 0 && (tstart[] = now)   # mark the first packet's arrival
        npkt[] += 1
    end
end

# --- intensity acquisition (optional second stream) ---------------------
# Two packet formats are accepted on IPORT, and which one you get is a property
# of the config, not of this script:
#
#   [peak, mean, saturated] as 3 doubles -- a udp_sink with "reduce": "stats".
#       24 bytes, so this runs UNDECIMATED at the full loop rate. Prefer it.
#   a raw frame -- a plain udp_sink. Reduced here instead of in the loop, which
#       costs 61.5 kB per packet on the wire and a full decode in julia, so it
#       only works decimated. Kept working because bode/probe configs already
#       have frame sinks lying around.
#
# Ring buffers, not push!/deleteat! vectors: at the full loop rate the trimming
# a growable vector needs is an O(n) memmove per packet, which is precisely the
# cost that made undecimated intensity look impossible. Sized N like the error
# rings, so the history covers the same span the time axis shows.
const int_t    = zeros(N)    # arrival time (s, wall clock)
const int_peak = zeros(N)    # brightest pixel in the frame (counts)
const int_mean = zeros(N)    # mean over the frame (counts)
const int_lock = ReentrantLock()
const int_widx = Ref(1)      # next write position in the intensity rings
const int_sat  = Ref(0.0)    # fraction of saturated pixels, newest frame
const int_dims = Ref((0, 0)) # frame size, for the panel title (0,0 = stats mode)
const int_npkt = Ref(0)

# How much intensity history to keep: the span the time axis actually shows,
# which is (N-1)/fs. That equals SECONDS only when the loop runs at the nominal
# FS -- N is sized from FS, so at the real 3788 Hz the axis is ~4.6x shorter
# than SECONDS, and trimming to SECONDS would hoard samples the panel clips off
# anyway. The 1 s floor keeps a usable trace on very short windows.
hist_span() = max((N - 1) / meas_fs(), 1.0)

isock = nothing
if IPORT > 0
    s = UDPSocket()
    if bind(s, ip"0.0.0.0", IPORT)
        global isock = s
        println("listening on ", IPORT, " (intensity: stats or raw frames)")
    else
        # not fatal: most configs have no frame sink on this port, and the
        # error plots are the point of this script
        @warn "couldn't bind port $IPORT; intensity panel disabled"
        close(s)
    end
end

iacq = isock === nothing ? nothing : @async while true
    chunk = read(IOBuffer(recv(isock)), AYLPChunk)
    m = chunk.data
    length(m) == 0 && continue
    local pk, mn, sat, dims
    if eltype(m) === Float64 && length(m) == 3
        # already reduced in the loop by "reduce": "stats"
        pk, mn, sat = m[1], m[2], m[3]
        dims = (0, 0)
    else
        # raw frame; sum() widens UInt8 to a machine word, so no overflow.
        # saturation matters because a pinned peak hides everything above it:
        # at 255 the centroid weights are clipped and the mean is the only
        # honest brightness left.
        pk = Float64(maximum(m))
        mn = Float64(sum(m)) / length(m)
        sat = count(==(typemax(eltype(m))), m) / length(m)
        dims = size(m)
    end
    now = time()
    lock(int_lock) do
        i = int_widx[]
        int_t[i] = now; int_peak[i] = pk; int_mean[i] = mn
        int_widx[] = i == N ? 1 : i + 1
        int_sat[] = sat
        int_dims[] = dims
        int_npkt[] += 1
    end
end

const RENDER_PERIOD = 0.2   # seconds between frames (~5 Hz)

let last_save = time()
    while true
        # snapshot the ring under lock, then do the heavy work unlocked
        local sy, sx, np
        lock(buf_lock) do
            i = widx[]
            sy = snapshot(buf_y, i)
            sx = snapshot(buf_x, i)
            np = npkt[]   # real samples so far, to skip the zero-prefilled tail
        end

        # axes built from the *measured* sample rate, recomputed each frame
        fs   = meas_fs()
        nyq  = fs / 2
        freqs = (0:N÷2) .* (fs / N)
        tvec  = range(-(N-1)/fs, 0; length=N)
        # Left edge of the time-series panels. The ring holds N samples, which
        # is SECONDS long only at the nominal FS -- at a measured 400 Hz an
        # `-s 3` window is really 6.1 s of data, and at 3788 Hz it is 0.65 s.
        # Show the last SECONDS of it, or the whole record when the record is
        # the shorter of the two, so the axis never runs off past the data.
        # The spectrum still uses the full N-sample record either way, so
        # narrowing the view costs no frequency resolution.
        xlo = max(first(tvec), -SECONDS)
        # spectrum x-axis edges: custom --fmin/--fmax, clamped to [0, Nyquist]
        fmax = FMAX > 0 ? min(FMAX, nyq) : nyq
        fmin = (FMIN > 0 && FMIN < fmax) ? FMIN : 0.0
        # tick spacing: custom --xtick, else ~8 readable ticks across the
        # displayed band, rounded to a multiple of 10 Hz
        tick = XTICK > 0 ? XTICK : max(round(Int, (fmax - fmin) / 8 / 10) * 10, 10)
        # first tick at a round multiple of the spacing at or above fmin
        xt = (ceil(fmin / tick) * tick):tick:fmax

        # one-sided amplitude spectrum, dB re peak; remove the mean first so the
        # nonzero resting centroid doesn't dominate the DC bin and bury the real
        # vibration content under the peak-normalized dB scale
        spec_y = abs.(rfft((sy .- sum(sy)/N) .* hann))
        spec_x = abs.(rfft((sx .- sum(sx)/N) .* hann))
        # 0 dB is the loudest bin *inside the displayed band*, not the loudest bin
        # anywhere. Otherwise raising --fmin only slides the axis: low-frequency
        # drift still sets the reference, and everything you were trying to look
        # at stays pinned near the -60 dB floor. With the default fmin=0 and
        # fmax=Nyquist the band is the whole spectrum, so this changes nothing.
        db_y   = todb(spec_y, freqs, fmin, fmax)
        db_x   = todb(spec_x, freqs, fmin, fmax)

        # windowed mean + RMS error (px) over the real, on-screen samples; shown
        # only after the warmup so an early noisy estimate isn't displayed
        nvalid   = min(np, N)
        show_rms = tstart[] > 0.0 && time() - tstart[] >= AVG_DELAY
        mean_y_px, rms_y_px = show_rms ? mean_rms_px(sy, nvalid, PIX_PER_UNIT) : (NaN, NaN)
        mean_x_px, rms_x_px = show_rms ? mean_rms_px(sx, nvalid, PIX_PER_UNIT) : (NaN, NaN)

        p1 = plot(tvec, sy .* PIX_PER_UNIT;
                  label="y (tip)", ylabel="error (px)", color=:blue, lw=1,
                  xlim=(xlo, 0),
                  title="Time series   (fs ≈ $(round(fs, digits=1)) Hz)")
        if show_rms
            hline!(p1, [mean_y_px]; color=:black, ls=:dot, lw=1,
                   label="mean = $(round(mean_y_px, sigdigits=3)) px")
            hline!(p1, [mean_y_px + rms_y_px, mean_y_px - rms_y_px];
                   color=:black, ls=:dash, lw=1,
                   label="RMS about mean = $(round(rms_y_px, sigdigits=3)) px")
        end
        # 5 dominant peaks in the displayed band, recomputed every frame
        peaks_y = top_peaks(freqs, db_y, fmin, fmax, 5)
        peaks_x = top_peaks(freqs, db_x, fmin, fmax, 5)

        p2 = plot(freqs, db_y;
                  label="y (tip)", xlabel="freq (Hz)", ylabel="dB", color=:blue, lw=1,
                  xlim=(fmin, fmax), xticks=xt, ylim=(-60, 0),
                  title="Spectrum", legend=:topright)
        scatter!(p2, [p[1] for p in peaks_y], [p[2] for p in peaks_y];
                 marker=:circle, ms=4, color=:blue, label=peak_label(peaks_y, ""))
        p3 = plot(tvec, sx .* PIX_PER_UNIT;
                  label="x (tilt)", xlabel="time (s)", ylabel="error (px)",
                  color=:red, lw=1, xlim=(xlo, 0))
        if show_rms
            hline!(p3, [mean_x_px]; color=:black, ls=:dot, lw=1,
                   label="mean = $(round(mean_x_px, sigdigits=3)) px")
            hline!(p3, [mean_x_px + rms_x_px, mean_x_px - rms_x_px];
                   color=:black, ls=:dash, lw=1,
                   label="RMS about mean = $(round(rms_x_px, sigdigits=3)) px")
        end
        p4 = plot(freqs, db_x;
                  label="x (tilt)", xlabel="freq (Hz)", ylabel="dB", color=:red, lw=1,
                  xlim=(fmin, fmax), xticks=xt, ylim=(-60, 0), legend=:topright)
        scatter!(p4, [p[1] for p in peaks_x], [p[2] for p in peaks_x];
                 marker=:circle, ms=4, color=:red, label=peak_label(peaks_x, ""))

        # intensity panel, only once samples have actually arrived
        local it, ip, im, isat, idims, inp
        lock(int_lock) do
            i = int_widx[]
            it = snapshot(int_t, i)
            ip = snapshot(int_peak, i)
            im = snapshot(int_mean, i)
            isat = int_sat[]; idims = int_dims[]; inp = int_npkt[]
        end
        # keep the real samples (skipping the zero-prefilled part of the ring)
        # that fall inside the displayed span; times increase along the tail, so
        # a binary search finds the left edge
        nivalid = min(inp, N)
        tnow = time()
        if nivalid > 0
            it = @view it[end-nivalid+1:end]
            ip = @view ip[end-nivalid+1:end]
            im = @view im[end-nivalid+1:end]
            k = searchsortedfirst(it, tnow - hist_span())
            it = @view it[k:end]; ip = @view ip[k:end]; im = @view im[k:end]
        end
        if inp > 0 && !isempty(it)
            trel = it .- tnow          # newest sample sits at ~0, as in p1/p3
            # arrival rate over the window, so a stalled or slowed source is
            # visible rather than silently freezing the trace
            irate = length(it) > 1 ?
                (length(it) - 1) / max(it[end] - it[1], 1e-9) : 0.0
            # no legend box: two traces on two axes are identified by the
            # colour-matched axis labels, and a box would sit on top of the
            # data in a panel this short
            p5 = plot(trel, ip;
                      xlabel="time (s)", ylabel="peak (counts)",
                      color=:green, lw=1, legend=false,
                      yguidefontcolor=:green, ytickfontcolor=:green,
                      xlim=(xlo, 0), ylim=(0, 255),
                      title="Intensity   (" *
                            (idims == (0, 0) ? "in-loop stats" :
                             "$(idims[1])x$(idims[2]) frames") * ", " *
                            "$(round(irate, digits=1)) Hz, " *
                            "sat $(round(100*isat, digits=2))%)")
            # mean rides a separate axis: on a mostly-dark frame it is a couple
            # of counts against a peak of ~250, so a shared 0-255 axis would
            # flatten it onto the x axis and show nothing
            plot!(twinx(p5), trel, im;
                  ylabel="mean (counts)", color=:purple, lw=1, legend=false,
                  yguidefontcolor=:purple, ytickfontcolor=:purple,
                  xlim=(xlo, 0), right_margin=12Plots.mm)
            # intensity gets a shorter row: it is a slow, mostly-flat trace, and
            # an equal third of the figure would come out of the error panels
            lay = @layout [a b; c d; e{0.22h}]
            pl = plot(p1, p2, p3, p4, p5; layout=lay, size=(1200,800))
        else
            pl = plot(p1, p2, p3, p4; layout=(2,2), size=(1200,600))
        end
        display(pl)

        if time() - last_save >= SAVE_EVERY
            savefig(pl, save_path())
            last_save = time()
        end

        # surface a crashed acquisition task instead of rendering stale data
        istaskdone(acq) && wait(acq)
        iacq !== nothing && istaskdone(iacq) && wait(iacq)
        # yield for the rest of the cadence so acquisition drains whatever
        # queued in the kernel while this frame was drawing
        sleep(RENDER_PERIOD)
    end
end

close(sock)
isock === nothing || close(isock)
