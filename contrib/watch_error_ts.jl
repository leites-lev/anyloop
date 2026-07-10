#!/usr/bin/julia
# Rolling time-series + PSD plot of the [y, x] error signal from center_of_mass.
# Listens on UDP port 64732.
include("anyloop.jl")
using .Anyloop
using Plots; gr()
using Sockets
using FFTW
using Dates

const FS = 815.0
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
#                 (default 64). The conversion is pixels = value * (SIZE - 1)/2.
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
function parse_args(args)
    outfile = joinpath(@__DIR__, "..", "spectrum.png")
    timestamp = false
    seconds = DEFAULT_SECONDS
    com = false
    pixel = 64.0    # span the CoM is normalized across (image size in track mode)
    fmin  = 0.0     # spectrum min frequency (Hz); 0 = from DC
    fmax  = 0.0     # spectrum max frequency (Hz); 0 = auto (Nyquist)
    xtick = 0.0     # spectrum x-axis tick spacing (Hz); 0 = auto
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
    return outfile, timestamp, seconds, com, pixel, fmin, fmax, xtick
end

const OUTFILE, TIMESTAMP, SECONDS, COM, PIXEL, FMIN, FMAX, XTICK = parse_args(ARGS)
# center_of_mass output is normalized to -1:1 across PIXEL px -- the whole image
# in track mode, the region otherwise; convert to pixels with value*(PIXEL-1)/2
const PIX_PER_UNIT = (PIXEL - 1) / 2
const N = max(round(Int, SECONDS * FS), 2)
const SAVE_EVERY = 400.0  # seconds between saves

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
                  label="x (tilt)", xlabel="time (s)", ylabel="error (px)", color=:red, lw=1)
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
        pl = plot(p1, p2, p3, p4; layout=(2,2), size=(1200,600))
        display(pl)

        if time() - last_save >= SAVE_EVERY
            savefig(pl, save_path())
            last_save = time()
        end

        # surface a crashed acquisition task instead of rendering stale data
        istaskdone(acq) && wait(acq)
        # yield for the rest of the cadence so acquisition drains whatever
        # queued in the kernel while this frame was drawing
        sleep(RENDER_PERIOD)
    end
end

close(sock)
