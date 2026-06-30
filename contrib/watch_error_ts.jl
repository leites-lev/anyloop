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
# --pixel/-p SIZE: region (subaperture) size in pixels, used to convert the
#                 center_of_mass output (normalized to -1:1 across the region)
#                 into pixels for the time-series panels (default 248). The
#                 conversion is pixels = value * (SIZE - 1)/2. Only affects the
#                 time series; the spectrum is unchanged.
# --fmin/-m HZ:   lower limit of the spectrum frequency axis in Hz (default 0).
#                 Ignored unless it is below the effective max frequency.
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
    pixel = 248.0   # region size in pixels (default 248x248)
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
# center_of_mass output is normalized to -1:1 across the region; convert to
# pixels with value * (PIXEL - 1)/2 for the time-series panels
const PIX_PER_UNIT = (PIXEL - 1) / 2
const N = max(round(Int, SECONDS * FS), 2)
const SAVE_EVERY = 40.0  # seconds between saves

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

# --com: text readout of the centroid, no plotting
if COM
    println("--com: printing [y, x] CoM (~5 Hz) with running mean of valid samples; Ctrl-C to stop")
    let sum_y = 0.0, sum_x = 0.0, n = 0, last_print = 0.0
        while true
            chunk = read(IOBuffer(recv(sock)), AYLPChunk)
            @assert length(chunk.data) == 2 "expected 2-element [y,x] vector"
            y = chunk.data[1]; x = chunk.data[2]
            # skip non-finite centroids (empty/low-signal frames) so one bad
            # frame doesn't poison the cumulative mean with NaN
            if isfinite(y) && isfinite(x)
                sum_y += y; sum_x += x; n += 1
            end
            now = time()
            if now - last_print >= 0.2
                mean_str = n > 0 ?
                    string("y=", round(sum_y/n, digits=4), " x=", round(sum_x/n, digits=4)) :
                    "(no valid samples yet)"
                println("y=", round(y, digits=4), "  x=", round(x, digits=4),
                        "   mean: ", mean_str, "  (", n, " valid)")
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

# RMS (in px) about the window mean (the moving average) over the newest nvalid
# samples of an oldest->newest window — i.e. the std dev of the on-screen signal,
# excluding any DC bias. The tail selection skips the zero-prefilled part of the
# ring before it has filled.
function rms_px(win, nvalid, scale)
    nvalid < 1 && return NaN
    v = @view win[end-nvalid+1:end]
    m = sum(v) / nvalid
    return sqrt(sum(x -> abs2(x - m), v) / nvalid) * scale
end

# oldest -> newest copy of a ring buffer, given the next-write index i
snapshot(buf, i) = vcat(buf[i:end], buf[1:i-1])

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
        db_y   = 20 .* log10.(spec_y ./ max(maximum(spec_y), 1e-12))
        db_x   = 20 .* log10.(spec_x ./ max(maximum(spec_x), 1e-12))

        # windowed RMS error (px) over the real, on-screen samples; shown only
        # after the warmup so an early noisy estimate isn't displayed
        nvalid   = min(np, N)
        show_rms = tstart[] > 0.0 && time() - tstart[] >= AVG_DELAY
        rms_y_px = show_rms ? rms_px(sy, nvalid, PIX_PER_UNIT) : NaN
        rms_x_px = show_rms ? rms_px(sx, nvalid, PIX_PER_UNIT) : NaN

        p1 = plot(tvec, sy .* PIX_PER_UNIT;
                  label="y (tip)", ylabel="error (px)", color=:blue, lw=1,
                  title="Time series   (fs ≈ $(round(fs, digits=1)) Hz)")
        if show_rms
            hline!(p1, [rms_y_px, -rms_y_px]; color=:black, ls=:dash, lw=1,
                   label="RMS = $(round(rms_y_px, sigdigits=3)) px")
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
            hline!(p3, [rms_x_px, -rms_x_px]; color=:black, ls=:dash, lw=1,
                   label="RMS = $(round(rms_x_px, sigdigits=3)) px")
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
