#!/usr/bin/env python3
"""Read a T-Deck SD rxlog and answer "what was the radio doing, when, and did it cost us?"

    python3 tools/rxlog_timeline.py /path/rx-boot0014.log [--origin LAT,LON] [--csv out.csv]

Row shape (see the '# fmt' header the firmware writes):
    <ms> <ev> <len> <rssi|toa_ms|sats> <snr|hdop> <detail>
Events: rx, tx, pong, miss, gps, rtc, noise, note, chunk.

What it produces
  * wall clock, if any 'rtc' row exists (ms -> epoch mapping from the moment of sync)
  * a radio-state timeline: every TX interval (tx rows carry ToA) and every RX interval
    (arrival is the END of the packet; the start is arrival - ToA(len) at SF9/BW125/CR4:6)
  * duty cycle: TX / RX / idle share of the session
  * Range: per-responder answer rate, RSSI, hop mix; each miss classified as
        self-inflicted  our own TX overlapped the 1..4 s window a PONG would have landed in
        channel-busy    somebody else's frame occupied that window (we heard it)
        silent          nothing on air: real loss, or nobody answered
  * distance from origin per gps row (first fix if --origin not given), and a per-minute
    table of distance vs. per-responder median RSSI so a walk reads as a range curve
"""
import argparse, math, re, sys, collections, datetime

SF, BW, CR_DENOM, PREAMBLE = 9, 125000.0, 6, 8   # lora_rf.h: SF9 / BW125 / CR4:6 / preamble 8, explicit header, CRC off

def toa_ms(nbytes, sf=SF, bw=BW, cr=CR_DENOM - 4, preamble=PREAMBLE, crc=0, header=0):
    tsym = (2 ** sf) / bw
    de = 1 if (sf >= 11 and bw == 125000) else 0
    num = 8 * nbytes - 4 * sf + 28 + 16 * crc - 20 * header
    den = 4 * (sf - 2 * de)
    npay = 8 + max(math.ceil(num / den) * (cr + 4), 0)
    return ((preamble + 4.25) + npay) * tsym * 1000

def hav(a, b, c, d):
    r = 6371000.0
    p1, p2 = math.radians(a), math.radians(c)
    h = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(d - b) / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))

ROW = re.compile(r"^(\d+) (\w+) (\d+) (-?\d+) (-?[\d.]+) ?(.*)$")

def load(path):
    rows, hdr = [], []
    for line in open(path, encoding="utf-8", errors="replace"):
        line = line.rstrip("\n")
        if line.startswith("#"):
            hdr.append(line); continue
        m = ROW.match(line)
        if m:
            rows.append(dict(ms=int(m[1]), ev=m[2], len=int(m[3]), a=int(m[4]), b=float(m[5]), d=m[6]))
    return hdr, rows

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--origin", help="LAT,LON of the base; default = first gps fix")
    ap.add_argument("--csv", help="write the per-event timeline as CSV")
    a = ap.parse_args()
    hdr, rows = load(a.log)
    if not rows:
        sys.exit("no rows")

    # --- wall clock: the LAST rtc row wins (a later sync is a better one)
    epoch0 = None
    for r in rows:
        if r["ev"] == "rtc":
            m = re.search(r"epoch=(\d+)", r["d"])
            if m: epoch0 = int(m[1]) - r["ms"] / 1000.0
    def wall(ms):
        if epoch0 is None: return f"{ms/1000:8.1f}s"
        return datetime.datetime.fromtimestamp(epoch0 + ms / 1000.0).strftime("%H:%M:%S")

    t0, t1 = rows[0]["ms"], rows[-1]["ms"]
    span = (t1 - t0) / 1000.0
    print(f"file {a.log}")
    for h in hdr[:1]: print(h)
    print(f"span {span:.0f} s   rows {len(rows)}   wall clock: {'yes (rtc row)' if epoch0 else 'NONE - millis only'}")

    # --- radio state intervals
    # tx: a = ToA ms (firmware >= 2026-09-10); older logs have 0 there, so compute it from len
    tx = [(r["ms"], r["ms"] + (r["a"] or toa_ms(r["len"])), r) for r in rows if r["ev"] == "tx"]
    rx = [(r["ms"] - toa_ms(r["len"]), r["ms"], r) for r in rows if r["ev"] in ("rx", "pong", "noise", "chunk")]
    tx_busy = sum(e - s for s, e, _ in tx); rx_busy = sum(e - s for s, e, _ in rx)
    print(f"duty: TX {tx_busy/1000:.1f} s ({100*tx_busy/(span*1000):.1f}%)   RX {rx_busy/1000:.1f} s ({100*rx_busy/(span*1000):.1f}%)   idle {100*(1-(tx_busy+rx_busy)/(span*1000)):.1f}%")

    # --- gps / distance
    fixes = []
    for r in rows:
        if r["ev"] == "gps":
            m = re.match(r"(-?\d+\.\d+),(-?\d+\.\d+)", r["d"])
            if m: fixes.append((r["ms"], float(m[1]), float(m[2]), r["a"], r["b"]))
    nofix = sum(1 for r in rows if r["ev"] == "gps" and r["d"].startswith("nofix"))
    if a.origin:
        olat, olon = map(float, a.origin.split(","))
    elif fixes:
        olat, olon = fixes[0][1], fixes[0][2]
    else:
        olat = olon = None
    print(f"gps rows: {len(fixes)} fixes, {nofix} nofix" + (f"   origin {olat:.6f},{olon:.6f}" if olat is not None else "   (no position: distance unavailable)"))
    def dist_at(ms):
        if olat is None or not fixes: return None
        best = min(fixes, key=lambda f: abs(f[0] - ms))
        if abs(best[0] - ms) > 15000: return None
        return hav(olat, olon, best[1], best[2])

    # --- Range
    pings = [r for r in rows if r["ev"] == "tx" and "PING|" in r["d"]]
    pongs = [r for r in rows if r["ev"] == "pong"]
    misses = [r for r in rows if r["ev"] == "miss"]
    # seq restarts at 0 every time the Range app is reopened, so one file can hold the
    # same seq several times: a reply belongs to the LAST PING of that seq before it.
    ping_seqs = collections.defaultdict(list)
    for r in pings:
        m = re.search(r"PING\|(\d+)\|", r["d"])
        if m: ping_seqs[int(m[1])].append(r)
    def ping_for(seq, before_ms):
        c = [p for p in ping_seqs.get(seq, []) if p["ms"] <= before_ms]
        return c[-1] if c else None
    if pings:
        print(f"\nRange: PING {len(pings)}  PONG {len(pongs)}  miss {len(misses)}")
        by = collections.defaultdict(list)
        for r in pongs:
            m = re.match(r"(\w+) h(\d) seq (\d+)( late)?", r["d"])
            if m: by[m[1]].append((r, int(m[2]), int(m[3]), bool(m[4])))
        for k, v in sorted(by.items(), key=lambda kv: -len(kv[1])):
            rs = sorted(x[0]["a"] for x in v)
            hops = collections.Counter(x[1] for x in v)
            print(f"  {k:5} answered {len(v):3}/{len(pings)} ({100*len(v)/len(pings):3.0f}%)  rssi {rs[0]}/{rs[len(rs)//2]}/{rs[-1]}  hops {dict(sorted(hops.items()))}  late {sum(x[3] for x in v)}")
        # miss classification: the PONG window is 1..4 s after the PING left
        cls = collections.Counter()
        for r in misses:
            m = re.search(r"seq (\d+)", r["d"]); seq = int(m[1]) if m else -1
            p = ping_for(seq, r["ms"])
            if not p: cls["unknown"] += 1; continue
            w0, w1 = p["ms"] + 1000, p["ms"] + 4000
            own = any(s < w1 and e > w0 for s, e, rr in tx if rr is not p)
            other = any(s < w1 and e > w0 for s, e, rr in rx)
            cls["self-inflicted" if own else ("channel-busy" if other else "silent")] += 1
        print(f"  misses: {dict(cls)}")
        # A damaged frame landing 1.0..2.0 s after a PING is where a fixed-hold PONG
        # arrives; two responders on the same hold collide exactly there.
        dmg = [r["ms"] for r in rows if r["ev"] == "noise" or (r["ev"] == "rx" and r["d"].startswith("CORRUPT"))]
        coll = sum(1 for p in pings if any(1000 <= t - p["ms"] <= 2000 for t in dmg))
        print(f"  PINGs with a damaged frame at +1.0..2.0 s (same-slot PONG collision signature): {coll}/{len(pings)}")
        bins = collections.Counter(round((t - p["ms"]) / 1000, 1) for p in pings for t in dmg if 500 <= t - p["ms"] <= 7500)
        print(f"  damaged-frame arrival after PING (s): {dict(sorted(bins.items()))}")
        # who answered together: a responder that only survives when another is silent
        # is being trampled by that one (or by its relay forward). Every pair, per PING.
        who = {p["ms"]: set() for p in pings}
        for r in pongs:
            m = re.match(r"(\w+) h\d seq (\d+)", r["d"])
            if m:
                p = ping_for(int(m[2]), r["ms"])
                if p: who[p["ms"]].add(m[1])
        names = sorted(by)
        print("  co-response per PING (rows: A answered / A silent; cols: B answered / B silent):")
        for i, A in enumerate(names):
            for B in names[i + 1:]:
                c = collections.Counter((A in v, B in v) for v in who.values())
                print(f"    {A}×{B}: both {c[(True,True)]:3}  {A}-only {c[(True,False)]:3}  {B}-only {c[(False,True)]:3}  neither {c[(False,False)]:3}")
        # arrival delay per responder, PING tx start -> PONG rx end
        for k in names:
            d = sorted((x[0]["ms"] - (ping_for(x[2], x[0]["ms"]) or x[0])["ms"]) / 1000 for x in by[k])
            print(f"  {k} delay: med {d[len(d)//2]:.2f}s  range {d[0]:.2f}-{d[-1]:.2f}")

        # per-minute walk table
        print("\n  min  wall      dist   " + "  ".join(f"{k:>10}" for k in sorted(by)) + "   miss")
        nb = int(span // 60) + 1
        for b in range(nb):
            lo, hi = t0 + b * 60000, t0 + (b + 1) * 60000
            if not any(lo <= p["ms"] < hi for p in pings): continue
            d = dist_at(lo + 30000)
            cells = []
            for k in sorted(by):
                vv = [x[0]["a"] for x in by[k] if lo <= x[0]["ms"] < hi]
                cells.append(f"{len(vv):2}@{sorted(vv)[len(vv)//2]:>4}" if vv else "     -    ")
            mm = sum(1 for r in misses if lo <= r["ms"] < hi)
            print(f"  {b:3}  {wall(lo)}  {d:5.0f}m  " if d is not None else f"  {b:3}  {wall(lo)}     -   ", "  ".join(f"{c:>10}" for c in cells), f"   {mm}")

    if a.csv:
        with open(a.csv, "w") as f:
            f.write("ms,wall,ev,len,a,b,start_ms,end_ms,dist_m,detail\n")
            for r in rows:
                s = e = ""
                if r["ev"] == "tx": s, e = r["ms"], r["ms"] + r["a"]
                elif r["ev"] in ("rx", "pong", "noise", "chunk"): s, e = round(r["ms"] - toa_ms(r["len"])), r["ms"]
                d = dist_at(r["ms"]); d = "" if d is None else f"{d:.0f}"
                f.write(f'{r["ms"]},{wall(r["ms"]).strip()},{r["ev"]},{r["len"]},{r["a"]},{r["b"]},{s},{e},{d},"{r["d"]}"\n')
        print(f"\ncsv -> {a.csv}")

if __name__ == "__main__":
    main()
