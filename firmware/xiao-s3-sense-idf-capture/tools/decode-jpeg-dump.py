"""Turn the capture firmware's serial dump into JPEG files, then let ffprobe
decide whether they are real pictures.

The board saying "I captured 150 frames" is a self-metric. This is the
outside instrument: the frames come off the chip as `JPEGDATA` hex lines, get
reassembled here, and are handed to ffprobe, which reports the geometry it
finds in the bitstream, and to ffmpeg, which has to actually decode the
entropy-coded data to produce a pixel checksum. A file that ffprobe reads but
ffmpeg cannot decode is a truncated frame, and that distinction is the whole
point of running both.

Three independent things get checked per frame:

1. The byte count the firmware announced matches the bytes that arrived.
2. The JPEG markers are intact: it starts with SOI and ends with EOI. A
   camera driver handing back a short buffer produces exactly this failure,
   and it is the one a frame counter cannot see.
3. ffprobe's width and height match what the firmware configured, and ffmpeg
   decodes it to completion.

Usage:
    python decode-jpeg-dump.py <monitor-capture.txt> [outdir]
"""
import os
import re
import subprocess
import sys

BEGIN = re.compile(
    r"^\s*JPEG begin index=(\d+) bytes=(\d+) sequence=(\d+) width=(\d+) height=(\d+)"
)
DATA = re.compile(r"^\s*JPEGDATA\s+([0-9a-fA-F]+)\s*$")
END = re.compile(r"^\s*JPEG end index=(\d+)")

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


def parse(path):
    """Yield (announced, bytes) per frame in the capture."""
    cur = None
    buf = bytearray()
    for line in open(path, encoding="utf-8", errors="replace"):
        m = BEGIN.match(line)
        if m:
            cur = {
                "index": int(m.group(1)),
                "bytes": int(m.group(2)),
                "sequence": int(m.group(3)),
                "width": int(m.group(4)),
                "height": int(m.group(5)),
            }
            buf = bytearray()
            continue
        if cur is None:
            continue
        m = DATA.match(line)
        if m:
            buf += bytes.fromhex(m.group(1))
            continue
        if END.match(line):
            yield cur, bytes(buf)
            cur = None


def ffprobe(path):
    try:
        r = subprocess.run(
            ["ffprobe", "-hide_banner", "-v", "error", "-show_entries",
             "stream=width,height,codec_name,pix_fmt",
             "-of", "default=noprint_wrappers=1", path],
            capture_output=True, text=True, timeout=60,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return {"error": str(e)}
    out = {}
    for line in r.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    if r.returncode != 0:
        out["error"] = r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "ffprobe failed"
    return out


def ffmpeg_decodes(path):
    """Decoding is the check ffprobe cannot make: a truncated frame still has
    a readable header."""
    try:
        r = subprocess.run(
            ["ffmpeg", "-hide_banner", "-v", "error", "-i", path,
             "-f", "framecrc", "-"],
            capture_output=True, text=True, timeout=60,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return False, str(e)
    crc = ""
    for line in r.stdout.splitlines():
        if line.startswith("0,"):
            crc = line.strip().split(",")[-1]
    return r.returncode == 0 and bool(crc), (crc or r.stderr.strip()[:120])


def main() -> int:
    src = sys.argv[1]
    outdir = sys.argv[2] if len(sys.argv) > 2 else "frames"
    os.makedirs(outdir, exist_ok=True)

    frames = list(parse(src))
    if not frames:
        print("no JPEG frames found in the capture", file=sys.stderr)
        return 1
    print(f"{len(frames)} frame(s) reassembled from the dump\n")

    failures = 0
    for meta, data in frames:
        path = os.path.join(outdir, f"frame{meta['index']}.jpg")
        open(path, "wb").write(data)
        print(f"frame {meta['index']}  seq={meta['sequence']}  -> {path}")

        size_ok = len(data) == meta["bytes"]
        print(
            f"  bytes announced={meta['bytes']} arrived={len(data)}"
            f"  {'match' if size_ok else 'MISMATCH'}"
        )
        soi = data[:2] == SOI
        eoi = data[-2:] == EOI
        print(
            f"  markers  SOI={'yes' if soi else 'NO'}  EOI={'yes' if eoi else 'NO'}"
            f"   ({'intact' if soi and eoi else 'TRUNCATED'})"
        )

        probe = ffprobe(path)
        if "error" in probe:
            print(f"  ffprobe  {probe['error']}")
        else:
            geo_ok = (
                probe.get("width") == str(meta["width"])
                and probe.get("height") == str(meta["height"])
            )
            print(
                f"  ffprobe  {probe.get('codec_name')} "
                f"{probe.get('width')}x{probe.get('height')} {probe.get('pix_fmt')}"
                f"   {'matches the firmware' if geo_ok else 'DISAGREES with the firmware'}"
            )
            if not geo_ok:
                failures += 1

        ok, detail = ffmpeg_decodes(path)
        print(f"  ffmpeg   {'decoded, crc=' + detail if ok else 'FAILED: ' + detail}")
        if not (size_ok and soi and eoi and ok):
            failures += 1
        print()

    print(f"Verdict: {len(frames) - min(failures, len(frames))}/{len(frames)} frames "
          f"survived every check")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
