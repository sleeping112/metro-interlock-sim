#!/usr/bin/env python3
"""Generate small procedural station-sign textures (no external downloads)."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parents[1] / (
    "ws_metro/src/metro_description/models/metro_station/textures"
)

FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REGULAR = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def font(size, bold=True):
    try:
        return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)
    except OSError:
        return ImageFont.load_default()


def station_name():
    image = Image.new("RGB", (1024, 192), (12, 30, 54))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, 1024, 14), fill=(230, 180, 30))
    draw.text((34, 46), "METRO CENTRAL", font=font(74), fill=(245, 247, 250))
    draw.text((38, 132), "LINE 1  ·  CENTRAL STATION", font=font(30, False),
              fill=(170, 190, 215))
    image.save(OUT / "station_name.png")


def line_map():
    image = Image.new("RGB", (768, 384), (245, 246, 248))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, 768, 12), fill=(12, 30, 54))
    draw.text((32, 28), "LINE 1", font=font(42), fill=(12, 30, 54))
    draw.text((32, 82), "CENTRAL ↔ RIVERSIDE", font=font(24, False),
              fill=(70, 82, 96))
    y = 220
    draw.line((40, y, 728, y), fill=(20, 90, 170), width=16)
    stations = [
        ("AIRPORT", 70), ("NORTH GATE", 170), ("MUSEUM", 270),
        ("CENTRAL", 390), ("MARKET", 500), ("RIVERSIDE", 650),
    ]
    for name, x in stations:
        radius = 14 if name != "CENTRAL" else 20
        draw.ellipse((x - radius, y - radius, x + radius, y + radius),
                     fill=(255, 255, 255), outline=(20, 90, 170), width=6)
        draw.text((x - 30, y + 34), name, font=font(16, False),
                  fill=(12, 30, 54))
    image.save(OUT / "line_map.png")


def exit_sign():
    image = Image.new("RGB", (512, 128), (20, 120, 70))
    draw = ImageDraw.Draw(image)
    draw.text((30, 28), "EXIT  →", font=font(64), fill=(245, 250, 245))
    image.save(OUT / "exit_sign.png")


def mind_the_gap():
    image = Image.new("RGB", (768, 128), (236, 186, 26))
    draw = ImageDraw.Draw(image)
    draw.text((30, 24), "MIND THE GAP  ·  STAND CLEAR", font=font(40),
              fill=(30, 30, 30))
    image.save(OUT / "mind_the_gap.png")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    station_name()
    line_map()
    exit_sign()
    mind_the_gap()
    print(f"wrote station textures to {OUT}")


if __name__ == "__main__":
    main()
