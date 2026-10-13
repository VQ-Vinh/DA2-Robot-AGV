#!/usr/bin/env python3
"""Tao mat na vung cam (keepout) cho Nav2 tu ban do kho.

Vung cam = cac o ke mini (agv_mission/config/shelves.yaml): khong cho lap duong xuyen gam ke
(xe vua lot giua 4 chan, nhung chi docking_server moi duoc chui vao, cham va thang hang).
Truoc day vung cam la 2 pallet thap ma lidar o 0.17 m khong thay; lidar da ha xuong 0.12 m
nen pallet nam ngay trong ban do.

  python3 make_keepout_mask.py            # doc maps/warehouse.yaml, ghi maps/keepout_mask.*

Dinh dang theo nav2_costmap_filters_demo: mode scale, den (0) = 100 = cam, trang = tu do.
"""
import os

import yaml
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, '..', 'maps')

SHELVES = os.path.join(HERE, '..', '..', 'agv_mission', 'config', 'shelves.yaml')
MARGIN = 0.05   # m, noi rong moi canh


def zones():
    """(tam x, tam y, dai x, rong y) frame map: moi o ke = ke + 4 cm moi ben (vach son tren san), va hanh lang
    sau moi day o (tu mep o toi |y| = back_keepout_y): qua hep cho xe cho pallet quay (REPORT.md 4.19)."""
    lay = yaml.safe_load(open(SHELVES))
    size = lay['shelf']['size'] + 0.08
    out = [(sl['x'], sl['y'], size, size) for sl in lay['slots'].values()]
    back = lay['shelf'].get('back_keepout_y')
    if back:
        for side in (1, -1):
            row = [sl for sl in lay['slots'].values() if sl['y'] * side > 0]
            if not row:
                continue
            x0 = min(sl['x'] for sl in row) - size / 2
            x1 = max(sl['x'] for sl in row) + size / 2
            y0 = min(abs(sl['y']) for sl in row) + size / 2
            out.append(((x0 + x1) / 2, side * (y0 + back) / 2, x1 - x0, back - y0))
    return out


def main():
    meta = yaml.safe_load(open(os.path.join(MAPS, 'warehouse.yaml')))
    w, h = Image.open(os.path.join(MAPS, meta['image'])).size
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]

    mask = Image.new('L', (w, h), 255)
    draw = ImageDraw.Draw(mask)
    zs = zones()
    for cx, cy, sx, sy in zs:
        x0 = cx - sx / 2 - MARGIN
        x1 = cx + sx / 2 + MARGIN
        y0 = cy - sy / 2 - MARGIN
        y1 = cy + sy / 2 + MARGIN
        # Anh: cot = (x - ox) / res, hang dem tu tren xuong = h - (y - oy) / res
        draw.rectangle([(x0 - ox) / res, h - (y1 - oy) / res,
                        (x1 - ox) / res, h - (y0 - oy) / res], fill=0)

    mask.save(os.path.join(MAPS, 'keepout_mask.pgm'))
    with open(os.path.join(MAPS, 'keepout_mask.yaml'), 'w', newline='\n') as f:
        f.write(f"image: keepout_mask.pgm\nmode: scale\nresolution: {res}\n"
                f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f'mat na {w}x{h}, {len(zs)} vung cam')


if __name__ == '__main__':
    main()
