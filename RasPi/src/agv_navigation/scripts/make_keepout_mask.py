#!/usr/bin/env python3
"""Tao mat na vung cam (keepout) cho Nav2 tu ban do kho.

Vung cam:
  1. Cac o ke mini (agv_mission/config/shelves.yaml): khong cho lap duong xuyen gam ke
     (xe vua lot giua 4 chan, nhung chi docking_server moi duoc chui vao, cham va thang hang).
  2. Dai phia SAU moi day ke (tu mep o ke ra toi tuong, suot chieu dai day): loi do chi rong ~1.17 m
     (mep ke -> ke co dinh sat tuong), xe dang cho ke (0.88 x 0.78 m, duong cheo 1.18 m) khong quay
     dau duoc. Khi loi giua bi chan, Nav2 lap duong vong ra sau day ke va xe ket o do (REPORT.md 4.17).
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
BACK_DEPTH = 3.0   # m, dai cam sau day ke keo ra toi ngoai ban do (qua tuong)


def zones():
    """(tam x, tam y, dai x, rong y) frame map: moi o ke = ke + 4 cm moi ben (vach son tren san)."""
    lay = yaml.safe_load(open(SHELVES))
    size = lay['shelf']['size'] + 0.08
    out = [(sl['x'], sl['y'], size, size) for sl in lay['slots'].values()]
    # dai sau moi day ke: cac o cung huong (yaw) la mot day, "sau" = phia xe huong toi khi chui vao
    rows = {}
    for sl in lay['slots'].values():
        rows.setdefault(round(sl['yaw'], 2), []).append(sl)
    for yaw, slots in rows.items():
        xs = [sl['x'] for sl in slots]
        side = 1.0 if yaw > 0 else -1.0                 # day bac (yaw +pi/2): phia sau la +y
        y_edge = slots[0]['y'] + side * size / 2
        depth = BACK_DEPTH
        out.append(((min(xs) + max(xs)) / 2, y_edge + side * depth / 2, max(xs) - min(xs) + size, depth))
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
