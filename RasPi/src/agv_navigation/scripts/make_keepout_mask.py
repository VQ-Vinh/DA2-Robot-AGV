#!/usr/bin/env python3
"""Tao mat na vung cam (keepout) cho Nav2 tu ban do kho.

Pallet cao 15 cm, lidar quet o 17 cm nen khong thay -> Nav2 phai biet truoc qua
mat na: cung kich thuoc / goc toa do voi ban do, pixel den = cam vao.

  python3 make_keepout_mask.py            # doc maps/warehouse.yaml, ghi maps/keepout_mask.*

Dinh dang theo nav2_costmap_filters_demo: mode scale, den (0) = 100 = cam, trang = tu do.
"""
import os

import yaml
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, '..', 'maps')

# Frame map = world doi di cho xe xuat phat luc lap ban do (world x = -4.5, y = 0)
START_X, START_Y = -4.5, 0.0
MARGIN = 0.05   # m, noi rong moi canh
# (tam x, tam y, dai x, rong y) trong frame world, lay tu agv_gazebo/worlds/warehouse.sdf
ZONES = [
    (5.0, 2.8, 1.0, 0.8),    # pallet_1
    (5.0, -2.8, 1.0, 0.8),   # pallet_2
]


def main():
    meta = yaml.safe_load(open(os.path.join(MAPS, 'warehouse.yaml')))
    w, h = Image.open(os.path.join(MAPS, meta['image'])).size
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]

    mask = Image.new('L', (w, h), 255)
    draw = ImageDraw.Draw(mask)
    for cx, cy, sx, sy in ZONES:
        x0 = cx - START_X - sx / 2 - MARGIN
        x1 = cx - START_X + sx / 2 + MARGIN
        y0 = cy - START_Y - sy / 2 - MARGIN
        y1 = cy - START_Y + sy / 2 + MARGIN
        # Anh: cot = (x - ox) / res, hang dem tu tren xuong = h - (y - oy) / res
        draw.rectangle([(x0 - ox) / res, h - (y1 - oy) / res,
                        (x1 - ox) / res, h - (y0 - oy) / res], fill=0)

    mask.save(os.path.join(MAPS, 'keepout_mask.pgm'))
    with open(os.path.join(MAPS, 'keepout_mask.yaml'), 'w', newline='\n') as f:
        f.write(f"image: keepout_mask.pgm\nmode: scale\nresolution: {res}\n"
                f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f'mat na {w}x{h}, {len(ZONES)} vung cam')


if __name__ == '__main__':
    main()
