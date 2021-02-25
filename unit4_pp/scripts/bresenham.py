#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Implementation of Bresenham's line drawing algorithm
See en.wikipedia.org/wiki/Bresenham's_line_algorithm
Source: The bresenham module https://github.com/encukou/bresenham
Copyright © 2016 Petr Viktorin
encukou/bresenham is licensed under the MIT License
Usage:
For example, the coordinates of a line from (-1, -4) to (3, 2), are:
list(bresenham(-1, -4, 3, 2))
[(-1, -4), (0, -3), (0, -2), (1, -1), (2, 0), (2, 1), (3, 2)]
"""


def bresenham(x0, y0, x1, y1):
    """Yield integer coordinates on the line from (x0, y0) to (x1, y1).

    Input coordinates should be integers.

    The result will contain both the start and the end point.
    """
    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy
