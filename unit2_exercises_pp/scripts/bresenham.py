class bresenham:
    # parameters are only 'start' point and 'end' point
    def __init__(self, start, end):
        self.start = list(start)
        self.end = list(end)
        self.path = []
        self.toggle=0

        # if start point and end point are same, then we don't need to calculate line.
        if start[0]-end[0]+start[1]-end[1]==0:
            return None

        # we should consider which variable is more steep between x and y.
        # return true if x is more steep than y.
        self.steep = abs(self.end[1]-self.start[1]) > abs(self.end[0]-self.start[0])

        # if x is more steep than y, swap x and y.
        # cause we assume that x should be gentle than y
        if self.steep:
            self.start = self.swap(self.start[0],self.start[1])
            self.end = self.swap(self.end[0],self.end[1])

        # if x of start is bigger than x of end, then toggle each other.
        # because we assume that x of end point is bigger than x of start point.
        if self.start[0] > self.end[0]:
            self.toggle=1
            #print 'flippin and floppin'
            _x0 = int(self.start[0])
            _x1 = int(self.end[0])
            self.start[0] = _x1
            self.end[0] = _x0

            _y0 = int(self.start[1])
            _y1 = int(self.end[1])
            self.start[1] = _y1
            self.end[1] = _y0

        # after all settings are done, we begin besenham algorithm.
        dx = self.end[0] - self.start[0]
        dy = abs(self.end[1] - self.start[1])
        error = 0
        derr = dy/float(dx)

        ystep = 0
        y = self.start[1]

        if self.start[1] < self.end[1]: ystep = 1
        else: ystep = -1

        # if we've changed x and y, we should draw line to y,x.
        # otherwise draw line to x,y.
        for x in range(self.start[0],self.end[0]+1):
            if self.steep:
                self.path.append((y,x))
            else:
                self.path.append((x,y))

            error += derr

            # if error is bigger than 0.5, then add ystep to y and make error to 0
            # because if the error is greater than 0.5, we know that the line has moved upward one pixel.
            if error >= 0.5:
                y += ystep
                error -= 1.0

        if self.toggle==1:
            self.path.reverse()

    def swap(self,n1,n2):
        return [n2,n1]