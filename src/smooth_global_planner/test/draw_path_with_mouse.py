#!/usr/bin/env python
from tkinter import *

def paint(event):
    x1,y1 = (event.x - 0.1),(event.y - 0.1)
    x2,y2 = (event.x + 0.1),(event.y + 0.1)
    print('x1',x1,'y1',y1)
    data.write(str(x1) + ' ' + str(500-y1) + '\n')
    cv.create_oval(x1, y1, x2, y2, fill='green')

data = open('path_point.txt','w')
root = Tk()
cv = Canvas(root, width=1200, height=500)
cv.pack(expand=YES, fill=BOTH)
cv.bind('<B1-Motion>',paint)

mainloop()
data.close()