import numpy as np
import cv2

fname = input('Enter filename to convert: ')
im = cv2.imread(fname)
im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
fname = fname[:-4]

txt = open(fname + '.txt', 'w')
for row in range(im.shape[0]):
  for col in range(im.shape[1]):
    txt.write('  ' + str(int(im[row, col] == 0)))

  txt.write('\n')

txt.close()
