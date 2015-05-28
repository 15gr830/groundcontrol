#!/usr/bin/env python

takeoff_alt = 0.8
size_of_square = 0.50
threshold = 0.1
sandbox_buffer = 0.30

sandbox = [1, 1.5, 2]
safezone = [sandbox[0]-sandbox_buffer, sandbox[1]-sandbox_buffer, sandbox[2]-sandbox_buffer]

takeoff = [[0, 0, takeoff_alt]]

landing = [[0, 0, 0]]

# square = [[0, size_of_square, takeoff_alt],
#           [size_of_square, 0, takeoff_alt],
#           [0, -size_of_square, takeoff_alt], 
#           [-size_of_square, 0, takeoff_alt]]

square = [[0,1, takeoff_alt]]