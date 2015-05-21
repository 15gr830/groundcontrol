#!/usr/bin/env python

takeoff_alt = 1
size_of_square = 0.5
threshold = 0.3
sandbox_buffer = 0.3

sandbox = [2, 2, 2]
safezone = [sandbox[0]-sandbox_buffer, sandbox[1]-sandbox_buffer, sandbox[2]-sandbox_buffer]

takeoff = [[0, 0, takeoff_alt]]

landing = [[0, 0, takeoff_alt*0.6],
           [0, 0, takeoff_alt*0.4],
           [0, 0, takeoff_alt*0.2],
           [0, 0, 0]]

square = [[0, size_of_square, takeoff_alt],
          [size_of_square, 0, takeoff_alt],
          [0, -size_of_square, takeoff_alt], 
          [-size_of_square, 0, takeoff_alt]]