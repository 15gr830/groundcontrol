#!/usr/bin/env python

takeoff_alt = 1
size_of_squar = 0.5
threshold = 0.1

sandbox = [1.5, 1.5, 2]

takeoff = [[0, 0, takeoff_alt*0.4],
           [0, 0, takeoff_alt]]

landing = [[0, 0, takeoff_alt*0.6],
           [0, 0, takeoff_alt*0.4],
           [0, 0, takeoff_alt*0.2],
           [0, 0, 0]]

square = [[0, size_of_squar, takeoff_alt],
          [size_of_squar, 0, takeoff_alt],
          [0, -size_of_squar, takeoff_alt], 
          [-size_of_squar, 0, takeoff_alt]]