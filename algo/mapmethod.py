#!/usr/bin/env python3
import numpy as np

def map_from_descriptor(hex_str):
        pass

def map_from_file(path):
    with open(path) as f:
        return np.genfromtxt(f, dtype=int, delimiter=1)

def file_from_map(mapdata,path):
    return np.savetxt(path, mapdata, delimiter=1) 