import numpy as np
from algo.mapmethod import map_from_file


def MDFString():
    map = map_from_file('./map/currentmap.txt')

    bits = ''
    for row in map[::-1, :]:
        for bit in row:
            if bit == 2:
                bits += '1'
            elif bit != 0:
                bits += '0'
    bits += '0'*(4 - len(bits) % 4)
    hex_str = ['%X' % int(bits[i:i+4], 2)
               for i in range(0, len(bits)-3, 4)]
    retString = 'MDF : ' + ''.join(hex_str) + '\n'+'MDF : '
    descriptor = np.zeros([20, 15]).astype(int)
    descriptor[map[::-1, :] != 0] = 1
    bits = '11'
    for row in descriptor:
        string = ""
        for i in row:
            string += str(i)
        bits += string
    bits += '11'
    hex_str = ['%X' % int(bits[i:i+4], 2)
               for i in range(0, len(bits)-3, 4)]
    retString += ''.join(hex_str) + '\n'
    print(retString)
    # arr = []
    # for i in retString:
    #     arr.append(i)
    # for i in range(len(arr)):
    #     if arr[i] == 'F':
    #         arr[i] = 'G'
    # retString = ""
    # for i in arr:
    #     retString+=i

    return retString
MDFString()