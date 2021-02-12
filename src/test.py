###
# File created by Leonardo Cencetti on 2/12/21
###
import numpy as np


def copyVec(v1: np.ndarray, v2: np.ndarray):
    dim = v1.size
    for d in range(dim):
        v2.flat[d] = v1.flat[d]
    return v2



if __name__ == '__main__':
    a = np.ones(12)
    b = np.zeros([3, 4])
    copyVec(a, b)
    print(b)
