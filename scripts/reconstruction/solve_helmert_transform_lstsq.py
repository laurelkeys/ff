import numpy as np
import numpy.linalg as la


# def helmert_A_B(x_source, x_target):
#     """ Reference: https://gis.stackexchange.com/a/84454 """
#     [x0, y0, z0], [xt, yt, zt] = x_source, x_target
#     #        [tx  ty  tz   S  S*rx  S*ry  S*rz]
#     ax, bx = [ 1,  0,  0, x0,    0,   z0,  -y0], xt
#     ay, by = [ 0,  1,  0, y0,  -z0,    0,   x0], yt
#     az, bz = [ 0,  0,  1, z0,   y0,  -x0,    0], zt
#     return np.vstack([ax, ay, az]), np.vstack([bx, by, bz])


def helmert_A(x_source):
    [x0, y0, z0] = x_source
    return np.vstack([
        [1, 0, 0, x0, 0, z0, -y0],
        [0, 1, 0, y0, -z0, 0, x0],
        [0, 0, 1, z0, y0, -x0, 0],
    ])


def helmert_B(x_target):
    return x_target.reshape((3, 1))


# NOTE
#   [tx, ty, tz] are dispacements in meters
#   S = (1 + s * 1e6), with s given in ppm
#   [rx, ry, rz] are rotations in radians


if __name__ == "__main__":

    xs_airsim = np.array([
        [-11.156498908996582, 2.528249740600586, -24.40956687927246],
        [-11.463881492614746, -16.7890567779541, -24.380529403686523],
        [12.605552673339844, 14.389413833618164, -24.343830108642578],
        [12.161497116088867, -12.899473190307617, -24.37879180908203],
    ])

    xs_meshroom = np.array([
        [0.63688847290049799, -0.17012024221971922, 0.57579369576066597],
        [-0.072042902759522631, 0.0036060512159483207, 0.35551471170296589],
        [0.68793063728854542, -0.92985931502239416, 1.221081888966717],
        [-0.27034800528631592, -0.77850072331406606, 0.85769238693834615],
    ])

    # As, Bs = [], []
    # for x_airsim, x_meshroom in zip(xs_airsim, xs_meshroom):
    #     a, b = helmert_A_B(x_source=x_meshroom, x_target=x_airsim)
    #     As.append(a)
    #     Bs.append(b)
    # A, B = np.vstack(As), np.vstack(Bs)

    A = np.vstack([helmert_A(x_source) for x_source in xs_meshroom])
    B = np.vstack([helmert_B(x_target) for x_target in xs_airsim])
    X, residuals, rank, singular = la.lstsq(A, B, rcond=None)  # A @ X = B

    print(f"{X = }")

    print(f"{A.shape = }")  # A = "Coefficient" matrix (M, N)
    print(f"{B.shape = }")  # B = "Dependent variables" (M, K)
    print(f"{X.shape = }")  # X = Least-squares solution (N, K)

    print(f"{A @ X = }")
    print(f"{B = }")

    print(f"{residuals = }")  # (0,) if rank < N or M <= N else (K,)
    print(f"{rank = }")  # rank of matrix A
    print(f"{singular = }")  # min(M, N), singular values of A

    # NOTE
    #   [X1 X2 X3 X4 X5 X6 X7] = [tx ty tz S S*rx S*ry S*rz]
    #   => [tx, ty, tz] = [X1, X2, X3]
    #   => S = X4 ==> s = (X4 - 1) * 1e-6
    #   => [rx, ry, rz] = (1 / X4) * [X5, X6, X7]

    [X1, X2, X3, X4, X5, X6, X7] = X.reshape((7,))
    [tx, ty, tz] = [X1, X2, X3]
    S, s = X4, (X4 - 1) * 1e-6  # s in ppm
    [rx, ry, rz] = (1 / X4) * np.array([X5, X6, X7])

    print(f"{[tx, ty, tz] = }")
    print(f"{[rx, ry, rz] = }")
    print(f"{S = }")
    print(f"{s = }")
