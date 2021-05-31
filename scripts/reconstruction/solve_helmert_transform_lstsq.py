import numpy as np
import numpy.linalg as la


def compute_helmert_A_b(xs_source, xs_target):
    """ Reference: https://gis.stackexchange.com/a/84454 """
    compute_b = lambda xt, yt, zt: np.vstack([xt, yt, zt])
    compute_a = lambda x0, y0, z0: np.vstack(
        [[1, 0, 0, x0, 0, z0, -y0], [0, 1, 0, y0, -z0, 0, x0], [0, 0, 1, z0, y0, -x0, 0]]
    )

    # #        [tx  ty  tz   S  S*rx  S*ry  S*rz]
    # ax, bx = [ 1,  0,  0, x0,    0,   z0,  -y0], xt
    # ay, by = [ 0,  1,  0, y0,  -z0,    0,   x0], yt
    # az, bz = [ 0,  0,  1, z0,   y0,  -x0,    0], zt
    # A = np.vstack([ax, ay, az])
    # b = np.vstack([bx, by, bz])

    As = np.vstack([compute_a(*x_source) for x_source in xs_source])
    bs = np.vstack([compute_b(*x_target) for x_target in xs_target])

    return As, bs


def solve_helmert_lstsq(As, bs):
    xs, residuals, rank, singular = la.lstsq(As, bs, rcond=None)

    M, N = As.shape  # "coefficient" matrix
    m, K = bs.shape  # "dependent variables"
    n, k = xs.shape  # least-squares solution
    assert M == m and N == n and K == k  # As @ xs = bs

    [x0, x1, x2, x3, x4, x5, x6] = xs.reshape((7,))  # [tx ty tz S S*rx S*ry S*rz]
    [tx, ty, tz] = [x0, x1, x2]  # dispacements in meters
    s = (x3 - 1) * 1e-6  # S = 1 + (s * 10e6), with s in ppm
    [rx, ry, rz] = [x4 / x3, x5 / x3, x6 / x3]  # angles in radians

    return ([tx, ty, tz], s, [rx, ry, rz]), (xs, residuals, rank, singular)


def compute_helmert_matrix(tx, ty, tz, s, rx, ry, rz):
    t = np.array([tx, ty, tz]).reshape((3, 1))
    R = np.array([[1, -rz, ry], [rz, 1, -rx], [-ry, rx, 1]])
    return np.vstack((np.hstack((s * R, t)), [0, 0, 0, 1]))


if __name__ == "__main__":
    xs_airsim = np.array(
        [
            [-11.156498908996582, 2.528249740600586, -24.40956687927246],
            [-11.463881492614746, -16.7890567779541, -24.380529403686523],
            [12.605552673339844, 14.389413833618164, -24.343830108642578],
            [12.161497116088867, -12.899473190307617, -24.37879180908203],
        ]
    )

    xs_meshroom = np.array(
        [
            [0.63688847290049799, -0.17012024221971922, 0.57579369576066597],
            [-0.072042902759522631, 0.0036060512159483207, 0.35551471170296589],
            [0.68793063728854542, -0.92985931502239416, 1.221081888966717],
            [-0.27034800528631592, -0.77850072331406606, 0.85769238693834615],
        ]
    )

    As, bs = compute_helmert_A_b(xs_source=xs_meshroom, xs_target=xs_airsim)
    helmert_solution, lstsq_solution = solve_helmert_lstsq(As, bs)
    [tx, ty, tz], s, [rx, ry, rz] = helmert_solution
    xs, residuals, rank, singular = lstsq_solution

    print(f"{xs.shape = }, {As.shape = }, {bs.shape = }")
    xs = xs.squeeze()
    As = As.squeeze()
    bs = bs.squeeze()
    print(f"{xs.shape = }, {As.shape = }, {bs.shape = }")

    print(f"{xs = }")
    print(f"{As @ xs = }")
    print(f"{bs = }")

    # NOTE residuals is the squared Euclidean 2-norm for each column in `bs - As @ xs`
    print(f"{residuals = }")  # sums of squared residuals
    print(f"{rank = }")  # rank of matrix As
    print(f"{singular = }")  # singular values of As

    print(f"{[tx, ty, tz] = }")  # [-11.174689594235451, -17.173427747128173, -22.082146377245863]
    print(f"{[rx, ry, rz] = }")  # [0.9088041769608071, -0.13435949734005437, -4.220948131978764]
    print(f"{s = }")  # -7.385906801894286e-06
    S = 1 + (s * 10e6)
    print(f"{S = }")

    matrix = compute_helmert_matrix(tx, ty, tz, s, rx, ry, rz)
    print(f"{matrix = }")
    # np.array(
    #     [
    #         [-7.38590680e-06, -3.11755295e-05,  9.92366725e-07, -1.11746896e+01],
    #         [ 3.11755295e-05, -7.38590680e-06,  6.71234295e-06, -1.71734277e+01],
    #         [-9.92366725e-07, -6.71234295e-06, -7.38590680e-06, -2.20821464e+01],
    #         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00],
    #     ]
    # )
