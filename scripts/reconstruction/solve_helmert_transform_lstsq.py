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

    [tx, ty, tz, S, Srx, Sry, Srz] = xs.reshape((7,))
    # NOTE S = 1 + (s * 10e6), with s in ppm: s = (S - 1) * 1e-6
    [rx, ry, rz] = [Srx / S, Sry / S, Srz / S]  # angles in radians

    return (tx, ty, tz, S, rx, ry, rz), (xs, residuals, rank, singular)


def compute_helmert_matrix(tx, ty, tz, S, rx, ry, rz):
    t = np.array([tx, ty, tz]).reshape((3, 1))
    R = np.array([[1, -rz, ry], [rz, 1, -rx], [-ry, rx, 1]])
    return np.vstack((np.hstack((S * R, t)), [0, 0, 0, 1]))


def print_helmert_solution(tx, ty, tz, S, rx, ry, rz):
    print(f"{[tx, ty, tz] = }")
    print(f"{[rx, ry, rz] = }")
    print(f"{S = }")
    s = (S - 1) * 1e-6  # ppm
    print(f"{s = } = (S - 1) * 1e-6")


def print_lstsq_solution(As, bs, xs, residuals, rank, singular):
    print(f"{xs.shape = }, {As.shape = }, {bs.shape = }")
    As, bs, xs = As.squeeze(), bs.squeeze(), xs.squeeze()
    print(f"{xs.shape = }, {As.shape = }, {bs.shape = }")

    print(f"{xs = }")
    print(f"{As @ xs = }")
    print(f"{bs = }")

    # NOTE residuals is the squared Euclidean 2-norm for each column in `bs - As @ xs`
    print(f"{residuals = }")  # sums of squared residuals
    print(f"{rank = }")  # rank of matrix As
    print(f"{singular = }")  # singular values of As


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
    tx, ty, tz, S, rx, ry, rz = helmert_solution
    xs, residuals, rank, singular = lstsq_solution
    matrix = compute_helmert_matrix(tx, ty, tz, S, rx, ry, rz)

    print_lstsq_solution(As, bs, xs, residuals, rank, singular)
    print_helmert_solution(tx, ty, tz, S, rx, ry, rz)
    print(f"{matrix = }")
