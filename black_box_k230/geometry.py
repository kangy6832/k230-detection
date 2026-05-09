"""Geometry helpers: corner ordering, square filtering, planar pose, and 3D corner recovery."""

import math


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(v):
    return math.sqrt(_dot(v, v))


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _mul_scalar(v, s):
    return [v[0] * s, v[1] * s, v[2] * s]


def _mat3_mul_vec(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def _mat3_inv(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]

    A = e * i - f * h
    B = -(d * i - f * g)
    C = d * h - e * g
    D = -(b * i - c * h)
    E = a * i - c * g
    F = -(a * h - b * g)
    G = b * f - c * e
    H = -(a * f - c * d)
    I = a * e - b * d

    det = a * A + b * B + c * C
    if abs(det) < 1e-9:
        return None

    inv_det = 1.0 / det
    return [
        [A * inv_det, D * inv_det, G * inv_det],
        [B * inv_det, E * inv_det, H * inv_det],
        [C * inv_det, F * inv_det, I * inv_det],
    ]


def _solve_linear_system(a, b):
    """Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [row[:] + [b[idx]] for idx, row in enumerate(a)]

    for col in range(n):
        pivot = col
        max_abs = abs(m[col][col])
        for row in range(col + 1, n):
            v = abs(m[row][col])
            if v > max_abs:
                max_abs = v
                pivot = row

        if max_abs < 1e-9:
            return None

        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]

        pivot_val = m[col][col]
        for j in range(col, n + 1):
            m[col][j] /= pivot_val

        for row in range(n):
            if row == col:
                continue
            factor = m[row][col]
            if abs(factor) < 1e-12:
                continue
            for j in range(col, n + 1):
                m[row][j] -= factor * m[col][j]

    return [m[i][n] for i in range(n)]


def order_corners_tl_tr_br_bl(points):
    """Return corners ordered as top-left, top-right, bottom-right, bottom-left."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) != 4:
        return None

    sums = [p[0] + p[1] for p in pts]
    diffs = [p[0] - p[1] for p in pts]

    tl = pts[sums.index(min(sums))]
    br = pts[sums.index(max(sums))]
    tr = pts[diffs.index(max(diffs))]
    bl = pts[diffs.index(min(diffs))]

    ordered = [tl, tr, br, bl]
    # Reject duplicates caused by degenerate detections.
    if len(set(ordered)) != 4:
        return None
    return ordered


def _distance(p0, p1):
    dx = p0[0] - p1[0]
    dy = p0[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


def is_square_like(ordered_points, max_aspect_deviation, max_side_rel_err):
    if ordered_points is None:
        return False

    tl, tr, br, bl = ordered_points
    top = _distance(tl, tr)
    right = _distance(tr, br)
    bottom = _distance(br, bl)
    left = _distance(bl, tl)

    if min(top, right, bottom, left) < 2.0:
        return False

    mean_side = (top + right + bottom + left) / 4.0
    side_err = max(
        abs(top - mean_side),
        abs(right - mean_side),
        abs(bottom - mean_side),
        abs(left - mean_side),
    ) / mean_side

    width = (top + bottom) * 0.5
    height = (left + right) * 0.5
    aspect = width / height if height > 1e-6 else 999.0

    return (
        abs(aspect - 1.0) <= max_aspect_deviation
        and side_err <= max_side_rel_err
    )


def solve_homography(world_xy, image_uv):
    """Solve homography H from 4 world plane points to 4 image points.

    H maps [X, Y, 1]^T -> [u, v, 1]^T up to scale.
    """
    if len(world_xy) != 4 or len(image_uv) != 4:
        return None

    a = []
    b = []
    for (xw, yw), (u, v) in zip(world_xy, image_uv):
        a.append([xw, yw, 1.0, 0.0, 0.0, 0.0, -u * xw, -u * yw])
        b.append(u)
        a.append([0.0, 0.0, 0.0, xw, yw, 1.0, -v * xw, -v * yw])
        b.append(v)

    h = _solve_linear_system(a, b)
    if h is None:
        return None

    h11, h12, h13, h21, h22, h23, h31, h32 = h
    return [
        [h11, h12, h13],
        [h21, h22, h23],
        [h31, h32, 1.0],
    ]


def solve_pose_from_square(image_points_tl_tr_br_bl, camera_matrix, square_size_cm):
    """Estimate pose from a known planar square and return (R, t, reproj_err).

    R is 3x3, t is 3x1 in camera coordinates. World square lies on Z=0 plane,
    centered at origin with side length square_size_cm.
    """
    half = square_size_cm * 0.5
    world_xy = [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
    ]

    h = solve_homography(world_xy, image_points_tl_tr_br_bl)
    if h is None:
        return None, None, None

    kinv = _mat3_inv(camera_matrix)
    if kinv is None:
        return None, None, None

    b1 = _mat3_mul_vec(kinv, [h[0][0], h[1][0], h[2][0]])
    b2 = _mat3_mul_vec(kinv, [h[0][1], h[1][1], h[2][1]])
    b3 = _mat3_mul_vec(kinv, [h[0][2], h[1][2], h[2][2]])

    n1 = _norm(b1)
    n2 = _norm(b2)
    if n1 < 1e-9 or n2 < 1e-9:
        return None, None, None

    scale = 2.0 / (n1 + n2)
    r1 = _mul_scalar(b1, scale)
    r2 = _mul_scalar(b2, scale)
    t = _mul_scalar(b3, scale)

    r1n = _mul_scalar(r1, 1.0 / max(_norm(r1), 1e-9))
    r2_ortho = _sub(r2, _mul_scalar(r1n, _dot(r2, r1n)))
    r2n = _mul_scalar(r2_ortho, 1.0 / max(_norm(r2_ortho), 1e-9))
    r3n = _cross(r1n, r2n)

    r3n_norm = _norm(r3n)
    if r3n_norm < 1e-9:
        return None, None, None
    r3n = _mul_scalar(r3n, 1.0 / r3n_norm)

    # Keep Z-forward convention: if normal points backward, flip frame.
    if r3n[2] < 0.0:
        r1n = _mul_scalar(r1n, -1.0)
        r2n = _mul_scalar(r2n, -1.0)
        r3n = _mul_scalar(r3n, -1.0)
        t = _mul_scalar(t, -1.0)

    r = [
        [r1n[0], r2n[0], r3n[0]],
        [r1n[1], r2n[1], r3n[1]],
        [r1n[2], r2n[2], r3n[2]],
    ]

    reproj = compute_reprojection_error_px(
        image_points_tl_tr_br_bl,
        r,
        t,
        camera_matrix,
        square_size_cm,
    )
    return r, t, reproj


def project_world_point(world_xyz, r, t, camera_matrix):
    xc = (
        r[0][0] * world_xyz[0]
        + r[0][1] * world_xyz[1]
        + r[0][2] * world_xyz[2]
        + t[0]
    )
    yc = (
        r[1][0] * world_xyz[0]
        + r[1][1] * world_xyz[1]
        + r[1][2] * world_xyz[2]
        + t[1]
    )
    zc = (
        r[2][0] * world_xyz[0]
        + r[2][1] * world_xyz[1]
        + r[2][2] * world_xyz[2]
        + t[2]
    )

    if abs(zc) < 1e-9:
        return None

    fx = camera_matrix[0][0]
    fy = camera_matrix[1][1]
    cx = camera_matrix[0][2]
    cy = camera_matrix[1][2]

    u = fx * (xc / zc) + cx
    v = fy * (yc / zc) + cy
    return (u, v)


def compute_reprojection_error_px(image_points_tl_tr_br_bl, r, t, camera_matrix, square_size_cm):
    half = square_size_cm * 0.5
    world_points = [
        (-half, -half, 0.0),
        (half, -half, 0.0),
        (half, half, 0.0),
        (-half, half, 0.0),
    ]

    err = 0.0
    count = 0
    for idx, wp in enumerate(world_points):
        uv = project_world_point(wp, r, t, camera_matrix)
        if uv is None:
            continue
        du = uv[0] - image_points_tl_tr_br_bl[idx][0]
        dv = uv[1] - image_points_tl_tr_br_bl[idx][1]
        err += math.sqrt(du * du + dv * dv)
        count += 1

    return err / count if count > 0 else 1e9


def square_world_to_camera_points(r, t, square_size_cm):
    half = square_size_cm * 0.5
    world_points = [
        (-half, -half, 0.0),
        (half, -half, 0.0),
        (half, half, 0.0),
        (-half, half, 0.0),
    ]

    out = []
    for wp in world_points:
        xc = r[0][0] * wp[0] + r[0][1] * wp[1] + r[0][2] * wp[2] + t[0]
        yc = r[1][0] * wp[0] + r[1][1] * wp[1] + r[1][2] * wp[2] + t[1]
        zc = r[2][0] * wp[0] + r[2][1] * wp[1] + r[2][2] * wp[2] + t[2]
        out.append((xc, yc, zc))
    return out


def ema_points(prev_points, new_points, alpha):
    if prev_points is None:
        return new_points
    out = []
    for idx in range(len(new_points)):
        px, py, pz = prev_points[idx]
        nx, ny, nz = new_points[idx]
        out.append(
            (
                alpha * nx + (1.0 - alpha) * px,
                alpha * ny + (1.0 - alpha) * py,
                alpha * nz + (1.0 - alpha) * pz,
            )
        )
    return out
