#!/usr/bin/env python

from __future__ import print_function

import numpy as np

# From scikit-image:
# https://github.com/scikit-image/scikit-image/blob/master/skimage/transform/_geometric.py


def _umeyama(src, dst, estimate_scale=True):
    """Estimate N-D similarity transformation with or without scaling.
    Parameters
    ----------
    src : (M, N) array
        Source coordinates.
    dst : (M, N) array
        Destination coordinates.
    estimate_scale : bool
        Whether to estimate scaling factor.
    Returns
    -------
    T : (N + 1, N + 1)
        The homogeneous similarity transformation matrix. The matrix contains
        NaN values only if the problem is not well-conditioned.
    References
    ----------
    .. [1] "Least-squares estimation of transformation parameters between two
            point patterns", Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
    """

    num = src.shape[0]
    dim = src.shape[1]

    # Compute mean of src and dst.
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)

    # Subtract mean from src and dst.
    src_demean = src - src_mean
    dst_demean = dst - dst_mean

    # Eq. (38).
    A = np.dot(dst_demean.T, src_demean) / num

    # Eq. (39).
    d = np.ones((dim, ), dtype=np.double)
    if np.linalg.det(A) < 0:
        d[dim - 1] = -1

    T = np.eye(dim + 1, dtype=np.double)

    U, S, V = np.linalg.svd(A)

    # Eq. (40) and (43).
    rank = np.linalg.matrix_rank(A)
    if rank == 0:
        return np.nan * T
    elif rank == dim - 1:
        if np.linalg.det(U) * np.linalg.det(V) > 0:
            T[:dim, :dim] = np.dot(U, V)
        else:
            s = d[dim - 1]
            d[dim - 1] = -1
            T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V))
            d[dim - 1] = s
    else:
        T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V))

    if estimate_scale:
        # Eq. (41) and (42).
        scale = 1.0 / src_demean.var(axis=0).sum() * np.dot(S, d)
    else:
        scale = 1.0

    T[:dim, dim] = dst_mean - scale * np.dot(T[:dim, :dim], src_mean.T)

    print(T)

    return T, scale


def umeyama(src, dst, estimate_scale=True):
    assert src.shape[0] == 3
    assert src.shape[0] == dst.shape[0]
    assert src.shape[1] > 3
    assert src.shape[1] == dst.shape[1]
    return _umeyama(src.T, dst.T, estimate_scale)


def test_umeyama():
    p_A = np.random.rand(3, 10)
    R_BA = np.array([
        [0.9689135, -0.0232753, 0.2463025],  #
        [0.0236362, 0.9997195, 0.0014915],  #
        [-0.2462682, 0.0043765, 0.9691918]
    ])
    B_t_BA = np.array([[1], [2], [3]])
    p_B = np.dot(R_BA, p_A) + B_t_BA
    p_B = 1.5 * p_B

    # Reconstruct the transformation with ralign.ralign
    T_BA_hat, c = umeyama(p_A, p_B)
    R_BA_hat = T_BA_hat[0:3, 0:3]
    B_t_BA_hat = T_BA_hat[3, 0:3]
    print("Rotation matrix=\n", R_BA_hat, sep='')
    print("Scaling coefficient=", c, sep='')
    print("Translation vector=", B_t_BA_hat, sep='')


def main():
    test_umeyama()
