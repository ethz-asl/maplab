#!/usr/bin/env python

import numpy as np

# From https://gist.github.com/CarloNicolini/7118015


def umeyama(X, Y):
  assert X.shape[0] == 3
  assert Y.shape[0] == 3
  assert X.shape[1] > 0
  assert Y.shape[1] > 0

  m, n = X.shape

  mx = X.mean(1)
  my = Y.mean(1)

  Xc = X - np.tile(mx, (n, 1)).T
  Yc = Y - np.tile(my, (n, 1)).T

  sx = np.mean(np.sum(Xc * Xc, 0))
  sy = np.mean(np.sum(Yc * Yc, 0))
  Sxy = np.dot(Yc, Xc.T) / n

  U, D, V = np.linalg.svd(Sxy, full_matrices=True, compute_uv=True)
  V = V.T.copy()

  r = np.rank(Sxy)
  d = np.linalg.det(Sxy)
  S = np.eye(m)
  if r > (m - 1):
    if (np.det(Sxy) < 0):
      S[m, m] = -1
    elif (r == m - 1):
      if (np.det(U) * np.det(V) < 0):
        S[m, m] = -1
    else:
      R = np.eye(2)
      c = 1
      t = np.zeros(2)
      return R, c, t

  R = np.dot(np.dot(U, S), V.T)

  c = np.trace(np.dot(np.diag(D), S)) / sx
  t = my - c * np.dot(R, mx)

  return R, t, c


def test_umeyama():
  p_A = np.random.rand(3, 10)
  R_BA = np.array([[0.9689135, -0.0232753, 0.2463025],
                   [0.0236362, 0.9997195, 0.0014915],
                   [-0.2462682, 0.0043765, 0.9691918]])
  B_t_BA = np.array([[1], [2], [3]])
  p_B = np.dot(R_BA, p_A) + B_t_BA

  # Reconstruct the transformation with ralign.ralign
  R_BA_hat, B_t_BA_hat, c = umeyama(p_A, p_B)
  print "Rotation matrix=\n", R_BA_hat
  print "Scaling coefficient=", c
  print "Translation vector=", B_t_BA_hat


if __name__ == '__main__':
  test_umeyama()
