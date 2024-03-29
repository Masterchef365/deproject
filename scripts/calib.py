import numpy as np
import sys
from os.path import join

root_path = sys.argv[1]
calib_points_path = join(root_path, "calib_points.csv")
matrix_path = join(root_path, "matrix.csv")
predict_path = join(root_path, "predict.csv")

# Load file
f = np.loadtxt(calib_points_path, delimiter=',')
xyz = f[:, :3]
uv = f[:, 3:]

u = uv[:, 0]
v = uv[:, 1]

# Construct homogenous matrices
xyz1 = np.ones((len(xyz), 4))
xyz1[:, 0:3] = xyz

# Calculate model accuracy
def deproject(abc):
    return np.dot(xyz1, abc[:4]) / np.dot(xyz1, abc[4:])


def calc_mse(abc, u):
    res = deproject(abc)
    d = res - u
    return np.dot(d.T, d)


def grad(abc, u, xyz1):
    m = np.dot(xyz1, abc[:4])
    p = np.dot(xyz1, abc[4:])

    grad = np.zeros((len(u), 8))

    mp = m / np.abs(p)
    mp = mp[:, np.newaxis]
    mp = mp[:, (0, 0, 0, 0)]

    grad[:, :4] = xyz1
    grad[:, 4:] = xyz1 * mp

    s = np.sign(m - u * p)
    sp = s / np.abs(p)
    sp = sp[:, np.newaxis]
    sp = sp[:, tuple([0]*8)]

    grad *= sp

    return np.average(grad, axis=0)


lr = 1e-3
gamma = 0.9

iters = 50_000
n_samples = 100


# Initialize random weights
def solve(u):
    np.random.seed(0)
    abc = np.random.rand(8) * 2. - 1.
    v = np.zeros_like(grad(abc, u, xyz1))

    for i in range(iters):
        samples = np.random.randint(0, len(u), size=n_samples)

        xyz1_sample = xyz1[samples]
        u_sample = u[samples]

        v = gamma * v + lr * grad(abc - gamma * v, u_sample, xyz1_sample)
        abc -= v

        if i % 1000 == 0:
            rmse = np.sqrt(calc_mse(abc, u))
            print(rmse)
            sys.stdout.flush()

    return abc

soln_u = solve(u)
soln_v = solve(v)

u_pred = deproject(soln_u)
v_pred = deproject(soln_v)

out = np.zeros((len(xyz), 6))
out[:, :3] = xyz
out[:, 3] = u_pred
out[:, 4] = v_pred

idx = np.bitwise_and(
    np.bitwise_and(out[:, 3] >= 0, out[:, 3] <= 1),
    np.bitwise_and(out[:, 4] >= 0, out[:, 4] <= 1),
)
out = out[idx]

np.savetxt(predict_path, out, delimiter=',')

matrix = np.array([soln_u, soln_v])

np.savetxt(matrix_path, matrix, delimiter=',')
