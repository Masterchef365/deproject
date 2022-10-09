import numpy as np
import sys

# Load file
f = np.loadtxt("data.csv", delimiter=',')
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

    mp = np.abs(m / p - u)
    j = m - u * p

    a = j / (p**2 * mp)
    b = -(j * m) / (p**3 * mp)

    a = a[:, np.newaxis]
    a = a[:, (0, 0, 0, 0)]

    b = b[:, np.newaxis]
    b = b[:, (0, 0, 0, 0)]

    grad[:, :4] = xyz1 * a
    grad[:, 4:] = xyz1 * b

    return np.average(grad, axis=0)


lr = 1e-2
gamma = 0.9

iters = 10_000
n_samples = 1000
tolerance = 0.1

# Initialize random weights
retry = 0
while True:
    np.random.seed(retry)
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
            #print(rmse, list(abc))
            print(rmse)
            sys.stdout.flush()

    print(f"RETRY_{retry}")
    print(abc)
    sys.stdout.flush()
    retry += 1

    if abs(rmse) < tolerance:
        break
