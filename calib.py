import numpy as np

# Load file
f = np.loadtxt("data.csv", delimiter=',')
xyz = f[:, :3]
uv = f[:, 3:]

u = uv[:, 0]
v = uv[:, 1]

# Construct homogenous matrices
xyz1 = np.ones((len(xyz), 4))
xyz1[:, 0:3] = xyz

# Initialize random weights
np.random.seed(1)
abc = np.random.rand(8) * 2. - 1.


# Calculate model accuracy
def deproject(abc):
    return np.dot(xyz1, abc[:4]) / np.dot(xyz1, abc[4:])


def calc_mse(abc, u):
    res = deproject(abc)
    d = res - u
    return np.dot(d.T, d)


def grad(abc, u):
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

iters = 100000000000

v = np.zeros_like(grad(abc, u))

for _ in range(iters):
    v = gamma * v + lr * grad(abc, u)
    abc -= v
    print(np.sqrt(calc_mse(abc, u)))
