import numpy as np

# Load file
f = np.loadtxt("data.csv", delimiter=',')
xyz = f[:, :3]
uv = f[:, 3:]

# Construct homogenous matrices
xyz1 = np.ones((len(xyz), 8))
xyz1[:, 0:3] = xyz
xyz1[:, 4:7] = xyz

xyz1_u = xyz1.copy()
xyz1_u[:, 4:] *= uv[:, (0, 0, 0, 0)]

xyz1_v = xyz1.copy()
xyz1_v[:, 4:] *= uv[:, (1, 1, 1, 1)]


# Solve for null space
# https://scicomp.stackexchange.com/a/2511
def nullspace(a):
    q, r = np.linalg.qr(a.T)

    eps = 1e-1  # Tolerance
    r = len(list(filter(lambda x: abs(x) > eps, list(np.diag(r)))))

    soln = q[:, len(q)-r]

    return soln
    print("soln: ", soln)


def accuracy(soln, x):
    res = np.dot(x, soln)
    #print("res: ", res)

    mse = np.dot(res.T, res) / len(x)
    print("MSE: ", mse)


soln_u = nullspace(xyz1_u)
soln_v = nullspace(xyz1_v)

accuracy(soln_u, xyz1_u)
accuracy(soln_v, xyz1_v)
