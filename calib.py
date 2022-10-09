import numpy as np

# Load file
f = np.loadtxt("out.csv", delimiter=',')
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
q, r = np.linalg.qr(xyz1_u.T)

eps = 1e-1
r = len(list(filter(lambda x: abs(x) > eps, list(np.diag(r)))))

soln = q[:, len(q)-r]
print("soln: ", soln)

res = np.dot(xyz1, soln)
print("res: ", res)

mse = np.dot(res.T, res) / len(xyz)
print("MSE: ", mse)
