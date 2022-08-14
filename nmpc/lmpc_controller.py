import numpy as np
Np = 20
Nc = 10
inter = 0.1
A = np.array([[0,1,0,0],[0,0,0,0],[0,0,0,1],[0,0,0,0]])
B = np.array([[0,0],[1,0],[0,0],[0,1]])
row_A = A.shape[0]
col_A = A.shape[1]
row_B = B.shape[0]
col_B = B.shape[1]
R = 0.1 * np.eye(Nc * col_B)
Ak = A * inter + np.eye(row_A)
Bk = B * inter
F = []
for i in range(Np):
	F.extend((A**(i+1)).tolist())
F = np.array(F)
PHI = np.zeros((Np*row_A, Nc*col_B))
for i in range(Np):
	for j in range(Nc):
		if j<=i:
			PHI[i*row_A:(i+1)*row_A, j*col_B:(j+1)*col_B] = Ak**(i-j) @ Bk
		else:
			PHI[i*row_A:(i+1)*row_A, j*col_B:(j+1)*col_B] = np.zeros((row_A, col_B))
			
EX = np.array([[1], [1], [0], [0]])
U = -np.linalg.inv(PHI.T @ PHI + R) @ PHI.T @ F @ EX
#ux = U[0] + Uxr
#uy = U[1] + Uyr
print(U[0],U[1])


