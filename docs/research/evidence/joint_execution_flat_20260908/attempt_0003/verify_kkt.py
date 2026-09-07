"""Independent original-coordinate certificate; no production solver import."""
import json
from pathlib import Path
import numpy as np
p = Path(__file__).parent
tokens = iter((p / "qp.txt").read_text().split())
q = {}
for name in tokens:
    r, c = int(next(tokens)), int(next(tokens))
    q[name] = np.array([float(next(tokens)) for _ in range(r*c)]).reshape(r,c)
audit = json.loads((p / "independent.json").read_text())
active = audit["independent_qp_optimum"]["active_inequality_rows"]
H, A, E = q["H"], q["Aineq"], q["Aeq"]
g, b, d = (q[k].ravel() for k in ("g", "bineq", "beq"))
W = np.vstack((E, A[active]))
k = W.shape[0]
K = np.block([[H, W.T], [W, np.zeros((k,k))]])
sol = np.linalg.solve(K, np.r_[-g, d, b[active]])
x, lam = sol[:len(g)], sol[len(g):]
eq = float(np.max(np.abs(E@x-d)))
inequality = float(max(0, np.max(A@x-b)))
stationarity = float(np.max(np.abs(H@x+g+W.T@lam)))
minimum_multiplier = float(np.min(lam[len(d):]))
objective = float(.5*x@H@x+g@x)
assert np.linalg.eigvalsh(H)[0] > 0
assert eq < 1e-7 and inequality < 1e-7 and stationarity < 1e-7
assert minimum_multiplier >= 0
assert abs(objective-17524222.67510441) < .01
print(json.dumps(dict(objective=objective, equality=eq, inequality=inequality,
                     stationarity=stationarity, minimum_multiplier=minimum_multiplier)))
