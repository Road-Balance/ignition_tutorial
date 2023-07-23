import sympy as sy

q1, q2 = sy.symbols('q1 q2')
l, r = sy.symbols('l r')

S = 1/r * sy.Matrix([
    [sy.cos(q1), sy.sin(q1), l/2 * sy.sin(q1)],
    [sy.cos(q2), sy.sin(q2), -l/2 * sy.sin(q2)],
])

print(S.inv())