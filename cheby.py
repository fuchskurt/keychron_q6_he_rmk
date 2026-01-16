# Convert power-basis cubic f(x)=a0+a1 x + a2 x^2 + a3 x^3
# into Chebyshev coefficients on t in [-1,1], where x maps from [xmin,xmax].

import sympy as sp

a0 = sp.Float(426.88962)
a1 = sp.Float(-0.48358)
a2 = sp.Float(2.04637e-4)
a3 = sp.Float(-2.99368e-8)

xmin = sp.Float(1200.0)
xmax = sp.Float(3500.0)

t = sp.Symbol('t')
m = (xmax - xmin) / 2
b = (xmax + xmin) / 2

x = m*t + b
g = sp.expand(a3*x**3 + a2*x**2 + a1*x + a0)  # g(t) in power basis

# g(t) = p0 + p1 t + p2 t^2 + p3 t^3
p3 = sp.expand(g).coeff(t, 3)
p2 = sp.expand(g).coeff(t, 2)
p1 = sp.expand(g).coeff(t, 1)
p0 = sp.expand(g).coeff(t, 0)

# Convert power -> Chebyshev using:
# t^2 = (T2 + T0)/2
# t^3 = (T3 + 3 T1)/4
c3 = p3/4
c2 = p2/2
c1 = p1 + 3*p3/4
c0 = p0 + p2/2

print("c0 =", float(c0))
print("c1 =", float(c1))
print("c2 =", float(c2))
print("c3 =", float(c3))

