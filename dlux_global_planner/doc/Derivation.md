# Full Derivation
We start with our basic equation definitions.
 * !eq[P(X) = \frac{-\beta + \sqrt{\beta^2 - 4 \gamma}}{2}]
 * !eq[\beta = -\Big(P(C) + P(A)\Big)]
 * !eq[\gamma = \frac{P(A)^2 + P(C)^2 - h^2}{2}]

## Reformulate
First we want to reformulate in terms of our new !eq[\delta] variable.
 * !eq[\delta = \frac{P(C) - P(A)}{h}]

We can rewrite this so we have a new definition of !eq[P(C)]
 * !eq[P(C) = h\delta  + P(A)]

This allows us to derive new values for the other equations.
 * !eq[\beta = -\Big(P(C) + P(A)\Big)]
 * !eq[\beta = -\Big(h\delta + P(A) + P(A)\Big)]
 * !eq[\beta = -h\delta - 2 P(A)]
 * !eq[\gamma = \frac{P(A)^2 + P(C)^2 - h^2}{2}]
 * !eq[\gamma = \frac{P(A)^2 + \Big(P(A)+h\delta\Big)^2 - h^2}{2}]
 * !eq[\gamma = P(A)^2 + h\delta P(A) + \frac{h^2\delta^2 - h^2}{2}]
 * !eq[P(X) = \frac{-\beta + \sqrt{\beta^2 - 4 \gamma}}{2}]
 * !eq[P(X) = \frac{-(-h\delta - 2 P(A)) + \sqrt{\Big(-h\delta - 2 P(A)\Big)^2 - 4 \Big(P(A)^2 + h\delta P(A) + \frac{h^2\delta^2 - h^2}{2}\Big)}}{2}]
 * !eq[P(X) = \frac{h\delta + 2 P(A) + \sqrt{\Big(-h\delta - 2 P(A)\Big)^2 - 4 P(A)^2 -4 h\delta P(A) - 2h^2\delta^2 + 2h^2}}{2}]
 * !eq[P(X) = \frac{h\delta + 2 P(A) + \sqrt{4P(A)^2 + 4h\delta P(A) + h^2\delta^2 - 4 P(A)^2 -4 h\delta P(A) - 2h^2\delta^2 + 2h^2}}{2}]
 * !eq[P(X) = \frac{h\delta + 2 P(A) + \sqrt{-h^2\delta^2 + 2h^2}}{2}]
 * !eq[P(X) = P(A) + \frac{h}{2} \Big(\delta + \sqrt{2-\delta^2}\Big)]

We're going to call this a new function !eq[f(\delta)], so now !eq[P(X)] is in terms of !eq[f(\delta)]
 * !eq[f(\delta) = P(A) + \frac{h}{2} \Big(\delta + \sqrt{2-\delta^2}\Big)]

## Taylor Series Expansion
For computational efficiency, we are going to compute the Taylor series expansion. So first, we'll need the derivatives of !eq[f(\delta)].
 * !eq[f'(\delta) = \frac{h}{2} (1 - \frac{\delta}{\sqrt{2-\delta^2}})]
 * !eq[f''(\delta) = \frac{-h}{(2-\delta^2)^\frac{3}{2}}]

### 0th Order Taylor
 * !eq[f_0(\delta, a) = f(a)]
 * !eq[f_0(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2}\Big)]

### 1st Order Taylor
 * !eq[f_1(\delta, a) = f(a) + f'(a) (\delta - a)]
 * !eq[f_1(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2}\Big) + \frac{h}{2} (1 - \frac{a}{\sqrt{2-a^2}}) (\delta - a)]
 * !eq[f_1(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2} + (1 - \frac{a}{\sqrt{2-a^2}}) (\delta - a)\Big)]
 * !eq[f_1(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2} + \delta - a - \frac{a\delta}{\sqrt{2-a^2}} + \frac{a^2}{\sqrt{2-a^2}}\Big)]
 * !eq[f_1(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \sqrt{2-a^2} + \frac{a^2}{\sqrt{2-a^2}}\Big)]
 * !eq[f_1(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \frac{2}{\sqrt{2-a^2}}\Big)]

### 2nd Order Taylor
 * !eq[f_2(\delta, a) = f(a) + f'(a) (\delta - a) + \frac{f''(a)}{2!}(\delta - a)^2]
 * !eq[f_2(\delta, a) = f_1(\delta, a) + \frac{1}{2} \frac{-h}{(2-a^2)^\frac{3}{2}} (\delta - a)^2]
 * !eq[f_2(\delta, a) = f_1(\delta, a) + \frac{h}{2} \frac{-1}{(2-a^2)^\frac{3}{2}} (\delta^2 - 2\delta a + a^2)]
 * !eq[f_2(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \frac{2}{\sqrt{2-a^2}}\Big) + \frac{h}{2} \frac{-1}{(2-a^2)^\frac{3}{2}} (\delta^2 - 2\delta a + a^2)]
 * !eq[f_2(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \frac{2}{\sqrt{2-a^2}} + \frac{-1}{(2-a^2)^\frac{3}{2}} (\delta^2 - 2\delta a + a^2)\Big)]
 * !eq[f_2(\delta, a) = P(A) + h\Big(c_0(a) + c_1(a)\delta + c_2(a) \delta^2\Big)]
 * !eq[c_0(a) = \frac{1}{2}\Big(\frac{2}{\sqrt{2-a^2}} - \frac{a^2}{(2-a^2)^\frac{3}{2}}\Big)]
 * !eq[c_0(a) = \frac{4 - 3a^2}{2(2-a^2)^\frac{3}{2}}]

 * !eq[c_1(a) = \frac{1}{2}\Big(1 - \frac{a}{\sqrt{2-a^2}} + \frac{2a}{(2-a^2)^\frac{3}{2}} \Big)]
 * !eq[c_2(a) = \frac{-1}{2(2-a^2)^\frac{3}{2}}]

## Exact coefficients
Now that we have the general equations for the Taylor series, we can evaluate it at different values of a in the range `[0, 1]`.

| !eq[a] | !eq[c_0(a)] | !eq[c_1(a)] | !eq[c_2(a)] |
| ------ | ----------- | ----------- | ----------- |
|  0.0   |    0.7071   |    0.5000   |   -0.1768   |
|  0.5   |    0.7019   |    0.5270   |   -0.2160   |
|  1.0   |    0.5000   |    1.0000   |   -0.5000   |

Historically, the values used by [`navfn`](https://github.com/ros-planning/navigation/blob/1f335323a605b49b4108a845c55a7c1ba93a6f2e/navfn/src/navfn.cpp#L509) are

|  !eq[c_0]   |  !eq[c_1]   |  !eq[c_2]   |
| ----------- | ----------- | ----------- |
|    0.7040   |   0.5307    |   -0.2301   |

You can see these values plotted [here](https://www.desmos.com/calculator/vbpkey1mt6).

The historical values are pretty close to the values for !eq[\delta=0.5], although the exact reason for the difference is unknown, but its close enough to not be overly concerning.
