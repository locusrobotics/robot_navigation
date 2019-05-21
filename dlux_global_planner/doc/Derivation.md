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

And then let's evaluate them at !eq[\delta=0]. Here's the first derivative
 * !eq[f'(0) = \frac{h}{2} (1 - \frac{0}{\sqrt{2-0^2}})]
 * !eq[f'(0) = \frac{h}{2}]

and the second
 * !eq[f''(0) = \frac{-h}{(2-0^2)^\frac{3}{2}}]
 * !eq[f''(0) = \frac{-h}{2^\frac{3}{2}}]

Now we can start to expand the Taylor series with !eq[\delta=0].

### 0th Order Taylor
 * !eq[f_0(\delta) = f(0)]
 * !eq[f_0(\delta) = P(A) + \frac{h}{2}(0+\sqrt{2-0^2})]
 * !eq[f_0(\delta) = P(A) + h\frac{\sqrt{2}}{2}]

### 1st Order Taylor
 * !eq[f_1(\delta) = f(0) + f'(0) (\delta - 0)]
 * !eq[f_1(\delta) = P(A) + h\frac{\sqrt{2}}{2} + \frac{h}{2}\delta]
 * !eq[f_1(\delta) = P(A) + h\Big(\frac{\sqrt{2}}{2} + \frac{1}{2} \delta\Big)]

### 2nd Order Taylor
 * !eq[f_2(\delta) = f(0) + f'(0) (\delta - 0) + \frac{f''(0)}{2!}(\delta - 0)^2]
 * !eq[f_2(\delta) = P(A) + h\Big(\frac{\sqrt{2}}{2} + \frac{1}{2} \delta\Big) + \frac{-h}{2^\frac{3}{2}}\frac{1}{2}\delta^2]
 * !eq[f_2(\delta) = P(A) + h\Big(\frac{\sqrt{2}}{2} + \frac{1}{2} \delta - \frac{1}{2^\frac{5}{2}}\delta^2\Big)]

## Exact coefficients
More generally, we can say
 * !eq[f_2(\delta) = P(A) + h (c_0 + c_1\delta + c_2 \delta^2 )]

If we evaluate at !eq[\delta=0] (as we did above), then we get
 * !eq[c_0 = \frac{\sqrt{2}}{2} \approx 0.7071]
 * !eq[c_1 = \frac{1}{2} = 0.5]
 * !eq[c_2 = \frac{-1}{2^\frac{5}{2}} \approx -0.1768]

Historically, the values used by [`navfn`](https://github.com/ros-planning/navigation/blob/1f335323a605b49b4108a845c55a7c1ba93a6f2e/navfn/src/navfn.cpp#L509) are
 * !eq[c_0 = 0.7040]
 * !eq[c_1 = 0.5307]
 * !eq[c_2 = -0.2301]

The exact reason for the difference in the values for !eq[\delta=0] and `navfn` is unknown at this time, but the plots are close enough to each other that it is not overly concerning.
