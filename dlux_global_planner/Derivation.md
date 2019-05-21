# Full Derivation
We start with our basic equation definitions.
 * ![P(X) = \frac{-\beta + \sqrt{\beta^2 - 4 \gamma}}{2}](doc/PX_frac_beta_sqrtbeta_2_4gamma2.gif)
 * ![\beta = -\Big(P(C) + P(A)\Big)](doc/beta__BigPC_PABig.gif)
 * ![\gamma = \frac{P(A)^2 + P(C)^2 - h^2}{2}](doc/gamma_fracPA_2_PC_2_h_22.gif)

## Reformulate
First we want to reformulate in terms of our new ![\delta](doc/delta.gif) variable.
 * ![\delta = \frac{P(C) - P(A)}{h}](doc/delta_fracPC_PAh.gif)

We can rewrite this so we have a new definition of ![P(C)](doc/PC.gif)
 * ![P(C) = h\delta  + P(A)](doc/PC_hdelta_PA.gif)

This allows us to derive new values for the other equations.
 * ![\beta = -\Big(P(C) + P(A)\Big)](doc/beta__BigPC_PABig.gif)
 * ![\beta = -\Big(h\delta + P(A) + P(A)\Big)](doc/beta__Bighdelta_PA_PABig.gif)
 * ![\beta = -h\delta - 2 P(A)](doc/beta__hdelta_2PA.gif)
 * ![\gamma = \frac{P(A)^2 + P(C)^2 - h^2}{2}](doc/gamma_fracPA_2_PC_2_h_22.gif)
 * ![\gamma = \frac{P(A)^2 + \Big(P(A)+h\delta\Big)^2 - h^2}{2}](doc/gamma_fracPA_2_BigPA_hdeltaBig_2_h_22.gif)
 * ![\gamma = P(A)^2 + h\delta P(A) + \frac{h^2\delta^2 - h^2}{2}](doc/gamma_PA_2_hdeltaPA_frach_2delta_2_h_22.gif)
 * ![P(X) = \frac{-\beta + \sqrt{\beta^2 - 4 \gamma}}{2}](doc/PX_frac_beta_sqrtbeta_2_4gamma2.gif)
 * ![P(X) = \frac{-(-h\delta - 2 P(A)) + \sqrt{\Big(-h\delta - 2 P(A)\Big)^2 - 4 \Big(P(A)^2 + h\delta P(A) + \frac{h^2\delta^2 - h^2}{2}\Big)}}{2}](doc/PX_frac__hdelta_2PA_sqrtBig_hdelta_2PABig_2_4BigPA_2_hdeltaPA_frach_2delta_2_h_22Big2.gif)
 * ![P(X) = \frac{h\delta + 2 P(A) + \sqrt{\Big(-h\delta - 2 P(A)\Big)^2 - 4 P(A)^2 -4 h\delta P(A) - 2h^2\delta^2 + 2h^2}}{2}](doc/PX_frachdelta_2PA_sqrtBig_hdelta_2PABig_2_4PA_2_4hdeltaPA_2h_2delta_2_2h_22.gif)
 * ![P(X) = \frac{h\delta + 2 P(A) + \sqrt{4P(A)^2 + 4h\delta P(A) + h^2\delta^2 - 4 P(A)^2 -4 h\delta P(A) - 2h^2\delta^2 + 2h^2}}{2}](doc/PX_frachdelta_2PA_sqrt4PA_2_4hdeltaPA_h_2delta_2_4PA_2_4hdeltaPA_2h_2delta_2_2h_22.gif)
 * ![P(X) = \frac{h\delta + 2 P(A) + \sqrt{-h^2\delta^2 + 2h^2}}{2}](doc/PX_frachdelta_2PA_sqrt_h_2delta_2_2h_22.gif)
 * ![P(X) = P(A) + \frac{h}{2} \Big(\delta + \sqrt{2-\delta^2}\Big)](doc/PX_PA_frach2Bigdelta_sqrt2_delta_2Big.gif)

We're going to call this a new function ![f(\delta)](doc/fdelta.gif), so now ![P(X)](doc/PX.gif) is in terms of ![f(\delta)](doc/fdelta.gif)
 * ![f(\delta) = P(A) + \frac{h}{2} \Big(\delta + \sqrt{2-\delta^2}\Big)](doc/fdelta_PA_frach2Bigdelta_sqrt2_delta_2Big.gif)

## Taylor Series Expansion
For computational efficiency, we are going to compute the Taylor series expansion. So first, we'll need the derivatives of ![f(\delta)](doc/fdelta.gif).
 * ![f'(\delta) = \frac{h}{2} (1 - \frac{\delta}{\sqrt{2-\delta^2}})](doc/f_delta_frach21_fracdeltasqrt2_delta_2.gif)
 * ![f''(\delta) = \frac{-h}{(2-\delta^2)^\frac{3}{2}}](doc/f__delta_frac_h2_delta_2_frac32.gif)

And then let's evaluate them at ![\delta=0](doc/delta_0.gif). Here's the first derivative
 * ![f'(0) = \frac{h}{2} (1 - \frac{0}{\sqrt{2-0^2}})](doc/f_0_frach21_frac0sqrt2_0_2.gif)
 * ![f'(0) = \frac{h}{2}](doc/f_0_frach2.gif)

and the second
 * ![f''(0) = \frac{-h}{(2-0^2)^\frac{3}{2}}](doc/f__0_frac_h2_0_2_frac32.gif)
 * ![f''(0) = \frac{-h}{2^\frac{3}{2}}](doc/f__0_frac_h2_frac32.gif)

Now we can start to expand the Taylor series with ![\delta=0](doc/delta_0.gif).

### 0th Order Taylor
 * ![f_0(\delta) = f(0)](doc/f_0delta_f0.gif)
 * ![f_0(\delta) = P(A) + \frac{h}{2}(0+\sqrt{2-0^2})](doc/f_0delta_PA_frach20_sqrt2_0_2.gif)
 * ![f_0(\delta) = P(A) + h\frac{\sqrt{2}}{2}](doc/f_0delta_PA_hfracsqrt22.gif)

### 1st Order Taylor
 * ![f_1(\delta) = f(0) + f'(0) (\delta - 0)](doc/f_1delta_f0_f_0delta_0.gif)
 * ![f_1(\delta) = P(A) + h\frac{\sqrt{2}}{2} + \frac{h}{2}\delta](doc/f_1delta_PA_hfracsqrt22_frach2delta.gif)
 * ![f_1(\delta) = P(A) + h\Big(\frac{\sqrt{2}}{2} + \frac{1}{2} \delta\Big)](doc/f_1delta_PA_hBigfracsqrt22_frac12deltaBig.gif)

### 2nd Order Taylor
 * ![f_2(\delta) = f(0) + f'(0) (\delta - 0) + \frac{f''(0)}{2!}(\delta - 0)^2](doc/f_2delta_f0_f_0delta_0_fracf__02_delta_0_2.gif)
 * ![f_2(\delta) = P(A) + h\Big(\frac{\sqrt{2}}{2} + \frac{1}{2} \delta\Big) + \frac{-h}{2^\frac{3}{2}}\frac{1}{2}\delta^2](doc/f_2delta_PA_hBigfracsqrt22_frac12deltaBig_frac_h2_frac32frac12delta_2.gif)
 * ![f_2(\delta) = P(A) + h\Big(\frac{\sqrt{2}}{2} + \frac{1}{2} \delta - \frac{1}{2^\frac{5}{2}}\delta^2\Big)](doc/f_2delta_PA_hBigfracsqrt22_frac12delta_frac12_frac52delta_2Big.gif)

## Exact coefficients
More generally, we can say
 * ![f_2(\delta) = P(A) + h (c_0 + c_1\delta + c_2 \delta^2 )](doc/f_2delta_PA_hc_0_c_1delta_c_2delta_2.gif)

If we evaluate at ![\delta=0](doc/delta_0.gif) (as we did above), then we get
 * ![c_0 = \frac{\sqrt{2}}{2} \approx 0.7071](doc/c_0_fracsqrt22approx0_7071.gif)
 * ![c_1 = \frac{1}{2} = 0.5](doc/c_1_frac12_0_5.gif)
 * ![c_2 = \frac{-1}{2^\frac{5}{2}} \approx -0.1768](doc/c_2_frac_12_frac52approx_0_1768.gif)

Historically, the values used by [`navfn`](https://github.com/ros-planning/navigation/blob/1f335323a605b49b4108a845c55a7c1ba93a6f2e/navfn/src/navfn.cpp#L509) are
 * ![c_0 = 0.7040](doc/c_0_0_7040.gif)
 * ![c_1 = 0.5307](doc/c_1_0_5307.gif)
 * ![c_2 = -0.2301](doc/c_2__0_2301.gif)

The exact reason for the difference in the values for ![\delta=0](doc/delta_0.gif) and `navfn` is unknown at this time, but the plots are close enough to each other that it is not overly concerning.
