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

### 0th Order Taylor
 * ![f_0(\delta, a) = f(a)](doc/f_0delta_a_fa.gif)
 * ![f_0(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2}\Big)](doc/f_0delta_a_PA_frach2Biga_sqrt2_a_2Big.gif)

### 1st Order Taylor
 * ![f_1(\delta, a) = f(a) + f'(a) (\delta - a)](doc/f_1delta_a_fa_f_adelta_a.gif)
 * ![f_1(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2}\Big) + \frac{h}{2} (1 - \frac{a}{\sqrt{2-a^2}}) (\delta - a)](doc/f_1delta_a_PA_frach2Biga_sqrt2_a_2Big_frach21_fracasqrt2_a_2delta_a.gif)
 * ![f_1(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2} + (1 - \frac{a}{\sqrt{2-a^2}}) (\delta - a)\Big)](doc/f_1delta_a_PA_frach2Biga_sqrt2_a_2_1_fracasqrt2_a_2delta_aBig.gif)
 * ![f_1(\delta, a) = P(A) + \frac{h}{2} \Big(a + \sqrt{2-a^2} + \delta - a - \frac{a\delta}{\sqrt{2-a^2}} + \frac{a^2}{\sqrt{2-a^2}}\Big)](doc/f_1delta_a_PA_frach2Biga_sqrt2_a_2_delta_a_fracadeltasqrt2_a_2_fraca_2sqrt2_a_2Big.gif)
 * ![f_1(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \sqrt{2-a^2} + \frac{a^2}{\sqrt{2-a^2}}\Big)](doc/f_1delta_a_PA_frach2Bigdelta_fracadeltasqrt2_a_2_sqrt2_a_2_fraca_2sqrt2_a_2Big.gif)
 * ![f_1(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \frac{2}{\sqrt{2-a^2}}\Big)](doc/f_1delta_a_PA_frach2Bigdelta_fracadeltasqrt2_a_2_frac2sqrt2_a_2Big.gif)

### 2nd Order Taylor
 * ![f_2(\delta, a) = f(a) + f'(a) (\delta - a) + \frac{f''(a)}{2!}(\delta - a)^2](doc/f_2delta_a_fa_f_adelta_a_fracf__a2_delta_a_2.gif)
 * ![f_2(\delta, a) = f_1(\delta, a) + \frac{1}{2} \frac{-h}{(2-a^2)^\frac{3}{2}} (\delta - a)^2](doc/f_2delta_a_f_1delta_a_frac12frac_h2_a_2_frac32delta_a_2.gif)
 * ![f_2(\delta, a) = f_1(\delta, a) + \frac{h}{2} \frac{-1}{(2-a^2)^\frac{3}{2}} (\delta^2 - 2\delta a + a^2)](doc/f_2delta_a_f_1delta_a_frach2frac_12_a_2_frac32delta_2_2deltaa_a_2.gif)
 * ![f_2(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \frac{2}{\sqrt{2-a^2}}\Big) + \frac{h}{2} \frac{-1}{(2-a^2)^\frac{3}{2}} (\delta^2 - 2\delta a + a^2)](doc/f_2delta_a_PA_frach2Bigdelta_fracadeltasqrt2_a_2_frac2sqrt2_a_2Big_frach2frac_12_a_2_frac32delta_2_2deltaa_a_2.gif)
 * ![f_2(\delta, a) = P(A) + \frac{h}{2} \Big(\delta - \frac{a\delta}{\sqrt{2-a^2}} + \frac{2}{\sqrt{2-a^2}} + \frac{-1}{(2-a^2)^\frac{3}{2}} (\delta^2 - 2\delta a + a^2)\Big)](doc/f_2delta_a_PA_frach2Bigdelta_fracadeltasqrt2_a_2_frac2sqrt2_a_2_frac_12_a_2_frac32delta_2_2deltaa_a_2Big.gif)
 * ![f_2(\delta, a) = P(A) + h\Big(c_0(a) + c_1(a)\delta + c_2(a) \delta^2\Big)](doc/f_2delta_a_PA_hBigc_0a_c_1adelta_c_2adelta_2Big.gif)
 * ![c_0(a) = \frac{1}{2}\Big(\frac{2}{\sqrt{2-a^2}} - \frac{a^2}{(2-a^2)^\frac{3}{2}}\Big)](doc/c_0a_frac12Bigfrac2sqrt2_a_2_fraca_22_a_2_frac32Big.gif)
 * ![c_0(a) = \frac{4 - 3a^2}{2(2-a^2)^\frac{3}{2}}](doc/c_0a_frac4_3a_222_a_2_frac32.gif)

 * ![c_1(a) = \frac{1}{2}\Big(1 - \frac{a}{\sqrt{2-a^2}} + \frac{2a}{(2-a^2)^\frac{3}{2}} \Big)](doc/c_1a_frac12Big1_fracasqrt2_a_2_frac2a2_a_2_frac32Big.gif)
 * ![c_2(a) = \frac{-1}{2(2-a^2)^\frac{3}{2}}](doc/c_2a_frac_122_a_2_frac32.gif)

## Exact coefficients
Now that we have the general equations for the Taylor series, we can evaluate it at different values of a in the range `[0, 1]`.

| ![a](doc/a.gif) | ![c_0(a)](doc/c_0a.gif) | ![c_1(a)](doc/c_1a.gif) | ![c_2(a)](doc/c_2a.gif) |
| ------ | ----------- | ----------- | ----------- |
|  0.0   |    0.7071   |    0.5000   |   -0.1768   |
|  0.5   |    0.7019   |    0.5270   |   -0.2160   |
|  1.0   |    0.5000   |    1.0000   |   -0.5000   |

Historically, the values used by [`navfn`](https://github.com/ros-planning/navigation/blob/1f335323a605b49b4108a845c55a7c1ba93a6f2e/navfn/src/navfn.cpp#L509) are

|  ![c_0](doc/c_0.gif)   |  ![c_1](doc/c_1.gif)   |  ![c_2](doc/c_2.gif)   |
| ----------- | ----------- | ----------- |
|    0.7040   |   0.5307    |   -0.2301   |

You can see these values plotted [here](https://www.desmos.com/calculator/vbpkey1mt6).

The historical values are pretty close to the values for ![\delta=0.5](doc/delta_0_5.gif), although the exact reason for the difference is unknown, but its close enough to not be overly concerning.
