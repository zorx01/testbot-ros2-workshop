# Transformation Matrices - testbot

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | backward_left_wheel |
| $L_{2}$ | Backward_Right_wheel |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_2 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_4 | continuous (rad) | $L_{0}$ | $L_{2}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [continuous] Revolute_2 (q1)
  |   L1: backward_left_wheel
  +-- [continuous] Revolute_4 (q2)
      L2: Backward_Right_wheel
```

## Transforms

## Revolute_2

$L_{0}$ **base_link** -> $L_{1}$ **backward_left_wheel** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (-0.081, 0.129, 0.0155) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.081 \\
0 & 0 & 1 & 0.129 \\
0 & -1 & 0 & 0.0155 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_4

$L_{0}$ **base_link** -> $L_{2}$ **Backward_Right_wheel** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (-0.081, -0.129, 0.0155) m
- **origin rpy**: (1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.081 \\
0 & 0 & -1 & -0.129 \\
0 & 1 & 0 & 0.0155 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

