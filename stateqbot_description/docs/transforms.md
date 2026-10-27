# Transformation Matrices - stateqbot

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | Right_wheel |
| $L_{2}$ | left_wheel |
| $L_{3}$ | lidar |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Right_wheel | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | left_wheel | continuous (rad) | $L_{0}$ | $L_{2}$ |
| $q_{3}$ | lidar | continuous (rad) | $L_{0}$ | $L_{3}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [continuous] Right_wheel (q1)
  |   L1: Right_wheel
  |-- [continuous] left_wheel (q2)
  |   L2: left_wheel
  +-- [continuous] lidar (q3)
      L3: lidar
```

## Transforms

## Right_wheel

$L_{0}$ **base_link** -> $L_{1}$ **Right_wheel** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (-0.053333, -0.052, 0.02) m
- **origin rpy**: (1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.053333 \\
0 & 0 & -1 & -0.052 \\
0 & 1 & 0 & 0.02 \\
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

## left_wheel

$L_{0}$ **base_link** -> $L_{2}$ **left_wheel** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (-0.053333, 0.052, 0.02) m
- **origin rpy**: (-1.570796, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & -0.053333 \\
0 & 0 & 1 & 0.052 \\
0 & -1 & 0 & 0.02 \\
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

## lidar

$L_{0}$ **base_link** -> $L_{3}$ **lidar** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.04, 0, 0.045) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{0}_{3}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0.04 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0.045 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

