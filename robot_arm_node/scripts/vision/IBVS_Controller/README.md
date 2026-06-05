# Image-Based Visual Servoing Controller

An implementation of an image-based visual servoing controller, as described in
[Chaumette & Hutchinson (2006)](https://ieeexplore.ieee.org/document/4015997).
Developed by Jay Rana (contact: jay.rana1@gmail.com).

## Quick Start

### Camera Frame

The camera frame is assumed to be: origin at the center of the image, +x goes to the right,
+y goes downwards, and +z goes into the image. All velocities are given relative to this frame
(e.g. a positive x velocity means to move right).

![ENEE408I Final Report Image Frame](https://github.com/user-attachments/assets/b0ded35e-6c07-4474-9544-c9e73043281f)

### Point Format

Each point is a tuple of three floats.

- The first value is the normalized x coordinate, in the range `(-1.0, 1.0)`.
- The second value is the normalized y coordinate, in the range `(-1.0, 1.0)`.
- The third value is the depth of the point in meters, and must be greater than `0.0`.

### Control Modes

Several control modes are supported (note: this follows the camera frame noted above):

- Two degrees of freedom: x velocity and z velocity (`control_mode='2xz'`)
- Two degrees of freedom: z velocity and y angular velocity (`control_mode='2zy'`)
- Four degrees of freedom: x velocity, y velocity, z velocity, and y angular velocity (`control_mode='4xyzy'`)

### Interaction Modes

Several interaction matrix modes are supported:

- Only use the current positions of each point in the error interaction matrix estimate (`interaction_mode='curr'`)
- Only use the desired positions of each point in the error interaction matrix estimate (`interaction_mode='desired'`)
- Use the mean of the error interaction matrix estimates from the current and desired positions (`interaction_mode='mean'`)

### Controller Loop

**Step 1.** Instantiate the controller with a control mode, interaction mode, and the number of
points (must be greater than 0):

```python
controller = IBVS_Controller(control_mode='2xz', interaction_mode='curr', num_pts=2)
```

**Step 2.** Set the lambda matrix:

```python
controller.set_lambda_matrix(lambdas=[2.0, 5.0])
```

**Step 3.** Set the desired positions of each point in the image:

```python
controller.set_desired_points(desired_pts=[(-0.5, -0.5, 1.0), (0.5, 0.5, 1.0)])
```

**Step 4.** For each control loop iteration:

**4a.** Set the current positions of each point:

```python
controller.set_current_points(curr_pts=[(-0.2, -0.2, 5.0), (0.2, 0.2, 5.0)])
```

**4b.** Check if the error is within some threshold:

```python
if np.linalg.norm(controller.errs) < 0.1:
    break
```

**4c.** Calculate the interaction matrix for this iteration:

```python
controller.calculate_interaction_matrix()
```

**4d.** Calculate the output velocities:

```python
vels = controller.calculate_velocities()
```

**4e.** Apply the output velocities to your motor controllers (note: your robot may have a
different frame than your camera).

## Implementation Details

The general control equation is:

```text
vels = -1 * lambda_matrix * L_e_est_pinv * errs
```

where:

- `vels` — output velocity vector, dimensions `d x 1`
- `lambda_matrix` — diagonal scaling matrix, dimensions `d x d`
- `L_e_est_pinv` — Moore-Penrose pseudoinverse of the error interaction matrix estimate, dimensions `d x 2p`
- `errs` — error vector between current and desired points, dimensions `2p x 1`
- `d` — number of degrees of freedom
- `p` — number of points supplied to the controller

To set the lambda matrix, call `set_lambda_matrix()` with a list of scalars whose length equals
the number of degrees of freedom. The list becomes a diagonal matrix.

To compute `L_e_est_pinv`, call `calculate_interaction_matrix()` after setting points:

- `curr` mode: uses the Moore-Penrose pseudoinverse of the **current** interaction matrix.
- `desired` mode: uses the Moore-Penrose pseudoinverse of the **desired** interaction matrix.
- `mean` mode: uses `0.5 * pinv(L_e + L_e_desired)`.

For `set_current_points()`: if using `desired` mode, the depth value (third element) is ignored
and may be set to `None`. Otherwise the depth must be provided.

For `set_desired_points()`: if using `curr` mode, the depth value is ignored and may be set to
`None`. Otherwise the depth must be provided.

When both current and desired points are set, the error vector is calculated automatically via
`calculate_error_vector()`.

Once the lambda matrix, `L_e_est_pinv`, and error vector are ready, call `calculate_velocities()`
to get the output velocities as a NumPy array, ordered as listed in the associated control mode.

---

## 基于图像的视觉伺服控制器（中文说明）

本项目实现了一种基于图像的视觉伺服（IBVS）控制器，算法参考自
[Chaumette & Hutchinson (2006)](https://ieeexplore.ieee.org/document/4015997)。
由 Jay Rana 开发（联系方式：jay.rana1@gmail.com）。

### 快速入门

#### 相机坐标系

相机坐标系约定如下：原点位于图像中心，+x 向右，+y 向下，+z 指向图像内部。
所有速度均相对于此坐标系表示（例如，x 速度为正表示向右移动）。

![ENEE408I Final Report Image Frame](https://github.com/user-attachments/assets/b0ded35e-6c07-4474-9544-c9e73043281f)

#### 点的格式

每个点是一个包含三个浮点数的元组：

- 第一个值为归一化的 x 坐标，范围为 `(-1.0, 1.0)`。
- 第二个值为归一化的 y 坐标，范围为 `(-1.0, 1.0)`。
- 第三个值为该点的深度（单位：米），必须大于 `0.0`。

#### 控制模式

支持以下几种控制模式（注意：均基于上述相机坐标系）：

- 2 自由度：x 线速度 + z 线速度（`control_mode='2xz'`）
- 2 自由度：z 线速度 + y 角速度（`control_mode='2zy'`）
- 4 自由度：x 线速度、y 线速度、z 线速度 + y 角速度（`control_mode='4xyzy'`）

#### 交互矩阵模式

支持以下几种交互矩阵估计模式：

- 仅使用当前点位置估计误差交互矩阵（`interaction_mode='curr'`）
- 仅使用期望点位置估计误差交互矩阵（`interaction_mode='desired'`）
- 使用当前与期望位置的均值估计误差交互矩阵（`interaction_mode='mean'`）

#### 控制循环

**第 1 步：** 实例化控制器，指定控制模式、交互矩阵模式和点的数量（必须大于 0）：

```python
controller = IBVS_Controller(control_mode='2xz', interaction_mode='curr', num_pts=2)
```

**第 2 步：** 设置 lambda 矩阵：

```python
controller.set_lambda_matrix(lambdas=[2.0, 5.0])
```

**第 3 步：** 设置各点的期望位置：

```python
controller.set_desired_points(desired_pts=[(-0.5, -0.5, 1.0), (0.5, 0.5, 1.0)])
```

**第 4 步：** 在每次控制循环中：

**4a.** 设置各点的当前位置：

```python
controller.set_current_points(curr_pts=[(-0.2, -0.2, 5.0), (0.2, 0.2, 5.0)])
```

**4b.** 检查误差是否低于阈值：

```python
if np.linalg.norm(controller.errs) < 0.1:
    break
```

**4c.** 计算本次迭代的交互矩阵：

```python
controller.calculate_interaction_matrix()
```

**4d.** 计算输出速度：

```python
vels = controller.calculate_velocities()
```

**4e.** 将输出速度发送至电机控制器（注意：机器人坐标系可能与相机坐标系不同，需进行转换）。

### 实现细节

通用控制方程为：

```text
vels = -1 * lambda_matrix * L_e_est_pinv * errs
```

其中：

- `vels` — 输出速度向量，维度 `d x 1`
- `lambda_matrix` — 对角增益矩阵，维度 `d x d`
- `L_e_est_pinv` — 误差交互矩阵估计值的 Moore-Penrose 伪逆，维度 `d x 2p`
- `errs` — 当前点与期望点之间的误差向量，维度 `2p x 1`
- `d` — 控制自由度数量
- `p` — 提供给控制器的点的数量

`set_lambda_matrix()` 接受一个长度等于自由度数量的标量列表，并将其构造为对角矩阵。

`calculate_interaction_matrix()` 在设置点位后调用，根据交互矩阵模式计算伪逆：

- `curr` 模式：对**当前**交互矩阵取伪逆。
- `desired` 模式：对**期望**交互矩阵取伪逆。
- `mean` 模式：计算 `0.5 * pinv(L_e + L_e_desired)`。

`set_current_points()`：若使用 `desired` 模式，深度值（元组第三个元素）将被忽略，可设为
`None`；其他模式下必须提供深度值。

`set_desired_points()`：若使用 `curr` 模式，深度值将被忽略，可设为 `None`；其他模式下必须
提供深度值。

当当前点和期望点均已设置时，误差向量会由 `calculate_error_vector()` 自动计算。

lambda 矩阵、`L_e_est_pinv` 和误差向量均准备好后，调用 `calculate_velocities()` 即可获得
输出速度（NumPy 数组），顺序与对应控制模式中列出的一致。
