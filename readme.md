---
title: GAMES101 PA 报告兼课程重点算法回顾
date: 2022-1-18 21:19:54
categories:
- 理论
- 理论/计算机图形学
tags:
- 图形学
mathjax: true
---

![output-4](https://s2.loli.net/2022/01/28/hIQTsycbV6xeojp.png)

GAMES101 2020-Spring 对应的 8 次 PA 整理。

仓库地址：https://github.com/c7w/GAMES101-HW-2020-Spring

*　　Warning: FYI, these codes may be ugly and buggy.*

<!--more-->

## PA1

> 本次作业的任务是填写一个**旋转矩阵**和一个**透视投影矩阵**。
>
> 给定三维下三个点 $v_0(2.0, 0.0, −2.0), v_1(0.0, 2.0, −2.0), v_2(−2.0, 0.0, −2.0)$​，你需要将这三个点的坐标变换为屏幕坐标并在屏幕上绘制出对应的线框三角形 (在代码框架中，我们已经提供了 draw_triangle 函数，所以你只需要去构建变换矩阵即可)。
>
> 简而言之，我们需要进行模型、视图、投影、视口等变换来将三角形显示在屏幕上。在提供的代码框架中，我们留下了模型变换和投影变换的部分给你去完成。

### 算法回顾

拍照的第一步是模型变换，也就是把模型放在合适的位置上。第二步是找好角度放相机（View Transformation），也就是视图变换。第三步是做投影变换，将照片定格。

+ 视图变换：定义 $M_{view}$ 变换相机，使得经过该变换满足 $\vec e = \vec 0, \hat g = -\hat z, \hat t = \hat y $，其中：$\vec e$ 为相机所在位置， $\hat g$ 为相机看的方向，$\hat t$ 为相机的向上方向。

$$
M_{view} = R_{view}T_{view} \ (**)\\

T_{view} = \begin{bmatrix} 1&0&0&-x_e\\
0&1&0&-y_e\\
0&0&1&-z_e\\
0&0&0&1\end{bmatrix}\\

R_{view}^{-1} = \begin{bmatrix} x_{\hat g \times \hat t}&x_{\hat t}&x_{-\hat g}&0\\
y_{\hat g \times \hat t}&y_{\hat t}&y_{-\hat g}&0\\
z_{\hat g \times \hat t}&z_{\hat t}&z_{-\hat g}&0\\
0&0&0&1\end{bmatrix}\\

R_{view} = (R_{view}^{-1})^T =
\begin{bmatrix} x_{\hat g \times \hat t}&y_{\hat g \times \hat t}&z_{\hat g \times \hat t}&0\\
x_{\hat t}&y_{\hat t}&z_{\hat t}&0\\
x_{-\hat g}&y_{-\hat g}&z_{-\hat g}&0\\
0&0&0&1\end{bmatrix}\\
$$

我们要做的，是把所有物体都应用这个变换，保证相机与物体的相对位置不变。

+ 投影变换

**首先将摄像机的视锥压成正方体 $M_{persp\rightarrow ortho}$​，然后进行正交投影 $M_{ortho}$​​。**

这里我们略去透视投影矩阵的推导，直接给出透视投影转换为正交投影矩阵的表示形式。具体的推导可以通过考虑以下特殊点，代入特殊值来决定矩阵的元素。

+ 近平面的坐标不改变
+ 远平面的 $x,y$ 坐标被压缩至与近平面相同
+ 近平面中心点与远平面中心点的位置不变

$$
M_{persp\rightarrow ortho} = 
\begin{bmatrix}
n & 0 & 0 & 0 \\
0 & n & 0 & 0 \\
0 & 0 & n+f & -nf \\
0 & 0 & 1 & 0
\end{bmatrix} \\\\
$$

值得注意的是，在我们平常的应用中，我们并不是直接使用 $l, r, b,t$ 来描述一个近平面的位置，而是更倾向于使用 $fovY$​(**field-of-view**, 垂直视角) 和 **aspect ratio** 这两个量来描述一个近平面。

使用这两个量我们可以轻松地计算出 $l,r,b,t$ 四个近平面参数。

![image-20211106214646265](https://i.loli.net/2021/11/06/NZbxWisdEH78plf.png)

然后我们使用正交投影映射到 $[-1, 1]^3$。
$$
M_{\text {ortho }}=\left[\begin{array}{cccc}
\frac{2}{r-l} & 0 & 0 & 0 \\
0 & \frac{2}{t-b} & 0 & 0 \\
0 & 0 & \frac{2}{n-f} & 0 \\
0 & 0 & 0 & 1
\end{array}\right]\left[\begin{array}{cccc}
1 & 0 & 0 & -\frac{r+l}{2} \\
0 & 1 & 0 & -\frac{t+b}{2} \\
0 & 0 & 1 & -\frac{n+f}{2} \\
0 & 0 & 0 & 1
\end{array}\right]
$$

### 实验结果

![1](https://s2.loli.net/2022/01/28/ftcK5nhEdYaGwPp.png)

## PA2

> 任务如下：
>
> + `rasterize_triangle()`: 执行三角形栅格化算法
> + `static bool insideTriangle()`: 测试点是否在三角形内
> + 正确实现深度缓冲算法
> + Super-sampling

### 算法回顾

+ 检测点在三角形内

我们考虑如下的三个叉积。$\overrightarrow {P_0P_1} \times \overrightarrow {P_0Q}$, $\overrightarrow {P_1P_2} \times \overrightarrow {P_1Q}$, $\overrightarrow {P_2P_0} \times \overrightarrow {P_2Q}$ 如果 z 坐标的符号相同，那么点 Q 就一定在三角形内。但是，如果对屏幕的所有元素采样，造成了没有必要的资源浪费。我们使用**包围盒**（Bounding Box）的概念，取三角形边界点的 x, y 坐标分别的最小值或最大值，作为包围盒 x, y 坐标的最小值与最大值。这样我们就能得到一张带锯齿的图像了。

+ Super-sampling

而在我们具体解决问题的时候，我们选择 Supersampling 的方式，也就是将某个像素分为 $N\times N$​ 个采样点，然后对这些采样点的像素值取平均。

+ 深度缓冲

为了解决这个问题，图形学引入了深度缓存的概念。想法就是对屏幕的所有像素额外记录其当前显示物体的最浅深度（深度取正值，表示距离相机的远近）。这样类似于动态规划的算法最终复杂度是 $O(n)$ 的。而且如果我们假设在同一深度处不会出现两个模型，那么不同模型的着色顺序对结果是没有影响的。

![image-20211110140334554](https://i.loli.net/2021/11/10/vkn5R7zfhoNZTjC.png)

### 实验结果

![2-1](https://s2.loli.net/2022/01/28/ma6wt8dVnjGMTq4.png)

![2-2](https://s2.loli.net/2022/01/28/GDWpa1w9JLNsXz6.png)

## PA3

> 任务：
>
> + 参数插值
> + Blinn-Phong 模型
> + Texture Mapping
> + Bump Mapping & Displacement Mapping
> + 尝试更多模型
> + 双线性插值

### 算法回顾

+ 重心坐标插值

当我们知道了三角形的三个顶点的属性的时候，如果我们想要实现在三角形内部属性的平滑过渡，就要引入重心坐标的概念。对于一个三角形 ABC 来说，其中 $(x,y) = \alpha A + \beta B + \gamma C$，若 $\alpha + \beta + \gamma = 1$，则称 $(\alpha, \beta, \gamma)$ 为该三角形内的 $(x,y)$ 点的重心坐标，其中 $\alpha, \beta, \gamma \ge 0$。

![image-20211111231732485](https://i.loli.net/2021/11/11/U4jEISXchwARgZV.png)

+ BP

$$
\begin{aligned}
L &=L_{a}+L_{d}+L_{s} \\
&=k_{a} I_{a}+k_{d}\left(I / r^{2}\right) \max (0, \mathbf{n} \cdot \mathbf{l})+k_{s}\left(I / r^{2}\right) \max (0, \mathbf{n} \cdot \mathbf{h})^{p}
\end{aligned}
$$



### 实验结果

+ Normal

![output-2](https://s2.loli.net/2022/01/28/K8ISoYs59h2wdPg.png)

+ BP

![output-3](https://s2.loli.net/2022/01/28/oVBEDL2NuwRpkGO.png)

+ Texture

![output-4](https://s2.loli.net/2022/01/28/hIQTsycbV6xeojp.png)

+ Bump Mapping & Displacement Mapping

![output-5](https://s2.loli.net/2022/01/28/HJiSMxQa4udzjPF.png)

![output-6](https://s2.loli.net/2022/01/28/vXV89tQPTSUpDeA.png)

+ Bunny

![bunny](https://s2.loli.net/2022/01/28/ERkHN7PlVSQF4qm.png)

+ 双线性插值前/后 (Shrink texture.png by 50%)

![output-small](https://s2.loli.net/2022/01/28/fm9AjRGu2rngCLw.png)

![output-small2](https://s2.loli.net/2022/01/28/I5jeWBENHgndFtC.png)

### 实验框架

+ `main.cpp`
  + `Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)`
  + `Eigen::Matrix4f get_model_matrix(float angle)`
  + `Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)`
  + `struct Light {Eigen::Vector3f position; Eigen::Vector3f intensity;}; `
  + Shaders
    + `Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)`
    + `Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)`
      + 定义 $k_d, k_s, k_a, lights, amb\_light, eye\_pos$​
      + 返回 $result\_color * 255.0$
    + `Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)`
    + `Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)`
  + `int main()`
    + 读取模型，然后将模型中的各个面建立 Triangle 对象
    + 建立 Rasterizer 类对象，设置材质
    + 调用渲染函数，然后使用 OpenCV 库保存渲染结果
+ `OBJ_Loader.h`：加载模型用
+ `Rasterizer.hpp/Rasterizer.cpp`
  + 其中的 rasterizer 子类内包括了一些必备的结构和函数
  + `std::vector<Eigen::Vector3f> frame_buf;`
  + `std::vector<float> depth_buf;`

+ `Texture.hpp/Texture.cpp`
  + `cv::Mat image_data;` // Raw data
  + `int width, height;`
  + `Vector3f getColor(float u, float v);`
+ `Triangle.hpp/Triangle.cpp`
  + `Vector4f v[3];` // Original coordinates of the triangle
  + `Vector3f color[3]; Vector2f tex_coords[3]; Vector3f normal[3];` // Color, texture_coord, normal vector for each vertex

## PA4

> 使用 **De Casteljau 算法** 生成贝塞尔曲线。

### 算法回顾

（1）先考虑三个控制点，生成二次贝塞尔曲线。

![image-20211201194418429](https://i.loli.net/2021/12/01/fE6FrtaL2sDQIGT.png)

（2）再考虑四个控制点，生成三次贝塞尔曲线。

![image-20211201194528603](https://i.loli.net/2021/12/01/wR7VDnSPCrLxkKY.png)

(3) 代数形式：插值！

![image-20211201194639946](https://i.loli.net/2021/12/01/pmW5vVa61urdbxI.png)

给定 $n+1$ 个控制点，有贝塞尔曲线如下：
$$
b^n(t) = b^n_0(t) = \sum_{j=0}^nb_jB_j^n(t) \\
B_{i}^{n}(t)=\left(\begin{array}{l}
n\\
i
\end{array}\right) t^{i}(1-t)^{n-i}
$$

### 实验结果

![4](https://s2.loli.net/2022/01/29/kHpBU6wfgcvxi7W.png)

## PA5

> 在光线追踪中，最重要的操作之一就是找到光线与物体的交点。
>
> 一旦找到光线与物体的交点，就可以执行着色并返回像素颜色。
>
> 在这次作业中，我们需要实现两个部分：光线的生成和光线与三角的相交。
>
> 具体来说，我们需要修改：
>
> + `Renderer.cpp` 中的 `Render()`：这里你需要为每个像素生成一条对应的光线，然后调用函数 `castRay()` 来得到颜色，最后将颜色存储在帧缓冲区的相应像素中。
> + `Triangle.hpp` 中的 `rayTriangleIntersect()`: v0, v1, v2 是三角形的三个顶点，orig 是光线的起点，dir 是光线单位化的方向向量。tnear, u, v 是你需要使用我们课上推导的 Moller-Trumbore 算法来更新的参数。

### 算法回顾

+ Whitted-style Ray Tracing

![image-20220114155905503](https://s2.loli.net/2022/01/14/T1xAeVvrhOoGJK4.png)

+ Möller Trumbore 算法

Möller Trumbore 算法，可以直接判断交点是否在三角形内（需要用到克莱姆法则）：
$$
\begin{gathered}
\overrightarrow{\mathbf{O}}+t \overrightarrow{\mathbf{D}}=\left(1-b_{1}-b_{2}\right) \overrightarrow{\mathbf{P}}_{0}+b_{1} \overrightarrow{\mathbf{P}}_{1}+b_{2} \overrightarrow{\mathbf{P}}_{2} \\
\text { Where: } \\
{\left[\begin{array}{c}
t \\
b_{1} \\
b_{2}
\end{array}\right]=\frac{1}{\overrightarrow{\mathbf{S}}_{1} \bullet \overrightarrow{\mathbf{E}}_{1}}\left[\begin{array}{cc}
\overrightarrow{\mathbf{S}}_{2} \cdot \overrightarrow{\mathbf{E}}_{2} \\
\overrightarrow{\mathbf{S}}_{1} \cdot \overrightarrow{\mathbf{S}} \\
\overrightarrow{\mathbf{S}}_{2} \cdot \overrightarrow{\mathbf{D}}
\end{array}\right]} \\
\text { Cost = (1 div, 27 mul, 17 add) } \\
\overrightarrow{\mathbf{E}}_{1}=\overrightarrow{\mathbf{P}}_{1}-\overrightarrow{\mathbf{P}}_{0} \\
\overrightarrow{\mathbf{E}}_{2}=\overrightarrow{\mathbf{P}}_{2}-\overrightarrow{\mathbf{P}}_{0} \\
\overrightarrow{\mathbf{S}}=\overrightarrow{\mathbf{O}}-\overrightarrow{\mathbf{P}}_{0} \\
\overrightarrow{\mathbf{S}}_{1}=\overrightarrow{\mathbf{D}} \times \overrightarrow{\mathbf{E}}_{2} \\
\overrightarrow{\mathbf{S}}_{2}=\overrightarrow{\mathbf{S}} \times \overrightarrow{\mathbf{E}}_{1}
\end{gathered}
$$



### 实验结果

![5](https://s2.loli.net/2022/01/29/ZeSLqlF65o7vfD8.png)

## PA6

> 在之前的编程练习中，我们实现了基础的光线追踪算法，具体而言是光线传输、光线与三角形求交。
>
> 我们采用了这样的方法寻找光线与场景的交点：**遍历场景中的所有物体，判断光线是否与它相交。**
>
> 在场景中的物体数量不大时，该做法可以取得良好的结果，但当物体数量增多、模型变得更加复杂，该做法将会变得非常低效。
>
> 因此，我们需要加速结构来加速求交过程。
>
> 在本次练习中，我们重点关注：**物体划分算法 Bounding Volume Hierarchy (BVH)**。
>
> 本练习要求你实现 Ray-Bounding Volume 求交与 BVH 查找，附加内容为使用 **SAH 加速**算法对 BVH 查找进行改进。

### 算法回顾

#### 包围盒求交

我们将（包围盒）长方体理解成三个对面所分划出的空间的交集。我们一般使用的包围体积便是长方体，且长方体是与坐标轴平行的，即轴对齐包围盒（AABB）。选用 AABB 式包围盒有利于加速计算。判断光线和 AABB 类包围盒的求交方法如下：

![image-20220114163501816](https://s2.loli.net/2022/01/14/5rMlAnIw3gQBVNL.png)

核心思想：只有当光线进入所有三个对面后，光线才进入了包围盒；若光线从某一个对面出射，则光线便从包围盒中射出。于是我们可以计算三个对面分别对应的 $t_{min}$​​ 和 $t_{max}$​​，然后对其求交。当且仅当 $t_{enter} \lt t_{exit}$​​ 且 $t_{exit} \ge 0$​​，光线才可能与包围盒有交点。

#### BVH

通过对含有物体的向量沿轴划分建立二叉树，并且对于二叉树的每一个结点建立其包围盒。

![image-20220116102930688](https://s2.loli.net/2022/01/16/zbLpGw5E7SCgQAu.png)

如何进行划分？

+ 沿着最长的维度来划分
+ 划分端点取中位数
  + 取三角形的重心

![image-20220116103520101](https://s2.loli.net/2022/01/16/xtKaWO7lJ2Rm8Xb.png)

使用 BVH 算法，即使包围盒可能在空间上有相交，但是我们却解决了 kd-Tree 存在的问题。

#### SAH 加速

（BVH）构建过程中最重要的问题就是如何对图元进行划分...划分时要**尽可能减少划分后两部分包围盒重叠的体积**，因为**重叠的体积越大，光线穿过重叠区域的可能性越大，遍历两个子树的可能性就越高，计算消耗越多**。

因为我们最终的目的是要减少划分后左右子节点重叠的体积，因此一般**在图元跨度最大的坐标轴上进行划分**。这里，图元的跨度可以用图元的包围盒来衡量也可以用图元的质心来衡量。

两种最简单的划分方法是：

+ 取坐标轴跨度的中点 $t_{\operatorname{mid}}=\frac{t_{\max }+t_{\min }}{2}$​ ，若节点的坐标小于 $t_{mid}$ 则将其划分到左节点，否则将其划分到右节点（中点划分）。
+ 最左边的 $\frac n 2$​ 个被划分到左节点，剩下的被划分到右节点（等量划分）。 // 作业框架中 Naive 的划分由此实现

一种更为常用且效果更好的方法是**基于表面积的启发式评估划分方法**（Surface Area Heuristic，SAH），这种方法通过对求交代价和遍历代价进行评估，给出了每一种划分的代价（Cost），而我们的目的便是去寻找代价最小的划分。

假设当前节点的包围体中存在 $n$​ 个物体，设对每一个物体求交的代价为 $t(i)$​​ ，如果不做划分依次对其求交则总的代价为：
$$
\sum t(i)=t(1)+t(2)+\cdots+t(n)
$$
如果这些物体划分为 2 组，这两组物体分别处于它们的包围盒 A 和 B 中。设光线击中它们的概率分别为 $p_A$​​​​​​​ 和 $p_B$​​​​​​ ，需要注意包围盒 A 和 B 之间存在重叠，且它们并不一定会填满其父节点的包围体，因此 $p_A$​​​ 和 $p_B$​​ 的和不一定为1，且它们的和越大说明 A 和 B 的重叠程度越大。综上所述，当前节点求交的代价可以写为：
$$
c(A,B) = p_A \sum_{i \in A}t(i) + p_B \sum_{i \in B}t(i) + t_{traverse}
$$
其中 $t_{traverse}$​​​ 代表遍历树状结构的代价。一般来说，我们假设对所有图元的求交代价是相同的，可设 $t(i) \equiv 1$​​​，又遍历的代价小于求交的代价，可设 $t_{traverse} = 0.125$​​ 。设包围盒A中图元的个数为 $a$​，B中图元的个数为 $b$，则：
$$
c(A,B) = \frac {S(A)} {S(C)}a + \frac {S(B)} {S(C)}b + 0.125
$$
SAH 考虑到了图元在空间中的分布也考虑到了子节点包围体的重叠程度，在实际应用中拥有很好的效果。

然后我们就可以借助一定的辅助空间，在 $O(n)$​ 的时间内完成这个划分过程。这里我们<s>偷懒</s>采用一种 $O(n \log n)$ 的分划方式：

+ 将物体按照质心顺序排序. $O(n \log n)$
+ $\forall k$​，记录 $a_i := \cup_{i \le k} \ S(i)$​. $O(n)$​​
+ $\forall k$​，记录 $b_i := \cup_{i \gt k} \ S(i)$​. $O(n)$​
+ $\forall k$​，考虑划分 $[0, k] \ || \ [k+1, n)$​，计算其代价并更新最小代价. $O(n)$​

参考资料：https://zhuanlan.zhihu.com/p/50720158

### 实验结果

采用 SAH 加速前，进行 RT 用时 8 min 35 secs；使用 SAH 加速后，用时为 8 min 48 secs。

// Warning: this code may be buggy!

// 没错用了这 ** 优化又慢了我也不知道x

// 我感觉包围盒算法就需要大改，anyway 领会精神吧x

![6-1](https://s2.loli.net/2022/01/18/Y9Nh7QAUxfncCgV.png)

### 实验框架

+ `main.cpp`
  + 创建场景（1280x960）与预处理
    + 导入模型为 MeshTriangle 类对象，并添加入场景；
    + 将光源添加入场景；
    + 对场景执行 buildBVH()
  + 创建 `Renderer` 类对象渲染器，并开始渲染
+ `Vector.hpp, Vector.cpp`：向量类，提供了向量 `Vector3f`, `Vector2f` 的基本操作
+ `Object.hpp`：抽象类
  + 提供了以下接口：
    + `bool intersect(const Ray&)`
    + `bool intersect(const Ray& ray, float& distance, int& index) const`
    + `Intersection getIntersection(Ray _ray)`
    + `void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &, const Vector2f &, Vector3f &, Vector2f &) const`
    + `virtual Vector3f evalDiffuseColor(const Vector2f &) const`
    + `virtual Bounds3 getBounds()`
  + 总结来说，首先是判断有无交点和获取交点的接口，然后是获取表面属性和计算颜色的接口，然后是获取包围盒的接口。
  + `Triangle.hpp/Sphere.hpp`：继承自 `Object`，重载上述函数并实现为对应对象
+ `Ray.hpp`
  + `Vector3f orig, direction;`, `double t, t_min, t_max;`
+ `Light.hpp`：`Vector3f position, intensity;`
  + `AreaLight.hpp`：`Vector3f u, v, normal; double length;`
+ `Intersection.hpp`：
  + `bool happened;`
  + `Vector3f coords, normal;`
  + `double distance;`
  + `Object* obj;`
  + `Material* m;`
+ `Material.hpp`：存储表面属性
  + 表面类型： `enum MaterialType { DIFFUSE_AND_GLOSSY, REFLECTION_AND_REFRACTION, REFLECTION };`
  + 表面颜色（反射，辐射），折射率，BP 模型中的 $k_d, k_s, p$​
+ `Bounds3.hpp`：包围盒相关函数
  + 求对角线，维数，表面积，中心点；
  + 与另外一个包围盒求交，求并；
  + 检测两个包围盒是否互相包含或者重叠；
  + 判断与光线是否相交；
+ `BVH.hpp/BVH.cpp`：加速结构
  + `enum class SplitMethod { NAIVE, SAH };`
  + 与光线求交
    + `Intersection Intersect(const Ray &ray) const;`
    + `Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;`
    + `bool IntersectP(const Ray &ray) const;`
  + 本质是一颗二叉树，拥有树形结构
    + `BVHBuildNode* root;`
    + `BVHBuildNode* recursiveBuild(std::vector<Object*>objects);`
    + `BVHBuildNode* recursiveBuildSAH(std::vector<Object*>objects);`
  + 存储其图元 primitive 的指针
    + `std::vector<Object*> primitives;`
+ `Renderer.hpp / Renderer.cpp`
  + 给定一个场景，创建渲染任务
  + 管理任务过程中的 framebuffer 等等图像数据
  + 管理整个渲染流程以及数据最终的写入文件
+ `Scene.hpp / Scene.cpp`
  + 图像相关的设定
    + `int width, height;`
    + `double fov;`
    + `Vector3f backgroundColor;`
    + `maxDepth;`
  + 模型相关的设定 Get / Set
    + `void Add(Object *object);`
    + `void Add(std::unique_ptr<Light> light);`
  + 与光线求交的函数：`Intersection intersect(const Ray& ray) const;`
  + BVH 相关：
    + `BVHAccel *bvh = nullptr;`
    + `void buildBVH();`
  + 渲染主函数：
    + `Vector3f castRay(const Ray &ray, int depth) const;` // Important!
    + `bool trace(const Ray &ray, const std::vector<Object*> &objects, float &tNear, uint32_t &index, Object **hitObject);`

## PA7

> + 实现 Path Tracing 算法

### 算法回顾

```
def ray_generation(camPos, pixel):
	Uniformly choose N samples from the pixel
	pixel_radiance = 0.0
	for sample in the pixel:
		Shoot a ray r(camPos, cam_to_sample)
		if ray r hit the scene at p:
			pixel_radiance += 1 / N * shade(p, sample_to_cam)
    return pixel_radiance

def shade(p, wo):

	# Contributions from the light source
	L_dir = 0.0
	Uniformly sample the light at x' (pdf_light = 1/A)
	Shoot a ray from p to x'
	if the ray is not blocked in the middle:
		L_dir = L_i * f_r * cos \theta * cos \theta' / |x'-p|^2 / pdf_light
    
    # Contributions from other places
    L_indir = 0.0
    If test Russian Roulette with probability P_RR :
        Uniformly sample the hemisphere toward wi (pdf_hemi = 1 / 2pi)
        Trace a ray r(p, wi)
        If ray r hit a non-emitting object at q:
            L_indir = shade(q, -wi) * f_r * cos \theta / pdf_hemi / P_RR
    
    Return L_dir + L_indir
```

### 实验结果

+ SPP = 16 / 32 / 64

![7-16](https://s2.loli.net/2022/01/29/pwevmYcRyDqBPxh.png)

![7-32](https://s2.loli.net/2022/01/29/1OrJUsgTSLocbwK.png)

![7-64](https://s2.loli.net/2022/01/29/7aCgFk6KOdqS4bM.png)



## PA8

> + 连接绳子约束，正确的构造绳子 
> + 半隐式欧拉法
> + 显式欧拉法
> + 显式 Verlet
> + 阻尼

### 算法回顾

+ 显式/半隐式欧拉法

$$
\boldsymbol{f}_{a \rightarrow b}=k_{s} \frac{\boldsymbol{b}-\boldsymbol{a}}{\|\boldsymbol{b}-\boldsymbol{a}\|}(\|\boldsymbol{b}-\boldsymbol{a}\|-l) \\
f_{b \rightarrow a} = - f_{ a \rightarrow b}
$$

$$
\begin{aligned}
&\mathrm{F}=\mathrm{ma} \\
&\mathrm{v}(\mathrm{t}+1)=\mathrm{v}(\mathrm{t})+\mathrm{a}(\mathrm{t}) * \mathrm{dt} \\
&\mathrm{x}(\mathrm{t}+1)=\mathrm{x}(\mathrm{t})+\mathrm{v}(\mathrm{t}) * \mathrm{dt} \\
&\mathrm{x}(\mathrm{t}+1)=\mathrm{x}(\mathrm{t})+\mathrm{v}(\mathrm{t}+1) * \mathrm{dt}
\end{aligned}
$$

倒数第二行为显式欧拉法，最后一行为半隐式欧拉法。

+ 显式 Verlet

$$
x(t+1)=x(t)+[x(t)-x(t-1)]+a(t) * d t * d t
$$

+ 阻尼

$$
x(t+1) =x(t)+(1-\text {damping\_factor }) *[x(t)-x(t-1)]+a(t) * d t * d t
$$

