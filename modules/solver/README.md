# 模块：解算器

姿态解算。

## 点与向量

区分**点**与**向量**：
- 点：位置信息，有大小，无方向。
- 向量：位移信息，有大小，有方向。由两个点连接起来。

我们用空间内的一组基，来表示空间中的任意向量。
$$
\boldsymbol a=
\begin{bmatrix}
\boldsymbol e_1 & \boldsymbol e_2 & \boldsymbol e_3
\end{bmatrix}
\begin{bmatrix}
a_1\\
a_2\\
a_3
\end{bmatrix}
=a_1\boldsymbol e_1+a_2\boldsymbol e_2+a_3\boldsymbol e_3
$$
其中$\boldsymbol e_1,\boldsymbol e_2,\boldsymbol e_3$是一组基，$[a_1,a_2,a_3]^\text T$是向量$\boldsymbol a$在这组基下的坐标。

## 坐标系间的欧式变换

两个坐标系之间的变换，可以用一个旋转矩阵和一个平移向量来表示。

### 旋转变换

对于同一个向量，他不会随着坐标系的选择而改变。因此，在坐标变换过程中有下面的等式。

$$
[\boldsymbol e_1, \boldsymbol e_2, \boldsymbol e_3]
\begin{bmatrix}
\boldsymbol a_1\\
\boldsymbol a_2\\
\boldsymbol a_3
\end{bmatrix}
=
[\boldsymbol e'_1, \boldsymbol e'_2, \boldsymbol e'_3]
\begin{bmatrix}
\boldsymbol a'_1\\
\boldsymbol a'_2\\
\boldsymbol a'_3
\end{bmatrix}
$$

两边同时左乘 $[\boldsymbol e_1, \boldsymbol e_2, \boldsymbol e_3]^\text T$ ，得到
$$
\begin{bmatrix}
a_1\\
a_2\\
a_3
\end{bmatrix}
=
\begin{bmatrix}
\boldsymbol e_1^\text T\boldsymbol e'_1 & \boldsymbol e_1^\text T\boldsymbol e'_2 & \boldsymbol e_1^\text T\boldsymbol e'_3\\
\boldsymbol e_2^\text T\boldsymbol e'_1 & \boldsymbol e_2^\text T\boldsymbol e'_2 & \boldsymbol e_2^\text T\boldsymbol e'_3\\
\boldsymbol e_3^\text T\boldsymbol e'_1 & \boldsymbol e_3^\text T\boldsymbol e'_2 & \boldsymbol e_3^\text T\boldsymbol e'_3
\end{bmatrix}
\begin{bmatrix}
a'_1\\
a'_2\\
a'_3
\end{bmatrix}
=\boldsymbol R\boldsymbol a'
$$
其中 $\boldsymbol R$ 是旋转矩阵。由两组积的内积组成，刻画了旋转前后坐标变换的关系。

n维旋转矩阵的集合是一个特殊的李群，称为特殊正交群$\mathrm {SO}(n)$。
$$
\mathrm{SO}(n)=\{\boldsymbol R\in\mathbb R^{n\times n}|\boldsymbol R^\text T\boldsymbol R=\boldsymbol I, \det(\boldsymbol R)=1\}
$$

### 平移变换

平移部分的变换可以用一个向量来表示，只需要将平移向量加到坐标上即可。

$$
\boldsymbol a'=\boldsymbol R\boldsymbol a+\boldsymbol t
$$
其中$\boldsymbol t$是平移向量。

### 变换矩阵与齐次坐标

在三维向量末尾加上一个 1 ，得到四维向量，称为齐次坐标。这样，旋转和平移可以统一表示为一个矩阵乘法。
$$
\begin{bmatrix}
\boldsymbol a'\\
1
\end{bmatrix}
=
\begin{bmatrix}
\boldsymbol R & \boldsymbol t\\
\boldsymbol 0^\text T & 1
\end{bmatrix}
\begin{bmatrix}
\boldsymbol a\\
1
\end{bmatrix}
=\boldsymbol T
\begin{bmatrix}
\boldsymbol a\\
1
\end{bmatrix}
$$
其中 $\boldsymbol R$ 是旋转矩阵，$\boldsymbol t$ 是平移向量。

n维空间中的变换矩阵是一个特殊的李群，记为 $\mathrm{SE}(n)$ 。
$$
\mathrm{SE}(n)=\{\boldsymbol T=\begin{bmatrix}
\boldsymbol R & \boldsymbol t\\
\boldsymbol 0^\text T & 1
\end{bmatrix}
\in\mathbb R^{(n+1)\times(n+1)}
| \boldsymbol R\in\mathrm{SO}(n), \boldsymbol t\in\mathbb R^n\}
