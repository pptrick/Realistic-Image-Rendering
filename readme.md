# Realistic Image Rendering

> Copyright reserved @pptrick

真实感渲染是计算机图形学领域相当重要的研究方向，在游戏、影视制作以及模拟仿真等方面都有着重要应用；该方向既涉及光线追踪、光子映射等全局光照算法，也包括Bezier、B样条曲线曲面建模方法，涵盖面相当广泛。本项目旨在应用真实感渲染领域的一些主流算法，通过Bezier旋转体建模实现真实感场景绘制。



## 代码整体框架

**main.cpp**	主程序接口，设置物体位置参数，设置物体材质、光源性质、相机参数等文件的文件路径，调用初始化函数以及Ray Tracing主体函数；

**vector3D.h**	定义三维向量，并定义叉积、点积、归一化等向量常用运算；

**tracingtools.h**	声明点、线、面关系运算的常用函数。包括但不限于：反射折射、点线距离、包围盒求交；

**tracingtools.cpp**	定义点、线、面关系运算的常用函数。包括但不限于：反射折射、点线距离、包围盒求交；

**object.h**	定义物体类，继承有球、平面、房间类；

**object.cpp**	定义物体类的相关函数，包括但不限于：初始化、局域光照、法向量、光线求交；

**camera.h**	定义相机类，包括但不限于视点位置、主光轴、光圈；

**camera.cpp**	定义相机相关函数，包括但不限于：初始化相机函数、光线追踪函数、最近交点函数、图像生成函数；

**bezier.h**	定义Bezier旋转体类，包含位置、控制点、纹理等参数；

**bezier.cpp**	定义Bezier旋转体相关函数，初始化、局域光照、纹理映射、法向量、牛顿法光线求交；

**PhotonMapping.h**	定义kd-tree节点类以及光子图类，以kd-tree形式存储光子；

**PhotonMapping**	定义光子映射相关函数，包括但不限于：光子追踪函数、kd-tree构建函数、kd-tree搜索函数、光子生成器；



## Bezier旋转体建模与求交

**（代码见bezier.cpp）**

### Bezier曲线绘制

构建一个Bezier旋转体需要先绘制一条Bezier曲线，再让其绕中心轴旋转一周得到旋转体；因此Bezier曲线绘制是相当重要的一步。具体绘制方式如下：给定n+1个控制点f0, f1, ...，fn, 则由这一系列控制点生成的Bezier曲线在参数t处的坐标向量可表示为：

<a href="https://www.codecogs.com/eqnedit.php?latex=Q(f,t)=\sum_{i=0}^nf_iB_{in}(t)=\sum_{i=0}^nf_iC_{n}^it^i(1-t)^{n-i}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Q(f,t)=\sum_{i=0}^nf_iB_{in}(t)=\sum_{i=0}^nf_iC_{n}^it^i(1-t)^{n-i}" title="Q(f,t)=\sum_{i=0}^nf_iB_{in}(t)=\sum_{i=0}^nf_iC_{n}^it^i(1-t)^{n-i}" /></a>

其中B为Bernstein基。因此，在具体实现中只需要给定t和控制点，即可得到需要的坐标位置；需要说明的是，组合数的计算可以通过其递推式，用递归算法得到：

```C++
double Comb(int i, int n)
{
    if (i == n || i == 0)return 1;
    else if (i < n)return Comb(i, n - 1) + Comb(i - 1, n - 1);
    else return 0;
}
```

### Bezier旋转体建模

得到Bezier曲线之后，可令其绕中心轴旋转一周得到旋转体；具体方法是引入一个Φ角坐标衡量旋转角度，得到的曲面F(t,、Φ)由Bezier曲线参数t和旋转角参数φ共同确定。此时，笛卡尔坐标系下的坐标需要作如下变换：

<a href="https://www.codecogs.com/eqnedit.php?latex=y^\prime=y\\&space;z^\prime=-x\sin\phi\\&space;x^\prime=x\cos\phi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y^\prime=y\\&space;z^\prime=-x\sin\phi\\&space;x^\prime=x\cos\phi" title="y^\prime=y\\ z^\prime=-x\sin\phi\\ x^\prime=x\cos\phi" /></a>

在本例中采用了二段Bezier曲线拼接，因此还需要保证连接处的导数一致；下图是本例中构建的Bezier曲线：（openCV绘制）

<img src=".\ref\Bezier.png" style="zoom:80" />

### 牛顿梯度下降法求交

本例中采用的梯度下降法为通过调整t和Φ参数，使得旋转面上的估计点距离光线最近，从而实现求交。因此误差函数即取估计点到光线的“点线欧氏距离”。其中梯度的估计方法为，先估计Bezier点Q(f,t)的梯度的解析表示，再利用这个表示估计误差函数的数值梯度（L为误差函数）：

<a href="https://www.codecogs.com/eqnedit.php?latex=\frac{d}{dt}Q(f,t)=\sum_{i=0}^nf_i\frac{d}{dt}B_{in}(t)=\sum_{i=0}^nf_iC_{n}^i\frac{d}{dt}[t^i(1-t)^{n-i}]\\&space;\frac{\partial}{\partial{t}}L(t,\phi)=\frac{L(t&plus;\delta{t},\phi)-L(t,\phi)}{\delta{t}}\\&space;\frac{\partial}{\partial{\phi}}L(t,\phi)=\frac{L(t,\phi&plus;\delta{\phi})-L(t,\phi)}{\delta{\phi}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\frac{d}{dt}Q(f,t)=\sum_{i=0}^nf_i\frac{d}{dt}B_{in}(t)=\sum_{i=0}^nf_iC_{n}^i\frac{d}{dt}[t^i(1-t)^{n-i}]\\&space;\frac{\partial}{\partial{t}}L(t,\phi)=\frac{L(t&plus;\delta{t},\phi)-L(t,\phi)}{\delta{t}}\\&space;\frac{\partial}{\partial{\phi}}L(t,\phi)=\frac{L(t,\phi&plus;\delta{\phi})-L(t,\phi)}{\delta{\phi}}" title="\frac{d}{dt}Q(f,t)=\sum_{i=0}^nf_i\frac{d}{dt}B_{in}(t)=\sum_{i=0}^nf_iC_{n}^i\frac{d}{dt}[t^i(1-t)^{n-i}]\\ \frac{\partial}{\partial{t}}L(t,\phi)=\frac{L(t+\delta{t},\phi)-L(t,\phi)}{\delta{t}}\\ \frac{\partial}{\partial{\phi}}L(t,\phi)=\frac{L(t,\phi+\delta{\phi})-L(t,\phi)}{\delta{\phi}}" /></a>



通过调节t和Φ的学习率可以使得求交较快收敛。下图为Bezier旋转体建模效果图：

<img src=".\ref\cup.png" style="zoom:100" />

旋转体上的青花瓷纹理见后“纹理映射”。



## 光线追踪算法

### 反射与折射

本例中反射和折射主要采用矢量运算的方法计算。反射满足关系：

<a href="https://www.codecogs.com/eqnedit.php?latex=R=I-2(I\cdot&space;N)N" target="_blank"><img src="https://latex.codecogs.com/gif.latex?R=I-2(I\cdot&space;N)N" title="R=I-2(I\cdot N)N" /></a>

其中I为入射光方向，N为表面法向，R为反射光方向。折射满足关系：

<a href="https://www.codecogs.com/eqnedit.php?latex=T=\frac{\eta_i}{\eta_t}\cdot{I}&plus;(\frac{\eta_i}{\eta_t}(-I\cdot{N})-\cos{\theta_t})\cdot{N}\\&space;\cos{\theta_t=\sqrt{1-\frac{\eta_i^2(1-\cos^2{\theta_i})}{\eta_t^2}}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?T=\frac{\eta_i}{\eta_t}\cdot{I}&plus;(\frac{\eta_i}{\eta_t}(-I\cdot{N})-\cos{\theta_t})\cdot{N}\\&space;\cos{\theta_t=\sqrt{1-\frac{\eta_i^2(1-\cos^2{\theta_i})}{\eta_t^2}}}" title="T=\frac{\eta_i}{\eta_t}\cdot{I}+(\frac{\eta_i}{\eta_t}(-I\cdot{N})-\cos{\theta_t})\cdot{N}\\ \cos{\theta_t=\sqrt{1-\frac{\eta_i^2(1-\cos^2{\theta_i})}{\eta_t^2}}}" /></a>


**（代码见tracingtools.cpp）**

### 物体求交

球和平面的求交均可通过求解参数方程得到。对于房间（即长方体求交），可对六个面直接求交，并取先交的那个面。较为简单不再赘述。需要说明的是，球的求交也可以用矢量运算的方法得到，即只涉及矢量的加减和点乘，相比于直接求解方程更加高效，本例采用的就是这种方法。

**（代码见object.cpp）**



## 光子映射算法

**（代码见PhotonMapping.cpp）**

### 光子生成与存储

本例主要采用光子映射算法估计焦散和阴影。光子由一个光子生成器（函数）生成，并由一个散播器将光子立体角均匀地从光源散播至房间各个角落。被吸收面吸收的光子随即进入一个kd-tree中，以节点形式存储。需要说明的是，光子在运动的过程中仍然遵循Ray Tracing中的折射反射吸收定律。

光子存储在kd-tree(3d-tree)中并分别依次按x,y,z方向大小排序进入子节点。在本例中约使用了数量为10^7数量级的光子。

### 光子统计与亮度估计

光子的散布和kd-tree的构建在程序的初始化阶段完成。之后的光线追踪过程中，在估计局部光照时会加入光子数的估计，从而重新调整该点亮度。

统计光子数的方法即为kd-tree的近邻搜索。先设定一个搜索范围，然后剪枝地搜索二叉树结点，直至不存在范围内节点（递归实现）。下附搜索代码（部分省略）：

```C++
void PhotonMap::RegionFind(Vector3D Goal,double range, node* p, int& photon_num)
{
    if (p == NULL)return;
    else if (inrange(Goal, range, p)){
        photon_num++;
        RegionFind(Goal, range, p->l_child, photon_num);
        RegionFind(Goal, range, p->r_child, photon_num);
    }
    else {
        if (Goal.x + range < p->node_x)RegionFind(Goal, range, p->l_child, photon_num);
        else if(Goal.x - range > p->node_x)RegionFind(Goal, range, p->r_child, photon_num);
        else{
            RegionFind(Goal, range, p->l_child, photon_num);
            RegionFind(Goal, range, p->r_child, photon_num);
        }
    }
}
```

亮度估计是在局部光照Phong模型的基础上，根据该点附近小立方体内光子数量多少，结合该点法向量对该点亮度进行重新估计。满足关系：

<a href="https://www.codecogs.com/eqnedit.php?latex=I=\alpha&space;I_0&plus;\beta&space;(N\cdot&space;V)n" target="_blank"><img src="https://latex.codecogs.com/gif.latex?I=\alpha&space;I_0&plus;\beta&space;(N\cdot&space;V)n" title="I=\alpha I_0+\beta (N\cdot V)n" /></a>

其中I0为原始亮度，α、β为系数，n为光子数，N为法向，V为视线方向。

### 焦散与阴影

按照上述方法重新估计后可得到阴影和透明物体的焦散。效果如下图所示：

<img src=".\ref\pm.png" style="zoom:80" /> 



## 纹理映射

### Bezier旋转体纹理映射

本例中的Bezier旋转体（花瓶）贴有青花瓷纹理。具体贴图方法为，在光线追踪返回花瓶上某个点时会同时返回该点所在的Bezier参数(t,Φ)；因此，只需要建立(t,Φ)到纹理坐标(u,v)之间的一一映射，即可在计算局部光照时加入纹理的对应点颜色值。

**（代码见bezier.cpp）**

### 房间（平面）纹理映射

平面的纹理映射无法直接返回某点在平面上的位置参数，因此需要利用该点坐标计算其在平面的相对位置，进而与纹理坐标(u,v)建立对应关系。

**（代码见object.cpp）**



## 加速算法

### kd-tree加速

kd-tree加速算法主要应用在光子映射的光子存储和光子搜索部分。应用该算法可以使得每一次光子搜索复杂度降至O（logn）。kd-tree还可用于光线求交，由于本例中并未用到网格模型，因此没有实现这一部分功能。

**（代码见PhotonMapping.cpp）**

### 包围盒加速

包围盒加速主要用在Bezier旋转体上。由于每次应用牛顿法计算光线与Bezier体交点时都要耗费较长时间，因此要尽可能减少计算次数，很明显交不到的光线不应该计算。包围盒将Bezier体紧密包围，判断光线与包围盒是否有交采用延长线交点法（x方向交线段与y方向交线段是否有重合），逐面进行判断。

**（代码见tracingtools.cpp）**



## 结果

下图为本项目的渲染结果。构图采用日式风格房间和花瓶，混搭富有现代感的玻璃球。

<img src=".\ref\final.png" style="zoom:50" />



## 参考文献

[1]Jiaguang Sun, Shimin Hu. *计算机图形学（新版）.清华大学出版社*

[2]Junhui Deng. *数据结构（第三版）.清华大学出版社*

[3]Photon Mapping. https://en.wikipedia.org/wiki/Photon_mapping.

[4]Bezier Curve. https://en.wikipedia.org/wiki/Bézier_curve





