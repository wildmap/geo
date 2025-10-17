# geo 地理空间坐标

Geo 是一个用于处理地理空间数据和几何计算的 Go 语言包。它提供了多种几何对象的定义和操作方法，包括点、线段、矩形、圆形、三角形、凸多边形等，并实现了它们之间的空间关系判断和几何计算功能。

## 功能特性
### 基础几何对象
+ `Coord`: 表示二维坐标点`(X, Z)`
+ `Vector`: 表示二维向量，支持向量运算（加法、减法、点积、叉积等）
+ `Segment`: 表示线段，支持距离计算、平移等操作
+ `Rectangle`: 表示矩形，支持点包含判断、边界检测等
+ `Circle`: 表示圆形，支持与线段、多边形的相交判断等

### 复杂几何结构
+ `Triangle`: 表示三角形，支持点包含判断、邻接边检测等
+ `Convex`: 表示凸多边形，支持点包含判断、与其他几何体的相交检测等
+ `Border`: 表示边界区域，用于空间划分和位置状态判断 

### 几何计算功能
+ 点与几何体的位置关系判断
+ 几何体之间的相交检测
+ 距离计算（点到点、点到线段等）
+ 线段交点计算
+ 向量运算（点积、叉积、旋转、平移等）
+ 特殊路径生成（圆弧、螺旋线、布雷森汉姆直线等）

### 空间索引支持
+ 通过 Border 实现四叉树风格的空间划分
+ 支持快速定位几何体在空间中的位置状态

## 主要类型和接口
### Polygon 接口
Polygon 是一个接口，定义了多边形的基本操作：
+ IsCoordInside(p Coord) bool: 判断点是否在多边形内
+ GetVectors() []Vector: 获取多边形的向量表示
+ ToRect() (minX, minZ, maxX, maxZ int64): 获取包围矩形
+ GetIndex() int64: 获取唯一标识
+ GetEdgeIDs() []int64: 获取边的标识列表
+ GetEdgeMidCoords() []Coord: 获取边的中点坐标
+ GetVertices() []Vertice: 获取顶点列表

### LocationState
位置状态枚举，用于表示几何体在空间划分中的位置：
+ LeftTop: 左上区域
+ RightTop: 右上区域
+ LeftBottom: 左下区域
+ RightBottom: 右下区域

## 使用示例
下面是一个简单的使用示例：
```go
// 创建一个矩形
rect := NewRectangle(0, 0, 100, 100)

// 创建一个点
point := Coord{X: 50, Z: 50}

// 判断点是否在矩形内
if rect.IsCoordInside(point) {
    fmt.Println("Point is inside rectangle")
}

// 创建一个圆
circle := NewCirCle(Coord{X: 0, Z: 0}, 50)

// 创建一条线段
segment := NewSegment(Coord{X: -100, Z: 0}, Coord{X: 100, Z: 0})

// 判断线段是否与圆相交
if circle.IsIntersect(&segment) {
    fmt.Println("Segment intersects with circle")
}
```

## 应用场景
该包适用于需要进行几何计算和空间分析的应用场景，如：
+ 游戏开发中的碰撞检测
+ 地理信息系统(GIS)的空间查询
+ 计算机图形学中的几何处理
+ 机器人路径规划中的障碍物检测
+ 计算几何算法的实现基础

## 注意事项
+ 所有坐标计算均使用 int64 类型，以保证精度
+ 角度计算使用弧度制
+ 向量运算中，叉积的符号用于判断相对方向（左手坐标系）
+ 部分计算涉及浮点数运算，使用了适当的精度控制