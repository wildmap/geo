# geo - 高性能二维空间几何计算库 🎯

![Go Version](https://img.shields.io/badge/Go-1.25+-00ADD8?style=flat&logo=go)

---

## 📖 项目简介 (Introduction)

**geo** 是一个专为 Go 语言打造的**轻量级、高性能二维几何与空间计算库**。

在游戏服务端开发（如 MMORPG、MOBA）、地图导航及物流调度等场景中，经常需要处理大量的坐标运算与空间关系判断。**geo** 库采用了 **X-Z 平面坐标系**（常见于 3D 游戏服务端的地面投影）以及 **int32 整数运算**，旨在解决浮点数运算带来的精度误差与性能损耗问题，提供了一套稳定、高效的几何原语与算法集合。

### 🎮 核心应用场景

*   **游戏开发**：
    *   技能范围判定（圆形/扇形/矩形 AOE）
    *   单位碰撞检测与物理模拟
    *   NavMesh 导航网格寻路辅助
    *   视野（FOV）计算与遮挡检测
*   **地图应用**：
    *   地理围栏（Geo-fencing）判定
    *   区域包含关系查询（点是否在多边形内）
    *   路径规划预处理与可达性分析
*   **计算几何**：
    *   向量运算与坐标变换
    *   凸包计算与多边形合并
    *   图形相交检测与分离轴定理（SAT）应用

### 💡 设计亮点

*   ⚡ **整数运算优先**：核心计算使用 `int32`，避免浮点误差，提升性能
*   🧮 **X-Z 坐标系**：符合游戏引擎习惯（Y 轴为高度，X-Z 为平面）
*   🔧 **零依赖核心**：仅依赖标准库 `math`，易于集成
*   🎯 **实战导向**：API 设计源自真实游戏服务端需求

---

## ✨ 核心特性 (Features)

### 🧱 丰富的几何图元

*   **基础类型**：
    *   [`Coord`](coord.go) - 二维坐标点（X, Z）
    *   [`Vector`](vector.go) - 位置向量（支持点积、叉积、旋转）
    *   [`Segment`](segment.go) - 线段（支持距离计算、投影点）
    *   [`Line`](line.go) - 直线方程（Ax + By + C = 0）
    *   [`Edge`](edge.go) - 边与顶点（图形构成元素）

*   **复杂形状**：
    *   [`Rectangle`](rectangle.go) - 轴对齐矩形（AABB）
    *   [`Circle`](circle.go) - 圆形（支持与线段、多边形相交检测）
    *   [`Triangle`](triangle.go) - 三角形（重心计算、点包含判断）
    *   [`Convex`](convex.go) - 凸多边形（合并、射线法/叉积法判定）
    *   [`Border`](border.go) - 边界区域（四象限位置判定）

### 🎯 高效的空间算法

#### 包含关系判断
*   ✅ 点是否在矩形内（叉积法）
*   ✅ 点是否在圆形内（距离比较）
*   ✅ 点是否在三角形内（重心坐标法）
*   ✅ 点是否在凸多边形内（射线法、叉积法、二分法）

#### 相交检测
*   ⚔️ 线段与圆的交点计算
*   ⚔️ 圆与多边形的碰撞检测
*   ⚔️ 两矩形的相交区域计算
*   ⚔️ 线段与线段的跨立实验（Straddle Test）

#### 距离计算
*   📏 点到点的欧几里得距离
*   📏 点到线段的最短距离
*   📏 点到直线的垂直距离

### 🛠️ 实用的工具函数

*   **向量运算**：
    *   加法、减法、点积、叉积
    *   向量长度、长度平方
    *   向量旋转（支持任意弧度）
    *   向量截断与缩放
    *   向量夹角计算

*   **路径生成**：
    *   [`GetBresenhamCoord`](geo.go#L142) - Bresenham 直线算法（整数光栅化）
    *   [`GetArcCoords`](geo.go#L64) - 圆弧路径采样
    *   [`GetSpiralCoords`](geo.go#L75) - 螺旋线路径生成
    *   [`GetCoordsAround`](geo.go#L13) - 圆周均匀分布点

*   **边界判定**：
    *   [`Border.CoordLocation`](border.go#L58) - 判断点相对于边界的位置（左上/右上/左下/右下）
    *   [`Border.RectLocation`](border.go#L27) - 判断矩形与边界的重叠象限

*   **几何变换**：
    *   [`Segment.Pan`](segment.go#L47) - 线段平行移动（法向量方向）
    *   [`Vector.Rotate`](vector.go#L95) - 向量旋转（左手坐标系）
    *   [`CalMidCoord`](geo.go#L208) - 计算两点中点

---

## 🚀 使用指南 (Usage)

### 📦 安装

在项目中引入该模块：

```bash
go get github.com/wildmap/geo
```

或者在 `go.mod` 中添加：

```go
require github.com/wildmap/geo v1.0.0
```

### 💻 代码示例

#### 1️⃣ 基础坐标与向量运算

```go
package main

import (
	"fmt"
	"math"
	"github.com/wildmap/geo"
)

func main() {
	// 定义两个坐标点 (X, Z)
	p1 := geo.NewCoord(100, 100)
	p2 := geo.NewCoord(200, 200)

	// 计算两点之间的距离
	dist := geo.CalDstCoordToCoord(p1, p2)
	fmt.Printf("两点距离: %.2f\n", dist) // 输出: 两点距离: 141.42

	// 向量操作：从 p1 指向 p2 的向量
	vec := geo.NewVector(p1, p2)
	fmt.Printf("向量: X=%d, Z=%d\n", vec.X, vec.Z) // 输出: 向量: X=100, Z=100

	// 计算向量长度
	length := vec.Length()
	fmt.Printf("向量长度: %.2f\n", length) // 输出: 向量长度: 141.42

	// 向量旋转 90 度（π/2 弧度）- 注意：Rotate 参数单位为弧度
	rotatedVec := vec.Rotate(math.Pi / 2)
	fmt.Printf("旋转后向量: X=%d, Z=%d\n", rotatedVec.X, rotatedVec.Z)

	// 向量缩放至原长度的 50%
	scaledVec := vec.Trunc(0.5)
	fmt.Printf("缩放后向量: X=%d, Z=%d\n", scaledVec.X, scaledVec.Z)
}
```

#### 2️⃣ 碰撞检测（圆形 vs 线段）

```go
package main

import (
	"fmt"
	"github.com/wildmap/geo"
)

func main() {
	// 创建一个圆形：圆心 (100, 100)，半径 50
	circle := geo.NewCirCle(geo.NewCoord(100, 100), 50)

	// 创建一条线段：从 (50, 50) 到 (150, 150)
	segment := geo.NewSegment(geo.NewCoord(50, 50), geo.NewCoord(150, 150))

	// 判断线段是否与圆相交
	if circle.IsIntersect(&segment) {
		fmt.Println("✅ 检测到碰撞！")
		
		// 获取第一个交点坐标
		if point, hit := circle.GetLineCross(&segment); hit {
			fmt.Printf("交点坐标: (%d, %d)\n", point.X, point.Z)
		}
	} else {
		fmt.Println("❌ 无碰撞")
	}

	// 计算圆外一点到圆上的最近点
	outsidePoint := geo.NewCoord(200, 200)
	intersectPoint := circle.GetIntersectCoord(outsidePoint)
	fmt.Printf("圆上最近点: (%d, %d)\n", intersectPoint.X, intersectPoint.Z)
}
```

#### 3️⃣ 区域判定（点是否在凸多边形内）

```go
package main

import (
	"fmt"
	"github.com/wildmap/geo"
)

func main() {
	// 定义一个三角形（特殊的凸多边形）
	t := &geo.Triangle{
		Index: 1,
		Vertices: []geo.Vertice{
			{Index: 0, Coord: geo.NewCoord(0, 0)},
			{Index: 1, Coord: geo.NewCoord(100, 0)},
			{Index: 2, Coord: geo.NewCoord(50, 100)},
		},
	}

	// 计算并缓存三角形重心
	t.CalCenter()
	fmt.Printf("三角形重心: (%d, %d)\n", t.Center.X, t.Center.Z)

	// 直接使用三角形判断
	target1 := geo.NewCoord(20, 20)
	if t.IsCoordInside(target1) {
		fmt.Printf("✅ 点 (%d, %d) 在三角形内\n", target1.X, target1.Z)
	}

	// 转换为凸多边形对象（支持更多操作）
	convex := geo.NewConvex(t, 1)

	// 判断点是否在凸多边形内（叉积法，更高效）
	target2 := geo.NewCoord(200, 200)
	if convex.IsCoordInside(target2) {
		fmt.Printf("✅ 点 (%d, %d) 在凸多边形内\n", target2.X, target2.Z)
	} else {
		fmt.Printf("❌ 点 (%d, %d) 在凸多边形外\n", target2.X, target2.Z)
	}

	// 获取凸多边形的中心点
	center := convex.GetCenterCoord()
	fmt.Printf("凸多边形中心点: (%d, %d)\n", center.X, center.Z)
}
```

#### 4️⃣ 矩形边界与象限判定

```go
package main

import (
	"fmt"
	"github.com/wildmap/geo"
)

func main() {
	// 创建一个边界：左下角 (0, 0)，宽 1000，高 1000
	border := geo.NewBorder(0, 0, 1000, 1000)

	// 创建一个矩形
	rect := geo.NewRectangle(200, 200, 100, 100)

	// 判断矩形相对于边界的位置（可能跨多个象限）
	location := rect.GetLocationToBorder(&border)
	fmt.Printf("矩形位置状态: %d\n", location)

	// 检查矩形是否在左上象限
	if location&geo.LeftTop != 0 {
		fmt.Println("矩形部分区域在左上象限")
	}

	// 判断点的位置
	point := geo.NewCoord(800, 800)
	pointLoc := point.GetLocationToBorder(&border)
	switch pointLoc {
	case geo.LeftTop:
		fmt.Println("点在左上象限")
	case geo.RightTop:
		fmt.Println("点在右上象限")
	case geo.LeftBottom:
		fmt.Println("点在左下象限")
	case geo.RightBottom:
		fmt.Println("点在右下象限")
	default:
		fmt.Println("点在边界外")
	}

	// 矩形内随机生成一个坐标
	randomPoint := rect.RandCoord()
	fmt.Printf("随机点: (%d, %d)\n", randomPoint.X, randomPoint.Z)
}
```

#### 5️⃣ 路径生成与采样

```go
package main

import (
	"fmt"
	"math"
	"github.com/wildmap/geo"
)

func main() {
	// 使用 Bresenham 算法生成整数直线路径
	start := geo.NewCoord(0, 0)
	end := geo.NewCoord(100, 50)
	linePath := geo.GetBresenhamCoord(start, end)
	fmt.Printf("直线路径包含 %d 个点\n", len(linePath))

	// 生成圆周上均匀分布的 8 个点
	center := geo.NewCoord(500, 500)
	circlePoint := geo.NewCoord(600, 500) // 圆上一点（半径100）
	circlePoints := geo.GetCoordsAround2(circlePoint, center, 8)
	fmt.Printf("圆周采样 8 个点:\n")
	for i, p := range circlePoints {
		fmt.Printf("  点%d: (%d, %d)\n", i+1, p.X, p.Z)
	}

	// 生成圆弧路径（从 circlePoint 开始，逆时针旋转 90 度）
	arcPath := geo.GetArcCoords(circlePoint, center, math.Pi/2)
	fmt.Printf("圆弧路径包含 %d 个点\n", len(arcPath))

	// 生成螺旋线路径（半径逐渐增大）
	spiralPath := geo.GetSpiralCoords(circlePoint, center, math.Pi*2, 50)
	fmt.Printf("螺旋线路径包含 %d 个点\n", len(spiralPath))
}
```

#### 6️⃣ 线段相交与距离计算

```go
package main

import (
	"fmt"
	"github.com/wildmap/geo"
)

func main() {
	// 定义两条线段
	seg1 := geo.NewSegment(geo.NewCoord(0, 0), geo.NewCoord(100, 100))
	seg2 := geo.NewSegment(geo.NewCoord(0, 100), geo.NewCoord(100, 0))

	// 判断两条线段是否相交
	if crossPoint, ok := geo.GetCrossCoord(seg1.A, seg1.B, seg2.A, seg2.B); ok {
		fmt.Printf("✅ 两条线段相交于: (%d, %d)\n", crossPoint.X, crossPoint.Z)
	} else {
		fmt.Println("❌ 两条线段不相交")
	}

	// 计算点到线段的最短距离
	point := geo.NewCoord(50, 0)
	distance := seg1.DistanceToPoint(point)
	fmt.Printf("点到线段的距离: %.2f\n", distance)

	// 获取线段上距离点最近的点（投影点）
	closestPoint := seg1.ClosestPoint(point)
	fmt.Printf("线段上最近点: (%d, %d)\n", closestPoint.X, closestPoint.Z)

	// 线段平行移动（向左侧移动 10 个单位）
	pannedSeg := seg1.Pan(10, true)
	fmt.Printf("平移后线段: (%d, %d) -> (%d, %d)\n",
		pannedSeg.A.X, pannedSeg.A.Z, pannedSeg.B.X, pannedSeg.B.Z)
}
```

---

## 🔧 核心 API 概览

### 坐标与向量

| 函数/方法 | 说明 |
|---------|------|
| `NewCoord(x, z int32) Coord` | 创建坐标点 |
| `CalDstCoordToCoord(c1, c2 Coord) float64` | 计算两点欧几里得距离 |
| `NewVector(start, end Coord) Vector` | 创建向量 |
| `Vector.Dot(v *Vector) float64` | 向量点积 |
| `Vector.Cross(v *Vector) int64` | 向量叉积（判断方向） |
| `Vector.Length() float64` | 向量模长 |
| `Vector.Rotate(angle float64) Vector` | 向量旋转（弧度） |
| `Vector.GetAngle(v *Vector) float64` | 计算两向量夹角 |

### 几何形状

| 类型 | 核心方法 |
|-----|---------|
| **Circle** | `IsIntersect`, `GetLineCross`, `IsInterPolygon` |
| **Rectangle** | `IsCoordInside`, `RandCoord`, `GetVerticeCoords` |
| **Triangle** | `IsCoordInside`, `CalCenter`, `ToRect` |
| **Convex** | `IsCoordInside`, `MergeTriangle`, `GetVectors` |
| **Segment** | `DistanceToPoint`, `ClosestPoint`, `Pan` |

### 路径生成

| 函数 | 说明 |
|-----|------|
| `GetBresenhamCoord(p1, p2 Coord) []Coord` | Bresenham 直线算法 |
| `GetArcCoords(start, center Coord, angle float64) []Coord` | 圆弧路径采样 |
| `GetSpiralCoords(...) []Coord` | 螺旋线路径生成 |
| `GetCoordsAround2(circle, center Coord, n int) []Coord` | 圆周均匀采样 |

---

## 🎓 算法说明

### 点在多边形内判断

本库提供多种判断算法：

1. **射线法（Ray Casting）** - [`Convex.IsCoordInside1`](convex.go#L121)
   - 适用于任意凸多边形
   - 从点发射一条射线，统计与多边形边的交点个数

2. **叉积法（Cross Product）** - [`Convex.IsCoordInside`](convex.go#L213)
   - 高效判断凸多边形
   - 检查点是否在所有边的同一侧

3. **二分搜索法** - [`Convex.IsCoordInside2`](convex.go#L175)
   - 针对顶点数较多的凸多边形优化
   - 先二分定位点所在扇区，再判断是否在三角形内

### 线段相交判断

使用 **跨立实验（Straddle Test）**：
1. 排斥实验：快速矩形包围盒检测
2. 跨立实验：通过叉积判断端点是否分居两侧

参考实现：[`segment.go`](segment.go#L74)

---

## 📌 注意事项

1. **坐标系统**：本库使用 **X-Z 平面坐标系**（Y 轴为高度），符合 Unity/Unreal 等 3D 引擎习惯
2. **整数精度**：核心坐标使用 `int32`，避免浮点误差，但几何计算结果（如距离、角度）为 `float64`
3. **角度单位**：向量旋转函数 [`Vector.Rotate`](vector.go#L95) 使用 **弧度制**（Radian），而非角度制
4. **左手坐标系**：向量叉积结果 > 0 表示向量在左侧，< 0 表示在右侧
5. **依赖项**：本库依赖 [`github.com/wildmap/utility`](https://github.com/wildmap/utility) 提供的浮点数比较工具
