package geo

import (
	"math"

	"github.com/wildmap/utility/xlog"
)

// Convex 表示由若干三角形合并而成的凸多边形。
// 在导航网格（NavMesh）系统中，将相邻的三角形合并为凸多边形，
// 可以减少寻路节点数量、提升 A* 等算法的搜索效率。
// WtCoord 用于存储该凸多边形的加权坐标，供外部寻路系统使用。
type Convex struct {
	Index          int32       // 凸多边形序号，全局唯一标识
	Vertices       []Vertice   // 凸多边形的顶点列表（顺序决定方向）
	MergeTriangles []*Triangle // 合并前组成该凸多边形的原始三角形列表
	EdgeIDs        []int32     // 凸多边形各边的唯一序号（用于邻接关系查询）
	WtCoord        Coord       // 加权坐标，由外部系统设置，供寻路启发函数使用
}

// NewConvex 将单个三角形包装为凸多边形，作为合并的初始单元。
// 凸多边形的合并从单三角形出发，逐步吸纳相邻三角形，
// 初始化时直接复制三角形的三个顶点和边序号，保持结构一致性。
func NewConvex(t *Triangle, id int32) *Convex {
	return &Convex{
		Index: id,
		Vertices: []Vertice{
			t.Vertices[0],
			t.Vertices[1],
			t.Vertices[2],
		},
		MergeTriangles: []*Triangle{t},
		EdgeIDs:        t.EdgeIDs,
	}
}

// ToRect 计算凸多边形的轴对齐包围盒（AABB），用于空间索引的粗筛。
// 遍历所有顶点取极值，时间复杂度 O(n)，适合顶点数量有限的场景。
func (c *Convex) ToRect() (minX, minZ, maxX, maxZ int32) {
	minX = int32(math.MaxInt32)
	minZ = int32(math.MaxInt32)
	for _, v := range c.Vertices {
		minX = min(v.Coord.X, minX)
		minZ = min(v.Coord.Z, minZ)
		maxX = max(v.Coord.X, maxX)
		maxZ = max(v.Coord.Z, maxZ)
	}
	return minX, minZ, maxX, maxZ
}

// MergeTriangle 将新三角形的额外顶点插入当前凸多边形的顶点序列。
// 合并的前提是新三角形与当前凸多边形共享一条边（p1、p2 为共享边顶点），
// p3 为新三角形中不属于共享边的顶点列表。
// 合并成功后顶点列表仍保持凸性，失败时不修改顶点列表并返回 false。
//
// 插入策略：在共享顶点位置后插入 p3，并尝试两种顶点排列方向，
// 通过 IsConvex 校验最终结果是否仍为凸多边形。
func (c *Convex) MergeTriangle(p1, p2 Vertice, p3 []Vertice) bool {
	for index, v := range c.Vertices {
		if v.Index == p1.Index || v.Index == p2.Index {
			newVertices := make([]Vertice, len(c.Vertices))
			copy(newVertices, c.Vertices)
			// 在共享顶点的正确位置插入新顶点
			if index != 0 || c.Vertices[index+1].Index == p2.Index || c.Vertices[index+1].Index == p1.Index {
				// 尝试两次插入：因为无法预先确定 p3 的排列方向（顺时针或逆时针）
				for range 2 {
					newVertices = make([]Vertice, len(c.Vertices)+len(p3))
					insertIndex := 0
					for insertIndex <= index {
						newVertices[insertIndex] = c.Vertices[insertIndex]
						insertIndex++
					}
					for _, p := range p3 {
						newVertices[insertIndex] = p
						insertIndex++
					}
					for _, p := range c.Vertices[index+1:] {
						newVertices[insertIndex] = p
						insertIndex++
					}
					if IsConvex(newVertices) {
						c.Vertices = newVertices
						return true
					}
					// 反转 p3 的顺序后再次尝试插入
					for i, j := 0, len(p3)-1; i < j; i, j = i+1, j-1 {
						p3[i], p3[j] = p3[j], p3[i]
					}
				}
			} else {
				for range 2 {
					newVertices = make([]Vertice, len(c.Vertices))
					copy(newVertices, c.Vertices)
					newVertices = append(newVertices, p3...)
					if IsConvex(newVertices) {
						c.Vertices = newVertices
						return true
					}
					// 反转 p3 的顺序后再次尝试追加
					for i, j := 0, len(p3)-1; i < j; i, j = i+1, j-1 {
						p3[i], p3[j] = p3[j], p3[i]
					}
				}
			}
			return false
		}
	}
	return false
}

// GetVectors 返回凸多边形按逆时针排列的顶点位置向量列表。
// 逆时针排列是圆-多边形碰撞检测（IsInterPolygon）等算法的前提条件：
// 叉积 > 0 时为逆时针，若检测到顺时针则反转顶点列表再输出。
// 注意：本方法操作的是副本，不会修改原始顶点列表。
func (c *Convex) GetVectors() []Vector {
	vertices := make([]Vertice, len(c.Vertices))
	copy(vertices, c.Vertices)

	// 检查前三个顶点的叉积：叉积 < 0 表示顺时针排列，需要反转
	if len(vertices) >= 3 && CrossProduct(vertices[0], vertices[1], vertices[2]) < 0 {
		for i, j := 0, len(vertices)-1; i < j; i, j = i+1, j-1 {
			vertices[i], vertices[j] = vertices[j], vertices[i]
		}
	}

	vecs := make([]Vector, 0, len(vertices))
	for _, v := range vertices {
		vecs = append(vecs, NewVectorByCoord(v.Coord))
	}
	return vecs
}

// TriangleHasCoord 查找包含指定点的原始三角形并返回其序号。
// 通过遍历合并三角形列表，利用三角形的点包含判断完成定位。
// 未找到时返回 -1，供导航网格寻路时确定点所在的导航节点。
func (c *Convex) TriangleHasCoord(p Coord) int32 {
	for _, t := range c.MergeTriangles {
		if t.IsCoordInside(p) {
			return t.GetIndex()
		}
	}
	return -1
}

// isCoordInsideRayCasting 通过射线法（Ray Casting）判断点是否在凸多边形内部。
// 从目标点沿某方向发射一条射线，统计与多边形各边的交叉次数：奇数次表示在内部。
// 本实现先调用 CounterClockWiseSort 确保顶点逆时针排列，
// 再通过扫描线与各边的交点判断。边界点（点在边上）直接返回 true。
// 对于单三角形凸多边形，直接委托给 Triangle.IsCoordInside 以提升性能。
func (c *Convex) isCoordInsideRayCasting(p Coord) bool {
	if len(c.MergeTriangles) == 1 {
		return c.MergeTriangles[0].IsCoordInside(p)
	}
	c.CounterClockWiseSort()
	x := p.X
	z := p.Z
	sz := len(c.Vertices)
	isIn := false

	for i := range sz {
		j := i - 1
		if i == 0 {
			j = sz - 1
		}
		vi := c.Vertices[i]
		vj := c.Vertices[j]

		xmin := vi.Coord.X
		xmax := vj.Coord.X
		if xmin > xmax {
			t := xmin
			xmin = xmax
			xmax = t
		}
		zmin := vi.Coord.Z
		zmax := vj.Coord.Z
		if zmin > zmax {
			t := zmin
			zmin = zmax
			zmax = t
		}
		// 处理水平边：点在水平边上则在多边形内
		if vj.Coord.Z == vi.Coord.Z {
			if z == vi.Coord.Z && xmin <= x && x <= xmax {
				return true
			}
			continue
		}

		// 计算射线（z=const 的水平线）与当前边的交点 X 坐标
		xt := (vj.Coord.X-vi.Coord.X)*(z-vi.Coord.Z)/(vj.Coord.Z-vi.Coord.Z) + vi.Coord.X
		if xt == x && zmin <= z && z <= zmax {
			// 点恰好落在边 [vj, vi] 上
			return true
		}
		// 交点在点左侧且在边的 Z 范围内（不含上端点，防止顶点被计数两次）
		if x < xt && zmin <= z && z < zmax {
			isIn = !isIn
		}

	}
	return isIn
}

// isCoordInsideBinarySearch 通过二分搜索法判断点是否在凸多边形内部。
// 算法分三步：
//  1. 判断点是否在以 vertices[0] 为扇心的有效角度范围内（v0→v1 与 v0→v(n-1) 之间）；
//  2. 在有效范围内二分查找点落在哪两个相邻顶点确定的扇区；
//  3. 判断点是否在最终定位的三角形内部。
//
// 时间复杂度 O(log n)，适合顶点数量较多的凸多边形，但要求顶点有序。
func (c *Convex) isCoordInsideBinarySearch(p Coord) bool {
	numOfVertice := len(c.Vertices)
	// 顶点数不足 3 无法构成多边形
	if numOfVertice < 3 {
		return false
	}
	target := Vertice{Coord: p}
	vec1 := NewVector(c.Vertices[0].Coord, c.Vertices[1].Coord)
	vec2 := NewVector(c.Vertices[0].Coord, c.Vertices[numOfVertice-1].Coord)
	vec2Coord := NewVector(c.Vertices[0].Coord, p)
	vec2CoordLen := vec2Coord.Length()
	cp1 := vec1.Cross(&vec2Coord)
	cp2 := vec2.Cross(&vec2Coord)
	isCounterClockwise := cp1 > 0
	// 点在第一条或最后一条边所在射线上，且距离不超过边长，视为在边界上
	if cp1 == 0 && vec2CoordLen <= vec1.Length() || cp2 == 0 && vec2CoordLen <= vec2.Length() {
		return true
	}
	// 步骤一：点必须夹在 v0→v1 和 v0→v(n-1) 两条射线之间
	if (cp1 > 0) == (cp2 > 0) {
		return false
	}

	s := 1
	e := numOfVertice - 1
	// 步骤二：二分定位点所在扇区 [s, e]
	for e != s+1 {
		m := (s + e) / 2
		// 判断目标点相对于 v0→vm 是否与 v0→v1 同侧（逆时针）
		if (CrossProduct(target, c.Vertices[m], c.Vertices[0]) > 0) == isCounterClockwise {
			e = m
		} else {
			s = m
		}
	}
	// 步骤三：验证点相对于边 vs→ve 的方向与整体方向一致
	vec3 := NewVector(c.Vertices[s].Coord, c.Vertices[e].Coord)
	vecS2Coord := NewVector(c.Vertices[s].Coord, p)
	return vec3.Cross(&vecS2Coord) > 0 == isCounterClockwise
}

// IsCoordInside 通过叉积法判断点是否在凸多边形内部（包含边界）。
// 算法原理：对于逆时针排列的凸多边形，内部的点相对于每条边都在其左侧（叉积 ≥ 0）；
// 叉积为 0 表示点在边上，视为内部；一旦出现叉积符号不一致，即点在外部。
// 时间复杂度 O(n)，无需预排序，是通用性最好的判断方式。
func (c *Convex) IsCoordInside(p Coord) bool {
	numOfVertice := len(c.Vertices)
	if numOfVertice < 3 {
		return false
	}
	vec1 := NewVector(c.Vertices[0].Coord, c.Vertices[1].Coord)
	vec2 := NewVector(c.Vertices[0].Coord, p)
	firstCross := vec1.Cross(&vec2)
	// 以第一条边叉积的符号为基准，后续所有边必须与之一致
	isCounterClockwise := firstCross >= 0
	// 即使第一个叉积为 0（点在首边上），也须继续验证其余边，防止误判
	for i := 1; i < numOfVertice; i++ {
		v1 := NewVector(c.Vertices[i].Coord, c.Vertices[(i+1)%numOfVertice].Coord)
		v2 := NewVector(c.Vertices[i].Coord, p)
		cp := v1.Cross(&v2)
		if isCounterClockwise {
			if cp < 0 {
				return false
			}
		} else {
			if cp > 0 {
				return false
			}
		}
	}
	return true
}

// CheckConvex 校验凸多边形的合并结果是否正确，仅用于测试阶段。
// 双重校验：
//  1. 顶点列表是否满足凸性（IsConvex）；
//  2. 合并三角形覆盖的顶点集合是否与凸多边形顶点集合完全一致。
func (c *Convex) CheckConvex() bool {
	vers := make(map[int32]bool)
	for _, ver := range c.Vertices {
		vers[ver.Index] = false
	}
	if !IsConvex(c.Vertices) {
		xlog.Errorf("convex error not a convex")
		return false
	}
	count1 := len(vers)
	// 标记合并三角形中出现的所有顶点
	for _, triangle := range c.MergeTriangles {
		for _, ver := range triangle.GetVertices() {
			vers[ver.Index] = true
		}
	}
	count2 := len(vers)
	// 检查是否有顶点未被任何合并三角形覆盖
	for key, value := range vers {
		if !value {
			xlog.Errorf("convex error has not exist value %d", key)
			return false
		}
	}

	// 合并三角形覆盖的顶点数必须等于凸多边形顶点数
	if count1 != count2 {
		xlog.Errorf("convex error has not merged\nvertice convex vertices count: %d merge vertices count: %d\n", count1, count2)
		return false
	}
	return true
}

// CounterClockWiseSort 将顶点列表原地调整为逆时针排列。
// 通过检测前三个顶点的叉积符号判断当前排列方向：
// 叉积 < 0 表示顺时针，将整个顶点列表反转即可变为逆时针。
// 该操作为射线法等算法的前置步骤，保证方向假设成立。
func (c *Convex) CounterClockWiseSort() {
	if len(c.Vertices) < 3 {
		return
	}
	if CrossProduct(c.Vertices[0], c.Vertices[1], c.Vertices[2]) < 0 {
		// 顺时针排列，反转顶点列表为逆时针
		for i, j := 0, len(c.Vertices)-1; i < j; i, j = i+1, j-1 {
			c.Vertices[i], c.Vertices[j] = c.Vertices[j], c.Vertices[i]
		}
	}
}

// GetIndex 返回凸多边形的全局唯一序号。
func (c *Convex) GetIndex() int32 {
	return c.Index
}

// GetNeighborPoints 获取当前凸多边形与相邻多边形的共享边顶点及完整顶点列表。
// 仅当两个凸多边形恰好共享 2 个顶点（即一条完整的边）时，才视为相邻关系。
// 返回的顶点列表以共享边的起点为首元素，按 t2 的原始顺序循环排列，
// 便于后续寻路算法提取相邻边的中点作为导航途经点。
func (c *Convex) GetNeighborPoints(t2 Polygon) []Vertice {
	vertices := t2.GetVertices()
	numOfVecs := len(vertices)
	pointsIndexs := make([]int, 0, numOfVecs)

	for _, v := range c.Vertices {
		for j := range vertices {
			if v.Index == vertices[j].Index {
				pointsIndexs = append(pointsIndexs, j)
			}
		}
	}
	// 恰好 2 个共享顶点才构成相邻关系（共享一条边）
	if len(pointsIndexs) == 2 {
		points := make([]Vertice, numOfVecs)
		first := pointsIndexs[0]
		second := pointsIndexs[1]
		// 确保 first 是共享边的起点（first 的下一个顶点是 second）
		if (first+1)%numOfVecs != second {
			first = second
		}
		// 从共享边起点开始循环输出 t2 的完整顶点列表
		for i := range numOfVecs {
			points[i] = vertices[(first+i)%numOfVecs]
		}
		return points
	}
	return nil
}

// GetVertices 返回凸多边形的顶点列表。
func (c *Convex) GetVertices() []Vertice {
	return c.Vertices
}

// GetCenterCoord 计算凸多边形各顶点坐标的算术平均值（几何中心）。
// 使用 int64 累加防止多个 int32 坐标相加时溢出，最终截断为 int32。
// 注意：对于非均匀形状，算术平均值不等于面积加权重心，精度要求高时应使用 GetCentroid。
func (c *Convex) GetCenterCoord() Coord {
	length := int32(len(c.Vertices))
	if length == 0 {
		return Coord{}
	}
	var coordx, coordz int64
	for _, v := range c.Vertices {
		coordx += int64(v.Coord.X)
		coordz += int64(v.Coord.Z)
	}
	return Coord{
		X: int32(coordx / int64(length)),
		Z: int32(coordz / int64(length)),
	}
}

// GetCentroid 通过 Shoelace 公式计算凸多边形的面积加权质心（几何重心）。
// 质心公式：Cx = Σ[(xi + x(i+1)) * (xi*z(i+1) - x(i+1)*zi)] / (6A)，A 为有向面积。
// 全程使用 float64 计算，避免大坐标下 int64 中间值溢出。
// 当有向面积为 0（退化多边形）时，降级为算术平均值 GetCenterCoord。
func (c *Convex) GetCentroid() Coord {
	n := len(c.Vertices)
	if n == 0 {
		return Coord{}
	}
	// 第一轮循环：用 Shoelace 公式计算有向面积 S（含正负符号）
	var S float64
	for i := range n {
		next := (i + 1) % n
		xi := float64(c.Vertices[i].Coord.X)
		zi := float64(c.Vertices[i].Coord.Z)
		xni := float64(c.Vertices[next].Coord.X)
		zni := float64(c.Vertices[next].Coord.Z)
		S += xi*zni - xni*zi
	}
	S /= 2
	// 有向面积为 0 说明多边形退化（面积为零），降级使用几何中心
	if S == 0 {
		return c.GetCenterCoord()
	}
	// 第二轮循环：累加各边对质心坐标的贡献
	var centerX, centerZ float64
	for i := range n {
		next := (i + 1) % n
		xi := float64(c.Vertices[i].Coord.X)
		zi := float64(c.Vertices[i].Coord.Z)
		xni := float64(c.Vertices[next].Coord.X)
		zni := float64(c.Vertices[next].Coord.Z)
		_cross := xi*zni - xni*zi
		centerX += (xi + xni) * _cross
		centerZ += (zi + zni) * _cross
	}
	return Coord{
		X: int32(centerX / (6 * S)),
		Z: int32(centerZ / (6 * S)),
	}
}

// GetEdgeIDs 返回凸多边形各边的唯一序号列表，用于邻接关系查询。
func (c *Convex) GetEdgeIDs() []int32 {
	return c.EdgeIDs
}

// GetEdgeMidCoords 计算凸多边形各边的中点坐标列表，顺序与顶点列表一致。
// 边中点常用于导航网格寻路中相邻凸多边形之间的过渡点（Portal 点）。
func (c *Convex) GetEdgeMidCoords() []Coord {
	nums := len(c.Vertices)
	coords := make([]Coord, 0, nums)
	for i, v := range c.Vertices {
		coords = append(coords, CalMidCoord(v.Coord, c.Vertices[(i+1)%nums].Coord))
	}
	return coords
}
