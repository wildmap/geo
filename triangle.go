package geo

import (
	"math"
)

// Triangle 表示导航网格中的一个三角形单元。
// EdgeIDs 在系统启动时根据顶点序号生成，用于构建三角形间的邻接关系图。
// Center 为预计算的重心坐标，缓存于结构体中以加速寻路算法中的启发函数计算，
// 避免在每次路径查询时重复计算。
type Triangle struct {
	Index    int32     // 三角形序号，全局唯一标识
	Vertices []Vertice // 三个顶点列表（顺序决定法向量方向）
	EdgeIDs  []int32   // 三条边的唯一序号，启动时生成并缓存
	Center   Coord     // 三角形重心，由 CalCenter 预计算后缓存
}

// IsCoordInside 通过叉积法判断点是否在三角形内部（含边界）。
// 原理：从点 p 分别向三个顶点作向量，相邻向量两两叉积；
// 若所有叉积的符号一致（均 ≥ 0 或均 ≤ 0），则点在三角形内或边上。
// 叉积为 0 表示点与对应顶点连线共线（即点在边上），视为内部。
func (t *Triangle) IsCoordInside(p Coord) bool {
	pa := NewVector(p, t.Vertices[0].Coord)
	pb := NewVector(p, t.Vertices[1].Coord)
	pc := NewVector(p, t.Vertices[2].Coord)

	b1 := pa.Cross(&pb) >= 0
	b2 := pb.Cross(&pc) >= 0

	if b1 != b2 {
		return false
	}

	b3 := pc.Cross(&pa) >= 0
	return b2 == b3
}

// GetIndex 返回三角形的全局唯一序号。
func (t *Triangle) GetIndex() int32 {
	return t.Index
}

// GetLocationToBorder 判断三角形包围盒与边界的象限重叠关系。
// 将三角形退化为 AABB 后复用 RectLocation，避免精确的三角形-矩形相交计算。
func (t *Triangle) GetLocationToBorder(b *Border) LocationState {
	minX, minZ, maxX, maxZ := t.ToRect()
	return b.RectLocation(minX, minZ, maxX, maxZ)
}

// ToRect 计算三角形的轴对齐包围盒（AABB）的四个极值坐标。
// 遍历三个顶点取极值，用于空间索引的快速粗筛。
func (t *Triangle) ToRect() (minX, minZ, maxX, maxZ int32) {
	minX = int32(math.MaxInt32)
	minZ = int32(math.MaxInt32)
	for _, v := range t.Vertices {
		minX = min(v.Coord.X, minX)
		minZ = min(v.Coord.Z, minZ)
		maxX = max(v.Coord.X, maxX)
		maxZ = max(v.Coord.Z, maxZ)
	}
	return minX, minZ, maxX, maxZ
}

// GetEdgeIDs 返回三角形三条边的唯一序号列表。
func (t *Triangle) GetEdgeIDs() []int32 {
	return t.EdgeIDs
}

// GetEdgeMidCoords 返回三角形三条边的中点坐标列表，顺序与顶点列表一致。
// 边中点作为相邻三角形间的通行点（Portal），用于漏斗算法平滑路径。
func (t *Triangle) GetEdgeMidCoords() []Coord {
	coords := make([]Coord, 3)
	coords[0] = CalMidCoord(t.Vertices[0].Coord, t.Vertices[1].Coord)
	coords[1] = CalMidCoord(t.Vertices[1].Coord, t.Vertices[2].Coord)
	coords[2] = CalMidCoord(t.Vertices[2].Coord, t.Vertices[0].Coord)
	return coords
}

// GetVertices 返回三角形的顶点列表。
func (t *Triangle) GetVertices() []Vertice {
	return t.Vertices
}

// GetVectors 返回三角形按逆时针排列的顶点位置向量列表。
// 通过检测三顶点叉积符号确认排列方向，若为顺时针则交换后两个顶点使其变为逆时针，
// 保证与 IsInterPolygon 等要求逆时针输入的算法的接口约定一致。
func (t *Triangle) GetVectors() []Vector {
	vecs := make([]Vector, 3)
	vecs[0] = NewVectorByCoord(t.Vertices[0].Coord)
	vecs[1] = NewVectorByCoord(t.Vertices[1].Coord)
	vecs[2] = NewVectorByCoord(t.Vertices[2].Coord)

	// 叉积 < 0 表示顺时针排列，交换 vecs[1] 和 vecs[2] 反转为逆时针
	if CrossProduct(t.Vertices[0], t.Vertices[1], t.Vertices[2]) < 0 {
		vecs[1], vecs[2] = vecs[2], vecs[1]
	}

	return vecs
}

// GetNeighborEdgeNums 统计两个三角形之间共享顶点的数量。
// 两个三角形共享 2 个顶点即共享一条边（邻接三角形），
// 共享 3 个顶点表示同一个三角形。
func (t *Triangle) GetNeighborEdgeNums(t2 *Triangle) int {
	var cnt int
	for i := range t.Vertices {
		for j := range t2.Vertices {
			if t.Vertices[i].Index == t2.Vertices[j].Index {
				cnt++
			}
		}
	}
	return cnt
}

// CalCenter 计算并缓存三角形的重心坐标。
// 重心公式：重心 = (v0 + v1 + v2) / 3，即三顶点坐标的算术平均值。
// 使用 int64 累加三个 int32 坐标，防止求和溢出，最终截断为 int32。
// 预计算并缓存是为了在频繁调用 GetCenter 的寻路场景中避免重复计算。
func (t *Triangle) CalCenter() {
	x := (int64(t.Vertices[0].Coord.X) + int64(t.Vertices[1].Coord.X) + int64(t.Vertices[2].Coord.X)) / 3
	z := (int64(t.Vertices[0].Coord.Z) + int64(t.Vertices[1].Coord.Z) + int64(t.Vertices[2].Coord.Z)) / 3
	t.Center = Coord{
		X: int32(x),
		Z: int32(z),
	}
}

// GetCenter 返回三角形预计算的重心坐标。
// 需在 CalCenter 调用后使用，否则返回零值坐标。
func (t *Triangle) GetCenter() Coord {
	return t.Center
}
