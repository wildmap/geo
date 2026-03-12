package geo

// Vertice 表示导航网格中的顶点，由全局唯一序号和坐标共同描述。
// 序号用于在不同三角形、凸多边形之间共享同一顶点时进行快速比对，
// 避免纯坐标比较在浮点精度下可能出现的误判。
type Vertice struct {
	Index int32 // 顶点序号，全局唯一标识，用于邻接关系查询
	Coord Coord // 顶点坐标
}

// Edge 表示导航网格中连接两个顶点的边。
// IsAdjacency 标记该边是否为两个可通行三角形的公共边（导航邻接边），
// 非邻接边（障碍边）不参与寻路的邻接图构建。
// Inflects 存储寻路漏斗算法中计算出的边上拐点，预计算后缓存于此以复用。
type Edge struct {
	WtCoord            Coord       // 边的加权中心坐标，由外部寻路系统写入
	Vertices           [2]Vertice  // 构成该边的两个顶点（索引 0 和 1）
	AdjacenctTriangles []*Triangle // 共享该边的相邻三角形（最多两个）
	Inflects           [2]Vertice  // 漏斗算法预计算的拐点对
	IsAdjacency        bool        // 是否为两个可通行区域的邻接边
}

// CalMidCoord 计算该边两端点的中点坐标。
// 边中点在寻路的漏斗算法（Funnel Algorithm）中用作相邻区域的过渡点（Portal）。
func (e *Edge) CalMidCoord() Coord {
	return CalMidCoord(e.Vertices[0].Coord, e.Vertices[1].Coord)
}

// GenKey 生成该边的全局唯一整数键，用于边的哈希索引。
// 底层调用 GenEdgeKey，保证同一条边无论顶点顺序如何均产生相同的键值。
func (e *Edge) GenKey() int64 {
	return GenEdgeKey(e.Vertices[0].Index, e.Vertices[1].Index)
}

// GenEdgeKey 根据两个顶点序号生成无向边的唯一整数键。
// 通过强制将较小序号放在高 32 位，保证 (i,j) 与 (j,i) 产生相同的键，
// 满足无向边在哈希表中的唯一存储需求，避免重复建边。
func GenEdgeKey(i, j int32) int64 {
	if i < j {
		return int64(i)<<32 | int64(j)
	}
	return int64(j)<<32 | int64(i)
}
