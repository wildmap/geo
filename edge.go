package geo

// Vertice 顶点
type Vertice struct {
	Index int64 // 顶点序号，唯一标识
	Coord Coord // 顶点坐标
}

// Edge 边
type Edge struct {
	WtCoord            Coord       // 重心点
	Vertices           [2]Vertice  // 顶点数组，包含两个顶点0和1
	AdjacenctTriangles []*Triangle //相邻的两个三角形
	Inflects           [2]Vertice  // 拐点数组
	IsAdjacency        bool        // 是否为邻接边
}

// CalMidCoord 计算边的终点
func (e *Edge) CalMidCoord() Coord {
	return CalMidCoord(e.Vertices[0].Coord, e.Vertices[1].Coord)
}

// GenKey 生成边的序号
func (e *Edge) GenKey() int64 {
	return GenEdgeKey(e.Vertices[0].Index, e.Vertices[1].Index)
}

// GenEdgeKey 生成边的序号
func GenEdgeKey(i, j int64) int64 {
	if i < j {
		return 10000*i + j
	}
	return 10000*j + i
}
