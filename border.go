package geo

// LocationState 表示坐标点或矩形区域相对于边界的象限位置。
// 采用位掩码设计，支持通过按位或运算组合多个象限状态，
// 适用于四叉树等空间索引结构中的快速区域划分与定位。
type LocationState int

// 象限位置常量，以 iota 左移方式定义，便于位运算组合多个象限。
// 四象限以边界矩形的中心点为原点划分：
// X 轴方向：小于中心为左，大于等于中心为右；
// Z 轴方向：大于等于中心为上，小于中心为下。
const (
	LeftTop     = 1 << iota // 左上象限
	RightTop                // 右上象限
	LeftBottom              // 左下象限
	RightBottom             // 右下象限
)

// Border 表示一块具有四象限划分能力的矩形边界区域。
// 通过嵌入 Rectangle 复用基础矩形能力，并在此基础上提供
// 以中心点为基准的空间象限定位功能，常用于四叉树节点的区域分配。
type Border struct {
	Rectangle
}

// NewBorder 以左下角坐标及宽高创建边界对象。
func NewBorder(x, z, width, height int32) Border {
	return Border{
		Rectangle: NewRectangle(x, z, width, height),
	}
}

// RectLocation 判断给定矩形（由最小/最大坐标描述的 AABB）与边界的象限重叠关系。
// 返回位掩码，若矩形同时覆盖多个象限则对应位均被置位；返回 0 表示矩形完全在边界外。
// 该方法是四叉树插入/查询操作的核心，避免对每个象限单独遍历，性能优越。
func (b *Border) RectLocation(minX, minZ, maxX, maxZ int32) LocationState {
	// 快速排斥：矩形与边界的 AABB 不相交，直接返回 0
	if minX > b.X+b.Width ||
		minZ > b.Z+b.Height ||
		maxX < b.X ||
		maxZ < b.Z {
		return 0
	}

	var location LocationState
	// 以边界中心点为分界线划分四象限
	centerX := b.X + b.Width/2
	centerZ := b.Z + b.Height/2

	// 矩形左侧部分（minX < centerX）
	if minX < centerX {
		if maxZ >= centerZ {
			location |= LeftTop
		}
		if minZ < centerZ {
			location |= LeftBottom
		}
	}
	// 矩形右侧部分（maxX >= centerX）
	if maxX >= centerX {
		if maxZ >= centerZ {
			location |= RightTop
		}
		if minZ < centerZ {
			location |= RightBottom
		}
	}
	return location
}

// CoordLocation 判断给定坐标点在边界内所属的象限。
// 点必须位于边界内部，否则返回 0。与 RectLocation 的区别在于：
// 点只能归属于唯一一个象限，返回值是单一的位掩码常量。
func (b *Border) CoordLocation(p Coord) LocationState {
	// 点在边界矩形外部，无法归属任何象限
	if !b.IsCoordInside(p) {
		return 0
	}

	centerX := b.X + b.Width/2
	centerZ := b.Z + b.Height/2
	if p.X < centerX {
		if p.Z >= centerZ {
			return LeftTop
		}
		return LeftBottom
	}
	if p.Z >= centerZ {
		return RightTop
	}
	return RightBottom
}
