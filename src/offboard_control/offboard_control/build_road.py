from collections import deque
import sys

class GridSolver:
    def __init__(self):
        self.ROWS = 7
        self.COLS = 9
        self.BARRIER_COUNT = 3
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 右、下、左、上
        self.grid = None
        self.barriers = None    #(B,A)
        self.accessible_cells = []
        self.accessible_count = 0
        self.start = (0, 8)
        self._initialize_grid()
    
    def _initialize_grid(self):
        """初始化网格，所有单元格均可访问"""
        self.grid = [[1 for _ in range(self.COLS)] for _ in range(self.ROWS)]
        self.accessible_cells = []
        self.accessible_count = 0
    
    def set_barriers(self, barriers):
        """
        设置障碍物位置
        
        参数:
            barriers: 包含三个连续障碍物坐标的列表，每个坐标格式为(row, col)
        """
        if len(barriers) != self.BARRIER_COUNT:
            raise ValueError(f"必须提供{self.BARRIER_COUNT}个障碍物位置")
        
        # # 验证障碍物是否连续
        # if not self._check_barrier_continuity(barriers):
        #     raise ValueError("障碍物必须是水平或垂直连续的三连格")
        
        # 验证障碍物是否包含起点
        if any(barrier == self.start for barrier in barriers):
            raise ValueError("障碍物不能包含起点(0,0)")
        
        # 重置网格并设置障碍物
        self._initialize_grid()
        self.barriers = barriers
        for r, c in barriers:
            if not (0 <= r < self.ROWS and 0 <= c < self.COLS):
                raise ValueError(f"障碍物位置({r},{c})超出网格范围")
            self.grid[r][c] = 0
        
        # 检查连通性
        if not self.check_connectivity():
            raise ValueError("障碍物设置导致连通区域不足80%，请重新选择障碍物位置")
        
        # 收集可访问单元格
        self._collect_accessible_cells()
    
    def _check_barrier_continuity(self, barriers):
        """检查障碍物是否水平或垂直连续"""
        # 按行和列排序
        sorted_by_row = sorted(barriers, key=lambda x: (x[0], x[1]))
        sorted_by_col = sorted(barriers, key=lambda x: (x[1], x[0]))
        
        # 检查水平连续
        if (sorted_by_row[0][0] == sorted_by_row[1][0] == sorted_by_row[2][0] and
            sorted_by_row[0][1] + 1 == sorted_by_row[1][1] and
            sorted_by_row[1][1] + 1 == sorted_by_row[2][1]):
            return True
        
        # 检查垂直连续
        if (sorted_by_col[0][1] == sorted_by_col[1][1] == sorted_by_col[2][1] and
            sorted_by_col[0][0] + 1 == sorted_by_col[1][0] and
            sorted_by_col[1][0] + 1 == sorted_by_col[2][0]):
            return True
        
        return False
    
    def _collect_accessible_cells(self):
        """收集所有可访问的单元格"""
        self.accessible_cells = []
        for r in range(self.ROWS):
            for c in range(self.COLS):
                if self.grid[r][c] == 1:
                    self.accessible_cells.append((r, c))
        self.accessible_count = len(self.accessible_cells)
    
    def check_connectivity(self):
        """检查从起点出发是否至少能访问80%的可访问单元格"""
        if self.grid[0][0] != 1:
            return False
        
        visited = [[False for _ in range(self.COLS)] for _ in range(self.ROWS)]
        queue = deque()
        queue.append(self.start)
        visited[0][0] = True
        reachable_count = 1
        
        while queue:
            r, c = queue.popleft()
            
            for dr, dc in self.directions:
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.ROWS and 0 <= nc < self.COLS and
                    not visited[nr][nc] and self.grid[nr][nc] == 1):
                    visited[nr][nc] = True
                    queue.append((nr, nc))
                    reachable_count += 1
        
        total_accessible = self.ROWS * self.COLS - self.BARRIER_COUNT
        return reachable_count >= total_accessible * 0.8
    
    def ab_to_coordinate(self,A_str, B_str):
        """
        将 A 和 B 字符串转换为网络坐标 (x, y)。
        
        参数:
            A_str (str): 以 'A' 开头的字符串（例如 "A9"），代表行（y 坐标）。
            B_str (str): 以 'B' 开头的字符串（例如 "B1"），代表列（x 坐标）。
        
        返回:
            tuple: 网络坐标 (x, y) 作为整数。
        
        异常:
            ValueError: 如果输入字符串不以 'A' 或 'B' 开头，或数字部分无效。
        """
        # 验证 A_str 是否以 'A' 开头并有数字部分
        if not A_str.startswith('A'):
            raise ValueError(f"Invalid A string: '{A_str}'. Must start with 'A'.")
        y_part = A_str[1:]  # 提取数字部分（去掉 'A'）
        if not y_part.isdigit():
            raise ValueError(f"Invalid number in A string: '{A_str}'. Must have digits after 'A'.")
        
        # 验证 B_str 是否以 'B' 开头并有数字部分
        if not B_str.startswith('B'):
            raise ValueError(f"Invalid B string: '{B_str}'. Must start with 'B'.")
        x_part = B_str[1:]  # 提取数字部分（去掉 'B'）
        if not x_part.isdigit():
            raise ValueError(f"Invalid number in B string: '{B_str}'. Must have digits after 'B'.")
        
        # 转换为整数并返回坐标 (x, y)
        x = int(x_part) - 1
        y = int(y_part) - 1
        return (x, y)
    
    def print_grid(self):
        """打印当前网格状态"""
        print("\n当前网格状态 (S=起点, X=障碍物, .=可访问):")
        print("   " + " ".join(f"{j+1:2}" for j in range(self.COLS)))
        for i in range(self.ROWS):
            print(f"{i+1:2} ", end="")
            for j in range(self.COLS):
                if (i, j) == self.start:
                    print(" S ", end="")
                elif (i, j) in self.barriers:
                    print(" X ", end="")
                elif self.grid[i][j] == 1:
                    print(" . ", end="")
                else:
                    print(" ? ", end="")
            print()
        print()
    
    def find_shortest_path(self, start, end):
        """
        使用BFS查找两点间最短路径
        
        返回:
            path: 路径列表，包含从起点到终点的所有坐标
            steps: 实际步数（路径长度-1）
        """
        if start == end:
            return [start], 0
        
        visited = [[False for _ in range(self.COLS)] for _ in range(self.ROWS)]
        parent = [[(-1, -1) for _ in range(self.COLS)] for _ in range(self.ROWS)]
        queue = deque()
        
        queue.append(start)
        visited[start[0]][start[1]] = True
        parent[start[0]][start[1]] = (-1, -1)  # 起点无父节点
        
        found = False
        while queue:
            r, c = queue.popleft()
            if (r, c) == end:
                found = True
                break
            
            for dr, dc in self.directions:
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.ROWS and 0 <= nc < self.COLS and
                    not visited[nr][nc] and self.grid[nr][nc] == 1):
                    visited[nr][nc] = True
                    parent[nr][nc] = (r, c)
                    queue.append((nr, nc))
        
        if not found:
            return [], -1
        
        # 重建路径
        path = []
        current = end
        while current != (-1, -1):
            path.append(current)
            current = parent[current[0]][current[1]]
        path.reverse()
        return path, len(path) - 1
    
    def nearest_neighbor_tsp(self):
        """使用最近邻算法求解TSP访问顺序"""
        visited = [False] * self.accessible_count
        visit_order = []
        
        # 查找起点索引
        start_index = -1
        for i, cell in enumerate(self.accessible_cells):
            if cell == self.start:
                start_index = i
                break
        
        if start_index == -1:
            return []
        
        visit_order.append(self.start)
        visited[start_index] = True
        current = self.start
        
        while len(visit_order) < self.accessible_count:
            min_dist = float('inf')
            next_index = -1
            next_cell = None
            
            for i, cell in enumerate(self.accessible_cells):
                if not visited[i]:
                    dist = abs(current[0] - cell[0]) + abs(current[1] - cell[1])
                    if dist < min_dist:
                        min_dist = dist
                        next_index = i
                        next_cell = cell
            
            if next_index == -1:
                break
            
            visit_order.append(next_cell)
            visited[next_index] = True
            current = next_cell
        
        return visit_order
    
    def solve(self):
        """求解最短全遍历路径"""
        if not self.barriers:
            raise RuntimeError("请先设置障碍物")
        
        if self.accessible_count <= 1:
            return {
                'total_steps': 0,
                'path': [self.start],
                'barriers': self.barriers,
                'efficiency': 0.0,
                'point_info': [],  # 添加点信息字段
                'world_path': []   # 添加世界坐标路径字段
            }
        
        # 获取访问顺序
        visit_order = self.nearest_neighbor_tsp()
        
        # 构建完整路径
        full_path = [self.start]
        total_steps = 0
        
        # 计算每段路径，同时记录每段起止在 full_path 中的索引
        n = len(visit_order)
        segment_boundaries = [0]  # 每段 path 在 full_path 中的起始索引
        for i in range(n):
            start_point = visit_order[i]
            end_point = visit_order[(i + 1) % n]

            segment_path, segment_steps = self.find_shortest_path(start_point, end_point)
            if segment_steps < 0:
                raise RuntimeError(f"无法从{start_point}到达{end_point}")

            # 跳过第一个节点（已存在）
            full_path.extend(segment_path[1:])
            total_steps += segment_steps
            segment_boundaries.append(len(full_path) - 1)

        # 返航段起点: visit_order[-2] 即最后一个访问点 → start
        # segment_boundaries 中倒数第二段末尾 = 最后一段开始前 full_path 的最后一个索引
        # 返航段起始索引 = segment_boundaries[-2] (最后一段 BFS 的起点在 full_path 中的位置)
        return_start_index = segment_boundaries[-2]

        # 计算效率
        efficiency = (self.accessible_count / total_steps) * 100 if total_steps > 0 else 0.0

        # 生成点信息
        point_info = self.generate_point_info(full_path)

        # 转换到世界坐标系
        world_path = self.to_world_change(full_path)

        return {
            'total_steps': total_steps,
            'path': full_path,
            'barriers': self.barriers,
            'efficiency': efficiency,
            'point_info': point_info,
            'world_path': world_path,
            'return_start_index': return_start_index,  # 返航段在 path 中的起始索引
        }
    
    def draw_path(self, path):
        """
        在终端上绘制路径图
        
        参数:
            path: 路径列表，包含所有经过的坐标
        """
        if not path:
            print("路径为空，无法绘制")
            return
        
        # 创建网格的字符表示
        grid_chars = [[' ' for _ in range(self.COLS)] for _ in range(self.ROWS)]
        
        # 标记障碍物
        for r, c in self.barriers:
            grid_chars[r][c] = 'X'
        
        # 标记起点
        start_r, start_c = self.start
        grid_chars[start_r][start_c] = 'S'
        
        # 标记路径方向
        for i in range(1, len(path)):
            prev_r, prev_c = path[i-1]
            curr_r, curr_c = path[i]
            
            # 确定移动方向
            dr = curr_r - prev_r
            dc = curr_c - prev_c
            
            # 设置方向箭头
            if dr == 1:  # 向下
                arrow = '↓'
            elif dr == -1:  # 向上
                arrow = '↑'
            elif dc == 1:  # 向右
                arrow = '→'
            elif dc == -1:  # 向左
                arrow = '←'
            else:
                arrow = '•'  # 同一位置
            
            # 标记路径（起点和终点特殊处理）
            if (curr_r, curr_c) == self.start:
                grid_chars[curr_r][curr_c] = 'E'  # 终点
            elif grid_chars[curr_r][curr_c] == ' ':
                grid_chars[curr_r][curr_c] = arrow
        
        # 绘制网格边框
        print("\n" + "=" * (self.COLS * 4 + 1))
        print("路径图 (S=起点, E=终点, X=障碍物, 箭头=移动方向):")
        print("   " + " ".join(f"{j+1:2}" for j in range(self.COLS)))
        
        # 绘制网格内容
        for i in range(self.ROWS):
            print(f"{i+1:2} ", end="")
            for j in range(self.COLS):
                # 添加边框
                print("|" if j == 0 else "", end="")
                # 打印单元格内容
                print(f" {grid_chars[i][j]} ", end="")
            print("|")
            # 添加分隔线
            print("   " + "+---" * self.COLS + "+")
        
        # 显示路径统计信息
        print(f"路径长度: {len(path)} 步")
        print(f"访问格子: {len(set(path))} 个")
        print("=" * (self.COLS * 4 + 1))
    
    def generate_point_info(self, path):
        """
        生成路径中每个点的详细信息
        
        返回:
            包含每个点信息的列表，每个元素为字典格式:
            {
                "point": (row, col),         # 坐标
                "is_start": bool,            # 是否是起点
                "is_end": bool,              # 是否是终点
                "direction": str,            # 移动方向: '↑', '↓', '←', '→' 或 '' (起点/终点)
                "direction_name": str,       # 方向名称: 'up', 'down', 'left', 'right', 'start', 'end'
                "step": int,                 # 步数编号 (从0开始)
                "grid_position": (row+1, col+1)  # 网格中的实际位置(1-based)
            }
        """
        point_info = []
        
        for i, (r, c) in enumerate(path):
            # 确定移动方向
            direction = ''
            direction_name = ''
            
            if i == 0:
                # 起点
                direction = 'S'
                direction_name = 'start'
            elif i == len(path) - 1:
                # 终点
                direction = 'E'
                direction_name = 'end'
            else:
                # 计算移动方向
                prev_r, prev_c = path[i-1]
                next_r, next_c = path[i]
                
                dr = next_r - prev_r
                dc = next_c - prev_c
                
                if dr == 1:
                    direction = '↓'
                    direction_name = 'down'
                elif dr == -1:
                    direction = '↑'
                    direction_name = 'up'
                elif dc == 1:
                    direction = '→'
                    direction_name = 'right'
                elif dc == -1:
                    direction = '←'
                    direction_name = 'left'
                else:
                    direction = '•'
                    direction_name = 'stay'
            
            point_info.append({
                "point": (r, c),
                "is_start": (i == 0),
                "is_end": (i == len(path) - 1),
                "direction": direction,
                "direction_name": direction_name,
                "step": i,
                "grid_position": (r + 1, c + 1)  # 转换为1-based索引
            })
        
        return point_info
    
    def to_world_change(self, path):
        """
        将网格坐标系上的点转换成世界坐标系
        
        参数:
            path: 路径列表，包含所有经过的坐标 (row, col) - 0-based索引
            
        返回:
            世界坐标系路径列表，每个元素为 (x, y) 元组
            
        坐标系说明:
            - 网格坐标系: 
                - 原点在左上角 (0,0) 对应网格位置 (1,1)
                - 行增加方向向下 (row+)
                - 列增加方向向右 (col+)
                
            - 世界坐标系:
                - 原点在 (9,0) (1-based 网格位置)
                - 9格方向为世界坐标系的x半轴 (正方向为左)
                - 7格方向为世界坐标系的y负半轴 (负方向为下)
                - 转换公式为:
                    x = (8 - c) * SIZE  # 因为 (9,0) 对应 col=8 (0-based)
                    y = -r * SIZE

        """
        SIZE = 0.5  # 一格宽
        world_path = []
        
        for r, c in path:
            # 计算相对于网络原点 (0,8) 的偏移(对应索引：A9,B1)
            x = (8 - c) * SIZE
            y = -r * SIZE
            
            world_path.append((x, y))
        
        return world_path

# 使用示例
if __name__ == "__main__":
    solver = GridSolver()
    
    try:
        # # 设置障碍物（必须连续三格）
        barriers = [solver.ab_to_coordinate('A3', 'B4'), 
                    solver.ab_to_coordinate('A4', 'B4'), 
                    solver.ab_to_coordinate('A5', 'B4')]  # 连续
        # barriers = [(3, 2), (3, 3), (3, 4)]  # 水平连续
        solver.set_barriers(barriers)
        
        # 打印网格
        solver.print_grid()
        
        # 求解最短路径
        result = solver.solve()
        
        # 输出结果
        print("\n=== 求解结果 ===")
        print(f"总步数: {result['total_steps']}")
        print(f"可访问格子数: {solver.accessible_count}")
        print(f"路径效率: {result['efficiency']:.1f}%")
        print("\n障碍物位置:")
        for barrier in result['barriers']:
            print(f"({barrier[0]+1},{barrier[1]+1})", end=" ")
        
        # 绘制路径图
        solver.draw_path(result['path'])
        
        # 打印前10个点的信息
        print("\n路径点信息 (前10个点):")
        for i, info in enumerate(result['point_info'][:10]):
            print(f"步数 {info['step']:2d}: 位置 ({info['grid_position'][0]},{info['grid_position'][1]}) | "
                  f"方向: {info['direction']} ({info['direction_name']}) | "
                  f"起点: {'是' if info['is_start'] else '否'} | "
                  f"终点: {'是' if info['is_end'] else '否'}")
        
        # 打印最后1个点的信息
        if len(result['point_info']) > 10:
            last_info = result['point_info'][-1]
            print(f"...")
            print(f"步数 {last_info['step']:2d}: 位置 ({last_info['grid_position'][0]},{last_info['grid_position'][1]}) | "
                  f"方向: {last_info['direction']} ({last_info['direction_name']}) | "
                  f"起点: {'是' if last_info['is_start'] else '否'} | "
                  f"终点: {'是' if last_info['is_end'] else '否'}")
        
        # 打印世界坐标系路径
        print("\n世界坐标系路径 (前10个点):")
        for i, (x, y) in enumerate(result['world_path'][:10]):
            print(f"点 {i:2d}: (x={x:.2f}, y={y:.2f})")
        
        # 打印最后1个点
        if len(result['world_path']) > 10:
            x, y = result['world_path'][-1]
            print(f"...")
            print(f"点 {len(result['world_path'])-1:2d}: (x={x:.2f}, y={y:.2f})")
    
    except Exception as e:
        print(f"错误: {str(e)}", file=sys.stderr)