# Path Planning: A* 알고리즘 실습 자료

## 📚 목차
1. [A* 알고리즘 개요](#1-a-알고리즘-개요)
2. [핵심 개념](#2-핵심-개념)
3. [알고리즘 동작 원리](#3-알고리즘-동작-원리)
4. [코드 구조 분석](#4-코드-구조-분석)
5. [구현 세부사항](#5-구현-세부사항)

---

### ROS2 가상 맵에서 실행 방법

```bash
# 빌드
colcon build --packages-select astar_planner

# 실행
source install/setup.bash
ros2 launch astar_planner astar_planner.launch.py

# RViz2에서 "2D Goal Pose" 도구로 목표 설정
```

### ROS2 가제보에서 실행 방법

```bash
source install/setup.bash
ros2 launch astar_planner astar_planner.launch.py use_gazebo:=true
```

---

## 1. A* 알고리즘 개요

### 1.1 정의
- **A*** (A-star)는 그래프 탐색 및 경로 찾기에 사용되는 **최적 경로 탐색 알고리즘**
- Dijkstra 알고리즘에 **휴리스틱(heuristic)** 함수를 추가하여 탐색 효율을 향상
- 로봇 공학, 게임 AI, 내비게이션 시스템 등에 널리 사용

### 1.2 특징
- ✅ **완전성(Completeness)**: 경로가 존재하면 반드시 찾음
- ✅ **최적성(Optimality)**: 휴리스틱이 admissible하면 최적 경로 보장
- ✅ **효율성**: 목표 지향적 탐색으로 불필요한 노드 탐색 최소화

### 1.3 다른 알고리즘과의 비교

| 알고리즘 | 특징 | 시간 복잡도 |
|---------|------|-----------|
| **BFS** | 가중치 없는 그래프, 최단 경로 | O(V + E) |
| **Dijkstra** | 가중치 그래프, 최단 경로 | O((V + E) log V) |
| **A*** | 휴리스틱 사용, 목표 지향적 | O(b^d) (평균적으로 더 빠름) |
| **Greedy Best-First** | 휴리스틱만 사용, 최적성 보장 X | O(b^d) |

---

## 2. 핵심 개념

### 2.1 비용 함수

A* 알고리즘은 세 가지 비용 함수를 사용합니다:

#### **f(n) = g(n) + h(n)**

- **g(n)**: 시작점에서 현재 노드 n까지의 **실제 비용**
- **h(n)**: 현재 노드 n에서 목표까지의 **추정 비용** (휴리스틱)
- **f(n)**: 시작점에서 목표까지의 **총 예상 비용**

```
예시:
시작점 S → 현재 노드 N → 목표 G

g(N) = 5.0  (S에서 N까지 실제로 이동한 거리)
h(N) = 3.0  (N에서 G까지 예상 거리)
f(N) = 8.0  (총 예상 비용)
```

### 2.2 휴리스틱 함수 (Heuristic Function)

목표까지의 거리를 **추정**하는 함수입니다. 본 코드에서는 **유클리드 거리**를 사용합니다.

#### 주요 휴리스틱 종류:

1. **유클리드 거리 (Euclidean Distance)** - 본 구현에서 사용
   ```
   h(n) = √[(x₁ - x₂)² + (y₁ - y₂)²]
   ```
   - 8방향 이동 가능한 그리드에 적합
   - 실제 거리에 가장 근접

2. **맨해튼 거리 (Manhattan Distance)**
   ```
   h(n) = |x₁ - x₂| + |y₁ - y₂|
   ```
   - 4방향 이동만 가능한 그리드에 적합

3. **체비셰프 거리 (Chebyshev Distance)**
   ```
   h(n) = max(|x₁ - x₂|, |y₁ - y₂|)
   ```
   - 대각선 이동 비용이 직선 이동과 같을 때

#### Admissible Heuristic
- h(n)이 실제 비용을 **절대 과대평가하지 않으면** admissible
- Admissible 휴리스틱 → A*는 최적 경로 보장
- 유클리드 거리는 admissible (직선 거리가 항상 최단)

### 2.3 자료구조

#### Open Set (Priority Queue)
- 탐색 **대기 중인 노드**들
- f(n) 값이 **작은 순서**로 정렬 (최소 힙)
- 다음에 탐색할 노드를 빠르게 선택

#### Closed Set (Hash Map)
- 이미 **탐색 완료된 노드**들
- 중복 탐색 방지
- O(1) 조회 시간

---

## 3. 알고리즘 동작 원리

### 3.1 의사 코드 (Pseudocode)

```
function A_STAR(start, goal):
    open_set = priority_queue()
    closed_set = set()
    
    start.g = 0
    start.h = heuristic(start, goal)
    start.f = start.g + start.h
    
    open_set.push(start)
    
    while open_set is not empty:
        current = open_set.pop()  // f값이 가장 작은 노드
        
        if current == goal:
            return reconstruct_path(current)
        
        closed_set.add(current)
        
        for neighbor in get_neighbors(current):
            if neighbor in closed_set:
                continue
            
            tentative_g = current.g + distance(current, neighbor)
            
            if neighbor not in open_set or tentative_g < neighbor.g:
                neighbor.parent = current
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal)
                neighbor.f = neighbor.g + neighbor.h
                
                if neighbor not in open_set:
                    open_set.push(neighbor)
    
    return failure  // 경로 없음
```

### 3.2 단계별 설명

#### **Step 1: 초기화**
```
1. 시작 노드를 open_set에 추가
2. g(start) = 0, h(start) = heuristic(start, goal)
3. f(start) = g(start) + h(start)
```

#### **Step 2: 반복 탐색**
```
while open_set이 비어있지 않을 때까지:
    1. open_set에서 f값이 가장 작은 노드 선택
    2. 목표에 도달했는지 확인
    3. 현재 노드를 closed_set에 추가
    4. 이웃 노드들 검사
```

#### **Step 3: 이웃 노드 처리**
```
각 이웃 노드에 대해:
    1. 이미 closed_set에 있으면 스킵
    2. 새로운 g값 계산 (tentative_g)
    3. 더 좋은 경로를 발견했다면:
        - 부모 노드 업데이트
        - g, h, f 값 업데이트
        - open_set에 추가 (없다면)
```

#### **Step 4: 경로 재구성**
```
목표에 도달하면:
    1. 목표 노드부터 시작
    2. 각 노드의 parent를 따라 역추적
    3. 시작 노드까지 도달
    4. 경로를 뒤집어서 반환
```

---

## 4. 코드 구조 분석

### 4.1 프로젝트 구조

```
astar_planner/
├── include/astar_planner/
│   └── astar.hpp              # A* 알고리즘 헤더
├── src/
│   ├── astar.cpp              # A* 알고리즘 구현
│   ├── path_planner_node.cpp # ROS2 노드 (메인)
│   └── simulator_node.cpp    # 시뮬레이터
├── maps/
│   ├── simple_map.txt         # 간단한 테스트 맵
│   └── example_map.txt        # 복잡한 테스트 맵
├── config/
│   └── params.yaml            # 파라미터 설정
└── launch/
    └── astar_planner.launch.py # 실행 파일
```

### 4.2 클래스 다이어그램

```
┌─────────────────────────────┐
│      GridCell               │
├─────────────────────────────┤
│ + int x                     │
│ + int y                     │
│ + operator==()              │
└─────────────────────────────┘
           ▲
           │ uses
           │
┌─────────────────────────────┐
│         Node                │
├─────────────────────────────┤
│ + GridCell cell             │
│ + double g_cost             │
│ + double h_cost             │
│ + double f_cost             │
│ + GridCell parent           │
│ + operator>()               │
└─────────────────────────────┘
           ▲
           │ uses
           │
┌─────────────────────────────┐
│        AStar                │
├─────────────────────────────┤
│ - map_: vector<vector<int>> │
│ - map_width_: int           │
│ - map_height_: int          │
├─────────────────────────────┤
│ + setMap()                  │
│ + findPath()                │
│ - calculateHeuristic()      │
│ - isValid()                 │
│ - getNeighbors()            │
│ - reconstructPath()         │
└─────────────────────────────┘
```

---

## 5. 코드 구현 세부사항

### 5.1 데이터 구조

#### GridCell 구조체
```cpp
struct GridCell {
    int x;
    int y;
    
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};
```
- 그리드 맵의 **셀 좌표**를 표현
- 동등 비교 연산자 오버로딩

#### Node 구조체
```cpp
struct Node {
    GridCell cell;      // 노드의 위치
    double g_cost;      // 시작점부터의 실제 비용
    double h_cost;      // 목표까지의 휴리스틱 비용
    double f_cost;      // 총 비용 (g + h)
    GridCell parent;    // 부모 노드 (경로 재구성용)
    
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;  // 최소 힙용
    }
};
```

### 5.2 핵심 함수 상세 분석

#### 1) `findPath()` - 메인 알고리즘

```cpp
std::vector<GridCell> AStar::findPath(
    const GridCell& start, 
    const GridCell& goal)
{
    // 1. 유효성 검사
    if (!isValid(start) || !isValid(goal)) {
        return empty_path;
    }
    
    // 2. 자료구조 초기화
    priority_queue<Node, vector<Node>, greater<Node>> open_set;
    unordered_map<GridCell, bool, GridCellHash> closed_set;
    unordered_map<GridCell, double, GridCellHash> g_score;
    unordered_map<GridCell, GridCell, GridCellHash> came_from;
    
    // 3. 시작 노드 설정
    Node start_node;
    start_node.cell = start;
    start_node.g_cost = 0.0;
    start_node.h_cost = calculateHeuristic(start, goal);
    start_node.f_cost = start_node.g_cost + start_node.h_cost;
    
    open_set.push(start_node);
    g_score[start] = 0.0;
    
    // 4. 메인 루프
    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();
        
        // 목표 도달 확인
        if (current.cell == goal) {
            return reconstructPath(came_from, start, goal);
        }
        
        // 이미 처리된 노드 스킵
        if (closed_set[current.cell]) {
            continue;
        }
        
        closed_set[current.cell] = true;
        
        // 이웃 노드 처리
        for (const auto& neighbor : getNeighbors(current.cell)) {
            if (closed_set[neighbor]) continue;
            
            // 이동 비용 계산 (유클리드 거리)
            double dx = neighbor.x - current.cell.x;
            double dy = neighbor.y - current.cell.y;
            double movement_cost = sqrt(dx*dx + dy*dy);
            double tentative_g = current.g_cost + movement_cost;
            
            // 더 좋은 경로 발견 시 업데이트
            if (g_score.find(neighbor) == g_score.end() || 
                tentative_g < g_score[neighbor]) {
                
                came_from[neighbor] = current.cell;
                g_score[neighbor] = tentative_g;
                
                Node neighbor_node;
                neighbor_node.cell = neighbor;
                neighbor_node.g_cost = tentative_g;
                neighbor_node.h_cost = calculateHeuristic(neighbor, goal);
                neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost;
                
                open_set.push(neighbor_node);
            }
        }
    }
    
    return empty_path;  // 경로 없음
}
```

**주요 포인트:**
- Priority Queue로 **f값이 작은 노드부터 처리**
- Hash Map으로 **O(1) 조회 시간** 보장
- 대각선 이동 비용을 정확히 계산 (√2 ≈ 1.414)

#### 2) `calculateHeuristic()` - 휴리스틱 함수

```cpp
double AStar::calculateHeuristic(
    const GridCell& a, 
    const GridCell& b) const
{
    // 유클리드 거리
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}
```

**특징:**
- 8방향 이동에 최적화된 유클리드 거리
- Admissible 보장 (실제 거리보다 작거나 같음)

#### 3) `getNeighbors()` - 이웃 노드 탐색

```cpp
std::vector<GridCell> AStar::getNeighbors(const GridCell& cell) const
{
    std::vector<GridCell> neighbors;
    
    // 8방향 이동
    std::vector<std::pair<int, int>> directions = {
        {0, 1},   // 상
        {0, -1},  // 하
        {1, 0},   // 우
        {-1, 0},  // 좌
        {1, 1},   // 우상
        {1, -1},  // 우하
        {-1, 1},  // 좌상
        {-1, -1}  // 좌하
    };
    
    for (const auto& dir : directions) {
        GridCell neighbor = {cell.x + dir.first, cell.y + dir.second};
        if (isValid(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}
```

**특징:**
- **8-connected grid** (대각선 이동 가능)
- 유효한 이웃만 반환 (경계 및 장애물 체크)

#### 4) `reconstructPath()` - 경로 재구성

```cpp
std::vector<GridCell> AStar::reconstructPath(
    const std::unordered_map<GridCell, GridCell, GridCellHash>& came_from,
    const GridCell& start,
    const GridCell& goal) const
{
    std::vector<GridCell> path;
    GridCell current = goal;
    
    // 목표에서 시작으로 역추적
    while (!(current == start)) {
        path.push_back(current);
        auto it = came_from.find(current);
        if (it == came_from.end()) break;
        current = it->second;
    }
    
    path.push_back(start);
    std::reverse(path.begin(), path.end());  // 뒤집기
    
    return path;
}
```

**특징:**
- 부모 포인터를 따라 **역추적**
- 최종적으로 경로를 뒤집어 시작→목표 순서로 반환

### 5.3 시간 복잡도 분석

| 연산 | 시간 복잡도 | 설명 |
|-----|-----------|------|
| **Priority Queue Push** | O(log N) | 힙 삽입 |
| **Priority Queue Pop** | O(log N) | 힙 삭제 |
| **Hash Map 조회** | O(1) | 평균 |
| **이웃 탐색** | O(1) | 최대 8개 |
| **전체 알고리즘** | O(b^d) | b=분기계수, d=깊이 |

**실제 성능:**
- 휴리스틱 덕분에 Dijkstra보다 **훨씬 빠름**
- 최악의 경우: 모든 노드 탐색 (휴리스틱이 0일 때)
- 최선의 경우: 직선 경로만 탐색

### 5.4 공간 복잡도

- **Open Set**: O(N) - 최악의 경우 모든 노드
- **Closed Set**: O(N)
- **g_score Map**: O(N)
- **came_from Map**: O(N)
- **총 공간 복잡도**: O(N)

