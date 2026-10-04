# 厂内多巡检车无碰撞路径规划后端

基于 CBS（Conflict-Based Search）的多智能体路径规划 HTTP 服务。所有车辆在 t=0
同时出发，每步可移动到上下左右四邻格或原地等待；CBS 在保证各车独立最优的前提下做
联合协调，返回步数上限内的最小总到达时刻方案，并非按车序贪心。

## 构建与启动

```bash
go build ./...
go run ./cmd/server -port 8080   # 也可用环境变量 PORT=8080
```

- GET /healthz：健康检查。
- POST /plan：提交一次规划请求。每个请求拥有独立的约束树与节点预算；客户端断开
  会通过 context 立即停止搜索。

## 请求格式（JSON）

| 字段 | 类型 | 范围/要求 |
| --- | --- | --- |
| width / height | int | 1-20，矩形网格宽高，坐标从 0 开始 |
| obstacles | [{x,y}] | 障碍格列表，须在网格内，可重复（自动去重） |
| vehicles | 列表 | 1-5 辆，每辆含唯一字符串 id、整数 start、goal |
| max_steps | int | 1-64，单车上限步数 |
| budget | int | 1-20000，CBS 约束树节点展开预算（根节点计 1） |

校验失败返回 HTTP 400，JSON 形如 status=error 并带 error 文案。会拒绝越界坐标、起终点
落在障碍上、重复 ID、重复起点或重复终点、空 ID、超范围数值、非法 JSON 与未知字段。

## 响应格式与状态

- optimal：已找到并在约束树按总代价最优优先展开下证明最优，含 total_cost 与每车结果。
- infeasible：约束树已穷尽，该 max_steps 内确实无解，不返回任何车辆路径。
- undecided：CBS 节点预算耗尽或搜索被取消，尚未证明有无解；不能当作无解，也不返回
  部分车辆路线。
- cancelled：请求被客户端取消而中止。

每车结果：

- path：从时刻 0 到首次到达目标时刻的逐时刻坐标（长度等于到达时刻 + 1）。到达目标
  后车辆永久停留，后续时刻视为仍在目标格。
- arrival_time：首次到达目标的时刻；起终点相同为 0。
- cost：单车代价，等于到达时刻（等待计入代价；到达后停留不计）。
- total_cost：各车到达时刻之和。

expanded_nodes / remaining_budget 报告本次 CBS 预算消耗。

## 碰撞与运动语义

- 同一时刻两车不得占据同一格（顶点冲突）。
- 相邻两时刻不得迎面交换位置（边冲突）。
- 允许进入另一车同一时刻刚离开的格子（无追尾限制）。
- 只允许四邻格一步移动或等待，不得进入障碍或越界。
- 早到的车永久停在目标格；短路径结束后仍与他车未来占格做冲突检查，CBS 相应加入
  顶点/边约束并重算受影响车辆。

## 示例请求

协调成功（3x2 网格，两车对向而行，一车绕行下方避让格，最优总代价 6）：

```bash
curl -s -X POST localhost:8080/plan \
  -H 'Content-Type: application/json' \
  -d '{
    "width":3,"height":2,
    "obstacles":[],
    "vehicles":[
      {"id":"inspect-a","start":{"x":0,"y":0},"goal":{"x":2,"y":0}},
      {"id":"inspect-b","start":{"x":2,"y":0},"goal":{"x":0,"y":0}}
    ],
    "max_steps":12,"budget":2000
  }'
```

无解（1x3 走廊无避让位，两车无法互换顺序）：

```bash
curl -s -X POST localhost:8080/plan \
  -H 'Content-Type: application/json' \
  -d '{
    "width":3,"height":1,
    "vehicles":[
      {"id":"inspect-a","start":{"x":0,"y":0},"goal":{"x":2,"y":0}},
      {"id":"inspect-b","start":{"x":2,"y":0},"goal":{"x":0,"y":0}}
    ],
    "max_steps":20,"budget":20000
  }'
```

预算耗尽返回 undecided：把需要多代分裂的实例 budget 设为 1 即可观察。

## 关于点车辆近似

本模型将每辆巡检车视为占据单个网格点的质点：车体尺寸、转弯半径、最小安全车距、
加减速动力学均未建模；车辆朝向也不参与冲突判定。若实际车辆有占位面积，可在输入中
把车体膨胀为多个障碍/占用格后再接入本规划器。

## 代码结构

- internal/model：请求/响应模型与网格。
- internal/validate：地图与请求校验。
- internal/cbs：低层时空 A*（含约束寻路）与高层 CBS 冲突协调。
- internal/httpapi：chi 路由与 HTTP 适配。
- cmd/server：可配置端口的服务入口。

运行测试：

```bash
go test ./...
```
