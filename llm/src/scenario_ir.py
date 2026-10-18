"""
scenario_ir.py - 场景中间表示（Intermediate Representation）

解决的问题
----------
改造前，17 个 scenario_* 函数把场景逻辑硬编码在 Python 代码里：
场景只存在于运行中的 CARLA 世界，关掉即失；参数写死无法采样；无 seed 不可复现；
也无法导出成 OpenSCENARIO 文件。

本模块把"场景"从代码提升为**数据**，于是可以：
    生成 → 参数化采样 → 批量导出 → 校验 → 回灌 CARLA 执行

坐标与单位约定（原项目缺失，此处显式声明）
------------------------------------------
- 长度/距离：米 (m)
- 速度：米/秒 (m/s)
- 加速度：米/秒² (m/s²)
- 时间：秒 (s)
- 角度：度 (°)
- LanePosition.s 为沿车道的纵向弧长，lane_id/road_id 按 OpenDRIVE 约定，
  **以字符串存储**（右车道为负，如 "-1"）
"""
from __future__ import annotations

import enum
import hashlib
import json
from typing import Any, Dict, List, Optional, Union

from pydantic import BaseModel, Field, field_validator, model_validator


# =====================================================================
# 枚举
# =====================================================================
class EntityKind(str, enum.Enum):
    VEHICLE = "vehicle"
    PEDESTRIAN = "pedestrian"
    MISC_OBJECT = "misc_object"


class VehicleCategory(str, enum.Enum):
    CAR = "car"
    VAN = "van"
    TRUCK = "truck"
    BUS = "bus"
    BICYCLE = "bicycle"
    MOTORBIKE = "motorbike"
    TRAILER = "trailer"
    SEMITRAILER = "semitrailer"
    TRAIN = "train"
    TRAM = "tram"


class PedestrianCategory(str, enum.Enum):
    PEDESTRIAN = "pedestrian"
    WHEELCHAIR = "wheelchair"
    ANIMAL = "animal"


class PositionKind(str, enum.Enum):
    LANE = "lane"          # LanePosition：最精确，绑定路网
    WORLD = "world"        # WorldPosition：绝对坐标
    RELATIVE_LANE = "relative_lane"    # 相对某实体的车道位置
    RELATIVE_OBJECT = "relative_object"


class DynamicsShape(str, enum.Enum):
    STEP = "step"
    LINEAR = "linear"
    CUBIC = "cubic"
    SINUSOIDAL = "sinusoidal"


class DynamicsDimension(str, enum.Enum):
    TIME = "time"
    DISTANCE = "distance"
    RATE = "rate"


class BehaviorKind(str, enum.Enum):
    """行为类型。覆盖目标①要求的 ≥4 类。"""
    SPEED = "speed"                        # 绝对/相对速度
    LANE_CHANGE = "lane_change"            # 变道
    LANE_OFFSET = "lane_offset"            # 车道内横向偏移
    FOLLOW_TRAJECTORY = "follow_trajectory"  # 沿轨迹行驶（目标②用）
    LONGITUDINAL_DISTANCE = "longitudinal_distance"  # 纵向跟驰
    LATERAL_DISTANCE = "lateral_distance"
    TELEPORT = "teleport"


class TriggerKind(str, enum.Enum):
    """触发条件类型。覆盖目标①要求的 ≥5 种。"""
    TIME = "time"                          # SimulationTimeCondition
    DISTANCE = "distance"                  # DistanceCondition
    TIME_HEADWAY = "time_headway"          # TimeHeadwayCondition
    REACH_POSITION = "reach_position"      # ReachPositionCondition
    SPEED = "speed"                        # SpeedCondition
    RELATIVE_SPEED = "relative_speed"      # RelativeSpeedCondition
    COLLISION = "collision"                # CollisionCondition
    STANDSTILL = "standstill"              # StandStillCondition
    TIME_TO_COLLISION = "time_to_collision"  # TimeToCollisionCondition
    STORYBOARD_STATE = "storyboard_state"  # StoryboardElementStateCondition


class Rule(str, enum.Enum):
    EQUAL_TO = "equalTo"
    GREATER_THAN = "greaterThan"
    LESS_THAN = "lessThan"
    GREATER_OR_EQUAL = "greaterOrEqual"
    LESS_OR_EQUAL = "lessOrEqual"
    NOT_EQUAL_TO = "notEqualTo"


class ConditionEdge(str, enum.Enum):
    NONE = "none"
    RISING = "rising"
    FALLING = "falling"
    RISING_OR_FALLING = "risingOrFalling"


class Priority(str, enum.Enum):
    OVERRIDE = "override"     # 注意：overwrite 在 OSC 1.1 起已废弃
    OVERWRITE = "overwrite"
    PARALLEL = "parallel"
    SKIP = "skip"


# =====================================================================
# 参数空间（用于采样）
# =====================================================================
class ParamRange(BaseModel):
    """一个可采样参数的取值范围。

    kind:
      - "uniform" 连续均匀
      - "int"     整数均匀
      - "choice"  从 choices 中离散选取
      - "fixed"   固定值（不参与采样）
    """
    kind: str = "uniform"
    low: Optional[float] = None
    high: Optional[float] = None
    choices: Optional[List[Any]] = None
    value: Optional[Any] = None
    unit: str = ""

    @model_validator(mode="after")
    def _check(self):
        if self.kind in ("uniform", "int"):
            if self.low is None or self.high is None:
                raise ValueError("uniform/int 需要 low 与 high")
            if self.low > self.high:
                raise ValueError("low 不能大于 high")
        elif self.kind == "choice":
            if not self.choices:
                raise ValueError("choice 需要非空 choices")
        elif self.kind == "fixed":
            if self.value is None:
                raise ValueError("fixed 需要 value")
        else:
            raise ValueError("未知 kind: %s" % self.kind)
        return self

    def sample(self, rng) -> Any:
        """用给定 random.Random 采样出一个值。"""
        if self.kind == "uniform":
            return rng.uniform(self.low, self.high)
        if self.kind == "int":
            return rng.randint(int(self.low), int(self.high))
        if self.kind == "choice":
            return rng.choice(self.choices)
        return self.value


# =====================================================================
# 位置
# =====================================================================
class PositionSpec(BaseModel):
    """实体位置。优先用 lane（绑定路网），退化时用 world。"""
    kind: PositionKind = PositionKind.LANE

    # --- lane ---
    s: float = 0.0
    lane_offset: float = 0.0
    lane_id: str = "-1"
    road_id: str = "1"

    # --- world ---
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    h: float = 0.0

    # --- relative ---
    entity_ref: Optional[str] = None
    ds: float = 0.0
    d_lane: int = 0
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0

    @classmethod
    def lane(cls, s: float, lane_id: str = "-1", road_id: str = "1",
             offset: float = 0.0) -> "PositionSpec":
        return cls(kind=PositionKind.LANE, s=s, lane_id=str(lane_id),
                   road_id=str(road_id), lane_offset=offset)

    @classmethod
    def world(cls, x: float, y: float, z: float = 0.0,
              h: float = 0.0) -> "PositionSpec":
        return cls(kind=PositionKind.WORLD, x=x, y=y, z=z, h=h)

    @classmethod
    def relative_lane(cls, entity_ref: str, ds: float, d_lane: int = 0) -> "PositionSpec":
        return cls(kind=PositionKind.RELATIVE_LANE, entity_ref=entity_ref,
                   ds=ds, d_lane=d_lane)


# =====================================================================
# 实体
# =====================================================================
class BoundingBoxSpec(BaseModel):
    """OSC BoundingBox: length, width, height, x, y, z（中心相对坐标）"""
    length: float = 5.0
    width: float = 2.0
    height: float = 1.5
    x: float = 1.4
    y: float = 0.0
    z: float = 0.9


class EntitySpec(BaseModel):
    """一个场景实体。"""
    id: str = Field(..., description="唯一标识，如 ego / lead / ped")
    kind: EntityKind = EntityKind.VEHICLE
    vehicle_category: VehicleCategory = VehicleCategory.CAR
    pedestrian_category: PedestrianCategory = PedestrianCategory.PEDESTRIAN
    bounding_box: BoundingBoxSpec = Field(default_factory=BoundingBoxSpec)

    # 动力学上限
    max_speed: float = 69.4
    max_acceleration: float = 10.0
    max_deceleration: float = 10.0
    mass: float = 1500.0

    # 初始状态
    initial_position: PositionSpec = Field(
        default_factory=lambda: PositionSpec.lane(0.0))
    initial_speed: float = 0.0

    # CARLA 侧蓝图（供回灌 CARLA 用，OSC 不消费）
    carla_blueprint: Optional[str] = None

    # OSC 1.0 要求行人提供 model
    model3d: Optional[str] = None

    @field_validator("id")
    @classmethod
    def _id_ok(cls, v: str) -> str:
        if not v or not v.replace("_", "").replace("-", "").isalnum():
            raise ValueError("id 只能是字母/数字/下划线/连字符: %r" % v)
        return v


# =====================================================================
# 行为
# =====================================================================
class BehaviorSpec(BaseModel):
    """绑定到某实体的一个行为。"""
    kind: BehaviorKind
    entity_ref: str

    # speed
    target_speed: Optional[float] = None
    relative_speed: bool = False
    speed_entity_ref: Optional[str] = None   # 相对速度的参照实体

    # lane_change / lane_offset
    lane_change: Optional[int] = None        # +1 左移, -1 右移
    lane_offset_target: Optional[float] = None

    # longitudinal / lateral
    distance: Optional[float] = None
    distance_entity_ref: Optional[str] = None

    # follow_trajectory（目标②用）
    trajectory_file: Optional[str] = None
    # 直接给点：[[t, x, y], ...]，世界坐标（米）、时间（秒）。
    # 与 trajectory_file 二选一；给了这个就不读文件。
    trajectory_points: Optional[List[List[float]]] = None

    # 通用动力学
    dynamics_shape: DynamicsShape = DynamicsShape.LINEAR
    dynamics_dimension: DynamicsDimension = DynamicsDimension.TIME
    dynamics_value: float = 2.0

    name: Optional[str] = None

    @model_validator(mode="after")
    def _check(self):
        if self.kind == BehaviorKind.SPEED and self.target_speed is None:
            raise ValueError("speed 行为需要 target_speed")
        if self.kind == BehaviorKind.LANE_CHANGE:
            if self.lane_change is None:
                raise ValueError("lane_change 行为需要 lane_change 值")
            if self.lane_change == 0:
                raise ValueError("lane_change 不能为 0")
        if self.kind == BehaviorKind.FOLLOW_TRAJECTORY:
            if not self.trajectory_file and not self.trajectory_points:
                raise ValueError(
                    "follow_trajectory 需要 trajectory_file 或 trajectory_points")
            if self.trajectory_points is not None and len(self.trajectory_points) < 2:
                raise ValueError("trajectory_points 至少需要 2 个点")
            if self.trajectory_file and self.trajectory_points:
                raise ValueError(
                    "trajectory_file 与 trajectory_points 只能给一个")
        return self

    def resolved_name(self) -> str:
        return self.name or ("%s_%s" % (self.entity_ref, self.kind.value))


# =====================================================================
# 触发
# =====================================================================
class TriggerSpec(BaseModel):
    """一个触发条件。"""
    kind: TriggerKind
    rule: Rule = Rule.GREATER_THAN
    edge: ConditionEdge = ConditionEdge.RISING
    value: float = 0.0
    delay: float = 0.0

    # 实体类条件的参照
    entity_ref: Optional[str] = None
    # reach_position 用
    position: Optional[PositionSpec] = None
    tolerance: float = 1.0
    # 触发主体（EntityTrigger 需要）
    triggering_entity: Optional[str] = None

    name: Optional[str] = None

    @model_validator(mode="after")
    def _check(self):
        entity_kinds = {
            TriggerKind.TIME_HEADWAY, TriggerKind.DISTANCE, TriggerKind.RELATIVE_SPEED,
            TriggerKind.COLLISION, TriggerKind.TIME_TO_COLLISION, TriggerKind.STANDSTILL,
        }
        if self.kind in entity_kinds and not self.triggering_entity:
            # 允许缺省，导出时用第一个实体兜底；但记录需求
            pass
        if self.kind == TriggerKind.REACH_POSITION and self.position is None:
            raise ValueError("reach_position 触发需要 position")
        return self

    def resolved_name(self) -> str:
        return self.name or ("trig_%s_%g" % (self.kind.value, self.value))


# =====================================================================
# 场景
# =====================================================================
class ScenarioSpec(BaseModel):
    """一个完整场景的描述（与引擎无关）。"""
    name: str = "scenario"
    description: str = ""
    author: str = "DSH-MCP"

    # 路网
    xodr_file: Optional[str] = None
    map_name: Optional[str] = None      # CARLA 内置地图名

    # OSC 版本：0=1.0, 1=1.1, 2=1.2, 3=1.3
    osc_minor_version: int = 1

    entities: List[EntitySpec] = Field(default_factory=list)
    behaviors: List[BehaviorSpec] = Field(default_factory=list)
    triggers: List[TriggerSpec] = Field(default_factory=list)

    duration: float = 30.0
    priority: Priority = Priority.OVERRIDE

    # 采样与复现
    seed: Optional[int] = None
    template: Optional[str] = None
    params: Dict[str, Any] = Field(default_factory=dict)
    tags: List[str] = Field(default_factory=list)

    @model_validator(mode="after")
    def _check(self):
        if not self.entities:
            raise ValueError("场景至少需要 1 个实体")
        ids = [e.id for e in self.entities]
        dup = {i for i in ids if ids.count(i) > 1}
        if dup:
            raise ValueError("实体 id 重复: %s" % sorted(dup))
        id_set = set(ids)
        for b in self.behaviors:
            if b.entity_ref not in id_set:
                raise ValueError("行为引用了不存在的实体: %s" % b.entity_ref)
        for t in self.triggers:
            for ref in (t.entity_ref, t.triggering_entity):
                if ref and ref not in id_set:
                    raise ValueError("触发引用了不存在的实体: %s" % ref)
        if self.osc_minor_version not in (0, 1, 2, 3):
            raise ValueError("osc_minor_version 只能是 0/1/2/3")
        return self

    # ------------------------------------------------------------ 工具
    def ego(self) -> Optional[EntitySpec]:
        for e in self.entities:
            if e.id == "ego":
                return e
        return self.entities[0] if self.entities else None

    def entity(self, eid: str) -> Optional[EntitySpec]:
        for e in self.entities:
            if e.id == eid:
                return e
        return None

    def behaviors_of(self, eid: str) -> List[BehaviorSpec]:
        return [b for b in self.behaviors if b.entity_ref == eid]

    # ------------------------------------------------------ 序列化
    def to_dict(self) -> Dict[str, Any]:
        return self.model_dump(mode="json")

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ScenarioSpec":
        return cls.model_validate(d)

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=indent)

    @classmethod
    def from_json(cls, s: str) -> "ScenarioSpec":
        return cls.model_validate(json.loads(s))

    # ------------------------------------------------------ 指纹
    def fingerprint(self) -> str:
        """稳定指纹：忽略 name/seed/params，只反映场景结构。

        用于去重与回归对比 —— 相同结构必须得到相同指纹。
        """
        d = self.to_dict()
        for k in ("name", "description", "seed", "params", "author"):
            d.pop(k, None)
        blob = json.dumps(d, sort_keys=True, ensure_ascii=False)
        return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]

    # ------------------------------------------------------ 统计
    def stats(self) -> Dict[str, Any]:
        kinds = {}
        for b in self.behaviors:
            kinds[b.kind.value] = kinds.get(b.kind.value, 0) + 1
        tkinds = {}
        for t in self.triggers:
            tkinds[t.kind.value] = tkinds.get(t.kind.value, 0) + 1
        return {
            "entities": len(self.entities),
            "vehicles": sum(1 for e in self.entities if e.kind == EntityKind.VEHICLE),
            "pedestrians": sum(1 for e in self.entities
                               if e.kind == EntityKind.PEDESTRIAN),
            "behaviors": len(self.behaviors),
            "behavior_kinds": kinds,
            "triggers": len(self.triggers),
            "trigger_kinds": tkinds,
            "interactive": len(self.entities) > 1,
        }


# =====================================================================
# 模板定义
# =====================================================================
class TemplateSpec(BaseModel):
    """一个参数化场景模板。

    builder(params) -> ScenarioSpec 由 templates 模块提供；
    这里只描述"有哪些参数、默认值、可采样范围"。
    """
    name: str
    title: str
    description: str = ""
    tags: List[str] = Field(default_factory=list)
    params: Dict[str, ParamRange] = Field(default_factory=dict)
    # 该模板对应的 CARLA 侧原生函数名（复用已有资产）
    carla_function: Optional[str] = None

    def defaults(self) -> Dict[str, Any]:
        out = {}
        for k, r in self.params.items():
            if r.kind == "fixed":
                out[k] = r.value
            elif r.kind in ("uniform", "int"):
                out[k] = r.low
            elif r.kind == "choice":
                out[k] = r.choices[0]
        return out

    def sample_params(self, rng) -> Dict[str, Any]:
        return {k: r.sample(rng) for k, r in self.params.items()}
