"""
carla_runner.py - 把生成的路网与场景真正跑进 CARLA

补的是两块此前未闭环的能力：

  1. 路网落地：generate_rich_xodr 出来的 .xodr 以前只是文件，
     没有进 CARLA 的路径。CARLA 0.9.16 的 Client.generate_opendrive_world()
     可以直接吃 OpenDRIVE 字符串，这里封装成"生成/加载/布景/截图"一条链。

  2. 场景执行：仓库里没有 ScenarioRunner，.xosc 无法播放。这里做一个
     轻量执行器——把 ScenarioSpec 的实体按初始位置 spawn 进世界，
     把行为翻译成 CARLA 的 VehicleControl，跑固定帧数后截图。

注意：这不是完整的 OpenSCENARIO 播放器（不解析 .xosc、不支持 Story 分支、
不实现完整的 Trigger 语义），只覆盖本项目的模板能表达的那部分场景。
"""
from __future__ import annotations

import os
import queue
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import carla

from scenario_ir import EntityKind, PositionKind, ScenarioSpec


class CarlaRunnerError(RuntimeError):
    pass


# =====================================================================
# 连接
# =====================================================================
def connect(host: str = "127.0.0.1", port: int = 2000,
            timeout: float = 60.0, wait: float = 150.0,
            poll: float = 3.0) -> carla.Client:
    """连接 CARLA，带等待重试。

    刚切过世界的服务端会有几秒到几十秒不可用（要重建地图与导航网格），
    一次性 30 秒超时会偶发失败，所以这里轮询等待。
    """
    client = carla.Client(host, port)
    client.set_timeout(timeout)

    deadline = time.time() + wait
    last: Optional[Exception] = None
    while True:
        try:
            client.get_server_version()
            return client
        except Exception as e:
            last = e
            if time.time() >= deadline:
                break
            time.sleep(poll)

    raise CarlaRunnerError(
        "连不上 CARLA（%s:%d，等待 %.0f 秒）。请先启动 CarlaUE4.exe。"
        "原始错误：%s" % (host, port, wait, last))


# =====================================================================
# 路网加载
# =====================================================================
def load_xodr_world(client: carla.Client, xodr_path: str,
                    wall_height: float = 0.5,
                    additional_width: float = 0.5,
                    vertex_distance: float = 2.0,
                    max_road_length: float = 500.0,
                    pedestrian_nav: bool = False) -> carla.World:
    """把一个 .xodr 文件加载成 CARLA 世界。

    踩过的坑：PlanView 必须调用 adjust_roads_and_lanes() 才会有几何，
    否则写出的 <planView/> 是空的，CARLA 会一直等不到世界（表现为超时）。
    """
    if not os.path.exists(xodr_path):
        raise CarlaRunnerError("xodr 不存在: %s" % xodr_path)

    with open(xodr_path, "r", encoding="utf-8") as f:
        content = f.read()
    if "<planView/>" in content or "<planView />" in content:
        raise CarlaRunnerError(
            "xodr 的 planView 是空的，说明生成时漏了 adjust_roads_and_lanes()")

    params = carla.OpendriveGenerationParameters()
    params.vertex_distance = vertex_distance
    params.max_road_length = max_road_length
    params.wall_height = wall_height
    params.additional_width = additional_width
    params.enable_mesh_visibility = True
    params.enable_pedestrian_navigation = pedestrian_nav

    return client.generate_opendrive_world(content, params, True)


def load_builtin_world(client: carla.Client, map_name: str = "Town03") -> carla.World:
    return client.load_world(map_name)


# =====================================================================
# 布景
# =====================================================================
def clear_actors(world: carla.World) -> int:
    """清掉所有车辆、行人、传感器。"""
    n = 0
    for a in list(world.get_actors()):
        if a.type_id.startswith(("vehicle.", "walker.", "sensor.", "controller.")):
            try:
                a.destroy()
                n += 1
            except Exception:
                pass
    return n


def spawn_vehicles(world: carla.World, count: int = 5,
                   blueprint: str = "vehicle.tesla.model3",
                   autopilot: bool = False) -> List[carla.Actor]:
    """在生成点上路。"""
    bps = world.get_blueprint_library()
    bp = bps.find(blueprint)
    if bp is None:
        cands = list(bps.filter("vehicle.*"))
        if not cands:
            raise CarlaRunnerError("没有可用的车辆蓝图")
        bp = cands[0]
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "ego")

    spawn_points = world.get_map().get_spawn_points()
    out: List[carla.Actor] = []
    for i, sp in enumerate(spawn_points[:count]):
        try:
            v = world.try_spawn_actor(bp, sp)
            if v:
                v.set_autopilot(autopilot)
                out.append(v)
        except Exception:
            continue
    return out


def spawn_from_spec(world: carla.World, spec: ScenarioSpec,
                    blueprint: str = "vehicle.tesla.model3") -> Dict[str, Any]:
    """按 ScenarioSpec 的实体初始位置布景（轻量执行器）。

    LanePosition 转成最近的车道点；WorldPosition 直接用。
    返回 spawn 结果与失败原因。
    """
    bps = world.get_blueprint_library()
    vehicle_bp = bps.find(blueprint) or next(iter(bps.filter("vehicle.*")))
    walker_bp = bps.find("walker.pedestrian.0001")
    cmap = world.get_map()

    spawned: List[Tuple[str, carla.Actor]] = []
    failed: List[Dict[str, str]] = []

    for e in spec.entities:
        pos = e.initial_position
        try:
            if pos.kind == PositionKind.WORLD:
                tf = carla.Transform(
                    carla.Location(x=pos.x, y=pos.y, z=max(pos.z, 0.3) + 0.3),
                    carla.Rotation(yaw=pos.h))
            else:
                # LanePosition(s, offset, lane_id) -> 用 s 在路网上找最近点
                wps = cmap.generate_waypoints(5.0)
                if not wps:
                    failed.append({"id": e.id, "reason": "路网没有车道点"})
                    continue
                target = carla.Location(x=pos.s, y=pos.lane_offset, z=0.0)
                wp = min(wps, key=lambda w: w.transform.location.distance(target))
                tf = carla.Transform(
                    wp.transform.location + carla.Location(z=0.4),
                    wp.transform.rotation)

            bp = walker_bp if e.kind == EntityKind.PEDESTRIAN else vehicle_bp
            if bp is None:
                failed.append({"id": e.id, "reason": "找不到蓝图"})
                continue
            actor = world.try_spawn_actor(bp, tf)
            if actor is None:
                failed.append({"id": e.id, "reason": "生成点被占用"})
                continue
            if e.initial_speed > 0 and e.kind == EntityKind.VEHICLE:
                actor.set_target_velocity(
                    carla.Vector3D(x=e.initial_speed, y=0, z=0))
            spawned.append((e.id, actor))
        except Exception as ex:
            failed.append({"id": e.id, "reason": "%s: %s" % (type(ex).__name__, ex)})

    return {"spawned": spawned, "failed": failed}


# =====================================================================
# 截图
# =====================================================================
def _auto_views(world: carla.World) -> List[Dict[str, Any]]:
    """按地图实际范围自动构图。

    用 spawn_points[0] 当中心在长条路上会取到端点，画面大半是空地，
    所以改成用全部车道点的包围盒中心，并按范围决定相机高度。
    """
    cmap = world.get_map()
    pts = [w.transform.location for w in cmap.generate_waypoints(10.0)]
    if not pts:
        pts = [sp.location for sp in cmap.get_spawn_points()]
    if not pts:
        pts = [carla.Location(0, 0, 0)]

    xs = [p.x for p in pts]
    ys = [p.y for p in pts]
    w_span = max(xs) - min(xs)
    h_span = max(ys) - min(ys)

    # 长条路（高速公路匝道、直路）如果按整幅取景，7 米宽的路在画面里
    # 只有一条细线，看不清车道渐变。所以按长宽比判断：细长的地图只框
    # 中间一段，宽扁的地图（路口）框全幅。
    long_side = max(w_span, h_span, 20.0)
    short_side = max(min(w_span, h_span), 1.0)
    aspect = long_side / short_side
    if aspect > 5.0:
        # 细长路：视野覆盖约 90 m（足够看完整条渐变段），镜头放在前 1/4
        framed = max(short_side * 14.0, 90.0)
        height = framed * 0.55 + 10.0
        dist = framed * 0.5 + 16.0
    else:
        framed = long_side
        height = framed * 0.75 + 12.0
        dist = framed * 0.55 + 18.0

    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    cz = min(p.z for p in pts)

    if aspect > 5.0:
        # 车道渐变发生在路段前段，把镜头挪到前 1/4 处，而不是正中间
        if w_span >= h_span:
            cx = min(xs) + w_span * 0.25
        else:
            cy = min(ys) + h_span * 0.25

    return [
        {"name": "overhead",
         "location": (cx, cy, cz + height),
         "rotation": (-89.0, 0.0, 0.0)},
        {"name": "oblique",
         "location": (cx - dist * 0.6, cy - dist * 0.6, cz + dist * 0.5),
         "rotation": (-30.0, 45.0, 0.0)},
    ]


def capture_views(world: carla.World, out_dir: str,
                  views: Optional[Sequence[Dict[str, Any]]] = None,
                  width: int = 1280, height: int = 720,
                  warmup: int = 10) -> List[str]:
    """在世界里放相机取几帧图。

    views 每项形如 {"name": "overhead", "location": (x,y,z), "rotation": (p,y,r)}
    不传则自动按生成点布置一个俯视 + 一个斜视。
    """
    os.makedirs(out_dir, exist_ok=True)
    cams: List[Tuple[str, carla.Actor, Any]] = []
    saved: List[str] = []

    # 用同步模式保证能拿到帧
    settings = world.get_settings()
    old = (settings.synchronous_mode, settings.fixed_delta_seconds)
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    try:
        if not views:
            views = _auto_views(world)

        bps = world.get_blueprint_library()
        cam_bp = bps.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(width))
        cam_bp.set_attribute("image_size_y", str(height))
        cam_bp.set_attribute("fov", "90")

        for v in views:
            loc = carla.Location(*v["location"])
            rot = carla.Rotation(*v["rotation"])
            q: "queue.Queue" = queue.Queue()
            cam = world.spawn_actor(cam_bp, carla.Transform(loc, rot))
            cam.listen(q.put)
            cams.append((v["name"], cam, q))

        # 空转几帧让渲染稳定
        for _ in range(warmup):
            world.tick()

        for name, cam, q in cams:
            try:
                frame = q.get(timeout=20)
            except queue.Empty:
                continue
            p = os.path.join(out_dir, "%s.png" % name)
            frame.save_to_disk(p)
            for _ in range(3):
                world.tick()
            if os.path.exists(p):
                saved.append(p)
    finally:
        for name, cam, q in cams:
            try:
                cam.stop()
                cam.destroy()
            except Exception:
                pass
        settings = world.get_settings()
        settings.synchronous_mode = old[0]
        settings.fixed_delta_seconds = old[1]
        world.apply_settings(settings)

    return saved


# =====================================================================
# 一条链：布局 -> 世界 -> 布景 -> 截图
# =====================================================================
def run_layout_demo(client: carla.Client, layout_name: str, xodr_path: str,
                    out_dir: str, vehicle_count: int = 6,
                    width: int = 1280, height: int = 720) -> Dict[str, Any]:
    t0 = time.time()
    world = load_xodr_world(client, xodr_path)
    t_load = time.time() - t0

    n_clear = clear_actors(world)
    vehicles = spawn_vehicles(world, vehicle_count)
    images = capture_views(world, out_dir, width=width, height=height)

    m = world.get_map()
    return {
        "layout": layout_name,
        "map": m.name,
        "spawn_points": len(m.get_spawn_points()),
        "vehicles": len(vehicles),
        "cleared": n_clear,
        "images": images,
        "load_seconds": round(t_load, 2),
        "total_seconds": round(time.time() - t0, 2),
    }
