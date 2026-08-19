"""Runner headless reproducible del mismo controller autónomo puro."""
from __future__ import annotations
import argparse, csv, json, math, random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

try:
    from autonomous_controller import AutonomousController
    from geometric_planner import (GeometricPlanner, PlannerInput, PlannerState, TrackDirection,
        VehicleGeometry, VehicleState, VisibleObstacle, VisibleWall, rectangle_polygon,
        timing_percentiles, vehicle_step)
    from planner_rules import FIXED_RULES
    from planner_tuning import PlannerTuning, load_planner_tuning
    from scenario import ScenarioObject, generate_objects
    from track_config import INNER_WALL, OUTER_WALL, START_POSE, route_centerline, start_zone_contains, straight_sequence
except ImportError:
    from simulator.autonomous_controller import AutonomousController
    from simulator.geometric_planner import (GeometricPlanner, PlannerInput, PlannerState, TrackDirection,
        VehicleGeometry, VehicleState, VisibleObstacle, VisibleWall, rectangle_polygon,
        timing_percentiles, vehicle_step)
    from simulator.planner_rules import FIXED_RULES
    from simulator.planner_tuning import PlannerTuning, load_planner_tuning
    from simulator.scenario import ScenarioObject, generate_objects
    from simulator.track_config import INNER_WALL, OUTER_WALL, START_POSE, route_centerline, start_zone_contains, straight_sequence

PlannerConfig = PlannerTuning


@dataclass
class SensorModel:
    noise_position_cm: float=0.0
    noise_heading_deg: float=0.0
    latency_s: float=0.0
    dropout_probability: float=0.0


def _walls()->tuple[VisibleWall,...]:
    result=[]
    for prefix,rect in (("outer",OUTER_WALL),("inner",INNER_WALL)):
        polygon=rectangle_polygon(rect)
        result += [VisibleWall(f"{prefix}-{i}",a,b) for i,(a,b) in enumerate(zip(polygon,polygon[1:]+polygon[:1]))]
    return tuple(result)


def _visible(state:VehicleState,objects:list[ScenarioObject],sensor:SensorModel,rng:random.Random)->tuple[VisibleObstacle,...]:
    heading=state.heading_rad+math.radians(rng.gauss(0,sensor.noise_heading_deg)); result=[]
    for item in objects:
        dx,dy=item.x_cm-state.x_cm,item.y_cm-state.y_cm
        bearing=(math.atan2(dy,dx)-heading+math.pi)%(2*math.pi)-math.pi
        if math.hypot(dx,dy)>115 or abs(bearing)>math.radians(FIXED_RULES.horizontal_fov_deg/2) or rng.random()<sensor.dropout_probability:continue
        result.append(VisibleObstacle(item.object_id,item.x_cm+rng.gauss(0,sensor.noise_position_cm),
            item.y_cm+rng.gauss(0,sensor.noise_position_cm),item.width_cm,item.length_cm,item.color))
    return tuple(result)


def _visible_walls(state:VehicleState,walls:tuple[VisibleWall,...])->tuple[VisibleWall,...]:
    def seen(point):
        dx,dy=point[0]-state.x_cm,point[1]-state.y_cm
        bearing=(math.atan2(dy,dx)-state.heading_rad+math.pi)%(2*math.pi)-math.pi
        return math.hypot(dx,dy)<=120 and abs(bearing)<=math.radians(FIXED_RULES.horizontal_fov_deg/2)
    return tuple(w for w in walls if seen(w.start) or seen(w.end)
                 or seen(((w.start[0]+w.end[0])/2,(w.start[1]+w.end[1])/2)))


def _heading(state:VehicleState,route:tuple[tuple[float,float],...])->float:
    index=min(range(len(route)),key=lambda i:math.dist((state.x_cm,state.y_cm),route[i]))
    target_index=index;traveled=0.0
    lookahead_cm=max(FIXED_RULES.turn_radius_left_cm,FIXED_RULES.turn_radius_right_cm)
    while target_index+1<len(route) and traveled<lookahead_cm:
        traveled+=math.dist(route[target_index],route[target_index+1]);target_index+=1
    target=route[target_index]
    return math.atan2(target[1]-state.y_cm,target[0]-state.x_cm)


def _sector(state:VehicleState)->str|None:
    if 80<=state.x_cm<=220 and state.y_cm<90:return "top"
    if state.x_cm>210 and 80<=state.y_cm<=220:return "right"
    if 80<=state.x_cm<=220 and state.y_cm>210:return "bottom"
    if state.x_cm<90 and 80<=state.y_cm<=220:return "left"
    return None


def jsonable(value:Any)->Any:
    if hasattr(value,"value"):return value.value
    if hasattr(value,"__dataclass_fields__"):return {k:jsonable(v) for k,v in asdict(value).items()}
    if isinstance(value,dict):return {str(k):jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [jsonable(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def run_scenario(seed:int,scenario_index:int,sensor:SensorModel,duration_s:float,
                 fixed_speed_cm_s:float|None=None,planner_config:PlannerConfig|None=None):
    rng=random.Random(seed); objects=generate_objects(rng,scenario_index); tuning=planner_config or load_planner_tuning()
    if fixed_speed_cm_s is not None:tuning=tuning.with_overrides(fixed_speed_cm_s=fixed_speed_cm_s)
    geometry=VehicleGeometry(fixed_speed_cm_s=tuning.fixed_speed_cm_s,max_steering_rate_deg_s=tuning.max_steering_rate_deg_s,
        max_acceleration_cm_s2=tuning.max_acceleration_cm_s2,max_deceleration_cm_s2=tuning.max_deceleration_cm_s2)
    planner=GeometricPlanner(geometry,tuning); controller=AutonomousController(planner)
    state=VehicleState(*START_POSE); route=route_centerline(True); walls=_walls(); boundary=rectangle_polygon(OUTER_WALL)
    command=planner.plan(PlannerInput(state,drivable_boundary=boundary)).command
    rows=[];times=[];now=next_replan=0.0;collision=completed=passed=False;correct_side=True;no_safe=0
    order=straight_sequence(True);progress=0;last=None;route_valid=True
    while now<duration_s-1e-9:
        visible=_visible(state,objects,sensor,rng)
        data=PlannerInput(state,visible,_visible_walls(state,walls),boundary,TrackDirection.CLOCKWISE,_heading(state,route),now)
        if now>=next_replan-1e-9:
            result=controller.plan(data);command=result.command;next_replan=now+tuning.replanning_period_s;times.append(result.diagnostics.calculation_time_ms)
            no_safe += result.state is PlannerState.NO_SAFE_TRAJECTORY
            if result.best_candidate:correct_side &= result.best_candidate.correct_pass_side
            rows.append({"seed":seed,"scenario":scenario_index,"time_s":round(now,4),"x_cm":round(state.x_cm,4),"y_cm":round(state.y_cm,4),
                "heading_deg":round(math.degrees(state.heading_rad),4),"speed_cm_s":round(state.speed_cm_s,4),"planner_state":result.state.value,
                "selected_candidate":result.diagnostics.selected_candidate_id or "","selected_steering_angle_deg":round(command.steering_angle_deg,4),
                "selected_speed_cm_s":round(command.target_speed_cm_s,4),"minimum_clearance_cm":None if not math.isfinite(result.diagnostics.minimum_clearance_cm) else round(result.diagnostics.minimum_clearance_cm,4),
                "minimum_obstacle_clearance_cm":None if not math.isfinite(result.diagnostics.minimum_obstacle_clearance_cm) else round(result.diagnostics.minimum_obstacle_clearance_cm,4),
                "planning_time_ms":round(result.diagnostics.calculation_time_ms,4),"candidates_evaluated":result.diagnostics.candidates_evaluated,
                "no_safe_trajectory":result.state is PlannerState.NO_SAFE_TRAJECTORY,"planner_reason":result.reason})
        state=vehicle_step(state,command,tuning.simulation_dt_s,geometry)
        collision |= planner.collision_metrics(state,data)[0]
        passed |= any((state.x_cm-o.x_cm)*math.cos(state.heading_rad)+(state.y_cm-o.y_cm)*math.sin(state.heading_rad)>geometry.length_cm/2 for o in objects)
        sector=_sector(state)
        if sector and sector!=last:
            expected=order[min(progress,len(order)-1)]
            if sector==expected:progress+=1
            elif progress>0:route_valid=False
            last=sector
        completed=progress>=len(order) and now>20 and start_zone_contains(state.x_cm,state.y_cm)
        if collision or completed:break
        now+=tuning.simulation_dt_s
    summary={"seed":seed,"scenario":scenario_index,"objects":[jsonable(o) for o in objects],"collision":collision,
        "selected_angle_deg":rows[-1]["selected_steering_angle_deg"] if rows else 0,"minimum_distance_cm":min((r["minimum_clearance_cm"] for r in rows if r["minimum_clearance_cm"] is not None),default=None),
        "maneuver_completed":passed,"passed":passed,"straight_progress":progress,"next_straight_reached":progress>=2,"lap_completed":completed,
        "route_progress_valid":route_valid,"correct_pass_side":correct_side,"no_safe_trajectory_cycles":no_safe,"planning_cycles":len(rows),
        "timing":timing_percentiles(times),"sensor":jsonable(sensor),"cycles":rows}
    return summary,rows


def write_outputs(summaries,rows,output_dir:Path)->None:
    output_dir.mkdir(parents=True,exist_ok=True)
    (output_dir/"planner_results.json").write_text(json.dumps(jsonable(summaries),indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    with (output_dir/"planner_cycles.csv").open("w",newline="",encoding="utf-8") as handle:
        columns=sorted({key for row in rows for key in row});writer=csv.DictWriter(handle,fieldnames=columns);writer.writeheader();writer.writerows(rows)
    aggregate={"scenarios":len(summaries),"collisions":sum(bool(s["collision"]) for s in summaries),
        "maneuvers_completed":sum(bool(s["maneuver_completed"]) for s in summaries),"laps_completed":sum(bool(s["lap_completed"]) for s in summaries),
        "scenarios_reaching_next_straight":sum(bool(s["next_straight_reached"]) for s in summaries),"route_progress_valid":sum(bool(s["route_progress_valid"]) for s in summaries),
        "correct_pass_side":sum(bool(s["correct_pass_side"]) for s in summaries),"timing":timing_percentiles([r["planning_time_ms"] for r in rows])}
    (output_dir/"planner_summary.json").write_text(json.dumps(aggregate,indent=2)+"\n",encoding="utf-8")


def main()->None:
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--scenarios",type=int,default=20);parser.add_argument("--seed",type=int,default=20260815)
    parser.add_argument("--duration-s",type=float,default=20);parser.add_argument("--output-dir",type=Path,default=Path("/tmp/wro_planner_results"))
    parser.add_argument("--noise-position-cm",type=float,default=0);parser.add_argument("--noise-heading-deg",type=float,default=0);parser.add_argument("--latency-s",type=float,default=0)
    parser.add_argument("--dropout-probability",type=float,default=0);parser.add_argument("--fixed-speed-cm-s",type=float,default=None);parser.add_argument("--planner-config",type=Path,default=None)
    args=parser.parse_args();tuning=load_planner_tuning(args.planner_config)
    if args.fixed_speed_cm_s is not None:tuning=tuning.with_overrides(fixed_speed_cm_s=args.fixed_speed_cm_s)
    sensor=SensorModel(args.noise_position_cm,args.noise_heading_deg,args.latency_s,args.dropout_probability);summaries=[];rows=[]
    for index in range(args.scenarios):
        print(f"[{index+1}/{args.scenarios}] escenario {index}",flush=True);summary,cycle_rows=run_scenario(args.seed+index,index,sensor,args.duration_s,tuning.fixed_speed_cm_s,tuning);summaries.append(summary);rows+=cycle_rows
    write_outputs(summaries,rows,args.output_dir);print(json.dumps(json.loads((args.output_dir/"planner_summary.json").read_text()),indent=2))


if __name__=="__main__":main()
