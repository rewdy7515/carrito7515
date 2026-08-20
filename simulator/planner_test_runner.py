"""Runner headless reproducible del mismo controller autónomo puro."""
from __future__ import annotations
import argparse, json, math, random
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
    from scenario import ScenarioObject, generate_scenario, seat_slot
    from track_config import INNER_WALL, OUTER_WALL, START_POSE, route_centerline, start_zone_contains, straight_sequence
except ImportError:
    from simulator.autonomous_controller import AutonomousController
    from simulator.geometric_planner import (GeometricPlanner, PlannerInput, PlannerState, TrackDirection,
        VehicleGeometry, VehicleState, VisibleObstacle, VisibleWall, rectangle_polygon,
        timing_percentiles, vehicle_step)
    from simulator.planner_rules import FIXED_RULES
    from simulator.planner_tuning import PlannerTuning, load_planner_tuning
    from simulator.scenario import ScenarioObject, generate_scenario, seat_slot
    from simulator.track_config import INNER_WALL, OUTER_WALL, START_POSE, route_centerline, start_zone_contains, straight_sequence

PlannerConfig = PlannerTuning
TARGET_LAPS = 3
STRAIGHTS_PER_LAP = 4
TARGET_STRAIGHTS = TARGET_LAPS * STRAIGHTS_PER_LAP


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


def _collision_type(obstacle_clearance: float, wall_clearance: float) -> str | None:
    obstacle = obstacle_clearance <= 0.0
    wall = wall_clearance <= 0.0
    if obstacle and wall:
        return "wall_and_obstacle"
    if wall:
        return "wall"
    if obstacle:
        return "obstacle"
    return None


def jsonable(value:Any)->Any:
    if hasattr(value,"value"):return value.value
    if hasattr(value,"__dataclass_fields__"):return {k:jsonable(v) for k,v in asdict(value).items()}
    if isinstance(value,dict):return {str(k):jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [jsonable(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def run_scenario(seed:int,scenario_index:int,sensor:SensorModel,duration_s:float,
                 planner_config:PlannerConfig|None=None,
                 planning_budget_mode:str="candidate_count", mode:str="2",
                 base_seed:int|None=None, disable_safety_margins:bool=False):
    if mode not in {"1", "2"}:
        raise ValueError("mode 3 aún no está configurado; usa --mode 1 o --mode 2.")
    effective_seed=seed
    base_seed=effective_seed if base_seed is None else base_seed
    rng=random.Random(effective_seed)
    generated = None if mode == "1" else generate_scenario(rng, (scenario_index % 4) + 1)
    objects = [] if generated is None else list(generated.objects)
    tuning=planner_config or load_planner_tuning()
    if disable_safety_margins:
        tuning=tuning.with_overrides(
            disable_hard_safety_margins=True,
            allow_physical_collisions=True,
        )
    tuning=tuning.with_overrides(planning_budget_mode=planning_budget_mode)
    geometry=VehicleGeometry()
    planner=GeometricPlanner(geometry,tuning); controller=AutonomousController(planner)
    state=VehicleState(*START_POSE); route=route_centerline(True); walls=_walls(); boundary=rectangle_polygon(OUTER_WALL)
    command=planner.plan(PlannerInput(state,drivable_boundary=boundary)).command
    rows=[];times=[];now=next_replan=0.0;collision=completed=passed=False;correct_side=True;no_safe=0
    collision_type=None; seen_ids:set[str]=set(); crossed_ids:set[str]=set(); passed_ids:set[str]=set()
    observations={item.object_id:{"ever_detected":False,"first_detection_step":None,
                                  "first_detection_distance_cm":None,
                                  "first_detection_relative_x_cm":None,
                                  "first_detection_relative_y_cm":None}
                 for item in objects}
    distance_cm=0.0; reverse_distance_cm=0.0; step=0; stationary_time=0.0; stuck=False; failure_step=None
    reverse_recovery_attempted=False; replan_after_reverse=False; recovery_success=False
    first_failure_step=None; first_failure_time_s=None; nearest_failure_obstacle=None
    reverse_active=False; last_diagnostics=None
    order=straight_sequence(True)[:STRAIGHTS_PER_LAP]
    progress=0;last=None;route_valid=True
    while now<duration_s-1e-9:
        visible=_visible(state,objects,sensor,rng)
        seen_ids.update(item.object_id for item in visible)
        for item in visible:
            observation=observations[item.object_id]
            if not observation["ever_detected"]:
                dx=item.x_cm-state.x_cm; dy=item.y_cm-state.y_cm
                observation.update({
                    "ever_detected":True,"first_detection_step":step,
                    "first_detection_distance_cm":round(math.hypot(dx,dy),4),
                    "first_detection_relative_x_cm":round(dx*math.cos(state.heading_rad)+dy*math.sin(state.heading_rad),4),
                    "first_detection_relative_y_cm":round(-dx*math.sin(state.heading_rad)+dy*math.cos(state.heading_rad),4),
                })
        data=PlannerInput(state,visible,_visible_walls(state,walls),boundary,TrackDirection.CLOCKWISE,_heading(state,route),now,route)
        if now>=next_replan-1e-9:
            result=controller.plan(data);command=result.command
            next_replan = now + controller.execution_interval_s(
                state, command.target_speed_cm_s,
            )
            last_diagnostics=result.diagnostics
            reverse_recovery_attempted |= result.diagnostics.reverse_recovery_attempted
            if reverse_active and command.target_speed_cm_s >= 0.0:
                replan_after_reverse=True
                recovery_success |= result.best_candidate is not None and result.state is not PlannerState.NO_SAFE_TRAJECTORY
            reverse_active = command.target_speed_cm_s < 0.0
            planning_time_ms=(
                0.0
                if planning_budget_mode == "candidate_count"
                else result.diagnostics.calculation_time_ms
            )
            times.append(planning_time_ms)
            no_safe += result.state is PlannerState.NO_SAFE_TRAJECTORY
            if result.best_candidate:correct_side &= result.best_candidate.correct_pass_side
            rows.append({"seed":effective_seed,"base_seed":base_seed,"effective_seed":effective_seed,"scenario":scenario_index,"mode":mode,"step":step,"time_s":round(now,4),"x_cm":round(state.x_cm,4),"y_cm":round(state.y_cm,4),
                "heading_deg":round(math.degrees(state.heading_rad),4),"speed_cm_s":round(state.speed_cm_s,4),"planner_state":result.state.value,
                "selected_candidate":result.diagnostics.selected_candidate_id or "","selected_steering_angle_deg":round(command.steering_angle_deg,4),
                "selected_speed_cm_s":round(command.target_speed_cm_s,4),"minimum_clearance_cm":None if not math.isfinite(result.diagnostics.minimum_clearance_cm) else round(result.diagnostics.minimum_clearance_cm,4),
                "minimum_obstacle_clearance_cm":None if not math.isfinite(result.diagnostics.minimum_obstacle_clearance_cm) else round(result.diagnostics.minimum_obstacle_clearance_cm,4),
                "minimum_wall_clearance_cm":None if not math.isfinite(result.diagnostics.minimum_wall_clearance_cm) else round(result.diagnostics.minimum_wall_clearance_cm,4),
                "planning_time_ms":round(planning_time_ms,4),"candidates_evaluated":result.diagnostics.candidates_evaluated,
                "candidate_ids":"|".join(candidate.candidate_id for candidate in result.candidates),
                "budget_exhausted":result.diagnostics.budget_exhausted,
                "budget_reason":result.diagnostics.budget_reason or "",
                "straights_completed":progress,"target_straights":TARGET_STRAIGHTS,
                "laps_completed":progress // STRAIGHTS_PER_LAP,
                "visible_obstacles":len(visible),"expected_relevant_obstacles":len(objects),
                "no_safe_trajectory":result.state is PlannerState.NO_SAFE_TRAJECTORY,
                "planner_reason":result.reason,
                "no_safe_reason":result.diagnostics.no_safe_reason,
                "no_safe_detail":result.diagnostics.no_safe_detail,
            })
            if result.state is PlannerState.NO_SAFE_TRAJECTORY:
                # Este es el primer instante real en que el planner no pudo
                # producir una trayectoria segura. El escenario es terminal
                # aquí, no al final de duration_s.
                first_failure_step = step
                first_failure_time_s = now
                if visible:
                    nearest = min(
                        visible,
                        key=lambda item: math.hypot(
                            item.x_cm - state.x_cm, item.y_cm - state.y_cm
                        ),
                    )
                    dx = nearest.x_cm - state.x_cm
                    dy = nearest.y_cm - state.y_cm
                    nearest_failure_obstacle = {
                        "nearest_obstacle_id": nearest.object_id,
                        "nearest_obstacle_color": nearest.color.upper(),
                        "nearest_obstacle_distance_cm": round(math.hypot(dx, dy), 4),
                        "nearest_obstacle_relative_x_cm": round(
                            dx * math.cos(state.heading_rad) + dy * math.sin(state.heading_rad), 4
                        ),
                        "nearest_obstacle_relative_y_cm": round(
                            -dx * math.sin(state.heading_rad) + dy * math.cos(state.heading_rad), 4
                        ),
                    }
                break
        previous_state=state
        state=vehicle_step(state,command,FIXED_RULES.simulation_dt_s,geometry)
        distance_cm += math.dist((previous_state.x_cm,previous_state.y_cm),(state.x_cm,state.y_cm))
        if command.target_speed_cm_s < 0.0:
            reverse_distance_cm += math.dist((previous_state.x_cm,previous_state.y_cm),(state.x_cm,state.y_cm))
        step += 1
        collision_now, _, obstacle_clearance, wall_clearance = planner.collision_metrics(state,data)
        if collision_now and collision_type is None:
            collision_type = _collision_type(obstacle_clearance, wall_clearance)
            failure_step = step
        collision |= collision_now
        for item in objects:
            longitudinal = ((state.x_cm-item.x_cm)*math.cos(state.heading_rad)
                            + (state.y_cm-item.y_cm)*math.sin(state.heading_rad))
            if longitudinal > geometry.length_cm/2:
                crossed_ids.add(item.object_id)
                # PASSED requiere haberlo visto y haber cruzado su posición.
                # Un objeto cruzado sin detección se registra como fallo de
                # percepción, pero no como maniobra completada correctamente.
                if item.object_id in seen_ids:
                    passed_ids.add(item.object_id)
        passed |= len(passed_ids) > 0
        sector=_sector(state)
        if sector and sector!=last:
            expected=order[progress % STRAIGHTS_PER_LAP]
            if sector==expected:progress+=1
            elif progress>0:route_valid=False
            last=sector
        completed=progress>=TARGET_STRAIGHTS and start_zone_contains(state.x_cm,state.y_cm)
        if abs(command.target_speed_cm_s) > 1.0 and abs(state.speed_cm_s) < 0.5:
            stationary_time += FIXED_RULES.simulation_dt_s
        else:
            stationary_time = 0.0
        stuck = stationary_time >= 2.0 and not collision and not completed
        if collision or completed or stuck:
            failure_step = failure_step or step
            break
        now+=FIXED_RULES.simulation_dt_s
    completed_straights = min(progress, TARGET_STRAIGHTS)
    if first_failure_step is not None:
        termination_reason = "NO_SAFE_TRAJECTORY"
    elif collision:
        termination_reason = "collision"
    elif completed:
        termination_reason = "completed"
    elif stuck:
        termination_reason = "stuck"
    elif rows and rows[-1]["no_safe_trajectory"]:
        termination_reason = "NO_SAFE_TRAJECTORY"
    else:
        termination_reason = "timeout"
    if first_failure_step is not None:
        failure_step = first_failure_step
    elif failure_step is None:
        failure_step = step if termination_reason != "completed" else None
    clearance_values=[r["minimum_clearance_cm"] for r in rows if r["minimum_clearance_cm"] is not None]
    wall_values=[r["minimum_wall_clearance_cm"] for r in rows if r["minimum_wall_clearance_cm"] is not None]
    obstacle_values=[r["minimum_obstacle_clearance_cm"] for r in rows if r["minimum_obstacle_clearance_cm"] is not None]
    object_not_detected=any(
        item.object_id in crossed_ids and item.object_id not in seen_ids
        for item in objects
    )
    rule_broken=not route_valid or not correct_side
    failure_location=None if termination_reason == "completed" else {
        "lap": min(progress // STRAIGHTS_PER_LAP + 1, TARGET_LAPS),
        "straight": progress % STRAIGHTS_PER_LAP + 1,
        "step": failure_step,
    }
    summary={"scenario_index":scenario_index,"base_seed":base_seed,"effective_seed":effective_seed,
        "seed":effective_seed,"scenario":scenario_index,"mode":mode,
        "single_obstacle_straight":generated.single_obstacle_straight if generated else None,
        "objects":[jsonable(o) for o in objects],"collision":collision,
        "observations":observations,
        "collision_type":collision_type,"rule_broken":rule_broken,
        "object_not_detected":object_not_detected,
        "selected_angle_deg":rows[-1]["selected_steering_angle_deg"] if rows else 0,"minimum_distance_cm":min((r["minimum_clearance_cm"] for r in rows if r["minimum_clearance_cm"] is not None),default=None),
        "maneuver_completed":passed,"passed":passed,"straight_progress":progress,
        "passed_object_ids":sorted(passed_ids),"crossed_object_ids":sorted(crossed_ids),
        "straights_completed":completed_straights,"target_straights":TARGET_STRAIGHTS,
        "laps_completed":min(progress // STRAIGHTS_PER_LAP, TARGET_LAPS),
        "completed":completed,"next_straight_reached":progress>=2,"lap_completed":completed,
        "route_progress_valid":route_valid,"correct_pass_side":correct_side,"no_safe_trajectory_cycles":no_safe,"planning_cycles":len(rows),
        "maneuvers_completed":len(passed_ids),"obstacles_passed":len(passed_ids),
        "forward_candidates_valid":last_diagnostics.forward_candidates_valid if last_diagnostics else 0,
        "reverse_recovery_attempted":reverse_recovery_attempted,
        "reverse_distance_cm":round(reverse_distance_cm,4),
        "replan_after_reverse":replan_after_reverse,
        "recovery_success":recovery_success,
        "planner_diagnostics":jsonable(last_diagnostics) if last_diagnostics else None,
        "lap_time_s":round(now + FIXED_RULES.simulation_dt_s,4) if completed else None,
        "distance_cm":round(distance_cm,4),"min_clearance_cm":min(clearance_values,default=None),
        "elapsed_simulation_s":round(step*FIXED_RULES.simulation_dt_s,4),
        "min_wall_clearance_cm":min(wall_values,default=None),"min_obstacle_clearance_cm":min(obstacle_values,default=None),
        "failure_location":failure_location,
        "first_failure_step":first_failure_step,
        "first_failure_time_s":None if first_failure_time_s is None else round(first_failure_time_s,4),
        **(nearest_failure_obstacle or {
            "nearest_obstacle_id":None,
            "nearest_obstacle_color":None,
            "nearest_obstacle_distance_cm":None,
            "nearest_obstacle_relative_x_cm":None,
            "nearest_obstacle_relative_y_cm":None,
        }),
        "timing":timing_percentiles(times),"sensor":jsonable(sensor),
        "planning_budget_mode":planning_budget_mode,
        "safety_margins_disabled":disable_safety_margins,
        "required_clearance_cm":0.0 if tuning.disable_hard_safety_margins else tuning.mandatory_clearance_cm,
        "desired_clearance_cm":tuning.preferred_clearance_cm,
        "initial_pose":list(START_POSE),"track_direction":"CLOCKWISE",
        "termination_reason":termination_reason,
        "no_safe_reason":(
            last_diagnostics.no_safe_reason
            if termination_reason == "NO_SAFE_TRAJECTORY" and last_diagnostics else None
        ),
        "no_safe_detail":(
            last_diagnostics.no_safe_detail
            if termination_reason == "NO_SAFE_TRAJECTORY" and last_diagnostics else None
        ),
        "cycles":rows}
    return summary,rows


def write_outputs(summaries,rows,output_dir:Path,run_config:dict[str,Any])->None:
    output_dir.mkdir(parents=True,exist_ok=True)
    straight_numbers={"bottom":1,"left":2,"top":3,"right":4}
    scenario_rows=[]
    for s in summaries:
        last_cycle=s["cycles"][-1] if s["cycles"] else {}
        obstacle_rows=[]
        for obstacle in s["objects"]:
            straight, slot=seat_slot((obstacle["x_cm"],obstacle["y_cm"]))
            obstacle_id=obstacle["object_id"]
            passed=obstacle_id in s["passed_object_ids"]
            obstacle_rows.append({
                "id":int(obstacle_id),"straight":straight_numbers[straight],"slot":slot,
                "color":obstacle["color"].upper(),
                "ever_detected":s["observations"][obstacle_id]["ever_detected"],
                "crossed":obstacle_id in s["crossed_object_ids"],
                "passed":passed,
                "pass_side_correct":s["correct_pass_side"] if passed else None,
            })
        planner_data=s["planner_diagnostics"] or {}
        result={
            "completed":s["completed"],"termination_reason":s["termination_reason"].upper(),
            "laps_completed":s["laps_completed"],"straights_completed":s["straights_completed"],
            "distance_cm":s["distance_cm"],"elapsed_simulation_s":s["elapsed_simulation_s"],
            "no_safe_reason":s["no_safe_reason"],
            "no_safe_detail":s["no_safe_detail"],
        }
        failure=None if s["completed"] else {
            "failure_step":s["failure_location"]["step"],
            "failure_lap":s["failure_location"]["lap"],
            "failure_straight":s["failure_location"]["straight"],
            "first_failure_step":s["first_failure_step"],
            "first_failure_time_s":s["first_failure_time_s"],
            "vehicle_x_cm":last_cycle.get("x_cm"),"vehicle_y_cm":last_cycle.get("y_cm"),
            "vehicle_heading_deg":last_cycle.get("heading_deg"),"vehicle_speed_cm_s":last_cycle.get("speed_cm_s"),
            "reason":s["termination_reason"].upper(),
        }
        scenario_rows.append({
            "scenario":{
                "scenario_index":s["scenario_index"],"base_seed":s["base_seed"],"effective_seed":s["effective_seed"],
                "mode":int(s["mode"]),"direction":s["track_direction"],"start_straight":1,
                "single_obstacle_straight":straight_numbers.get(s["single_obstacle_straight"]),
                "target_laps":TARGET_LAPS,"target_straights":TARGET_STRAIGHTS,
            },
            "result":result,
            "failure":failure,
            "collisions":{
                "collision":s["collision"],"collision_type":s["collision_type"],
                "wall_collision":s["collision_type"] in {"wall","wall_and_obstacle"},
                "obstacle_collision":s["collision_type"] in {"obstacle","wall_and_obstacle"},
                "collision_obstacle_id":None,
            },
            "clearance":{
                "min_clearance_cm":s["min_clearance_cm"],"min_wall_clearance_cm":s["min_wall_clearance_cm"],
                "min_obstacle_clearance_cm":s["min_obstacle_clearance_cm"],
                "desired_clearance_cm":s["desired_clearance_cm"],
                "required_clearance_cm":s["required_clearance_cm"],
                "safety_margins_disabled":s["safety_margins_disabled"],
            },
            "perception":{
                "object_not_detected":s["object_not_detected"],
                "visible_obstacles_at_failure":last_cycle.get("visible_obstacles",0),
                "expected_relevant_obstacles":last_cycle.get("expected_relevant_obstacles",len(s["objects"])),
                "nearest_obstacle_id":s["nearest_obstacle_id"],
                "nearest_obstacle_color":s["nearest_obstacle_color"],
                "nearest_obstacle_distance_cm":s["nearest_obstacle_distance_cm"],
                "nearest_obstacle_relative_x_cm":s["nearest_obstacle_relative_x_cm"],
                "nearest_obstacle_relative_y_cm":s["nearest_obstacle_relative_y_cm"],
            },
            "obstacles":obstacle_rows,
            "rules":{
                "rule_broken":s["rule_broken"],"wrong_pass_side_count":0 if s["correct_pass_side"] else 1,
                "obstacles_passed_correctly":sum(item["passed"] and item["pass_side_correct"] is True for item in obstacle_rows),
            },
            "planner":{
                "planning_budget_mode":s["planning_budget_mode"],
                "candidates_generated":planner_data.get("candidates_generated",0),
                "candidates_valid":planner_data.get("forward_candidates_valid",0)+planner_data.get("reverse_candidates_valid",0),
                "forward_candidates_generated":planner_data.get("forward_candidates_generated",0),
                "forward_candidates_valid":planner_data.get("forward_candidates_valid",0),
                "reverse_candidates_generated":planner_data.get("reverse_candidates_generated",0),
                "reverse_candidates_valid":planner_data.get("reverse_candidates_valid",0),
                "rejection_reasons":planner_data.get("rejection_reasons",{}),
                "forward_rejections":planner_data.get("forward_rejections",{}),
                "reverse_rejections":planner_data.get("reverse_rejections",{}),
                "candidate_diagnostics":planner_data.get("candidate_diagnostics",[]),
                "representative_rejected_candidate":planner_data.get("representative_rejected_candidate"),
            },
            "avoidance":{
                "avoidance_started":s["maneuvers_completed"]>0,
                "avoidance_start_step":None,"avoidance_start_distance_cm":None,
                "distance_when_no_safe_trajectory_cm":s["nearest_obstacle_distance_cm"] if s["termination_reason"]=="NO_SAFE_TRAJECTORY" else None,
                "maneuvers_completed":s["maneuvers_completed"],
            },
            "reverse_recovery":{
                "reverse_available":True,"reverse_attempted":s["reverse_recovery_attempted"],
                "reverse_distance_cm":s["reverse_distance_cm"],
                "replan_after_reverse":s["replan_after_reverse"],"recovery_success":s["recovery_success"],
            },
        })
    def values(key:str)->list[float]:
        return [float(s[key]) for s in summaries if s[key] is not None]
    clearances=values("min_clearance_cm"); wall_clearances=values("min_wall_clearance_cm")
    obstacle_clearances=values("min_obstacle_clearance_cm")
    scenarios_completed=sum(bool(s["completed"]) for s in summaries)
    summary={
        "scenarios":len(summaries),"scenarios_completed":scenarios_completed,
        "success_rate":round(100*scenarios_completed/max(1,len(summaries)),1),
        "total_collisions":sum(bool(s["collision"]) for s in summaries),
        "wall_collisions":sum(s["collision_type"] in {"wall","wall_and_obstacle"} for s in summaries),
        "obstacle_collisions":sum(s["collision_type"] in {"obstacle","wall_and_obstacle"} for s in summaries),
        "rule_broken":sum(bool(s["rule_broken"]) for s in summaries),
        "object_not_detected":sum(bool(s["object_not_detected"]) for s in summaries),
        "no_safe_trajectory":sum(s["termination_reason"]=="NO_SAFE_TRAJECTORY" for s in summaries),
        "timeout":sum(s["termination_reason"]=="timeout" for s in summaries),
        "stuck":sum(s["termination_reason"]=="stuck" for s in summaries),
        "total_laps_completed":sum(int(s["laps_completed"]) for s in summaries),
        "total_straights_completed":sum(int(s["straights_completed"]) for s in summaries),
        "mean_straights_completed":round(sum(int(s["straights_completed"]) for s in summaries)/max(1,len(summaries)),1),
        "mean_laps_completed":round(sum(int(s["laps_completed"]) for s in summaries)/max(1,len(summaries)),1),
        "min_clearance_cm":{
            "overall_min":min(clearances,default=None),"mean":round(sum(clearances)/len(clearances),1) if clearances else None,
            "wall_min":min(wall_clearances,default=None),"obstacle_min":min(obstacle_clearances,default=None),
        },
        "planning_budget_mode":summaries[0].get("planning_budget_mode") if summaries else None,
        "safety_margins_disabled":bool(summaries and summaries[0].get("safety_margins_disabled")),
    }
    (output_dir/"config.json").write_text(
        json.dumps(jsonable(run_config),indent=2,ensure_ascii=False)+"\n",encoding="utf-8"
    )
    (output_dir/"summary.json").write_text(
        json.dumps(jsonable(summary),indent=2,ensure_ascii=False)+"\n",encoding="utf-8"
    )
    with (output_dir/"scenarios.jsonl").open("w",encoding="utf-8") as handle:
        for scenario in scenario_rows:
            handle.write(json.dumps(jsonable(scenario),ensure_ascii=False)+"\n")
    with (output_dir/"failures.jsonl").open("w",encoding="utf-8") as handle:
        for scenario in scenario_rows:
            if not scenario["result"]["completed"]:
                handle.write(json.dumps(jsonable(scenario),ensure_ascii=False)+"\n")


def main()->None:
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--scenarios",type=int,default=20);parser.add_argument("--seed",type=int,default=20260815)
    parser.add_argument("--duration-s",type=float,default=180,
                        help="Duración máxima; 180 s permite intentar las 3 vueltas a velocidad baja.")
    parser.add_argument("--output-dir",type=Path,default=Path("/tmp/wro_planner_results"))
    parser.add_argument("--noise-position-cm",type=float,default=0);parser.add_argument("--noise-heading-deg",type=float,default=0);parser.add_argument("--latency-s",type=float,default=0)
    parser.add_argument("--dropout-probability",type=float,default=0);parser.add_argument("--planner-config",type=Path,default=None)
    parser.add_argument("--planning-horizon-cm", type=float, default=None,
                        help="Longitud de cada plan completo que se compara.")
    parser.add_argument("--execution-horizon-min-cm", type=float, default=None,
                        help="Distancia mínima ejecutada entre comparaciones.")
    parser.add_argument("--execution-horizon-max-cm", type=float, default=None,
                        help="Distancia máxima ejecutada entre comparaciones.")
    parser.add_argument("--switch-margin", type=float, default=None,
                        help="Mejora mínima de score para cambiar el commitment.")
    parser.add_argument("--diagnostic-level", choices=("full", "summary", "off"),
                        default="summary",
                        help="Nivel de diagnóstico; los tests usan summary por defecto.")
    parser.add_argument("--mode", choices=("1", "2", "3"), default="2",
                        help="1=sin obstáculos, 2=obstáculos, 3=obstáculos + parking wall (aún inhabilitado).")
    parser.add_argument("--planning-budget-mode",choices=("candidate_count","time"),
                        default="candidate_count",
                        help="Presupuesto del planner: candidate_count es reproducible; time usa max_planning_time_ms.")
    parser.add_argument("--strict-deterministic",action="store_true",
                        help="Alias compatible: selecciona planning_budget_mode=candidate_count.")
    parser.add_argument("--disable-safety-margins",action="store_true",
                        help="Solo simulador: pone los márgenes artificiales en 0; mantiene geometría y colisiones.")
    args=parser.parse_args()
    if args.mode == "3":
        parser.error("El mode 3 (obstáculos + parking wall) está inhabilitado porque aún no tiene configuración.")
    tuning=load_planner_tuning(args.planner_config)
    tuning_overrides = {
        name: getattr(args, name)
        for name in (
            "planning_horizon_cm", "execution_horizon_min_cm",
            "execution_horizon_max_cm", "switch_margin",
        )
        if getattr(args, name) is not None
    }
    if tuning_overrides:
        tuning=tuning.with_overrides(**tuning_overrides)
    tuning=tuning.with_overrides(diagnostic_level=args.diagnostic_level)
    if args.disable_safety_margins:
        tuning=tuning.with_overrides(
            disable_hard_safety_margins=True,
            allow_physical_collisions=True,
        )
    sensor=SensorModel(args.noise_position_cm,args.noise_heading_deg,args.latency_s,args.dropout_probability);summaries=[];rows=[]
    selected_budget_mode="candidate_count" if args.strict_deterministic else args.planning_budget_mode
    for index in range(args.scenarios):
        effective_seed=args.seed+index
        print(f"[{index+1}/{args.scenarios}] index {index} | effective_seed {effective_seed} | mode {args.mode}",flush=True)
        summary,cycle_rows=run_scenario(effective_seed,index,sensor,args.duration_s,tuning,selected_budget_mode,args.mode,args.seed,args.disable_safety_margins)
        summaries.append(summary);rows+=cycle_rows
    run_config={
        "mode":args.mode,"scenarios":args.scenarios,"seed":args.seed,
        "duration_s":args.duration_s,"fixed_speed_cm_s":FIXED_RULES.fixed_speed_cm_s,
        "planning_budget_mode":selected_budget_mode,"simulation_dt_s":FIXED_RULES.simulation_dt_s,
        "disable_safety_margins":args.disable_safety_margins,
        "sensor":asdict(sensor),"planner_config":str(args.planner_config) if args.planner_config else None,
        "tuning":asdict(tuning),
    }
    write_outputs(summaries,rows,args.output_dir,run_config)
    print(json.dumps(json.loads((args.output_dir/"summary.json").read_text()),indent=2))


if __name__=="__main__":main()
