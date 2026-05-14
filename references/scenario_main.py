import math
import time
import threading

import carla
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3
from agents.navigation.basic_agent import BasicAgent


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def get_speed_ms(vehicle: carla.Vehicle) -> float:
    """차량 속도를 m/s 단위로 계산"""
    v = vehicle.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class PIController:
    """
    리더 차량 종방향 속도 제어용 간단한 PI 제어기
    - 현재 코드에서는 D항을 사용하지 않으므로 PI로 단순화
    """
    def __init__(self, kp, ki, i_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.i_limit = i_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def compute(self, err, dt, u_min, u_max):
        """
        PI 출력 계산 + anti-windup
        """
        u_unsat = self.kp * err + self.ki * self.integral
        u_sat = clamp(u_unsat, u_min, u_max)

        # 출력이 포화되지 않을 때만 적분
        if u_unsat == u_sat:
            self.integral += err * dt
            self.integral = clamp(self.integral, -self.i_limit, self.i_limit)

        return u_sat


class EpisodeController(Node):
    def __init__(self):
        super().__init__('episode_controller')

        # =========================
        # ROS2 interfaces
        # =========================
        self.create_subscription(Bool, '/start_new_episode', self.start_episode_callback, 10)
        self.create_subscription(Bool, '/start_drive', self.start_drive_callback, 10)

        self.create_subscription(Vector3, '/follower1_control_cmd', self.follower1_control_callback, 10)
        self.create_subscription(Vector3, '/follower2_control_cmd', self.follower2_control_callback, 10)
        self.create_subscription(Vector3, '/follower3_control_cmd', self.follower3_control_callback, 10)

        self.create_subscription(Float32, '/leader_vref_ms', self.leader_vref_callback, 10)

        self.arrival_publisher = self.create_publisher(Bool, '/leader_arrival_flag', 10)

        # =========================
        # Shared command states
        # =========================
        self._leader_vref_ms = 0.0
        self._leader_vref_lock = threading.Lock()

        # =========================
        # CARLA connection
        # =========================
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        self.spawn_points = self.carla_map.get_spawn_points()

        # =========================
        # Actor / agent states
        # =========================
        self.leader_vehicle = None
        self.follower1_vehicle = None
        self.follower2_vehicle = None
        self.follower3_vehicle = None

        self.agent = None
        self.follower_agent = None
        self.follower_agent2 = None
        self.follower_agent3 = None

        self.stop_requested = False
        self.start_driving = False
        self.drive_thread = None

        # =========================
        # Leader longitudinal control
        # =========================
        self.leader_pi = PIController(kp=0.6, ki=0.15, i_limit=8.0)

        self.brake_gain = 1.0
        self.vref_cmd = 0.0
        self.vref_rate_up = 0.6   # m/s^2
        self.vref_rate_dn = 4.0   # m/s^2

        # =========================
        # Debug printing
        # =========================
        self._last_print_t = None
        self.print_period = 0.3
        self._last_frame = None

    # =========================================================
    # Helper functions
    # =========================================================
    def _get_vehicle_by_role(self, role_name):
        """role_name으로 CARLA vehicle 찾기"""
        actors = self.world.get_actors().filter('vehicle.*')
        matched = [v for v in actors if v.attributes.get('role_name') == role_name]
        return matched[0] if matched else None

    def _place_and_freeze(self, vehicle, transform, z_offset=0.3):
        """
        차량을 지정 위치에 놓고 완전히 정지시킨다.
        주의: 동기화 tick은 MATLAB이 담당하므로 여기서 world.tick() 호출 금지
        """
        vehicle.set_autopilot(False)
        vehicle.set_simulate_physics(False)

        tf = carla.Transform(
            carla.Location(
                x=transform.location.x,
                y=transform.location.y,
                z=transform.location.z + z_offset
            ),
            transform.rotation
        )
        vehicle.set_transform(tf)

        try:
            vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        except Exception as e:
            print(f"[freeze] set_target_* failed: {e}")

        vehicle.apply_control(carla.VehicleControl(
            throttle=0.0,
            brake=1.0,
            steer=0.0,
            hand_brake=True,
            reverse=False
        ))

        vehicle.set_simulate_physics(True)

    def _release_all_brakes(self):
        """주행 시작 시 리더/팔로워 차량의 hand brake 해제"""
        for vehicle in [
            self.leader_vehicle,
            self.follower1_vehicle,
            self.follower2_vehicle,
            self.follower3_vehicle
        ]:
            if vehicle is not None:
                vehicle.apply_control(carla.VehicleControl(
                    throttle=0.0,
                    brake=0.0,
                    steer=0.0,
                    hand_brake=False,
                    reverse=False
                ))

    def _apply_follower_control(self, vehicle, agent, msg):
        """
        follower 차량 제어 공통 함수
        - longitudinal(throttle/brake): MATLAB에서 전달
        - lateral(steer): BasicAgent 사용
        """
        if vehicle is None or agent is None:
            return

        throttle = float(msg.x)
        brake = float(msg.y)
        steer = agent.run_step().steer

        vehicle.apply_control(carla.VehicleControl(
            throttle=throttle,
            brake=brake,
            steer=steer,
            hand_brake=False,
            reverse=False,
            manual_gear_shift=False
        ))

    # =========================================================
    # ROS2 callbacks
    # =========================================================
    def start_episode_callback(self, msg):
        if msg.data:
            self.reset_and_start()

    def start_drive_callback(self, msg: Bool):
        """
        MATLAB이 안정화 tick을 모두 보낸 뒤 True를 보내면 실제 주행 시작
        """
        self.start_driving = bool(msg.data)
        if self.start_driving:
            print("[START_DRIVE] 주행 허가 수신 -> 모든 차량 hand_brake 해제")
            self._release_all_brakes()

    def follower1_control_callback(self, msg):
        self._apply_follower_control(self.follower1_vehicle, self.follower_agent, msg)

    def follower2_control_callback(self, msg):
        self._apply_follower_control(self.follower2_vehicle, self.follower_agent2, msg)

    def follower3_control_callback(self, msg):
        self._apply_follower_control(self.follower3_vehicle, self.follower_agent3, msg)

    def leader_vref_callback(self, msg: Float32):
        """MATLAB에서 보내는 리더 목표 속도(m/s) 저장"""
        with self._leader_vref_lock:
            self._leader_vref_ms = float(msg.data)

    # =========================================================
    # Episode reset / route setup
    # =========================================================
    def reset_and_start(self):
        """
        새 에피소드 시작 시 수행:
        1) 제어기 초기화
        2) 차량 재배치 및 정지
        3) route 재설정
        4) leader drive thread 시작
        """
        self.leader_pi.reset()
        self.vref_cmd = 0.0
        self._last_frame = None

        print("\n새 에피소드 시작 요청 수신: 차량 초기화 + 리더 주행 준비")

        # leader 도착 플래그 초기화
        self.arrival_publisher.publish(Bool(data=False))
        print("'/leader_arrival_flag' -> False 초기화 완료")

        # 기존 주행 스레드 종료
        self.stop_requested = True
        if self.drive_thread and self.drive_thread.is_alive():
            print("이전 주행 스레드 종료 대기 중...")
            self.drive_thread.join()

        self.stop_requested = False
        self.start_driving = False

        # 차량 찾기
        self.leader_vehicle = self._get_vehicle_by_role('leader_vehicle')
        self.follower1_vehicle = self._get_vehicle_by_role('ego_vehicle_1')
        self.follower2_vehicle = self._get_vehicle_by_role('ego_vehicle_2')
        self.follower3_vehicle = self._get_vehicle_by_role('ego_vehicle_3')

        if self.leader_vehicle is None:
            print("'leader_vehicle'을 찾을 수 없습니다.")
            return

        # 리더 시작 위치 배치
        leader_start_wp = self.carla_map.get_waypoint(self.spawn_points[278].location)
        self._place_and_freeze(self.leader_vehicle, leader_start_wp.transform)

        # follower 차량을 leader 뒤쪽에 일정 거리로 배치
        vehicle_length = 7.94
        buffer_gap = 7.0
        total_gap = vehicle_length + buffer_gap

        follower_specs = [
            ('ego_vehicle_1', self.follower1_vehicle, 1),
            ('ego_vehicle_2', self.follower2_vehicle, 2),
            ('ego_vehicle_3', self.follower3_vehicle, 3),
        ]

        for role_name, vehicle, idx in follower_specs:
            if vehicle is None:
                print(f"차량 '{role_name}'을 찾을 수 없음")
                continue

            distance = idx * total_gap
            prev_wps = leader_start_wp.previous(distance)
            if not prev_wps:
                print(f"차량 '{role_name}' 배치를 위한 이전 waypoint 없음 ({distance:.2f} m)")
                continue

            self._place_and_freeze(vehicle, prev_wps[0].transform)

        # 리더 경로 계획
        self.agent = BasicAgent(self.leader_vehicle, target_speed=50)

        way_ids = [278, 238, 272, 222, 188, 352, 336, 305, 366, 278]
        waypoints = [self.carla_map.get_waypoint(self.spawn_points[i].location) for i in way_ids]

        route = []
        for i in range(len(waypoints) - 1):
            seg = self.agent.trace_route(waypoints[i], waypoints[i + 1])
            route.extend(seg)

        self.agent.set_global_plan(route)

        # follower는 lateral만 BasicAgent 사용
        if self.follower1_vehicle is not None:
            self.follower_agent = BasicAgent(self.follower1_vehicle, target_speed=70)
            self.follower_agent.set_global_plan(route)

        if self.follower2_vehicle is not None:
            self.follower_agent2 = BasicAgent(self.follower2_vehicle, target_speed=70)
            self.follower_agent2.set_global_plan(route)

        if self.follower3_vehicle is not None:
            self.follower_agent3 = BasicAgent(self.follower3_vehicle, target_speed=70)
            self.follower_agent3.set_global_plan(route)

        # 리더 주행 스레드 시작
        self.drive_thread = threading.Thread(target=self.leader_drive_loop, daemon=True)
        self.drive_thread.start()

    # =========================================================
    # Leader drive loop
    # =========================================================
    def leader_drive_loop(self):
        """
        리더 차량 주행 루프
        - steer: BasicAgent
        - throttle/brake: PI 속도제어
        - CARLA tick은 MATLAB이 주므로 새 frame이 들어왔을 때만 제어 수행
        """
        print("\n리더 차량 주행 시작 대기\n")

        while not self.stop_requested and not self.start_driving:
            time.sleep(0.005)

        if self.leader_vehicle is not None:
            self.leader_vehicle.apply_control(carla.VehicleControl(
                throttle=0.0,
                brake=0.0,
                steer=0.0,
                hand_brake=False,
                reverse=False
            ))

        print("리더 차량 주행 시작\n")

        while True:
            if self.stop_requested:
                print("이전 주행 중단 요청 수신 -> 루프 종료")
                break

            # 새 tick(frame) 도착했을 때만 제어를 갱신
            snap = self.world.get_snapshot()
            frame = snap.timestamp.frame

            if self._last_frame is None:
                self._last_frame = frame
                time.sleep(0.001)
                continue

            if frame == self._last_frame:
                time.sleep(0.001)
                continue

            self._last_frame = frame
            dt = 0.05

            # 경로 종료 시 MATLAB에 도착 알림
            if self.agent.done():
                print("리더 차량이 최종 목적지에 도착했습니다!")
                self.arrival_publisher.publish(Bool(data=True))
                print("'/leader_arrival_flag' 토픽으로 도착 메시지 전송 완료")
                break

            # BasicAgent는 횡방향 steer 계산
            steer = self.agent.run_step().steer

            # MATLAB에서 받은 리더 목표속도 읽기
            with self._leader_vref_lock:
                vref = float(self._leader_vref_ms)

            # vref 급변을 막기 위한 rate limiter
            dv = vref - self.vref_cmd
            max_up = self.vref_rate_up * dt
            max_dn = self.vref_rate_dn * dt
            dv = clamp(dv, -max_dn, max_up)
            self.vref_cmd += dv

            # 현재 속도와 속도 오차
            v_now = get_speed_ms(self.leader_vehicle)
            err = self.vref_cmd - v_now

            # PI 제어 출력
            U_MAX = 0.85
            U_MIN = -0.50
            u = self.leader_pi.compute(err, dt, U_MIN, U_MAX)

            # PI 출력 -> throttle / brake 변환
            THROTTLE_MAX = 0.85
            BRAKE_MAX = 0.50

            if u >= 0.0:
                throttle = clamp(u, 0.0, THROTTLE_MAX)
                brake = 0.0
            else:
                throttle = 0.0
                brake = clamp((-u) * self.brake_gain, 0.0, BRAKE_MAX)

            # 작은 값은 deadband 처리
            if throttle < 0.02:
                throttle = 0.0
            if brake < 0.02:
                brake = 0.0

            # 최종 제어 적용
            self.leader_vehicle.apply_control(carla.VehicleControl(
                throttle=throttle,
                brake=brake,
                steer=steer,
                hand_brake=False,
                reverse=False
            ))

            # 디버그 출력
            t_dbg = time.time()
            if self._last_print_t is None:
                self._last_print_t = t_dbg

            if (t_dbg - self._last_print_t) >= self.print_period:
                self._last_print_t = t_dbg
                ctrl = self.leader_vehicle.get_control()

                print(
                    f"[LEADER] gear={ctrl.gear:2d} | manual={ctrl.manual_gear_shift} | "
                    f"vref={vref:5.2f} | vref_cmd={self.vref_cmd:5.2f} | "
                    f"v_now={v_now:5.2f} | err={err:6.2f} | "
                    f"throttle={throttle:4.2f} | brake={brake:4.2f}"
                )

            time.sleep(0.003)


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
