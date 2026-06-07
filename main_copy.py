import asyncio
import time
import SPIMoveTest

try:
    from rplidarc1.scanner import RPLidar
except ImportError:
    from scanner import RPLidar


BASE_SPEED = 80        
TURN_SPEED = 200     
RIGHT_MOTOR_CORRECTION = 1.5 


KP = 0.55
KD = 1.0


TURN_90_DURATION = 0.45
TURN_180_DURATION = 2.0
SETTLE_DURATION = 1.7  


NO_WALL_THRESHOLD = 500 
FRONT_WALL_STOP = 150 


class MazeSolver:
    def __init__(self):
        self.robot = SPIMoveTest.RobotController()
        
        self.turn_dist_left = 2000.0
        self.turn_dist_right = 2000.0
        self.front_dist = 2000.0
        
        self.pid_dist_left = 500.0
        self.pid_dist_right = 500.0
        
        self.temp_min_left = 2000.0
        self.temp_min_right = 2000.0
        self.temp_turn_left = 2000.0
        self.temp_turn_right = 2000.0
        self.temp_front = 2000.0
        
        self.last_lidar_angle = 0.0
        
        self.last_error = 0.0
        self.current_correction = 0.0
        
        self.dir_left = 1
        self.dir_right = 1
        self.final_v_left = 0
        self.final_v_right = 0
        
        self.is_running = True
        self.first_scan_complete = False 

    def update_lidar_point(self, angle_deg, dist):
        if dist <= 0: return

        if self.last_lidar_angle > 300 and angle_deg < 60:
            self.pid_dist_left = self.temp_min_left
            self.pid_dist_right = self.temp_min_right
            self.turn_dist_left = self.temp_turn_left
            self.turn_dist_right = self.temp_turn_right
            self.front_dist = self.temp_front
            
            self.temp_min_left = 2000.0
            self.temp_min_right = 2000.0
            self.temp_turn_left = 2000.0
            self.temp_turn_right = 2000.0
            self.temp_front = 2000.0
            
            self.first_scan_complete = True 
            
        self.last_lidar_angle = angle_deg

        if 70 <= angle_deg <= 110:
            if dist < self.temp_min_right: self.temp_min_right = dist
        elif 250 <= angle_deg <= 290:
            if dist < self.temp_min_left: self.temp_min_left = dist
            
        if 85 < angle_deg < 95:
            if dist < self.temp_turn_right: self.temp_turn_right = dist
        elif 265 < angle_deg < 275:
            if dist < self.temp_turn_left: self.temp_turn_left = dist
            
        elif angle_deg > 345 or angle_deg < 15:
            if dist < self.temp_front: self.temp_front = dist

    def _set_motors(self, dir_l, pwm_l, dir_r, pwm_r):
        corrected_pwm_r = int(pwm_r * RIGHT_MOTOR_CORRECTION)
        corrected_pwm_r = max(0, min(255, corrected_pwm_r))
        pwm_l = max(0, min(255, int(pwm_l)))
        
        self.dir_left = dir_l
        self.dir_right = dir_r
        self.final_v_left = pwm_l
        self.final_v_right = corrected_pwm_r
        
        self.robot.start_move(dir_l, pwm_l, dir_r, corrected_pwm_r)

    async def turn_sequence_left(self):
        print(f"\n[АВТОПИЛОТ] Проход СЛЕВА. Перед={int(self.front_dist)}мм | Лево={int(self.turn_dist_left)}мм")
        self._set_motors(0, TURN_SPEED, 1, TURN_SPEED)
        await asyncio.sleep(TURN_90_DURATION)
        
        print("[АВТОПИЛОТ] Стабилизация.")
        self._set_motors(1, BASE_SPEED, 1, BASE_SPEED)
        await asyncio.sleep(SETTLE_DURATION)
        self.reset_pid()

    async def turn_sequence_right(self):
        print(f"\n[АВТОПИЛОТ] Проход СПРАВА. Перед={int(self.front_dist)}мм | Право={int(self.turn_dist_right)}мм")
        self._set_motors(1, TURN_SPEED, 0, TURN_SPEED)
        await asyncio.sleep(TURN_90_DURATION)
        
        print("[АВТОПИЛОТ] Стабилизация.")
        self._set_motors(1, BASE_SPEED, 1, BASE_SPEED)
        await asyncio.sleep(SETTLE_DURATION)
        self.reset_pid()

    async def turn_sequence_180(self):
        print(f"\n[АВТОПИЛОТ] ТУПИК. Разворот на 180.")
        self._set_motors(1, TURN_SPEED, 0, TURN_SPEED)
        await asyncio.sleep(TURN_180_DURATION)
        
        print("[АВТОПИЛОТ] Ищем стены...")
        self._set_motors(1, BASE_SPEED, 1, BASE_SPEED)
        await asyncio.sleep(SETTLE_DURATION)
            
        self.reset_pid()

    def reset_pid(self):
        self.last_error = 0.0
        self.current_correction = 0.0

    async def control_loop(self):
        print("[СИСТЕМА] Запуск петли управления. Ждем первый скан лидара...")
        while self.is_running:
            if not self.first_scan_complete:
                await asyncio.sleep(0.2)
                continue

            if self.front_dist < FRONT_WALL_STOP and self.turn_dist_left < NO_WALL_THRESHOLD and self.turn_dist_right < NO_WALL_THRESHOLD:
                print()
                await self.turn_sequence_180()
                
            elif self.turn_dist_left > NO_WALL_THRESHOLD:
                print()
                await self.turn_sequence_left()
                
            elif self.turn_dist_right > NO_WALL_THRESHOLD:
                print()
                await self.turn_sequence_right()
                
            else:
                error = self.pid_dist_left - self.pid_dist_right
                
                if error != self.last_error:
                    derivative = error - self.last_error
                    self.current_correction = (KP * error) + (KD * derivative)
                    self.last_error = error

                v_left_ideal = BASE_SPEED - self.current_correction
                v_right_ideal = BASE_SPEED + self.current_correction

                dir_l = 1 if v_left_ideal >= 0 else 0
                final_pwm_left = min(255, abs(int(v_left_ideal)))

                dir_r = 1 if v_right_ideal >= 0 else 0
                final_pwm_right = min(255, abs(int(v_right_ideal)))
                
                # Лог в консоль одной строкой
                dir_l_str = "+" if dir_l == 1 else "-"
                dir_r_str = "+" if dir_r == 1 else "-"
                print(f"\r[ПИД] Фронт:{int(self.front_dist):4d} | Л:{int(self.pid_dist_left):4d} П:{int(self.pid_dist_right):4d} | Ошибка:{int(error):4d} | Корр:{int(self.current_correction):4d} | Мот Л:{dir_l_str}{final_pwm_left:3d} П:{dir_r_str}{final_pwm_right:3d}   ", end="", flush=True)

                self._set_motors(dir_l, final_pwm_left, dir_r, final_pwm_right)

            await asyncio.sleep(0.01)

async def main():
    print("Инициализация RPLidar...")
    
    lidar = RPLidar("/dev/ttyUSB0", 460800)
    scan_task = asyncio.create_task(lidar.simple_scan())
    
    solver = MazeSolver()
    control_task = asyncio.create_task(solver.control_loop())

    try:
        while not lidar.stop_event.is_set():
            while not lidar.output_queue.empty():
                data = lidar.output_queue.get_nowait()
                dist = data.get('d_mm')
                angle_deg = data.get('a_deg')
                
                if dist is not None and isinstance(dist, (int, float)):
                    solver.update_lidar_point(angle_deg, dist)
                    
            await asyncio.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\nПрограмма остановлена (Ctrl+C)")
    except Exception as e:
        print(f"\nКритическая ошибка: {e}")
    finally:
        print("\nЭкстренное торможение...")
        solver.is_running = False
        lidar.stop_event.set()
        scan_task.cancel()
        control_task.cancel()
        
        solver.robot.stop()
        lidar.reset()

if __name__ == '__main__':
    asyncio.run(main())
