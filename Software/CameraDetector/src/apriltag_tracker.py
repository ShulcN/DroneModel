import cv2
import numpy as np
import time
import apriltag
import argparse
import threading
import socket
import json
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class AprilTagTracker:
    def __init__(self, tag_family="tag36h11", tag_size=0.1, camera_index=0, 
                 show_fps=True, debug=False, send_data=False, port=5555,
                 history_length=5, show_3d=False):
        """
        Инициализация трекера AprilTag
        
        Args:
            tag_family: Семейство тегов AprilTag
            tag_size: Размер тега в метрах
            camera_index: Индекс камеры
            show_fps: Показывать ли FPS
            debug: Включить ли режим отладки
            send_data: Отправлять ли данные по UDP
            port: Порт для UDP соединения
            history_length: Длина истории для сглаживания координат
            show_3d: Показывать ли 3D визуализацию положения
        """
        self.tag_family = tag_family
        self.tag_size = tag_size
        self.camera_index = camera_index
        self.show_fps = show_fps
        self.debug = debug
        self.send_data = send_data
        self.port = port
        self.history_length = history_length
        self.show_3d = show_3d
        
        # Загрузка параметров камеры
        self.camera_matrix, self.dist_coeffs = self.load_camera_params()
        
        # Инициализация детектора AprilTag
        options = apriltag.DetectorOptions(families=self.tag_family)
        self.detector = apriltag.Detector(options)
        
        # Для фильтрации координат
        self.position_history = {}  # Словарь для хранения истории для каждого тега
        
        # Для записи координат в файл
        if debug:
            self.log_file = open("apriltag_coordinates.csv", "w")
            self.log_file.write("timestamp,tag_id,x,y,z,roll,pitch,yaw\n")
            
        # Для UDP отправки
        if send_data:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
        # Для 3D визуализации
        if show_3d:
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')
            self.ax.set_zlabel('Z (m)')
            self.ax.set_title('AprilTag 3D Position')
            self.plot_thread = threading.Thread(target=self.update_plot)
            self.plot_thread.daemon = True
            self.position_3d = {}  # Словарь для хранения текущих позиций
            self.plot_running = True
            
        # Флаг для остановки трекера
        self.running = True
            
    def load_camera_params(self):
        """
        Загрузка параметров калибровки камеры
        """
        try:
            data = np.load('camera_calibration.npz')
            camera_matrix = data['camera_matrix']
            dist_coeffs = data['dist_coeffs']
            print("Загружены параметры калибровки камеры")
            return camera_matrix, dist_coeffs
        except:
            print("ВНИМАНИЕ: Файл калибровки не найден. Используются значения по умолчанию.")
            # Примерные значения матрицы камеры для камеры 640x480
            camera_matrix = np.array([
                [800, 0, 320],
                [0, 800, 240],
                [0, 0, 1]
            ], dtype=np.float32)
            
            dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
            
            return camera_matrix, dist_coeffs
            
    def update_plot(self):
        """
        Обновление 3D графика в отдельном потоке
        """
        while self.plot_running:
            if len(self.position_3d) > 0:
                self.ax.clear()
                self.ax.set_xlabel('X (m)')
                self.ax.set_ylabel('Y (m)')
                self.ax.set_zlabel('Z (m)')
                self.ax.set_title('AprilTag 3D Position')
                
                # Установка лимитов для осей
                self.ax.set_xlim([-1, 1])
                self.ax.set_ylim([-1, 1])
                self.ax.set_zlim([0, 3])
                
                # Рисуем положение каждого тега
                for tag_id, pos in self.position_3d.items():
                    x, y, z = pos
                    self.ax.scatter(x, y, z, label=f'Tag {tag_id}')
                    
                self.ax.legend()
                plt.pause(0.1)
            time.sleep(0.1)
    
    def start(self):
        """
        Запуск трекера AprilTag
        """
        # Инициализация видеопотока
        cap = cv2.VideoCapture(self.camera_index)
        
        # Установка разрешения камеры
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Для расчета FPS
        prev_frame_time = 0
        new_frame_time = 0
        
        print("Начало отслеживания AprilTag...")
        print("Нажмите 'q' для выхода")
        
        # Запуск потока обновления 3D графика
        if self.show_3d:
            self.plot_thread.start()
        
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    print("Ошибка при получении кадра")
                    break
                
                # Преобразование в grayscale для детектора
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Расчет FPS
                new_frame_time = time.time()
                fps = 1/(new_frame_time - prev_frame_time) if prev_frame_time > 0 else 0
                prev_frame_time = new_frame_time
                
                # Обнаружение тегов
                results = detector.detect(gray)
                
                # Отображаем FPS
                if self.show_fps:
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Обработка результатов
                for r in results:
                    # Обработка каждого обнаруженного тега
                    self.process_tag(r, frame)
                
                # Отображение кадра
                cv2.imshow("AprilTag Tracking", frame)
                
                # Выход по нажатию клавиши 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break
        
        finally:
            # Освобождение ресурсов
            cap.release()
            cv2.destroyAllWindows()
            
            if self.debug:
                self.log_file.close()
                print("\nКоординаты сохранены в файл apriltag_coordinates.csv")
                
            if self.show_3d:
                self.plot_running = False
                self.plot_thread.join()
                plt.close()
    
    def process_tag(self, r, frame):
        """
        Обработка обнаруженного тега
        
        Args:
            r: Результат обнаружения тега
            frame: Кадр для отображения
        """
        # Получаем ID тэга
        tag_id = r.tag_id
        
        # Координаты углов тэга на изображении
        corners = r.corners.astype(np.int32)
        
        # Рисуем контур тэга
        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
        
        # Вычисляем центр тэга
        center = np.mean(corners, axis=0).astype(int)
        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
        
        # Текст с ID тэга
        cv2.putText(frame, f"ID: {tag_id}", (center[0] + 10, center[1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Определение позиции и ориентации
        # Соответствие между 3D точками на теге и 2D точками на изображении
        obj_pts = np.array([
            [-self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2,  self.tag_size/2, 0],
            [-self.tag_size/2,  self.tag_size/2, 0]
        ])
        
        img_pts = r.corners
        
        # Решение PnP задачи
        success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.camera_matrix, self.dist_coeffs)
        
        if success:
            # Получаем координаты X, Y, Z относительно камеры
            x, y, z = tvec.flatten()
            
            # Инициализация истории позиций для тега, если еще не существует
            if tag_id not in self.position_history:
                self.position_history[tag_id] = []
            
            # Добавляем текущую позицию в историю
            self.position_history[tag_id].append((x, y, z))
            
            # Ограничиваем длину истории
            if len(self.position_history[tag_id]) > self.history_length:
                self.position_history[tag_id].pop(0)
            
            # Усредняем последние позиции
            x_avg, y_avg, z_avg = np.mean(self.position_history[tag_id], axis=0)
            
            # Сохраняем для 3D визуализации
            if self.show_3d:
                self.position_3d[tag_id] = (x_avg, y_avg, z_avg)
            
            # Преобразование вектора вращения в углы Эйлера
            rot_mat, _ = cv2.Rodrigues(rvec)
            euler_angles = cv2.RQDecomp3x3(rot_mat)[0]
            roll, pitch, yaw = euler_angles
            
            # Отображаем координаты
            cv2.putText(frame, f"X: {x_avg:.2f}m", (center[0] + 10, center[1] + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(frame, f"Y: {y_avg:.2f}m", (center[0] + 10, center[1] + 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(frame, f"Z: {z_avg:.2f}m", (center[0] + 10, center[1] + 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            if self.debug:
                cv2.putText(frame, f"Roll: {np.degrees(roll):.1f}°", (center[0] + 10, center[1] + 80), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(frame, f"Pitch: {np.degrees(pitch):.1f}°", (center[0] + 10, center[1] + 100), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(frame, f"Yaw: {np.degrees(yaw):.1f}°", (center[0] + 10, center[1] + 120), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Рисуем оси координат
            axis_length = self.tag_size
            axis_pts, _ = cv2.projectPoints(np.array([
                [0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]
            ]), rvec, tvec, self.camera_matrix, self.dist_coeffs)
            
            origin = tuple(axis_pts[0].ravel().astype(int))
            x_end = tuple(axis_pts[1].ravel().astype(int))
            y_end = tuple(axis_pts[2].ravel().astype(int))
            z_end = tuple(axis_pts[3].ravel().astype(int))
            
            cv2.line(frame, origin, x_end, (0, 0, 255), 2)  # X-axis (красный)
            cv2.line(frame, origin, y_end, (0, 255, 0), 2)  # Y-axis (зеленый)
            cv2.line(frame, origin, z_end, (255, 0, 0), 2)  # Z-axis (синий)
            
            # Запись данных в лог
            if self.debug:
                timestamp = time.time()
                self.log_file.write(f"{timestamp},{tag_id},{x_avg:.6f},{y_avg:.6f},{z_avg:.6f},"
                            f"{np.degrees(roll):.2f},{np.degrees(pitch):.2f},{np.degrees(yaw):.2f}\n")
                self.log_file.flush()
                
            # Отправка данных по UDP
            if self.send_data:
                data = {
                    "timestamp": time.time(),
                    "tag_id": int(tag_id),
                    "position": {
                        "x": float(x_avg),
                        "y": float(y_avg),
                        "z": float(z_avg)
                    },
                    "rotation": {
                        "roll": float(np.degrees(roll)),
                        "pitch": float(np.degrees(pitch)),
                        "yaw": float(np.degrees(yaw))
                    }
                }
                
                # Отправка данных по UDP
                try:
                    message = json.dumps(data).encode('utf-8')
                    self.sock.sendto(message, ('127.0.0.1', self.port))
                except:
                    if self.debug:
                        print("Ошибка отправки данных по UDP")
                
            # Выводим данные в консоль
            print(f"Tag {tag_id}: X={x_avg:.3f}m, Y={y_avg:.3f}m, Z={z_avg:.3f}m", end="\r")

def main():
    parser = argparse.ArgumentParser(description='AprilTag отслеживание для квадрокоптера')
    parser.add_argument('--family', type=str, default='tag36h11',
                        help='Семейство тегов AprilTag (по умолчанию: tag36h11)')
    parser.add_argument('--size', type=float, default=0.1,
                        help='Размер тега в метрах (по умолчанию: 0.1)')
    parser.add_argument('--camera', type=int, default=0,
                        help='Индекс камеры (по умолчанию: 0)')
    parser.add_argument('--no-fps', action='store_false', dest='show_fps',
                        help='Не показывать FPS')
    parser.add_argument('--debug', action='store_true',
                        help='Включить режим отладки с дополнительной информацией и логированием')
    parser.add_argument('--send-data', action='store_true',
                        help='Отправлять данные по UDP')
    parser.add_argument('--port', type=int, default=5555,
                        help='Порт для UDP соединения (по умолчанию: 5555)')
    parser.add_argument('--history-length', type=int, default=5,
                        help='Длина истории для сглаживания координат (по умолчанию: 5)')
    parser.add_argument('--3d', action='store_true', dest='show_3d',
                        help='Показывать 3D визуализацию положения')
    
    args = parser.parse_args()
    
    # Создание и запуск трекера
    tracker = AprilTagTracker(
        tag_family=args.family,
        tag_size=args.size,
        camera_index=args.camera,
        show_fps=args.show_fps,
        debug=args.debug,
        send_data=args.send_data,
        port=args.port,
        history_length=args.history_length,
        show_3d=args.show_3d
    )
    
    tracker.start()

if __name__ == "__main__":
    main()
