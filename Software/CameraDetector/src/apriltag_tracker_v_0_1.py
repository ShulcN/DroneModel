import cv2
import numpy as np
import time
import apriltag
import argparse

def load_camera_params():
    """
    Загрузка параметров калибровки камеры
    """
    try:
        data = np.load('camera_calibration.npz')
        camera_matrix = data['camera_matrix']
        dist_coeffs = data['dist_coeffs']
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

def track_apriltag(tag_family="tag36h11", tag_size=0.1, camera_index=0, show_fps=True, debug=False):
    """
    Отслеживание AprilTag в реальном времени
    
    Args:
        tag_family: Семейство тегов AprilTag (по умолчанию tag36h11)
        tag_size: Размер тега в метрах
        camera_index: Индекс камеры
        show_fps: Показывать ли FPS
        debug: Включить ли режим отладки с дополнительной информацией
    """
    # Загрузка параметров камеры
    camera_matrix, dist_coeffs = load_camera_params()
    
    # Инициализация видеопотока
    cap = cv2.VideoCapture(camera_index)
    
    # Установка разрешения камеры (если требуется)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Создание детектора AprilTag
    options = apriltag.DetectorOptions(families=tag_family)
    detector = apriltag.Detector(options)
    
    # Для расчета FPS
    prev_frame_time = 0
    new_frame_time = 0
    
    # Для фильтрации координат (простой фильтр скользящего среднего)
    position_history = []
    history_length = 5  # Количество последних позиций для усреднения
    
    # Для записи координат
    if debug:
        log_file = open("apriltag_coordinates.csv", "w")
        log_file.write("timestamp,tag_id,x,y,z,roll,pitch,yaw\n")
    
    print("Начало отслеживания AprilTag...")
    print("Нажмите 'q' для выхода")
    
    try:
        while True:
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
            if show_fps:
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                        help='Включить режим отладки с дополнительной информацией и логированием')
    
    args = parser.parse_args()
    
    track_apriltag(
        tag_family=args.family,
        tag_size=args.size,
        camera_index=args.camera,
        show_fps=args.show_fps,
        debug=args.debug
    )    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Обработка результатов
            for r in results:
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
                    [-tag_size/2, -tag_size/2, 0],
                    [ tag_size/2, -tag_size/2, 0],
                    [ tag_size/2,  tag_size/2, 0],
                    [-tag_size/2,  tag_size/2, 0]
                ])
                
                img_pts = r.corners
                
                # Решение PnP задачи
                success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)
                
                if success:
                    # Получаем координаты X, Y, Z относительно камеры
                    x, y, z = tvec.flatten()
                    
                    # Добавляем в историю для сглаживания
                    position_history.append((x, y, z))
                    if len(position_history) > history_length:
                        position_history.pop(0)
                    
                    # Усредняем последние позиции
                    x_avg, y_avg, z_avg = np.mean(position_history, axis=0)
                    
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
                    
                    if debug:
                        cv2.putText(frame, f"Roll: {np.degrees(roll):.1f}°", (center[0] + 10, center[1] + 80), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        cv2.putText(frame, f"Pitch: {np.degrees(pitch):.1f}°", (center[0] + 10, center[1] + 100), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        cv2.putText(frame, f"Yaw: {np.degrees(yaw):.1f}°", (center[0] + 10, center[1] + 120), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    # Рисуем оси координат
                    axis_length = tag_size
                    axis_pts, _ = cv2.projectPoints(np.array([
                        [0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]
                    ]), rvec, tvec, camera_matrix, dist_coeffs)
                    
                    origin = tuple(axis_pts[0].ravel().astype(int))
                    x_end = tuple(axis_pts[1].ravel().astype(int))
                    y_end = tuple(axis_pts[2].ravel().astype(int))
                    z_end = tuple(axis_pts[3].ravel().astype(int))
                    
                    cv2.line(frame, origin, x_end, (0, 0, 255), 2)  # X-axis (красный)
                    cv2.line(frame, origin, y_end, (0, 255, 0), 2)  # Y-axis (зеленый)
                    cv2.line(frame, origin, z_end, (255, 0, 0), 2)  # Z-axis (синий)
                    
                    # Запись данных в лог
                    if debug:
                        timestamp = time.time()
                        log_file.write(f"{timestamp},{tag_id},{x_avg:.6f},{y_avg:.6f},{z_avg:.6f},"
                                    f"{np.degrees(roll):.2f},{np.degrees(pitch):.2f},{np.degrees(yaw):.2f}\n")
                        log_file.flush()
                        
                    # Выводим данные в консоль
                    print(f"Tag {tag_id}: X={x_avg:.3f}m, Y={y_avg:.3f}m, Z={z_avg:.3f}m", end="\r")
            
            # Отображение кадра
            cv2.imshow("AprilTag Tracking", frame)
            
            # Выход по нажатию клавиши 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if debug:
            log_file.close()
            print("\nКоординаты сохранены в файл apriltag_coordinates.csv")

if __name__ == "__main__":
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
