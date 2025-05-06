import cv2
import numpy as np
import time

def calibrate_camera():
    """
    Калибровка камеры с использованием шахматного поля
    Возвращает матрицу камеры и коэффициенты искажений
    """
    # Размеры шахматного поля (число внутренних углов)
    CHECKERBOARD = (5, 7)  # Пример размера, измените в соответствии с вашим полем
    
    # Остановка калибровки после указанного количества успешных обнаружений
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    
    # Подготовка координат объекта (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    # Реальный размер квадрата в мм (из вашего условия - 31 мм)
    square_size = 0.031  # в метрах
    objp = objp * square_size
    
    # Смещение координатной системы робота относительно шахматного поля
    # X = 160 мм, Y = -124 мм (из вашего условия)
    robot_offset_x = 0.160  # в метрах
    robot_offset_y = -0.124  # в метрах
    
    # Массивы для хранения точек объекта и изображения со всех изображений
    objpoints = []  # 3d точки в реальном мире
    imgpoints = []  # 2d точки в плоскости изображения
    
    cap = cv2.VideoCapture(0)
    img_counter = 0
    required_samples = 25  # Количество образцов для калибровки
    
    print("Начало процесса калибровки...")
    print("Поместите шахматное поле перед камерой")
    print("Нажмите 'c' для захвата изображения, 'q' для завершения калибровки")
    print("Рекомендации для лучшей калибровки:")
    print("1. Держите шахматное поле неподвижно при съемке")
    print("2. Сделайте снимки с разных ракурсов и расстояний")
    print("3. Поместите шахматное поле в разных частях кадра")
    print("4. Убедитесь, что освещение равномерное, без бликов")
    while img_counter < required_samples:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка при получении кадра")
            break
        #lwr = np.array([0, 0, 143])
        #upr = np.array([179, 61, 252])
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #msk = cv2.inRange(hsv, lwr, upr)

        # Extract chess-board
        #krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
        #dlt = cv2.dilate(msk, krn, iterations=5)
        #res = 255 - cv2.bitwise_and(dlt, msk)

        # Displaying chess-board features
        #res = np.uint8(res)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Поиск углов шахматного поля
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        
        # Если нашли, добавляем точки объекта и изображения (после уточнения)
        if ret:
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)
            
            # Отображение рамки, чтобы показать, что шахматное поле обнаружено
            cv2.putText(frame, "Шахматное поле обнаружено!", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('Калибровка камеры', frame)
        
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('c') and ret:
            print('lol')
            # Уточняем положение углов
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            img_counter += 1
            print(f"Изображение {img_counter}/{required_samples} захвачено")
            time.sleep(0.5)  # Небольшая пауза для перемещения шахматного поля
    
    cap.release()
    cv2.destroyAllWindows()
    
    if img_counter > 0:
        print("Выполняется калибровка...")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )
        
        print("Калибровка завершена!")
        print(f"Матрица камеры:\n{camera_matrix}")
        print(f"Коэффициенты искажений:\n{dist_coeffs}")
        
        # Сохраняем результаты калибровки
        np.savez('camera_calibration.npz', 
                 camera_matrix=camera_matrix, 
                 dist_coeffs=dist_coeffs,
                 robot_offset_x=robot_offset_x,
                 robot_offset_y=robot_offset_y)
        
        print("Параметры калибровки сохранены в camera_calibration.npz")
        
        return camera_matrix, dist_coeffs, robot_offset_x, robot_offset_y
    else:
        print("Недостаточно изображений для калибровки")
        return None, None, None, None

def test_with_checkerboard():
    """
    Функция для тестирования определения расстояния с помощью шахматного поля
    """
    try:
        # Загрузка параметров калибровки
        data = np.load('camera_calibration.npz')
        camera_matrix = data['camera_matrix']
        dist_coeffs = data['dist_coeffs']
        robot_offset_x = data['robot_offset_x']
        robot_offset_y = data['robot_offset_y']
    except:
        print("Файл калибровки не найден. Сначала выполните калибровку.")
        return
    
    # Размеры шахматного поля
    CHECKERBOARD = (5, 7)  # То же, что и при калибровке
    square_size = 0.031  # метры (31 мм)
    
    # Подготовка 3D координат шахматного поля
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * square_size
    
    cap = cv2.VideoCapture(0)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка при получении кадра")
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Поиск углов шахматного поля
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        
        if ret:
            # Уточняем положение углов
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Рисуем углы
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
            
            # Решаем PnP для определения положения шахматного поля
            success, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)
            
            if success:
                # Получаем координаты
                x, y, z = tvec.flatten()
                
                # Расстояние до шахматного поля (z-координата)
                cv2.putText(frame, f"Distance: {z:.3f} m", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"X: {x:.3f} m", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Y: {y:.3f} m", (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Рисуем оси координат
                axis_length = 0.1  # 10 см
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
                
                # Применяем смещение робота к полученным координатам
                robot_x = x + robot_offset_x
                robot_y = y + robot_offset_y
                
                cv2.putText(frame, f"Robot X: {robot_x:.3f} m", (10, 120), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(frame, f"Robot Y: {robot_y:.3f} m", (10, 150), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        cv2.imshow('Тестирование с шахматным полем', frame)
        
        # Выход по нажатию клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("Выберите режим:")
    print("1 - Калибровка камеры")
    print("2 - Тестирование с шахматным полем")
    
    choice = input("Введите номер: ")
    
    if choice == '1':
        calibrate_camera()
    elif choice == '2':
        test_with_checkerboard()
    else:
        print("Неверный выбор")
